/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra_ros/frontend/ros_frontend_publisher.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <hydra/common/global_info.h>

namespace hydra {

using kimera_pgmo::MeshDeltaTypeAdapter;
using pose_graph_tools::PoseGraphTypeAdapter;
using BaseInterface = rclcpp::node_interfaces::NodeBaseInterface;
using rclcpp::CallbackGroupType;

namespace {

inline RosFrontendPublisher::Config get_config() {
  const auto odom_frame = GlobalInfo::instance().getFrames().odom;
  auto config = config::fromContext<RosFrontendPublisher::Config>("frontend");
  config.dsg_sender = config.dsg_sender.with_name("frontend").with_frame(odom_frame);
  return config;
}

}  // namespace

void declare_config(RosFrontendPublisher::Config& config) {
  using namespace config;
  name("RosFrontendPublisher::Config");
  field(config.dsg_sender, "");
  field(config.mesh_delta_queue_size, "mesh_delta_queue_size");
}

RosFrontendPublisher::RosFrontendPublisher(ianvs::NodeHandle nh)
    : config(config::checkValid(get_config())) {
  auto group = nh.as<BaseInterface>()->create_callback_group(
      CallbackGroupType::MutuallyExclusive);
  mesh_delta_server_ =
      nh.create_service<MeshDeltaSrv>("mesh_delta_query",
                                      &RosFrontendPublisher::processMeshDeltaQuery,
                                      this,
                                      rclcpp::ServicesQoS(),
                                      group);
  dsg_sender_ = std::make_unique<DsgSender>(config.dsg_sender, nh);
  mesh_graph_pub_ = nh.create_publisher<PoseGraphTypeAdapter>(
      "mesh_graph_incremental", rclcpp::QoS(100).transient_local());
  mesh_update_pub_ = nh.create_publisher<MeshDeltaTypeAdapter>(
      "full_mesh_update", rclcpp::QoS(100).transient_local());
}

void RosFrontendPublisher::call(uint64_t timestamp_ns,
                                const DynamicSceneGraph& graph,
                                const BackendInput& backend_input) const {
  // TODO(nathan) make sure pgmo stamps the deformation graph
  mesh_graph_pub_->publish(backend_input.deformation_graph);
  if (backend_input.mesh_update) {
    backend_input.mesh_update->timestamp_ns = timestamp_ns;
    mesh_update_pub_->publish(*backend_input.mesh_update);
    stored_delta_.insert(
        {backend_input.mesh_update->sequence_number, backend_input.mesh_update});
    if (config.mesh_delta_queue_size > 0 &&
        stored_delta_.size() > static_cast<size_t>(config.mesh_delta_queue_size)) {
      stored_delta_.erase(stored_delta_.begin());
    }
  }

  dsg_sender_->sendGraph(graph, rclcpp::Time(timestamp_ns));
}

void RosFrontendPublisher::processMeshDeltaQuery(
    const MeshDeltaSrv::Request::SharedPtr req,
    MeshDeltaSrv::Response::SharedPtr resp) {
  LOG(INFO) << "Received request for " << req->sequence_numbers.size()
            << " mesh deltas...";
  for (const auto& seq : req->sequence_numbers) {
    auto& msg = resp->deltas.emplace_back();
    // Check TypeAdater documentation TODO(Yun)
    if (!stored_delta_.count(seq)) {
      LOG(ERROR) << "Mesh delta sequence " << seq << " not found";
      continue;
    }
    mesh_delta_converter_.convert_to_ros_message(*stored_delta_.at(seq), msg);
  }
  LOG(INFO) << "Responding with " << resp->deltas.size() << " deltas...";
}

}  // namespace hydra
