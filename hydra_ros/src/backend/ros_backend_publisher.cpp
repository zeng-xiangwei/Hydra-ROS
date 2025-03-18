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
#include "hydra_ros/backend/ros_backend_publisher.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/ros.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <geometry_msgs/TransformStamped.h>
#include <gtsam/geometry/Pose3.h>
#include <hydra/common/global_info.h>
#include <kimera_pgmo_ros/visualization_functions.h>
#include <pose_graph_tools_msgs/PoseGraph.h>
#include <pose_graph_tools_ros/conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <visualization_msgs/Marker.h>

namespace hydra {

using kimera_pgmo::DeformationGraph;
using kimera_pgmo::KimeraPgmoConfig;
using kimera_pgmo_msgs::KimeraPgmoMesh;
using pose_graph_tools_msgs::PoseGraph;
using visualization_msgs::Marker;

void declare_config(RosBackendPublisher::Config& config) {
  using namespace config;
  name("RosBackendPublisher::Config");
  field(config.publish_mesh, "publish_mesh");
  field(config.publish_backend_tf, "publish_backend_tf");
  field(config.tf_pub_robot_frame, "tf_pub_robot_frame");
  field(config.tf_pub_map_frame, "tf_pub_map_frame");
  field(config.tf_pub_odom_frame, "tf_pub_odom_frame");
}

RosBackendPublisher::RosBackendPublisher(const ros::NodeHandle& nh)
    : config(config::checkValid(config::fromRos<Config>(nh))), nh_(nh) {
  mesh_mesh_edges_pub_ =
      nh_.advertise<Marker>("deformation_graph_mesh_mesh", 10, false);
  pose_mesh_edges_pub_ =
      nh_.advertise<Marker>("deformation_graph_pose_mesh", 10, false);
  pose_graph_pub_ = nh_.advertise<PoseGraph>("pose_graph", 10, false);
  mesh_graph_pub_ = nh_.advertise<PoseGraph>("mesh_graph", 10, false);

  const auto map_frame = GlobalInfo::instance().getFrames().map;
  dsg_sender_.reset(
      new hydra::DsgSender(nh_, map_frame, "backend", config.publish_mesh));
}

void RosBackendPublisher::call(uint64_t timestamp_ns,
                               const DynamicSceneGraph& graph,
                               const DeformationGraph& dgraph) const {
  ros::Time stamp;
  stamp.fromNSec(timestamp_ns);
  dsg_sender_->sendGraph(graph, stamp);

  if (pose_graph_pub_.getNumSubscribers() > 0) {
    publishPoseGraph(graph, dgraph);
  }

  if (mesh_graph_pub_.getNumSubscribers() > 0) {
    publishMeshGraph(graph, dgraph);
  }

  if (mesh_mesh_edges_pub_.getNumSubscribers() > 0 ||
      pose_mesh_edges_pub_.getNumSubscribers() > 0) {
    publishDeformationGraphViz(dgraph, timestamp_ns);
  }

  if (config.publish_backend_tf) {
    publishTf(graph, dgraph);
  }
}

std::string RosBackendPublisher::printInfo() const { return config::toString(config); }

void RosBackendPublisher::publishTf(const DynamicSceneGraph& graph,
                                    const DeformationGraph& dgraph) const {
  const auto& prefix = GlobalInfo::instance().getRobotPrefix();

  const auto& frames = GlobalInfo::instance().getFrames();
  std::string map_frame = config.tf_pub_map_frame;
  if (map_frame == "") {
    map_frame = frames.map;
  }

  std::string odom_frame = config.tf_pub_odom_frame;
  if (odom_frame == "") {
    odom_frame = frames.odom;
  }

  const auto layer_id = graph.getLayerKey(DsgLayers::AGENTS)->layer;
  const auto& agents = graph.getLayer(layer_id, prefix.key);
  int64_t agent_stamp = 0;

  gtsam::Pose3 map_T_odom;
  gtsam::Pose3 map_T_body;

  if (agents.numNodes()) {
    NodeSymbol pgmo_key(prefix.key, agents.numNodes() - 1);
    const auto& attrs = graph.getNode(pgmo_key).attributes<AgentNodeAttributes>();

    const auto& odom_T_body = dgraph.getInitialPose(prefix.key, pgmo_key.categoryId());
    map_T_body = gtsam::Pose3(gtsam::Rot3(attrs.world_R_body), attrs.position);
    map_T_odom = map_T_body * odom_T_body.inverse();

    agent_stamp =
        graph.getNode(pgmo_key).attributes<AgentNodeAttributes>().timestamp.count();
  }

  std::vector<geometry_msgs::TransformStamped> transforms;
  // Build map->odom transform message
  auto& tf = transforms.emplace_back();
  tf.header.frame_id = map_frame;
  tf.header.stamp = ros::Time::now();
  tf.child_frame_id = odom_frame;
  tf2::convert(map_T_odom.rotation().toQuaternion(), tf.transform.rotation);
  tf2::toMsg(map_T_odom.translation(), tf.transform.translation);

  if (config.tf_pub_robot_frame != "") {
    // Build map->robot transform message
    geometry_msgs::TransformStamped tf_robot;
    tf_robot.header.stamp = ros::Time::now();
    tf_robot.header.frame_id = map_frame;
    tf_robot.child_frame_id = config.tf_pub_robot_frame;
    tf2::convert(map_T_body.rotation().toQuaternion(), tf_robot.transform.rotation);
    tf2::toMsg(map_T_body.translation(), tf_robot.transform.translation);
    if (agent_stamp == 0) {
      // Publish identity map->robot TF if there are no agent nodes in the scene graph
      // yet
      tf_robot.header.stamp = ros::Time::now();
      transforms.push_back(tf_robot);
    } else if (agent_stamp != last_robot_tf_stamp_) {
      // Publish TF at most once per agent node
      tf_robot.header.stamp.fromNSec(agent_stamp);
      last_robot_tf_stamp_ = agent_stamp;
      transforms.push_back(tf_robot);
    }
  }

  tf_br_.sendTransform(transforms);
}

void RosBackendPublisher::publishPoseGraph(const DynamicSceneGraph& graph,
                                           const DeformationGraph& dgraph) const {
  const auto& prefix = GlobalInfo::instance().getRobotPrefix();
  const auto layer_id = graph.getLayerKey(DsgLayers::AGENTS)->layer;
  const auto& agent = graph.getLayer(layer_id, prefix.key);

  std::map<size_t, std::vector<size_t>> id_timestamps;
  id_timestamps[prefix.id] = std::vector<size_t>();
  auto& times = id_timestamps[prefix.id];
  for (const auto& [node_id, node] : agent.nodes()) {
    times.push_back(node->attributes<AgentNodeAttributes>().timestamp.count());
  }

  const auto pose_graph = *dgraph.getPoseGraph(id_timestamps, false, true);
  const auto map_frame = GlobalInfo::instance().getFrames().map;
  auto pose_graph_msg = pose_graph_tools::toMsg(pose_graph);
  pose_graph_msg.header.frame_id = map_frame;
  pose_graph_pub_.publish(pose_graph_msg);
}

void RosBackendPublisher::publishMeshGraph(const DynamicSceneGraph&,
                                           const DeformationGraph& dgraph) const {
  std::map<size_t, std::vector<size_t>> id_timestamps_temp;
  const auto mesh_graph = *dgraph.getPoseGraph(id_timestamps_temp, true, false);
  const auto map_frame = GlobalInfo::instance().getFrames().map;
  auto mesh_graph_msg = pose_graph_tools::toMsg(mesh_graph);
  mesh_graph_msg.header.frame_id = map_frame;
  mesh_graph_pub_.publish(mesh_graph_msg);
}

void RosBackendPublisher::publishDeformationGraphViz(const DeformationGraph& dgraph,
                                                     size_t timestamp_ns) const {
  ros::Time stamp;
  stamp.fromNSec(timestamp_ns);

  Marker mm_edges_msg;
  Marker pm_edges_msg;
  kimera_pgmo::fillDeformationGraphMarkers(dgraph,
                                           stamp,
                                           mm_edges_msg,
                                           pm_edges_msg,
                                           GlobalInfo::instance().getFrames().map);

  if (!mm_edges_msg.points.empty()) {
    mesh_mesh_edges_pub_.publish(mm_edges_msg);
  }
  if (!pm_edges_msg.points.empty()) {
    pose_mesh_edges_pub_.publish(pm_edges_msg);
  }
}

}  // namespace hydra
