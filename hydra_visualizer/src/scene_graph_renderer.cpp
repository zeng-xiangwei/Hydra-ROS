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
#include "hydra_visualizer/scene_graph_renderer.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/ros.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/printing.h>
#include <std_msgs/String.h>
#include <tf2_eigen/tf2_eigen.h>

#include "hydra_visualizer/color/colormap_utilities.h"
#include "hydra_visualizer/utils/config_manager.h"
#include "hydra_visualizer/utils/visualizer_utilities.h"

namespace hydra {
namespace {

inline std::string keyToLayerName(spark_dsg::LayerKey key) {
  std::stringstream ss;
  ss << "layer_" << key.layer;
  if (key.partition) {
    ss << "p" << key.partition;
  }

  return ss.str();
}

}  // namespace

using namespace spark_dsg;
using namespace visualizer;

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

struct MarkerNamespaces {
  static std::string layerNodeNamespace(LayerKey key) {
    return keyToLayerName(key) + "_nodes";
  }

  static std::string layerEdgeNamespace(LayerKey key) {
    return keyToLayerName(key) + "_edges";
  }

  static std::string layerTextNamespace(LayerKey key) {
    return keyToLayerName(key) + "_text";
  }

  static std::string layerBboxNamespace(LayerKey key) {
    return keyToLayerName(key) + "_bounding_boxes";
  }

  static std::string layerBoundaryNamespace(LayerKey key) {
    return keyToLayerName(key) + "_polygon_boundaries";
  }

  static std::string layerBoundaryEllipseNamespace(LayerKey key) {
    return keyToLayerName(key) + "_ellipsoid_boundaries";
  }

  static std::string layerBoundaryEdgeNamespace(LayerKey key) {
    return keyToLayerName(key) + "_polygon_boundaries_edges";
  }

  static std::string meshEdgeNamespace(LayerKey key) {
    return keyToLayerName(key) + "_mesh_edges";
  }
};

SceneGraphRenderer::SceneGraphRenderer(const ros::NodeHandle& nh)
    : nh_(nh), pub_(nh_.advertise<MarkerArray>("dsg_markers", 1, true)) {
  ConfigManager::init(nh_);
}

void SceneGraphRenderer::reset(const std_msgs::Header& header) {
  MarkerArray msg;
  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_.publish(msg);
  }

  // TODO(nathan) think if we actually want to do this
  // ConfigManager::reset();
}

bool SceneGraphRenderer::hasChange() const {
  return ConfigManager::instance().hasChange();
}

void SceneGraphRenderer::clearChangeFlag() {
  ConfigManager::instance().clearChangeFlags();
}

void SceneGraphRenderer::draw(const std_msgs::Header& header,
                              const DynamicSceneGraph& graph) {
  visualizer::GraphInfo info;
  const auto& manager = ConfigManager::instance();

  MarkerArray msg;
  for (const auto& [layer_id, layer] : graph.layers()) {
    const auto& conf = manager.getLayerConfig(layer_id);
    auto iter = info.layers.emplace(layer_id, conf.getInfo(graph)).first;
    drawLayer(header, iter->second, *layer, graph.mesh().get(), msg);
    for (const auto& [partition_id, partition] : graph.layer_partition(layer_id)) {
      const auto& conf = manager.getPartitionLayerConfig(layer_id);
      auto iter = info.layer_partitions.emplace(layer_id, conf.getInfo(graph)).first;
      drawLayer(header, iter->second, *partition, graph.mesh().get(), msg);
    }
  }

  const auto edges = makeGraphEdgeMarkers(
      header, info, graph, graph.interlayer_edges(), "interlayer_edges_");
  tracker_.add(edges, msg);

  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_.publish(msg);
  }
}

void SceneGraphRenderer::drawLayer(const std_msgs::Header& header,
                                   const LayerInfo& info,
                                   const SceneGraphLayer& layer,
                                   const Mesh* mesh,
                                   MarkerArray& msg) {
  if (!info.layer.visualize) {
    return;
  }

  if (info.layer.draw_frontier_ellipse) {
    info.filter = [](const SceneGraphNode& node) {
      try {
        return node.attributes<PlaceNodeAttributes>().real_place;
      } catch (const std::bad_cast&) {
        return true;
      }
    };
  }

  const auto node_ns = MarkerNamespaces::layerNodeNamespace(layer.id);
  tracker_.add(makeLayerNodeMarkers(header, info, layer, node_ns), msg);

  if (info.layer.use_text) {
    if (info.layer.use_layer_text) {
      LOG_FIRST_N(WARNING, 5) << "use_text and use_layer_text are mutually exclusive!";
    }

    const auto ns = MarkerNamespaces::layerTextNamespace(layer.id);
    tracker_.add(makeLayerNodeTextMarkers(header, info, layer, ns), msg);
  } else if (info.layer.use_layer_text && !layer.nodes().empty()) {
    const auto ns = MarkerNamespaces::layerTextNamespace(layer.id);
    tracker_.add(makeLayerTextMarker(header, info, layer, ns), msg);
  }

  if (info.layer.use_bounding_box) {
    const auto ns = MarkerNamespaces::layerBboxNamespace(layer.id);
    try {
      tracker_.add(makeLayerBoundingBoxes(header, info, layer, ns), msg);
    } catch (const std::bad_cast& e) {
      LOG_FIRST_N(WARNING, 5) << "unable to draw bounding boxes for layer " << layer.id
                              << ": " << e.what();
    }
  }

  if (info.layer.draw_boundaries) {
    const auto ns = MarkerNamespaces::layerBoundaryNamespace(layer.id);
    const auto edge_ns = MarkerNamespaces::layerBoundaryEdgeNamespace(layer.id);
    try {
      tracker_.add(makeLayerPolygonBoundaries(header, info, layer, ns), msg);
      if (info.layer.collapse_boundary) {
        tracker_.add(makeLayerPolygonEdges(header, info, layer, edge_ns), msg);
      }
    } catch (const std::bad_cast& e) {
      LOG_FIRST_N(WARNING, 5) << "Could not draw boundaries for layer " << layer.id
                              << ": " << e.what();
    }
  }

  if (info.layer.draw_boundary_ellipse) {
    const auto ns = MarkerNamespaces::layerBoundaryEllipseNamespace(layer.id);
    try {
      tracker_.add(makeLayerEllipseBoundaries(header, info, layer, ns), msg);
    } catch (const std::bad_cast& e) {
      LOG_FIRST_N(WARNING, 5) << "Could not draw boundary ellipses for layer "
                              << layer.id << ": " << e.what();
    }
  }

  if (info.layer.draw_frontier_ellipse) {
    tracker_.add(makeEllipsoidMarkers(header, info, layer, "frontier_ns"), msg);
    info.filter = {};  // we reset the manual filter to draw edges to frontiers
  }

  const auto edge_ns = MarkerNamespaces::layerEdgeNamespace(layer.id);
  tracker_.add(makeLayerEdgeMarkers(header, info, layer, edge_ns), msg);

  if (mesh && info.layer.draw_mesh_edges) {
    const std::string ns = MarkerNamespaces::meshEdgeNamespace(layer.id);
    tracker_.add(makeMeshEdgesMarker(header, info, layer, *mesh, ns), msg);
  }
}

}  // namespace hydra
