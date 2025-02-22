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
#include "hydra_visualizer/utils/visualizer_types.h"

#include <spark_dsg/node_attributes.h>

#include "hydra_visualizer/color/colormap_utilities.h"

namespace hydra::visualizer {

using namespace spark_dsg;

Color DefaultNodeColorFunction::operator()(const SceneGraphNode& node) const {
  try {
    return node.attributes<SemanticNodeAttributes>().color;
  } catch (const std::bad_cast&) {
    return {};
  }
}

std::string DefaultNodeTextFunction::operator()(const SceneGraphNode&) const {
  return "";
}

Color DefaultEdgeColorFunction::operator()(const SceneGraphNode&,
                                           const SceneGraphNode&,
                                           const SceneGraphEdge&,
                                           bool) const {
  return {};
}

double LayerInfo::getZOffset() const {
  return graph.collapse_layers ? 0 : layer.z_offset_scale * graph.layer_z_step;
}

bool LayerInfo::shouldVisualize(const spark_dsg::SceneGraphNode& node) const {
  if (!layer.visualize) {
    return false;
  }

  if (filter && !filter(node)) {
    return false;
  }

  return true;
}

LayerInfo GraphInfo::getLayerInfo(LayerKey key) const {
  if (key.partition) {
    auto iter = layer_partitions.find(key.layer);
    return iter == layer_partitions.end() ? LayerInfo() : iter->second;
  } else {
    auto iter = layers.find(key.layer);
    return iter == layers.end() ? LayerInfo() : iter->second;
  }
}

GraphInfo::EdgeInformation GraphInfo::getEdgeInfo(LayerKey source_layer,
                                                  const SceneGraphNode& source,
                                                  LayerKey target_layer,
                                                  const SceneGraphNode& target) const {
  const auto source_info = getLayerInfo(source_layer);
  if (!source_info.shouldVisualize(source)) {
    return {};
  }

  const auto target_info = getLayerInfo(target_layer);
  if (!target_info.shouldVisualize(target)) {
    return {};
  }

  if (!source_info.layer.draw_interlayer_edges ||
      !target_info.layer.draw_interlayer_edges) {
    return {};
  }

  const auto& info =
      source_info.layer.interlayer_edge_use_source ? source_info : target_info;
  const auto alpha = info.layer.interlayer_edge_alpha;
  const auto color = makeColorMsg(info.node_color(source), alpha);
  return {true,
          static_cast<size_t>(info.layer.interlayer_edge_insertion_skip),
          info.layer.interlayer_edge_scale,
          color,
          source_info.getZOffset(),
          target_info.getZOffset()};
}

}  // namespace hydra::visualizer
