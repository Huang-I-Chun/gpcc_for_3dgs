/* The copyright in this software is being made available under the BSD
 * Licence, included below.  This software may be subject to other third
 * party and contributor rights, including patent rights, and no such
 * rights are granted under this licence.
 *
 * Copyright (c) 2017-2018, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of the ISO/IEC nor the names of its contributors
 *   may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "pointset_processing.h"

#include "colourspace.h"
#include "hls.h"
#include "KDTreeVectorOfVectorsAdaptor.h"

#include <array>
#include <cstddef>
#include <set>
#include <vector>
#include <utility>
#include <map>

namespace pcc {

//============================================================================

template<typename UniqueFn, typename QFn>
SrcMappedPointSet
reducePointSet(const PCCPointSet3& src, UniqueFn uniqueFn, QFn qFn)
{
  SrcMappedPointSet dst;
  int numSrcPoints = src.getPointCount();

  // Build a map of duplicate points
  int numDstPoints = 0;
  if (1) {
    std::map<Vec3<int32_t>, int> qPosToSrcIdx;
    dst.srcIdxDupList.resize(numSrcPoints);
    for (int i = numSrcPoints - 1; i >= 0; i--) {
      // Attempt to insert quantised position
      auto res = qPosToSrcIdx.insert({uniqueFn(src[i]), i});

      // Append to linked list of same positions.
      // Index of the src point (i) or the index of the previous point with
      // the same quantised position
      dst.srcIdxDupList[res.first->second] ^= 0x80000000;
      dst.srcIdxDupList[i] = res.first->second | 0x80000000;
      res.first->second = i;
    }

    numDstPoints = qPosToSrcIdx.size();
  }

  // Number of quantised points is now known
  dst.cloud.resize(numDstPoints);
  dst.idxToSrcIdx.resize(numDstPoints);
  if (src.hasLaserAngles())
    dst.cloud.addLaserAngles();

  // Generate dst outputs
  for (int i = 0, dstIdx = 0; i < numSrcPoints; ++i) {
    // Find head of each linked list
    if (dst.srcIdxDupList[i] >= 0)
      continue;

    dst.srcIdxDupList[i] ^= 0x80000000;
    dst.idxToSrcIdx[dstIdx] = i;
    if (src.hasLaserAngles() == true)
      dst.cloud.setLaserAngle(dstIdx, src.getLaserAngle(i));
    dst.cloud[dstIdx++] = qFn(src[i]);
  }

  // Add attribute storage to match src
  dst.cloud.addRemoveAttributes(src.hasColors(), src.hasReflectances());
  dst.cloud.addRemoveOpacities(src.hasOpacities());
  dst.cloud.addRemovef_dc_0(src.hasf_dc_0());
  dst.cloud.addRemovef_dc_1(src.hasf_dc_1());
  dst.cloud.addRemovef_dc_2(src.hasf_dc_2());
  dst.cloud.addRemovescale_0(src.hasscale_0());
  dst.cloud.addRemovescale_1(src.hasscale_1());
  dst.cloud.addRemovescale_2(src.hasscale_2());
  dst.cloud.addRemoverot_0(src.hasrot_0());
  dst.cloud.addRemoverot_1(src.hasrot_1());
  dst.cloud.addRemoverot_2(src.hasrot_2());
  dst.cloud.addRemoverot_3(src.hasrot_3());
  dst.cloud.addRemovef_rest_0(src.hasf_rest_0());
  dst.cloud.addRemovef_rest_1(src.hasf_rest_1());
  dst.cloud.addRemovef_rest_2(src.hasf_rest_2());
  dst.cloud.addRemovef_rest_3(src.hasf_rest_3());
  dst.cloud.addRemovef_rest_4(src.hasf_rest_4());
  dst.cloud.addRemovef_rest_5(src.hasf_rest_5());
  dst.cloud.addRemovef_rest_6(src.hasf_rest_6());
  dst.cloud.addRemovef_rest_7(src.hasf_rest_7());
  dst.cloud.addRemovef_rest_8(src.hasf_rest_8());
  dst.cloud.addRemovef_rest_9(src.hasf_rest_9());
  dst.cloud.addRemovef_rest_10(src.hasf_rest_10());
  dst.cloud.addRemovef_rest_11(src.hasf_rest_11());
  dst.cloud.addRemovef_rest_12(src.hasf_rest_12());
  dst.cloud.addRemovef_rest_13(src.hasf_rest_13());
  dst.cloud.addRemovef_rest_14(src.hasf_rest_14());
  dst.cloud.addRemovef_rest_15(src.hasf_rest_15());
  dst.cloud.addRemovef_rest_16(src.hasf_rest_16());
  dst.cloud.addRemovef_rest_17(src.hasf_rest_17());
  dst.cloud.addRemovef_rest_18(src.hasf_rest_18());
  dst.cloud.addRemovef_rest_19(src.hasf_rest_19());
  dst.cloud.addRemovef_rest_20(src.hasf_rest_20());
  dst.cloud.addRemovef_rest_21(src.hasf_rest_21());
  dst.cloud.addRemovef_rest_22(src.hasf_rest_22());
  dst.cloud.addRemovef_rest_23(src.hasf_rest_23());
  dst.cloud.addRemovef_rest_24(src.hasf_rest_24());
  dst.cloud.addRemovef_rest_25(src.hasf_rest_25());
  dst.cloud.addRemovef_rest_26(src.hasf_rest_26());
  dst.cloud.addRemovef_rest_27(src.hasf_rest_27());
  dst.cloud.addRemovef_rest_28(src.hasf_rest_28());
  dst.cloud.addRemovef_rest_29(src.hasf_rest_29());
  dst.cloud.addRemovef_rest_30(src.hasf_rest_30());
  dst.cloud.addRemovef_rest_31(src.hasf_rest_31());
  dst.cloud.addRemovef_rest_32(src.hasf_rest_32());
  dst.cloud.addRemovef_rest_33(src.hasf_rest_33());
  dst.cloud.addRemovef_rest_34(src.hasf_rest_34());
  dst.cloud.addRemovef_rest_35(src.hasf_rest_35());
  dst.cloud.addRemovef_rest_36(src.hasf_rest_36());
  dst.cloud.addRemovef_rest_37(src.hasf_rest_37());
  dst.cloud.addRemovef_rest_38(src.hasf_rest_38());
  dst.cloud.addRemovef_rest_39(src.hasf_rest_39());
  dst.cloud.addRemovef_rest_40(src.hasf_rest_40());
  dst.cloud.addRemovef_rest_41(src.hasf_rest_41());
  dst.cloud.addRemovef_rest_42(src.hasf_rest_42());
  dst.cloud.addRemovef_rest_43(src.hasf_rest_43());
  dst.cloud.addRemovef_rest_44(src.hasf_rest_44());

  return dst;
}

//============================================================================
// Subsample a point cloud, retaining unique points only.
// Uniqueness is assessed by quantising each position by a multiplicative
// @sampleScale.  Output points are quantised by @quantScale with rounding,
// and translated by -@offset.
//
// NB: attributes are not processed.

SrcMappedPointSet
samplePositionsUniq(
  float sampleScale,
  float quantScale,
  Vec3<int> offset,
  const PCCPointSet3& src)
{
  auto diffScale = sampleScale / quantScale;

  return reducePointSet(
    src,
    [=](Vec3<int> point) {
      for (int k = 0; k < 3; k++)
        point[k] = std::round(std::round(point[k] * quantScale) * diffScale);
      return point;
    },
    [=](Vec3<int> point) {
      for (int k = 0; k < 3; k++)
        point[k] = std::round(point[k] * quantScale);
      return point - offset;
    });
}

//============================================================================
// Quantise the geometry of a point cloud, retaining unique points only.
// Points in the @src point cloud are translated by -@offset, quantised by a
// multiplicitive @scaleFactor with rounding, then clamped to @clamp.
//
// NB: attributes are not processed.

SrcMappedPointSet
quantizePositionsUniq(
  const float scaleFactor,
  const Vec3<int> offset,
  const Box3<int> clamp,
  const PCCPointSet3& src)
{
  auto qFn = [=](Vec3<int> point) {
    for (int k = 0; k < 3; k++) {
      double posk = std::round(point[k] * scaleFactor) - offset[k];
      point[k] = PCCClip(int32_t(posk), clamp.min[k], clamp.max[k]);
    }
    return point;
  };

  return reducePointSet(src, qFn, qFn);
}

//============================================================================
// Quantise the geometry of a point cloud, retaining duplicate points.
// Points in the @src point cloud are translated by -@offset, then quantised
// by a multiplicitive @scaleFactor with rounding.
//
// The destination and source point clouds may be the same object.
//
// NB: attributes are preserved

void
quantizePositions(
  const float scaleFactor,
  const Vec3<int> offset,
  const Box3<int> clamp,
  const PCCPointSet3& src,
  PCCPointSet3* dst)
{
  std::cout << "quantizePositions start" << std::endl;
  int numSrcPoints = src.getPointCount();
  std::cout << "src point:" << src.positions[0] << std::endl;
  std::cout << "src point:" << src.positions[1] << std::endl;
  std::cout << "src point:" << src.positions[2] << std::endl;
  std::cout << "src point:" << src.positions[3] << std::endl;

  // In case dst and src point clouds are the same, don't destroy src.
  if (&src != dst) {
    dst->clear();
    dst->addRemoveAttributes(src);
    dst->resize(numSrcPoints);
  }

  for (int i = 0; i < numSrcPoints; ++i) {
    const auto point = src[i];
    // std::cout << src[i] << std::endl;

    auto& dstPoint = (*dst)[i];
    for (int k = 0; k < 3; ++k) {
      double k_pos = std::round(point[k] * scaleFactor) - offset[k];
      dstPoint[k] = PCCClip(
        int32_t(k_pos), clamp.min[k], clamp.max[k]);  // quantize into int32
    }
  }
  std::cout << "offset: " << offset << std::endl;
  std::cout << "scaleFactor: " << scaleFactor << std::endl;

  // std::cout << "dst point:" << dst->positions[0] << std::endl;
  // std::cout << "dst point:" << dst->positions[1] << std::endl;
  // std::cout << "dst point:" << dst->positions[2] << std::endl;
  // std::cout << "dst point:" << dst->positions[3] << std::endl;

  // don't copy attributes if dst already has them
  if (&src == dst)
    return;

  if (src.hasColors()) {
    for (int i = 0; i < numSrcPoints; ++i)
      dst->setColor(i, src.getColor(i));
  }

  if (src.hasReflectances()) {
    for (int i = 0; i < numSrcPoints; ++i)
      dst->setReflectance(i, src.getReflectance(i));
  }

  if (src.hasOpacities()) {
    for (int i = 0; i < numSrcPoints; ++i)
      dst->setOpacity(i, src.getOpacity(i));
  }

  if (src.hasf_dc_0()) {
    for (int i = 0; i < numSrcPoints; ++i)
      dst->setf_dc_0(i, src.getf_dc_0(i));
  }

  if (src.hasf_dc_1()) {
    for (int i = 0; i < numSrcPoints; ++i)
      dst->setf_dc_1(i, src.getf_dc_1(i));
  }

  if (src.hasf_dc_2()) {
    for (int i = 0; i < numSrcPoints; ++i)
      dst->setf_dc_2(i, src.getf_dc_2(i));
  }

  if (src.hasscale_0()) {
    for (int i = 0; i < numSrcPoints; ++i)
      dst->setscale_0(i, src.getscale_0(i));
  }

  if (src.hasscale_1()) {
    for (int i = 0; i < numSrcPoints; ++i)
      dst->setscale_1(i, src.getscale_1(i));
  }

  if (src.hasscale_2()) {
    for (int i = 0; i < numSrcPoints; ++i)
      dst->setscale_2(i, src.getscale_2(i));
  }

  if (src.hasrot_0()) {
    for (int i = 0; i < numSrcPoints; ++i)
      dst->setrot_0(i, src.getrot_0(i));
  }

  if (src.hasrot_1()) {
    for (int i = 0; i < numSrcPoints; ++i)
      dst->setrot_1(i, src.getrot_1(i));
  }

  if (src.hasrot_2()) {
    for (int i = 0; i < numSrcPoints; ++i)
      dst->setrot_2(i, src.getrot_2(i));
  }

  if (src.hasrot_3()) {
    for (int i = 0; i < numSrcPoints; ++i)
      dst->setrot_3(i, src.getrot_3(i));
  }
  if (src.hasf_rest_0()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_0(i, src.getf_rest_0(i));
    }
  }
  if (src.hasf_rest_1()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_1(i, src.getf_rest_1(i));
    }
  }
  if (src.hasf_rest_2()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_2(i, src.getf_rest_2(i));
    }
  }
  if (src.hasf_rest_3()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_3(i, src.getf_rest_3(i));
    }
  }
  if (src.hasf_rest_4()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_4(i, src.getf_rest_4(i));
    }
  }
  if (src.hasf_rest_5()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_5(i, src.getf_rest_5(i));
    }
  }
  if (src.hasf_rest_6()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_6(i, src.getf_rest_6(i));
    }
  }
  if (src.hasf_rest_7()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_7(i, src.getf_rest_7(i));
    }
  }
  if (src.hasf_rest_8()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_8(i, src.getf_rest_8(i));
    }
  }
  if (src.hasf_rest_9()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_9(i, src.getf_rest_9(i));
    }
  }
  if (src.hasf_rest_10()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_10(i, src.getf_rest_10(i));
    }
  }
  if (src.hasf_rest_11()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_11(i, src.getf_rest_11(i));
    }
  }
  if (src.hasf_rest_12()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_12(i, src.getf_rest_12(i));
    }
  }
  if (src.hasf_rest_13()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_13(i, src.getf_rest_13(i));
    }
  }
  if (src.hasf_rest_14()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_14(i, src.getf_rest_14(i));
    }
  }
  if (src.hasf_rest_15()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_15(i, src.getf_rest_15(i));
    }
  }
  if (src.hasf_rest_16()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_16(i, src.getf_rest_16(i));
    }
  }
  if (src.hasf_rest_17()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_17(i, src.getf_rest_17(i));
    }
  }
  if (src.hasf_rest_18()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_18(i, src.getf_rest_18(i));
    }
  }
  if (src.hasf_rest_19()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_19(i, src.getf_rest_19(i));
    }
  }
  if (src.hasf_rest_20()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_20(i, src.getf_rest_20(i));
    }
  }
  if (src.hasf_rest_21()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_21(i, src.getf_rest_21(i));
    }
  }
  if (src.hasf_rest_22()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_22(i, src.getf_rest_22(i));
    }
  }
  if (src.hasf_rest_23()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_23(i, src.getf_rest_23(i));
    }
  }
  if (src.hasf_rest_24()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_24(i, src.getf_rest_24(i));
    }
  }
  if (src.hasf_rest_25()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_25(i, src.getf_rest_25(i));
    }
  }
  if (src.hasf_rest_26()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_26(i, src.getf_rest_26(i));
    }
  }
  if (src.hasf_rest_27()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_27(i, src.getf_rest_27(i));
    }
  }
  if (src.hasf_rest_28()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_28(i, src.getf_rest_28(i));
    }
  }
  if (src.hasf_rest_29()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_29(i, src.getf_rest_29(i));
    }
  }
  if (src.hasf_rest_30()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_30(i, src.getf_rest_30(i));
    }
  }
  if (src.hasf_rest_31()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_31(i, src.getf_rest_31(i));
    }
  }
  if (src.hasf_rest_32()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_32(i, src.getf_rest_32(i));
    }
  }
  if (src.hasf_rest_33()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_33(i, src.getf_rest_33(i));
    }
  }
  if (src.hasf_rest_34()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_34(i, src.getf_rest_34(i));
    }
  }
  if (src.hasf_rest_35()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_35(i, src.getf_rest_35(i));
    }
  }
  if (src.hasf_rest_36()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_36(i, src.getf_rest_36(i));
    }
  }
  if (src.hasf_rest_37()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_37(i, src.getf_rest_37(i));
    }
  }
  if (src.hasf_rest_38()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_38(i, src.getf_rest_38(i));
    }
  }
  if (src.hasf_rest_39()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_39(i, src.getf_rest_39(i));
    }
  }
  if (src.hasf_rest_40()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_40(i, src.getf_rest_40(i));
    }
  }
  if (src.hasf_rest_41()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_41(i, src.getf_rest_41(i));
    }
  }
  if (src.hasf_rest_42()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_42(i, src.getf_rest_42(i));
    }
  }
  if (src.hasf_rest_43()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_43(i, src.getf_rest_43(i));
    }
  }
  if (src.hasf_rest_44()) {
    for (int i = 0; i < numSrcPoints; ++i) {
      dst->setf_rest_44(i, src.getf_rest_44(i));
    }
  }

  if (src.hasLaserAngles()) {
    for (int i = 0; i < numSrcPoints; ++i)
      dst->setLaserAngle(i, src.getLaserAngle(i));
  }
}

//============================================================================
// Clamp point co-ordinates in @cloud to @bbox, preserving attributes.

void
clampVolume(Box3<double> bbox, PCCPointSet3* cloud)
{
  int numSrcPoints = cloud->getPointCount();

  for (int i = 0; i < numSrcPoints; ++i) {
    auto& point = (*cloud)[i];
    for (int k = 0; k < 3; ++k)
      point[k] = PCCClip(point[k], bbox.min[k], bbox.max[k]);
  }
}

//============================================================================
// Determine colour attribute values from a reference/source point cloud.
// For each point of the target p_t:
//  - Find the N_1 (1 < N_1) nearest neighbours in source to p_t and create
//    a set of points denoted by Ψ_1.
//  - Find the set of source points that p_t belongs to their set of N_2
//    nearest neighbours. Denote this set of points by Ψ_2.
//  - Compute the distance-weighted average of points in Ψ_1 and Ψ_2 by:
//        \bar{Ψ}_k = ∑_{q∈Ψ_k} c(q)/Δ(q,p_t)
//                    ----------------------- ,
//                    ∑_{q∈Ψ_k} 1/Δ(q,p_t)
//
// where Δ(a,b) denotes the Euclidian distance between the points a and b,
// and c(q) denotes the colour of point q.  Compute the average (or the
// weighted average with the number of points of each set as the weights)
// of \bar{Ψ}̅_1 and \bar{Ψ}̅_2 and transfer it to p_t.
//
// Differences in the scale and translation of the target and source point
// clouds, is handled according to:
//    posInTgt = posInSrc * sourceToTargetScaleFactor - targetToSourceOffset

bool
recolourColour(
  const AttributeDescription& attrDesc,
  const RecolourParams& params,
  const PCCPointSet3& source,
  double sourceToTargetScaleFactor,
  point_t targetToSourceOffset,
  PCCPointSet3& target)
{
  double targetToSourceScaleFactor = 1.0 / sourceToTargetScaleFactor;

  const size_t pointCountSource = source.getPointCount();
  const size_t pointCountTarget = target.getPointCount();
  if (!pointCountSource || !pointCountTarget || !source.hasColors()) {
    return false;
  }

  KDTreeVectorOfVectorsAdaptor<PCCPointSet3, double> kdtreeTarget(
    3, target, 10);
  KDTreeVectorOfVectorsAdaptor<PCCPointSet3, double> kdtreeSource(
    3, source, 10);

  target.addColors();
  std::vector<Vec3<attr_t>> refinedColors1;
  refinedColors1.resize(pointCountTarget);

  Vec3<double> clipMax = double((1 << attrDesc.bitdepth) - 1);

  double maxGeometryDist2Fwd = params.maxGeometryDist2Fwd < 512
    ? params.maxGeometryDist2Fwd
    : std::numeric_limits<double>::max();
  double maxGeometryDist2Bwd = params.maxGeometryDist2Bwd < 512
    ? params.maxGeometryDist2Bwd
    : std::numeric_limits<double>::max();
  double maxAttributeDist2Fwd = params.maxAttributeDist2Fwd < 512
    ? params.maxAttributeDist2Fwd
    : std::numeric_limits<double>::max();
  double maxAttributeDist2Bwd = params.maxAttributeDist2Bwd < 512
    ? params.maxAttributeDist2Bwd
    : std::numeric_limits<double>::max();

  // Forward direction
  const int num_resultsFwd = params.numNeighboursFwd;
  nanoflann::KNNResultSet<double> resultSetFwd(num_resultsFwd);
  std::vector<size_t> indicesFwd(num_resultsFwd);
  std::vector<double> sqrDistFwd(num_resultsFwd);
  for (size_t index = 0; index < pointCountTarget; ++index) {
    resultSetFwd.init(&indicesFwd[0], &sqrDistFwd[0]);

    Vec3<double> posInSrc =
      (target[index] + targetToSourceOffset) * targetToSourceScaleFactor;

    kdtreeSource.index->findNeighbors(
      resultSetFwd, &posInSrc[0], nanoflann::SearchParams(10));

    while (1) {
      if (indicesFwd.size() == 1)
        break;

      if (sqrDistFwd[int(resultSetFwd.size()) - 1] <= maxGeometryDist2Fwd)
        break;

      sqrDistFwd.pop_back();
      indicesFwd.pop_back();
    }

    bool isDone = false;
    if (params.skipAvgIfIdenticalSourcePointPresentFwd) {
      if (sqrDistFwd[0] < 0.0001) {
        refinedColors1[index] = source.getColor(indicesFwd[0]);
        isDone = true;
      }
    }

    if (isDone)
      continue;

    int nNN = indicesFwd.size();
    while (nNN > 0 && !isDone) {
      if (nNN == 1) {
        refinedColors1[index] = source.getColor(indicesFwd[0]);
        isDone = true;
        break;
      }

      std::vector<Vec3<attr_t>> colors;
      colors.resize(0);
      colors.resize(nNN);
      for (int i = 0; i < nNN; ++i) {
        for (int k = 0; k < 3; ++k) {
          colors[i][k] = double(source.getColor(indicesFwd[i])[k]);
        }
      }
      double maxAttributeDist2 = std::numeric_limits<double>::min();
      for (int i = 0; i < nNN; ++i) {
        for (int j = 0; j < nNN; ++j) {
          const double dist2 = (colors[i] - colors[j]).getNorm2<double>();
          if (dist2 > maxAttributeDist2) {
            maxAttributeDist2 = dist2;
          }
        }
      }
      if (maxAttributeDist2 > maxAttributeDist2Fwd) {
        --nNN;
      } else {
        Vec3<double> refinedColor(0.0);
        if (params.useDistWeightedAvgFwd) {
          double sumWeights{0.0};
          for (int i = 0; i < nNN; ++i) {
            const double weight = 1 / (sqrDistFwd[i] + params.distOffsetFwd);
            for (int k = 0; k < 3; ++k) {
              refinedColor[k] += source.getColor(indicesFwd[i])[k] * weight;
            }
            sumWeights += weight;
          }
          refinedColor /= sumWeights;
        } else {
          for (int i = 0; i < nNN; ++i) {
            for (int k = 0; k < 3; ++k) {
              refinedColor[k] += source.getColor(indicesFwd[i])[k];
            }
          }
          refinedColor /= nNN;
        }
        for (int k = 0; k < 3; ++k) {
          refinedColors1[index][k] =
            attr_t(PCCClip(round(refinedColor[k]), 0.0, clipMax[k]));
        }
        isDone = true;
      }
    }
  }

  // Backward direction
  const size_t num_resultsBwd = params.numNeighboursBwd;
  std::vector<size_t> indicesBwd(num_resultsBwd);
  std::vector<double> sqrDistBwd(num_resultsBwd);
  nanoflann::KNNResultSet<double> resultSetBwd(num_resultsBwd);

  struct DistColor {
    double dist;
    Vec3<attr_t> color;
  };
  std::vector<std::vector<DistColor>> refinedColorsDists2;
  refinedColorsDists2.resize(pointCountTarget);

  for (size_t index = 0; index < pointCountSource; ++index) {
    const Vec3<attr_t> color = source.getColor(index);
    resultSetBwd.init(&indicesBwd[0], &sqrDistBwd[0]);

    Vec3<double> posInTgt =
      source[index] * sourceToTargetScaleFactor - targetToSourceOffset;

    kdtreeTarget.index->findNeighbors(
      resultSetBwd, &posInTgt[0], nanoflann::SearchParams(10));

    for (int i = 0; i < num_resultsBwd; ++i) {
      if (sqrDistBwd[i] <= maxGeometryDist2Bwd) {
        refinedColorsDists2[indicesBwd[i]].push_back(
          DistColor{sqrDistBwd[i], color});
      }
    }
  }

  for (size_t index = 0; index < pointCountTarget; ++index) {
    std::sort(
      refinedColorsDists2[index].begin(), refinedColorsDists2[index].end(),
      [](const DistColor& dc1, const DistColor& dc2) {
        return dc1.dist < dc2.dist;
      });
  }

  for (size_t index = 0; index < pointCountTarget; ++index) {
    const Vec3<attr_t> color1 = refinedColors1[index];
    auto& colorsDists2 = refinedColorsDists2[index];
    if (colorsDists2.empty()) {
      target.setColor(index, color1);
      continue;
    }

    bool isDone = false;
    const Vec3<double> centroid1(color1[0], color1[1], color1[2]);
    Vec3<double> centroid2(0.0);
    if (params.skipAvgIfIdenticalSourcePointPresentBwd) {
      if (colorsDists2[0].dist < 0.0001) {
        auto temp = colorsDists2[0];
        colorsDists2.clear();
        colorsDists2.push_back(temp);
        for (int k = 0; k < 3; ++k) {
          centroid2[k] = colorsDists2[0].color[k];
        }
        isDone = true;
      }
    }

    if (!isDone) {
      int nNN = colorsDists2.size();
      while (nNN > 0 && !isDone) {
        nNN = colorsDists2.size();
        if (nNN == 1) {
          auto temp = colorsDists2[0];
          colorsDists2.clear();
          colorsDists2.push_back(temp);
          for (int k = 0; k < 3; ++k) {
            centroid2[k] = colorsDists2[0].color[k];
          }
          isDone = true;
        }
        if (!isDone) {
          std::vector<Vec3<double>> colors;
          colors.resize(0);
          colors.resize(nNN);
          for (int i = 0; i < nNN; ++i) {
            for (int k = 0; k < 3; ++k) {
              colors[i][k] = double(colorsDists2[i].color[k]);
            }
          }
          double maxAttributeDist2 = std::numeric_limits<double>::min();
          for (int i = 0; i < nNN; ++i) {
            for (int j = 0; j < nNN; ++j) {
              const double dist2 = (colors[i] - colors[j]).getNorm2<double>();
              if (dist2 > maxAttributeDist2) {
                maxAttributeDist2 = dist2;
              }
            }
          }
          if (maxAttributeDist2 <= maxAttributeDist2Bwd) {
            for (size_t k = 0; k < 3; ++k) {
              centroid2[k] = 0;
            }
            if (params.useDistWeightedAvgBwd) {
              double sumWeights{0.0};
              for (int i = 0; i < colorsDists2.size(); ++i) {
                const double weight =
                  1 / (sqrt(colorsDists2[i].dist) + params.distOffsetBwd);
                for (size_t k = 0; k < 3; ++k) {
                  centroid2[k] += (colorsDists2[i].color[k] * weight);
                }
                sumWeights += weight;
              }
              centroid2 /= sumWeights;
            } else {
              for (auto& coldist : colorsDists2) {
                for (int k = 0; k < 3; ++k) {
                  centroid2[k] += coldist.color[k];
                }
              }
              centroid2 /= colorsDists2.size();
            }
            isDone = true;
          } else {
            colorsDists2.pop_back();
          }
        }
      }
    }
    double H = double(colorsDists2.size());
    double D2 = 0.0;
    for (const auto color2dist : colorsDists2) {
      auto color2 = color2dist.color;
      for (size_t k = 0; k < 3; ++k) {
        const double d2 = centroid2[k] - color2[k];
        D2 += d2 * d2;
      }
    }
    const double r = double(pointCountTarget) / double(pointCountSource);
    const double delta2 = (centroid2 - centroid1).getNorm2<double>();
    const double eps = 0.000001;

    const bool fixWeight = 1;  // m42538
    if (!(fixWeight || delta2 > eps)) {
      // centroid2 == centroid1
      target.setColor(index, color1);
    } else {
      // centroid2 != centroid1
      double w = 0.0;

      if (!fixWeight) {
        const double alpha = D2 / delta2;
        const double a = H * r - 1.0;
        const double c = alpha * r - 1.0;
        if (fabs(a) < eps) {
          w = -0.5 * c;
        } else {
          const double delta = 1.0 - a * c;
          if (delta >= 0.0) {
            w = (-1.0 + sqrt(delta)) / a;
          }
        }
      }
      const double oneMinusW = 1.0 - w;
      Vec3<double> color0;
      for (size_t k = 0; k < 3; ++k) {
        color0[k] = PCCClip(
          round(w * centroid1[k] + oneMinusW * centroid2[k]), 0.0, clipMax[k]);
      }
      const double rSource = 1.0 / double(pointCountSource);
      const double rTarget = 1.0 / double(pointCountTarget);
      double minError = std::numeric_limits<double>::max();
      Vec3<double> bestColor(color0);
      Vec3<double> color;
      for (int32_t s1 = -params.searchRange; s1 <= params.searchRange; ++s1) {
        color[0] = PCCClip(color0[0] + s1, 0.0, clipMax[0]);
        for (int32_t s2 = -params.searchRange; s2 <= params.searchRange;
             ++s2) {
          color[1] = PCCClip(color0[1] + s2, 0.0, clipMax[1]);
          for (int32_t s3 = -params.searchRange; s3 <= params.searchRange;
               ++s3) {
            color[2] = PCCClip(color0[2] + s3, 0.0, clipMax[2]);

            double e1 = 0.0;
            for (size_t k = 0; k < 3; ++k) {
              const double d = color[k] - color1[k];
              e1 += d * d;
            }
            e1 *= rTarget;

            double e2 = 0.0;
            for (const auto color2dist : colorsDists2) {
              auto color2 = color2dist.color;
              for (size_t k = 0; k < 3; ++k) {
                const double d = color[k] - color2[k];
                e2 += d * d;
              }
            }
            e2 *= rSource;

            const double error = std::max(e1, e2);
            if (error < minError) {
              minError = error;
              bestColor = color;
            }
          }
        }
      }
      target.setColor(
        index,
        Vec3<attr_t>(
          attr_t(bestColor[0]), attr_t(bestColor[1]), attr_t(bestColor[2])));
    }
  }
  return true;
}

//============================================================================
// Determine reflectance attribute values from a reference/source point cloud.
// For each point of the target p_t:
//  - Find the N_1 (1 < N_1) nearest neighbours in source to p_t and create
//    a set of points denoted by Ψ_1.
//  - Find the set of source points that p_t belongs to their set of N_2
//    nearest neighbours. Denote this set of points by Ψ_2.
//  - Compute the distance-weighted average of points in Ψ_1 and Ψ_2 by:
//        \bar{Ψ}_k = ∑_{q∈Ψ_k} c(q)/Δ(q,p_t)
//                    ----------------------- ,
//                    ∑_{q∈Ψ_k} 1/Δ(q,p_t)
//
// where Δ(a,b) denotes the Euclidian distance between the points a and b,
// and c(q) denotes the colour of point q.  Compute the average (or the
// weighted average with the number of points of each set as the weights)
// of \bar{Ψ}̅_1 and \bar{Ψ}̅_2 and transfer it to p_t.
//
// Differences in the scale and translation of the target and source point
// clouds, is handled according to:
//    posInTgt = posInSrc * sourceToTargetScaleFactor - targetToSourceOffset

bool
recolourReflectance(
  const AttributeDescription& attrDesc,
  const RecolourParams& cfg,
  const PCCPointSet3& source,
  double sourceToTargetScaleFactor,
  point_t targetToSourceOffset,
  PCCPointSet3& target)
{
  double targetToSourceScaleFactor = 1.0 / sourceToTargetScaleFactor;

  const size_t pointCountSource = source.getPointCount();
  const size_t pointCountTarget = target.getPointCount();
  if (!pointCountSource || !pointCountTarget || !source.hasReflectances()) {
    return false;
  }
  KDTreeVectorOfVectorsAdaptor<PCCPointSet3, double> kdtreeTarget(
    3, target, 10);
  KDTreeVectorOfVectorsAdaptor<PCCPointSet3, double> kdtreeSource(
    3, source, 10);
  target.addReflectances();
  std::vector<attr_t> refinedReflectances1;
  refinedReflectances1.resize(pointCountTarget);

  double clipMax = (1 << attrDesc.bitdepth) - 1;

  double maxGeometryDist2Fwd = (cfg.maxGeometryDist2Fwd < 512)
    ? cfg.maxGeometryDist2Fwd
    : std::numeric_limits<double>::max();
  double maxGeometryDist2Bwd = (cfg.maxGeometryDist2Bwd < 512)
    ? cfg.maxGeometryDist2Bwd
    : std::numeric_limits<double>::max();
  double maxAttributeDist2Fwd = (cfg.maxAttributeDist2Fwd < 512)
    ? cfg.maxAttributeDist2Fwd
    : std::numeric_limits<double>::max();
  double maxAttributeDist2Bwd = (cfg.maxAttributeDist2Bwd < 512)
    ? cfg.maxAttributeDist2Bwd
    : std::numeric_limits<double>::max();

  // Forward direction
  const int num_resultsFwd = cfg.numNeighboursFwd;
  nanoflann::KNNResultSet<double> resultSetFwd(num_resultsFwd);
  std::vector<size_t> indicesFwd(num_resultsFwd);
  std::vector<double> sqrDistFwd(num_resultsFwd);
  for (size_t index = 0; index < pointCountTarget; ++index) {
    resultSetFwd.init(&indicesFwd[0], &sqrDistFwd[0]);

    Vec3<double> posInSrc =
      (target[index] + targetToSourceOffset) * targetToSourceScaleFactor;

    kdtreeSource.index->findNeighbors(
      resultSetFwd, &posInSrc[0], nanoflann::SearchParams(10));

    while (1) {
      if (indicesFwd.size() == 1)
        break;

      if (sqrDistFwd[int(resultSetFwd.size()) - 1] <= maxGeometryDist2Fwd)
        break;

      sqrDistFwd.pop_back();
      indicesFwd.pop_back();
    }

    bool isDone = false;
    if (cfg.skipAvgIfIdenticalSourcePointPresentFwd) {
      if (sqrDistFwd[0] < 0.0001) {
        refinedReflectances1[index] = source.getReflectance(indicesFwd[0]);
        isDone = true;
      }
    }

    if (isDone)
      continue;

    int nNN = indicesFwd.size();
    while (nNN > 0 && !isDone) {
      if (nNN == 1) {
        refinedReflectances1[index] = source.getReflectance(indicesFwd[0]);
        isDone = true;
        continue;
      }

      std::vector<attr_t> reflectances;
      reflectances.resize(0);
      reflectances.resize(nNN);
      for (int i = 0; i < nNN; ++i) {
        reflectances[i] = double(source.getReflectance(indicesFwd[i]));
      }
      double maxAttributeDist2 = std::numeric_limits<double>::min();
      for (int i = 0; i < nNN; ++i) {
        for (int j = 0; j < nNN; ++j) {
          const double dist2 = pow(reflectances[i] - reflectances[j], 2);
          if (dist2 > maxAttributeDist2)
            maxAttributeDist2 = dist2;
        }
      }
      if (maxAttributeDist2 > maxAttributeDist2Fwd) {
        --nNN;
      } else {
        double refinedReflectance = 0.0;
        if (cfg.useDistWeightedAvgFwd) {
          double sumWeights{0.0};
          for (int i = 0; i < nNN; ++i) {
            const double weight = 1 / (sqrDistFwd[i] + cfg.distOffsetFwd);
            refinedReflectance +=
              source.getReflectance(indicesFwd[i]) * weight;
            sumWeights += weight;
          }
          refinedReflectance /= sumWeights;
        } else {
          for (int i = 0; i < nNN; ++i)
            refinedReflectance += source.getReflectance(indicesFwd[i]);
          refinedReflectance /= nNN;
        }
        refinedReflectances1[index] =
          attr_t(PCCClip(round(refinedReflectance), 0.0, clipMax));
        isDone = true;
      }
    }
  }

  // Backward direction
  const size_t num_resultsBwd = cfg.numNeighboursBwd;
  std::vector<size_t> indicesBwd(num_resultsBwd);
  std::vector<double> sqrDistBwd(num_resultsBwd);
  nanoflann::KNNResultSet<double> resultSetBwd(num_resultsBwd);

  struct DistReflectance {
    double dist;
    attr_t reflectance;
  };
  std::vector<std::vector<DistReflectance>> refinedReflectancesDists2;
  refinedReflectancesDists2.resize(pointCountTarget);

  for (size_t index = 0; index < pointCountSource; ++index) {
    const attr_t reflectance = source.getReflectance(index);
    resultSetBwd.init(&indicesBwd[0], &sqrDistBwd[0]);

    Vec3<double> posInTgt =
      source[index] * sourceToTargetScaleFactor - targetToSourceOffset;

    kdtreeTarget.index->findNeighbors(
      resultSetBwd, &posInTgt[0], nanoflann::SearchParams(10));

    for (int i = 0; i < num_resultsBwd; ++i) {
      if (sqrDistBwd[i] <= maxGeometryDist2Bwd) {
        refinedReflectancesDists2[indicesBwd[i]].push_back(
          DistReflectance{sqrDistBwd[i], reflectance});
      }
    }
  }

  for (size_t index = 0; index < pointCountTarget; ++index) {
    std::sort(
      refinedReflectancesDists2[index].begin(),
      refinedReflectancesDists2[index].end(),
      [](const DistReflectance& dc1, const DistReflectance& dc2) {
        return dc1.dist < dc2.dist;
      });
  }

  for (size_t index = 0; index < pointCountTarget; ++index) {
    const attr_t reflectance1 = refinedReflectances1[index];
    auto& reflectancesDists2 = refinedReflectancesDists2[index];
    if (reflectancesDists2.empty()) {
      target.setReflectance(index, reflectance1);
      continue;
    }

    bool isDone = false;
    const double centroid1 = reflectance1;
    double centroid2 = 0.0;
    if (cfg.skipAvgIfIdenticalSourcePointPresentBwd) {
      if (reflectancesDists2[0].dist < 0.0001) {
        auto temp = reflectancesDists2[0];
        reflectancesDists2.clear();
        reflectancesDists2.push_back(temp);
        centroid2 = reflectancesDists2[0].reflectance;
        isDone = true;
      }
    }
    if (!isDone) {
      int nNN = reflectancesDists2.size();
      while (nNN > 0 && !isDone) {
        nNN = reflectancesDists2.size();
        if (nNN == 1) {
          auto temp = reflectancesDists2[0];
          reflectancesDists2.clear();
          reflectancesDists2.push_back(temp);
          centroid2 = reflectancesDists2[0].reflectance;
          isDone = true;
        }
        if (!isDone) {
          std::vector<double> reflectances;
          reflectances.resize(0);
          reflectances.resize(nNN);
          for (int i = 0; i < nNN; ++i) {
            reflectances[i] = double(reflectancesDists2[i].reflectance);
          }
          double maxAttributeDist2 = std::numeric_limits<double>::min();
          for (int i = 0; i < nNN; ++i) {
            for (int j = 0; j < nNN; ++j) {
              const double dist2 = pow(reflectances[i] - reflectances[j], 2);
              if (dist2 > maxAttributeDist2) {
                maxAttributeDist2 = dist2;
              }
            }
          }
          if (maxAttributeDist2 <= maxAttributeDist2Bwd) {
            centroid2 = 0;
            if (cfg.useDistWeightedAvgBwd) {
              double sumWeights{0.0};
              for (int i = 0; i < reflectancesDists2.size(); ++i) {
                const double weight =
                  1 / (sqrt(reflectancesDists2[i].dist) + cfg.distOffsetBwd);
                centroid2 += (reflectancesDists2[i].reflectance * weight);
                sumWeights += weight;
              }
              centroid2 /= sumWeights;
            } else {
              for (auto& refdist : reflectancesDists2) {
                centroid2 += refdist.reflectance;
              }
              centroid2 /= reflectancesDists2.size();
            }
            isDone = true;
          } else {
            reflectancesDists2.pop_back();
          }
        }
      }
    }
    double H = double(reflectancesDists2.size());
    double D2 = 0.0;
    for (const auto reflectance2dist : reflectancesDists2) {
      auto reflectance2 = reflectance2dist.reflectance;
      const double d2 = centroid2 - reflectance2;
      D2 += d2 * d2;
    }
    const double r = double(pointCountTarget) / double(pointCountSource);
    const double delta2 = pow(centroid2 - centroid1, 2);
    const double eps = 0.000001;

    const bool fixWeight = 1;  // m42538
    if (!(fixWeight || delta2 > eps)) {
      // centroid2 == centroid1
      target.setReflectance(index, reflectance1);
    } else {
      // centroid2 != centroid1
      double w = 0.0;

      if (!fixWeight) {
        const double alpha = D2 / delta2;
        const double a = H * r - 1.0;
        const double c = alpha * r - 1.0;
        if (fabs(a) < eps) {
          w = -0.5 * c;
        } else {
          const double delta = 1.0 - a * c;
          if (delta >= 0.0) {
            w = (-1.0 + sqrt(delta)) / a;
          }
        }
      }
      const double oneMinusW = 1.0 - w;
      double reflectance0;
      reflectance0 =
        PCCClip(round(w * centroid1 + oneMinusW * centroid2), 0.0, clipMax);
      const double rSource = 1.0 / double(pointCountSource);
      const double rTarget = 1.0 / double(pointCountTarget);
      double minError = std::numeric_limits<double>::max();
      double bestReflectance = reflectance0;
      double reflectance;
      for (int32_t s1 = -cfg.searchRange; s1 <= cfg.searchRange; ++s1) {
        reflectance = PCCClip(reflectance0 + s1, 0.0, clipMax);
        double e1 = 0.0;
        const double d = reflectance - reflectance1;
        e1 += d * d;
        e1 *= rTarget;

        double e2 = 0.0;
        for (const auto reflectance2dist : reflectancesDists2) {
          auto reflectance2 = reflectance2dist.reflectance;
          const double d = reflectance - reflectance2;
          e2 += d * d;
        }
        e2 *= rSource;

        const double error = std::max(e1, e2);
        if (error < minError) {
          minError = error;
          bestReflectance = reflectance;
        }
      }
      target.setReflectance(index, attr_t(bestReflectance));
    }
  }
  return true;
}

//============================================================================
// Colour attributes of a target point cloud given a source.
//
// Differences in the scale and translation of the target and source point
// clouds, is handled according to:
//   posInTgt = posInSrc * sourceToTargetScaleFactor - tgtToSrcOffset

int
recolour(
  const AttributeDescription& desc,
  const RecolourParams& cfg,
  const PCCPointSet3& source,
  float sourceToTargetScaleFactor,
  point_t tgtToSrcOffset,
  PCCPointSet3* target)
{
  // todo(df): fix the incorrect assumption here that 3-component
  // attributes are colour (and that single components are reflectance)
  if (desc.attributeLabel == KnownAttributeLabel::kColour) {
    bool ok = recolourColour(
      desc, cfg, source, sourceToTargetScaleFactor, tgtToSrcOffset, *target);

    if (!ok) {
      std::cout << "Error: can't transfer colors!" << std::endl;
      return -1;
    }
  }

  if (desc.attributeLabel == KnownAttributeLabel::kReflectance) {
    bool ok = recolourReflectance(
      desc, cfg, source, sourceToTargetScaleFactor, tgtToSrcOffset, *target);

    if (!ok) {
      std::cout << "Error: can't transfer reflectance!" << std::endl;
      return -1;
    }
  }

  return 0;
}

//============================================================================

void
convertGbrToYCgCoR(int bitDepth, PCCPointSet3& cloud)
{
  for (int i = 0; i < cloud.getPointCount(); i++) {
    auto& val = cloud.getColor(i);
    val = transformGbrToYCgCoR(bitDepth, val);
  }
}

//============================================================================

void
convertYCgCoRToGbr(int bitDepth, PCCPointSet3& cloud)
{
  for (int i = 0; i < cloud.getPointCount(); i++) {
    auto& val = cloud.getColor(i);
    val = transformYCgCoRToGbr(bitDepth, val);
  }
}

//============================================================================

void
convertGbrToYCbCrBt709(PCCPointSet3& cloud)
{
  for (int i = 0; i < cloud.getPointCount(); i++) {
    auto& val = cloud.getColor(i);
    val = transformGbrToYCbCrBt709(val);
  }
}

//============================================================================

void
convertYCbCrBt709ToGbr(PCCPointSet3& cloud)
{
  for (int i = 0; i < cloud.getPointCount(); i++) {
    auto& val = cloud.getColor(i);
    val = transformYCbCrBt709ToGbr(val);
  }
}

//============================================================================
double
roundAtDigit(double x, double digit)
{
  return std::round(x * digit) / digit;
}

//============================================================================

std::vector<int>
orderByAzimuth(
  PCCPointSet3& cloud,
  int start,
  int end,
  double recipBinWidth,
  Vec3<int32_t> origin)
{
  // build a list of inxdexes to sort
  auto pointCount = end - start;
  std::vector<int> order(pointCount);
  for (int i = 0; i < pointCount; i++)
    order[i] = start + i;

  std::sort(order.begin(), order.end(), [&](int aIdx, int bIdx) {
    auto a = cloud[aIdx] - origin;
    auto b = cloud[bIdx] - origin;

    double rA = hypot(a[0], a[1]);
    double phiA = atan2(a[1], a[0]);
    double tanThetaA = a[2] / rA;

    double rB = hypot(b[0], b[1]);
    double phiB = atan2(b[1], b[0]);
    double tanThetaB = b[2] / rB;

    // quantise azimith to specified precision
    if (recipBinWidth != 0.) {
      phiA = std::round(phiA * recipBinWidth);
      phiB = std::round(phiB * recipBinWidth);
    }

    // NB: the a < b comparison adds some stability to the sort.  It is not
    // required in an actual implementation.  Either slightly more performance
    // can be achieved by sorting by a second data dependent dimension, or
    // efficiency can be improved by removing the stability (at a cost of
    // being able to reproduce the exact same bitstream).

    return phiB != phiA ? phiA < phiB
      : rA != rB        ? rA < rB
                        : tanThetaA < tanThetaB;
  });

  return order;
}

//============================================================================

std::vector<int>
orderByAzimuth(
  PCCPointSet3& cloud,
  int start,
  int end,
  double recipBinWidth,
  Vec3<int32_t> origin,
  const int32_t positionAzimuthScaleLog2,
  const int32_t azimuthSpeed,
  const std::vector<int32_t>& angularTheta,
  const std::vector<int32_t>& angularZ)
{
  if (recipBinWidth != 0.) {
    recipBinWidth *= azimuthSpeed;
  }
  // build a list of inxdexes to sort
  auto pointCount = end - start;
  std::vector<int> order(pointCount);
  for (int i = 0; i < pointCount; i++)
    order[i] = i;

  int numLasers = angularZ.size();

  const int kpi = 1 << positionAzimuthScaleLog2 - 1;
  std::vector<point_t> lidarCoord(pointCount);
  for (int i = 0; i < pointCount; i++) {
    auto a = cloud[start + i] - origin;

    int32_t rA = int32_t(hypot(a[0], a[1]) * (1 << 8) + 0.5);
    double dphiA = (atan2(double(a[1]), double(a[0])) + M_PI);
    if (recipBinWidth != 0.) {
      dphiA = dphiA * recipBinWidth;
    } else {
      dphiA = dphiA * kpi / M_PI / azimuthSpeed;
    }
    int32_t phiIndexA = dphiA + 0.5;
    int32_t laserIndexA =
      findLaserPrecise(a, angularTheta.data(), angularZ.data(), numLasers);
    lidarCoord[i] = {rA, phiIndexA, laserIndexA};
  }

  std::sort(order.begin(), order.end(), [&](int aIdx, int bIdx) {
    auto a = lidarCoord[aIdx];
    auto b = lidarCoord[bIdx];

    return a[1] != b[1] ? a[1] < b[1]
      : a[2] != b[2]    ? a[2] < b[2]
                        : a[0] < b[0];
  });

  // now sort to minimize r-jump
  std::vector<int32_t> lastR(numLasers);
  for (int l = 0; l < numLasers; l++)
    lastR[l] = 0;

  int startRange = 0;
  int32_t startPhiIndex = lidarCoord[order[0]][1];
  int32_t startLaserIndex = lidarCoord[order[0]][2];
  for (int i = 0; i < pointCount; i++) {
    int32_t currentPhiIndex = lidarCoord[order[i]][1];
    int32_t currentLaserIndex = lidarCoord[order[i]][2];
    if (
      currentPhiIndex != startPhiIndex || currentLaserIndex != startLaserIndex
      || i == pointCount - 1) {
      // range completed
      int32_t minR = lidarCoord[order[startRange]][0];
      int32_t maxR = lidarCoord[order[i - 1]][0];

      // minimize r-jump
      if (
        std::abs(minR - lastR[startLaserIndex])
        > std::abs(maxR - lastR[startLaserIndex]))
        std::reverse(&order[startRange], &order[i]);

      // update for next range
      lastR[startLaserIndex] = lidarCoord[order[i - 1]][0];
      startPhiIndex = currentPhiIndex;
      startLaserIndex = currentLaserIndex;
      startRange = i;
    }  // end if range completed
  }

  return order;
}

//============================================================================
// Sorts according to azimuth.
// \param recipBinWidth is the reciprocal bin width used in sorting.
//        recipBinWidth = 0 disables binning.

void
sortByAzimuth(
  PCCPointSet3& cloud,
  int start,
  int end,
  double recipBinWidth,
  Vec3<int32_t> origin)
{
  auto pointCount = end - start;
  auto order = orderByAzimuth(cloud, start, end, recipBinWidth, origin);

  // inefficiently reorder the point cloud
  for (int i = 0; i < pointCount; i++) {
    while (order[i] - start != i) {
      cloud.swapPoints(order[i], order[order[i] - start]);
      std::swap(order[i], order[order[i] - start]);
    }
  }
}

//============================================================================
// Sort for LiDAR

void
sortByAzimuth(
  PCCPointSet3& cloud,
  int start,
  int end,
  double recipBinWidth,
  Vec3<int32_t> origin,
  const int32_t positionAzimuthScaleLog2,
  const int32_t azimuthSpeed,
  const std::vector<int32_t>& angularTheta,
  const std::vector<int32_t>& angularZ)
{
  auto pointCount = end - start;
  auto order = orderByAzimuth(
    cloud, start, end, recipBinWidth, origin, positionAzimuthScaleLog2,
    azimuthSpeed, angularTheta, angularZ);

  // inefficiently reorder the point cloud
  for (int i = 0; i < pointCount; i++) {
    while (order[i] != i) {
      cloud.swapPoints(start + order[i], start + order[order[i]]);
      std::swap(order[i], order[order[i]]);
    }
  }
}

//============================================================================

std::vector<int>
orderByRadius(PCCPointSet3& cloud, int start, int end, Vec3<int32_t> origin)
{
  // build a list of inxdexes to sort
  auto pointCount = end - start;
  std::vector<int> order(pointCount);
  for (int i = 0; i < pointCount; i++)
    order[i] = start + i;

  std::sort(order.begin(), order.end(), [&](int a, int b) {
    auto aPos = cloud[a] - origin;
    auto bPos = cloud[b] - origin;
    auto aT = aPos[0] * aPos[0] + aPos[1] * aPos[1];
    auto bT = bPos[0] * bPos[0] + bPos[1] * bPos[1];
    // NB: the a < b comparison adds some stability to the sort.  It is not
    // required in an actual implementation.  Either slightly more performance
    // can be achieved by sorting by a second data dependent dimension, or
    // efficiency can be improved by removing the stability (at a cost of
    // being able to reproduce the exact same bitstream).
    return aT != bT ? aT < bT : a < b;
  });

  return order;
}

//============================================================================

void
sortByRadius(PCCPointSet3& cloud, int start, int end, Vec3<int32_t> origin)
{
  auto pointCount = end - start;
  auto order = orderByRadius(cloud, start, end, origin);

  // inefficiently reorder the point cloud
  for (int i = 0; i < pointCount; i++) {
    while (order[i] - start != i) {
      cloud.swapPoints(order[i], order[order[i] - start]);
      std::swap(order[i], order[order[i] - start]);
    }
  }
}

//============================================================================

std::vector<int>
orderByLaserAngle(
  PCCPointSet3& cloud,
  int start,
  int end,
  double recipBinWidth,
  Vec3<int32_t> origin)
{
  // build a list of inxdexes to sort
  auto pointCount = end - start;
  std::vector<int> order(pointCount);
  for (int i = 0; i < pointCount; i++)
    order[i] = start + i;

  std::sort(order.begin(), order.end(), [&](int aIdx, int bIdx) {
    auto a = cloud[aIdx] - origin;
    auto b = cloud[bIdx] - origin;

    float rA = hypot(a[0], a[1]);
    float phiA = cloud.getLaserAngle(aIdx);
    float tanThetaA = a[2] / rA;

    float rB = hypot(b[0], b[1]);
    float phiB = cloud.getLaserAngle(bIdx);
    float tanThetaB = b[2] / rB;

    // quantise azimith to specified precision
    if (recipBinWidth != 0.) {
      phiA = std::round(phiA * recipBinWidth);
      phiB = std::round(phiB * recipBinWidth);
    }

    // NB: the aIdx < bIdx comparison adds some stability to the sort.  It is not
    // required in an actual implementation.  Either slightly more performance
    // can be achieved by sorting by a second data dependent dimension, or
    // efficiency can be improved by removing the stability (at a cost of
    // being able to reproduce the exact same bitstream).

    return phiB != phiA        ? phiA < phiB
      : rA != rB               ? rA < rB
      : tanThetaA != tanThetaB ? tanThetaA < tanThetaB
                               : aIdx < bIdx;
  });

  return order;
}

//============================================================================
// Sorts according to azimuth.
// \param recipBinWidth is the reciprocal bin width used in sorting.
//        recipBinWidth = 0 disables binning.

void
sortByLaserAngle(
  PCCPointSet3& cloud,
  int start,
  int end,
  double recipBinWidth,
  Vec3<int32_t> origin)
{
  auto pointCount = end - start;
  std::vector<int> order;
  if (cloud.hasLaserAngles())
    order = orderByLaserAngle(cloud, start, end, recipBinWidth, origin);
  else
    order = orderByAzimuth(cloud, start, end, recipBinWidth, origin);

  // inefficiently reorder the point cloud
  for (int i = 0; i < pointCount; i++) {
    while (order[i] - start != i) {
      cloud.swapPoints(order[i], order[order[i] - start]);
      std::swap(order[i], order[order[i] - start]);
    }
  }
}

}  // namespace pcc
