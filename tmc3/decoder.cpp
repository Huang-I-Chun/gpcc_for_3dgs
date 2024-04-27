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

#include "PCCTMC3Decoder.h"

#include <algorithm>
#include <cassert>
#include <string>

#include "AttributeCommon.h"
#include "PayloadBuffer.h"
#include "PCCPointSet.h"
#include "coordinate_conversion.h"
#include "geometry.h"
#include "geometry_octree.h"
#include "geometry_predictive.h"
#include "hls.h"
#include "io_hls.h"
#include "io_tlv.h"
#include "pcc_chrono.h"
#include "osspecific.h"

namespace pcc {

//============================================================================

PCCTMC3Decoder3::PCCTMC3Decoder3(const DecoderParams& params) : _params(params)
{
  init();
}

//----------------------------------------------------------------------------

void
PCCTMC3Decoder3::init()
{
  _firstSliceInFrame = true;
  _outputInitialized = false;
  _suppressOutput = 1;
  _sps = nullptr;
  _gps = nullptr;
  _spss.clear();
  _gpss.clear();
  _apss.clear();
  _refFrameSeq.clear();
  _refFrameAltSeq.clear();
  biPredDecodeParams.init();

  _ctxtMemOctreeGeom.reset(new GeometryOctreeContexts);
  _ctxtMemPredGeom.reset(new PredGeomContexts);
}

//----------------------------------------------------------------------------

PCCTMC3Decoder3::~PCCTMC3Decoder3() = default;

//============================================================================

static bool
payloadStartsNewSlice(PayloadType type)
{
  return type == PayloadType::kGeometryBrick
    || type == PayloadType::kFrameBoundaryMarker;
}

//============================================================================

bool
PCCTMC3Decoder3::dectectFrameBoundary(const PayloadBuffer* buf)
{
  // This may be from either geometry brick or attr param inventory
  int frameCtrLsb;

  switch (buf->type) {
  case PayloadType::kFrameBoundaryMarker: {
    // the frame boundary data marker explcitly indicates a boundary
    // However, this implementation doesn't flush the output, rather
    // this happens naturally when the frame boundary is detected by
    // a change in frameCtr.
    auto fbm = parseFrameBoundaryMarker(*buf);
    frameCtrLsb = fbm.fbdu_frame_ctr_lsb;
    break;
  }

  case PayloadType::kGeometryBrick: {
    activateParameterSets(parseGbhIds(*buf));
    auto gbh = parseGbh(*_sps, *_gps, *buf, nullptr, nullptr);
    frameCtrLsb = gbh.frame_ctr_lsb;
    break;
  }

  case PayloadType::kGeneralizedAttrParamInventory: {
    auto apih = parseAttrParamInventoryHdr(*buf);
    activateParameterSets(apih);
    // todo(conf): check lsb_bits is same as sps
    frameCtrLsb = apih.attr_param_frame_ctr_lsb;
    break;
  }

  // other data units don't indicate a boundary
  default: return false;
  }

  auto bdry = _frameCtr.isDifferentFrame(frameCtrLsb, _sps->frame_ctr_bits);
  _frameCtr.update(frameCtrLsb, _sps->frame_ctr_bits);

  return bdry;
}

//============================================================================

void
PCCTMC3Decoder3::outputCurrentCloud(PCCTMC3Decoder3::Callbacks* callback)
{
  if (_suppressOutput)
    return;

  std::swap(_outCloud.cloud, _accumCloud);

  // Apply global scaling to output for integer conformance
  // todo: add other output scaling modes
  // NB: if accumCloud is reused for future inter-prediction, global scaling
  //     must be applied to a copy.
  scaleGeometry(_outCloud.cloud, _sps->globalScale, _outCloud.outputFpBits);

  callback->onOutputCloud(_outCloud);

  std::swap(_outCloud.cloud, _accumCloud);
  _accumCloud.clear();
}

void
PCCTMC3Decoder3::storeCurrentCloudAsRef()
{
  if (_sps->inter_frame_prediction_enabled_flag && !_suppressOutput) {
    _refFrameSeq[_sps->sps_seq_parameter_set_id].cloud = _accumCloud;
    auto& altCloud = _refFrameAltSeq[_sps->sps_seq_parameter_set_id].cloud;
    altCloud.clear();
    altCloud.swap(_accumCloudAltPositions);
  }
}

void
PCCTMC3Decoder3::storeCurrentCloudAsBRef()
{
  if (_suppressOutput)
    return;

  if (_frameCtr == 0)
    _gbh.biPredictionEnabledFlag = false;

  if (!_gbh.biPredictionEnabledFlag) {
    biPredDecodeParams.refPointCloud2 = _accumCloud;
  } else {
    _refFrameSeq[_sps->sps_seq_parameter_set_id].cloud = _accumCloud;
  }

  if (_gps->biPredictionEnabledFlag == 2 && _gbh.biPredictionEnabledFlag)
    processHierarchicalGOFPost();
}

//============================================================================

void
PCCTMC3Decoder3::outputGOFCurrentCloud(PCCTMC3Decoder3::Callbacks* callback)
{
  if (!_gbh.biPredictionEnabledFlag) {
    if (
      (_outCloud.frameNum == 0)
      || ((_outCloud.frameNum - biPredDecodeParams.preOutFrameNum) == 1)) {
      compensateZ();
      outputCurrentCloud(callback);
      biPredDecodeParams.preOutFrameNum = _outCloud.frameNum;
    } else {
      _accumCloud.clear();
    }
    return;
  }

  if (_gps->biPredictionEnabledFlag == 1) {
    auto outputOrigin = _outCloud.outputOrigin;
    auto outputUnitLength = _outCloud.outputUnitLength;
    compensateZ();
    outputCurrentCloud(callback);
    biPredDecodeParams.preOutFrameNum = _outCloud.frameNum;
    if (_outCloud.frameNum + 1 == biPredDecodeParams.preIPFrame) {
      _accumCloud = biPredDecodeParams.refPointCloud2;
      _outCloud.frameNum = biPredDecodeParams.preIPFrame;
      _outCloud.outputOrigin = outputOrigin;
      _outCloud.outputUnitLength = outputUnitLength;
      compensateZ();
      outputCurrentCloud(callback);
      biPredDecodeParams.preOutFrameNum = _outCloud.frameNum;
    }
  } else {
    int outputFrameNum = biPredDecodeParams.preOutFrameNum + 1;
    int outputFrameIndexInGOF =
      outputFrameNum - biPredDecodeParams.prePreIPFrame;
    int currFrameNum = _outCloud.frameNum;

    const PCCPointSet3 _accumCloudTmp = _accumCloud;
    auto outputOrigin = _outCloud.outputOrigin;
    auto outputUnitLength = _outCloud.outputUnitLength;
    while (biPredDecodeParams.frameReadyForOutput(outputFrameIndexInGOF)) {
      _accumCloud =
        (outputFrameNum != currFrameNum
           ? hGOFDecodeParams.gof[outputFrameIndexInGOF]
           : _accumCloudTmp);

      if (outputFrameIndexInGOF != biPredDecodeParams.refTimesList.size() - 1)
        hGOFDecodeParams.clearFrame(outputFrameIndexInGOF);

      _outCloud.frameNum = outputFrameNum;
      _outCloud.outputOrigin = outputOrigin;
      _outCloud.outputUnitLength = outputUnitLength;
      compensateZ();
      outputCurrentCloud(callback);
      biPredDecodeParams.preOutFrameNum = _outCloud.frameNum;

      outputFrameNum++;
      outputFrameIndexInGOF++;
    }
  }
  _accumCloud.clear();
  return;
}

//============================================================================

void
PCCTMC3Decoder3::compensateZ()
{
  if (!_gps->geom_z_compensation_enabled_flag)
    return;

  auto plyScale = reciprocal(_sps->seqGeomScale);
  plyScale.numerator *= 1000;
  auto laserOrigin = _gps->gpsAngularOrigin;
  compensateZCoordinate(
    _accumCloud, _gps, plyScale, laserOrigin, _outCloud.outputUnitLength,
    _outCloud.outputOrigin);
}
//============================================================================

void
PCCTMC3Decoder3::startFrame()
{
  _outputInitialized = true;
  _firstSliceInFrame = true;
  _outCloud.frameNum = _frameCtr;

  // the following could be set once when the SPS is discovered
  _outCloud.setParametersFrom(*_sps, _params.outputFpBits);
  // create a new reference frame for the sps, if needed
  emplaceRefFrame(*_sps);
}

//============================================================================

void
PCCTMC3Decoder3::emplaceRefFrame(const SequenceParameterSet& sps)
{
  if (sps.inter_frame_prediction_enabled_flag) {
    _refFrameSeq.emplace(
      std::make_pair(sps.sps_seq_parameter_set_id, CloudFrame(_outCloud)));
    _refFrameAltSeq.emplace(
      std::make_pair(sps.sps_seq_parameter_set_id, CloudFrame(_outCloud)));
  }
}

//============================================================================

int
PCCTMC3Decoder3::decompress(
  const PayloadBuffer* buf, PCCTMC3Decoder3::Callbacks* callback)
{
  // Starting a new geometry brick/slice/tile, transfer any
  // finished points to the output accumulator
  if (!buf || payloadStartsNewSlice(buf->type)) {
    if (size_t numPoints = _currentPointCloud.getPointCount()) {
      for (size_t i = 0; i < numPoints; i++)
        for (int k = 0; k < 3; k++)
          _currentPointCloud[i][k] += _sliceOrigin[k];
      _accumCloud.append(_currentPointCloud);
    }
  }

  if (!buf) {
    // flush decoder, output pending cloud if any
    if (_gps->biPredictionEnabledFlag)
      outputGOFCurrentCloud(callback);
    else {
      std::cout << "Now output pointcloud" << std::endl;
      compensateZ();
      outputCurrentCloud(callback);
    }
    return 0;
  }

  // process a frame boundary
  //  - this may update FrameCtr
  //  - this will activate the sps for GeometryBrick and AttrParamInventory
  //  - after outputing the current frame, the output must be reinitialized
  if (dectectFrameBoundary(buf)) {
    if (_gps->biPredictionEnabledFlag) {
      storeCurrentCloudAsBRef();
      outputGOFCurrentCloud(callback);
    } else {
      storeCurrentCloudAsRef();
      compensateZ();
      outputCurrentCloud(callback);
    }
    _outputInitialized = false;
  }

  // process the buffer
  switch (buf->type) {
  case PayloadType::kSequenceParameterSet: {
    auto sps = parseSps(*buf);
    convertXyzToStv(&sps);
    storeSps(std::move(sps));
    return 0;
  }

  case PayloadType::kGeometryParameterSet: {
    auto gps = parseGps(*buf);
    // HACK: assume that an SPS has been received prior to the GPS.
    // This is not required, and parsing of the GPS is independent of the SPS.
    // todo(df): move GPS fixup to activation process
    _sps = &_spss.cbegin()->second;
    convertXyzToStv(*_sps, &gps);
    storeGps(std::move(gps));
    return 0;
  }

  case PayloadType::kAttributeParameterSet: {
    auto aps = parseAps(*buf);
    // HACK: assume that an SPS has been received prior to the APS.
    // This is not required, and parsing of the APS is independent of the SPS.
    // todo(df): move APS fixup to activation process
    _sps = &_spss.cbegin()->second;
    convertXyzToStv(*_sps, &aps);
    storeAps(std::move(aps));
    return 0;
  }

  case PayloadType::kFrameBoundaryMarker:
    if (!_outputInitialized)
      startFrame();
    return 0;

  case PayloadType::kGeometryBrick:
    if (!_outputInitialized)
      startFrame();

    // avoid accidents with stale attribute decoder on next slice
    _attrDecoder.reset();
    // Avoid dropping an actual frame
    _suppressOutput = false;

    return decodeGeometryBrick(*buf);

  case PayloadType::kAttributeBrick: decodeAttributeBrick(*buf); return 0;

  case PayloadType::kConstantAttribute:
    decodeConstantAttribute(*buf);
    return 0;

  case PayloadType::kTileInventory:
    // NB: the tile inventory is decoded in xyz order.  It may need
    //     conversion if it is used (it currently isn't).
    storeTileInventory(parseTileInventory(*buf));
    return 0;

  case PayloadType::kGeneralizedAttrParamInventory: {
    if (!_outputInitialized)
      startFrame();

    auto hdr = parseAttrParamInventoryHdr(*buf);
    assert(hdr.attr_param_sps_attr_idx < int(_sps->attributeSets.size()));
    auto& attrDesc = _outCloud.attrDesc[hdr.attr_param_sps_attr_idx];
    parseAttrParamInventory(attrDesc, *buf, attrDesc.params);
    return 0;
  }

  case PayloadType::kUserData: parseUserData(*buf); return 0;
  }

  // todo(df): error, unhandled payload type
  return 1;
}

//--------------------------------------------------------------------------

void
PCCTMC3Decoder3::storeSps(SequenceParameterSet&& sps)
{
  // todo(df): handle replacement semantics
  _spss.emplace(std::make_pair(sps.sps_seq_parameter_set_id, sps));
}

//--------------------------------------------------------------------------

void
PCCTMC3Decoder3::storeGps(GeometryParameterSet&& gps)
{
  // todo(df): handle replacement semantics
  _gpss.emplace(std::make_pair(gps.gps_geom_parameter_set_id, gps));
}

//--------------------------------------------------------------------------

void
PCCTMC3Decoder3::storeAps(AttributeParameterSet&& aps)
{
  // todo(df): handle replacement semantics
  _apss.emplace(std::make_pair(aps.aps_attr_parameter_set_id, aps));
}

//--------------------------------------------------------------------------

void
PCCTMC3Decoder3::storeTileInventory(TileInventory&& inventory)
{
  // todo(df): handle replacement semantics
  _tileInventory = inventory;
}

//==========================================================================

void
PCCTMC3Decoder3::activateParameterSets(const GeometryBrickHeader& gbh)
{
  // HACK: assume activation of the first SPS and GPS
  // todo(df): parse brick header here for propper sps & gps activation
  //  -- this is currently inconsistent between trisoup and octree
  assert(!_spss.empty());
  assert(!_gpss.empty());
  _sps = &_spss.cbegin()->second;
  _gps = &_gpss.cbegin()->second;

  _refFrame = _sps->inter_frame_prediction_enabled_flag
    ? &_refFrameSeq[_sps->sps_seq_parameter_set_id]
    : nullptr;
  _refFrameAlt = _sps->inter_frame_prediction_enabled_flag
    ? &_refFrameAltSeq[_sps->sps_seq_parameter_set_id]
    : nullptr;
}

//--------------------------------------------------------------------------

void
PCCTMC3Decoder3::activateParameterSets(const AttributeParamInventoryHdr& hdr)
{
  // HACK: assume activation of the first SPS and GPS
  // todo(df): parse brick header here for propper sps & gps activation
  //  -- this is currently inconsistent between trisoup and octree
  assert(!_spss.empty());
  assert(!_gpss.empty());
  _sps = &_spss.cbegin()->second;
  _gps = &_gpss.cbegin()->second;

  _refFrame = _sps->inter_frame_prediction_enabled_flag
    ? &_refFrameSeq[_sps->sps_seq_parameter_set_id]
    : nullptr;
  _refFrameAlt = _sps->inter_frame_prediction_enabled_flag
    ? &_refFrameAltSeq[_sps->sps_seq_parameter_set_id]
    : nullptr;
}
//--------------------------------------------------------------------------

void
PCCTMC3Decoder3::processHierarchicalGOF()
{
  auto& refCloud1 = attrInterPredParams.referencePointCloud;
  auto& refCloud2 =
    biPredDecodeParams.attrInterPredParams2.referencePointCloud;

  std::vector<point_t>& refPosSph1 = _refPosSph;
  std::vector<point_t>& refPosSph2 = biPredDecodeParams._refPosSph2;

  PredGeomPredictor& refFrameSph1 = _refFrameSph;
  PredGeomPredictor& refFrameSph2 = biPredDecodeParams._refFrameSph2;

  if (_gbh.biPredictionEnabledFlag) {
    const auto deltaIPFrame =
      biPredDecodeParams.preIPFrame - biPredDecodeParams.prePreIPFrame;
    if ((deltaIPFrame - 1) != hGOFDecodeParams.codeOrderList.size()) {
      hGOFDecodeParams.reInitializeLists(deltaIPFrame);
    }
    if (!biPredDecodeParams.preFrameAsBframe) {
      biPredDecodeParams.refTimesList = hGOFDecodeParams.refTimesList;
    }
  }

  if (!_gbh.biPredictionEnabledFlag) {
    if (hGOFDecodeParams.gof.size()) {
      biPredDecodeParams.refPointCloud2 = hGOFDecodeParams.gof.back();
      refCloud2 = hGOFDecodeParams.gof_spherical.back();
      refPosSph2 = hGOFDecodeParams.gof_posSph.back();
      hGOFDecodeParams.clearGofs();
    }
  } else {
    if (!hGOFDecodeParams.gof.size()) {
      hGOFDecodeParams.initializeGof(
        biPredDecodeParams.refTimesList.size(),
        _refFrameSeq[_sps->sps_seq_parameter_set_id].cloud,
        biPredDecodeParams.refPointCloud2, refCloud1, refCloud2, refPosSph1,
        refPosSph2);

      biPredDecodeParams.currFrameInGOF = 0;
    }

    biPredDecodeParams.currFrameIndexInGOF =
      hGOFDecodeParams.codeOrderList[biPredDecodeParams.currFrameInGOF];
    int _preRefFrame =
      hGOFDecodeParams.refFrameList[biPredDecodeParams.currFrameInGOF * 2];
    int _backRefFrame =
      hGOFDecodeParams.refFrameList[biPredDecodeParams.currFrameInGOF * 2 + 1];

    hGOFDecodeParams.updateReferenceFrames(
      _refFrameSeq[_sps->sps_seq_parameter_set_id].cloud,
      biPredDecodeParams.refPointCloud2, refCloud1, refCloud2, refPosSph1,
      refPosSph2, refFrameSph1, refFrameSph2, _preRefFrame, _backRefFrame);

    biPredDecodeParams.decrementRefTimesList(
      _preRefFrame, _backRefFrame, biPredDecodeParams.currFrameIndexInGOF);

    biPredDecodeParams.currFrameInGOF++;
  }
}

//--------------------------------------------------------------------------

void
PCCTMC3Decoder3::processHierarchicalGOFPost()
{
  hGOFDecodeParams.storeReferenceFrame(
    biPredDecodeParams.currFrameIndexInGOF,
    _refFrameSeq[_sps->sps_seq_parameter_set_id].cloud,
    attrInterPredParams.referencePointCloud, _refPosSph);
}
//==========================================================================
// Initialise the point cloud storage and decode a single geometry slice.

int
PCCTMC3Decoder3::decodeGeometryBrick(const PayloadBuffer& buf)
{
  std::cout << "PCCTMC3Decoder3::decodeGeometryBrick start" << std::endl;
  assert(buf.type == PayloadType::kGeometryBrick);
  std::cout << "positions bitstream size " << buf.size() << " B\n";

  // todo(df): replace with attribute mapping
  bool hasColour = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kColour;
    });

  bool hasReflectance = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kReflectance;
    });

  bool hasOpacity = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kOpacity;
    });

  bool hasf_dc_0 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_dc_0;
    });
  bool hasf_dc_1 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_dc_1;
    });
  bool hasf_dc_2 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_dc_2;
    });

  bool hasscale_0 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kscale_0;
    });
  bool hasscale_1 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kscale_1;
    });
  bool hasscale_2 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kscale_2;
    });

  bool hasrot_0 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::krot_0;
    });
  bool hasrot_1 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::krot_1;
    });
  bool hasrot_2 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::krot_2;
    });
  bool hasrot_3 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::krot_3;
    });
  bool hasf_rest_0 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_0;
    });
  bool hasf_rest_1 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_1;
    });
  bool hasf_rest_2 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_2;
    });
  bool hasf_rest_3 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_3;
    });
  bool hasf_rest_4 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_4;
    });
  bool hasf_rest_5 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_5;
    });
  bool hasf_rest_6 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_6;
    });
  bool hasf_rest_7 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_7;
    });
  bool hasf_rest_8 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_8;
    });
  bool hasf_rest_9 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_9;
    });
  bool hasf_rest_10 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_10;
    });
  bool hasf_rest_11 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_11;
    });
  bool hasf_rest_12 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_12;
    });
  bool hasf_rest_13 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_13;
    });
  bool hasf_rest_14 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_14;
    });
  bool hasf_rest_15 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_15;
    });
  bool hasf_rest_16 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_16;
    });
  bool hasf_rest_17 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_17;
    });
  bool hasf_rest_18 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_18;
    });
  bool hasf_rest_19 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_19;
    });
  bool hasf_rest_20 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_20;
    });
  bool hasf_rest_21 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_21;
    });
  bool hasf_rest_22 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_22;
    });
  bool hasf_rest_23 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_23;
    });
  bool hasf_rest_24 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_24;
    });
  bool hasf_rest_25 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_25;
    });
  bool hasf_rest_26 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_26;
    });
  bool hasf_rest_27 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_27;
    });
  bool hasf_rest_28 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_28;
    });
  bool hasf_rest_29 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_29;
    });
  bool hasf_rest_30 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_30;
    });
  bool hasf_rest_31 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_31;
    });
  bool hasf_rest_32 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_32;
    });
  bool hasf_rest_33 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_33;
    });
  bool hasf_rest_34 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_34;
    });
  bool hasf_rest_35 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_35;
    });
  bool hasf_rest_36 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_36;
    });
  bool hasf_rest_37 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_37;
    });
  bool hasf_rest_38 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_38;
    });
  bool hasf_rest_39 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_39;
    });
  bool hasf_rest_40 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_40;
    });
  bool hasf_rest_41 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_41;
    });
  bool hasf_rest_42 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_42;
    });
  bool hasf_rest_43 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_43;
    });
  bool hasf_rest_44 = std::any_of(
    _sps->attributeSets.begin(), _sps->attributeSets.end(),
    [](const AttributeDescription& desc) {
      return desc.attributeLabel == KnownAttributeLabel::kf_rest_44;
    });

  _currentPointCloud.clear();
  _currentPointCloud.addRemoveAttributes(hasColour, hasReflectance);
  _currentPointCloud.addRemoveOpacities(hasOpacity);
  _currentPointCloud.addRemovef_dc_0(hasf_dc_0);
  _currentPointCloud.addRemovef_dc_1(hasf_dc_1);
  _currentPointCloud.addRemovef_dc_2(hasf_dc_2);
  _currentPointCloud.addRemovescale_0(hasscale_0);
  _currentPointCloud.addRemovescale_1(hasscale_1);
  _currentPointCloud.addRemovescale_2(hasscale_2);
  _currentPointCloud.addRemoverot_0(hasrot_0);
  _currentPointCloud.addRemoverot_1(hasrot_1);
  _currentPointCloud.addRemoverot_2(hasrot_2);
  _currentPointCloud.addRemoverot_3(hasrot_3);
  _currentPointCloud.addRemovef_rest_0(hasf_rest_0);
  _currentPointCloud.addRemovef_rest_1(hasf_rest_1);
  _currentPointCloud.addRemovef_rest_2(hasf_rest_2);
  _currentPointCloud.addRemovef_rest_3(hasf_rest_3);
  _currentPointCloud.addRemovef_rest_4(hasf_rest_4);
  _currentPointCloud.addRemovef_rest_5(hasf_rest_5);
  _currentPointCloud.addRemovef_rest_6(hasf_rest_6);
  _currentPointCloud.addRemovef_rest_7(hasf_rest_7);
  _currentPointCloud.addRemovef_rest_8(hasf_rest_8);
  _currentPointCloud.addRemovef_rest_9(hasf_rest_9);
  _currentPointCloud.addRemovef_rest_10(hasf_rest_10);
  _currentPointCloud.addRemovef_rest_11(hasf_rest_11);
  _currentPointCloud.addRemovef_rest_12(hasf_rest_12);
  _currentPointCloud.addRemovef_rest_13(hasf_rest_13);
  _currentPointCloud.addRemovef_rest_14(hasf_rest_14);
  _currentPointCloud.addRemovef_rest_15(hasf_rest_15);
  _currentPointCloud.addRemovef_rest_16(hasf_rest_16);
  _currentPointCloud.addRemovef_rest_17(hasf_rest_17);
  _currentPointCloud.addRemovef_rest_18(hasf_rest_18);
  _currentPointCloud.addRemovef_rest_19(hasf_rest_19);
  _currentPointCloud.addRemovef_rest_20(hasf_rest_20);
  _currentPointCloud.addRemovef_rest_21(hasf_rest_21);
  _currentPointCloud.addRemovef_rest_22(hasf_rest_22);
  _currentPointCloud.addRemovef_rest_23(hasf_rest_23);
  _currentPointCloud.addRemovef_rest_24(hasf_rest_24);
  _currentPointCloud.addRemovef_rest_25(hasf_rest_25);
  _currentPointCloud.addRemovef_rest_26(hasf_rest_26);
  _currentPointCloud.addRemovef_rest_27(hasf_rest_27);
  _currentPointCloud.addRemovef_rest_28(hasf_rest_28);
  _currentPointCloud.addRemovef_rest_29(hasf_rest_29);
  _currentPointCloud.addRemovef_rest_30(hasf_rest_30);
  _currentPointCloud.addRemovef_rest_31(hasf_rest_31);
  _currentPointCloud.addRemovef_rest_32(hasf_rest_32);
  _currentPointCloud.addRemovef_rest_33(hasf_rest_33);
  _currentPointCloud.addRemovef_rest_34(hasf_rest_34);
  _currentPointCloud.addRemovef_rest_35(hasf_rest_35);
  _currentPointCloud.addRemovef_rest_36(hasf_rest_36);
  _currentPointCloud.addRemovef_rest_37(hasf_rest_37);
  _currentPointCloud.addRemovef_rest_38(hasf_rest_38);
  _currentPointCloud.addRemovef_rest_39(hasf_rest_39);
  _currentPointCloud.addRemovef_rest_40(hasf_rest_40);
  _currentPointCloud.addRemovef_rest_41(hasf_rest_41);
  _currentPointCloud.addRemovef_rest_42(hasf_rest_42);
  _currentPointCloud.addRemovef_rest_43(hasf_rest_43);
  _currentPointCloud.addRemovef_rest_44(hasf_rest_44);

  pcc::chrono::Stopwatch<pcc::chrono::utime_inc_children_clock> clock_user;
  clock_user.start();

  int gbhSize, gbfSize;
  _gbh = parseGbh(*_sps, *_gps, buf, &gbhSize, &gbfSize);
  _prevSliceId = _sliceId;
  _sliceId = _gbh.geom_slice_id;
  _sliceOrigin = _gbh.geomBoxOrigin;

  if (_frameCtr == 0) {
    _refFrameSph.setGlobalMotionEnabled(_gps->globalMotionEnabled);
    biPredDecodeParams._refFrameSph2.setGlobalMotionEnabled(
      _gps->globalMotionEnabled);
  } else if (_firstSliceInFrame) {
    if (_gps->biPredictionEnabledFlag) {
      if (_gps->biPredictionEnabledFlag == 2) {
        processHierarchicalGOF();
      }
      if (!_gbh.biPredictionEnabledFlag) {
        attrInterPredParams.referencePointCloud =
          biPredDecodeParams.attrInterPredParams2.referencePointCloud;
        _refFrameSeq[_sps->sps_seq_parameter_set_id].cloud =
          biPredDecodeParams.refPointCloud2;
      }
      if (_gps->predgeom_enabled_flag) {
        if (!_gbh.biPredictionEnabledFlag) {
          _refFrameSph.clearRefFrameCur();
          _refFrameSph.insert(biPredDecodeParams._refPosSph2);
          _refPosSph = biPredDecodeParams._refPosSph2;
        }
        if (_gps->globalMotionEnabled) {
          _refFrameSph.setFrameMovingState(_gbh.interFrameRefGmcFlag);
          _refFrameSph.setMotionParams(
            _gbh.gm_thresh, _gbh.gm_matrix, _gbh.gm_trans);
          _refFrameSph.setRefFrameCtr(_refFrameSph.getFrameCtr() + 1);
          if (_gbh.biPredictionEnabledFlag) {
            biPredDecodeParams._refFrameSph2.setFrameMovingState(
              _gbh.interFrameRefGmcFlag2);
            biPredDecodeParams._refFrameSph2.setMotionParams(
              _gbh.gm_thresh2, _gbh.gm_matrix2, _gbh.gm_trans2);
            biPredDecodeParams._refFrameSph2.setRefFrameCtr(
              biPredDecodeParams._refFrameSph2.getFrameCtr() + 1);
          }
        }
        _refFrameSph.updateFrame(*_gps);
        if (_gbh.biPredictionEnabledFlag) {
          biPredDecodeParams._refFrameSph2.updateFrame(*_gps);
        }
      }
    }

    else {
      if (_gps->globalMotionEnabled) {
        _refFrameSph.setFrameMovingState(_gbh.interFrameRefGmcFlag);
        _refFrameSph.setMotionParams(
          _gbh.gm_thresh, _gbh.gm_matrix, _gbh.gm_trans);
      }
      _refFrameSph.updateFrame(*_gps);
    }
  }

  // sanity check for loss detection
  if (_gbh.entropy_continuation_flag) {
    assert(!_firstSliceInFrame);
    assert(_gbh.prev_slice_id == _prevSliceId);
  } else {
    // forget (reset) all saved context state at boundary
    if (
      !_sps->inter_entropy_continuation_enabled_flag
      || !_gbh.interPredictionEnabledFlag) {
      _ctxtMemOctreeGeom->reset();
      _ctxtMemPredGeom->reset();
      for (auto& ctxtMem : _ctxtMemAttrs)
        ctxtMem.reset();
    }
  }

  // set default attribute values (in case an attribute data unit is lost)
  // NB: it is a requirement that geom_num_points_minus1 is correct
  _currentPointCloud.resize(_gbh.footer.geom_num_points_minus1 + 1);
  if (hasColour) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kColour;
      });

    Vec3<attr_t> defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      for (int k = 0; k < 3; k++)
        defAttrVal[k] = it->params.attr_default_value[k];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setColor(i, defAttrVal);
  }

  if (hasReflectance) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kReflectance;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setReflectance(i, defAttrVal);
  }

  if (hasOpacity) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kOpacity;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setOpacity(i, defAttrVal);
  }

  if (hasf_dc_0) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_dc_0;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_dc_0(i, defAttrVal);
  }
  if (hasf_dc_1) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_dc_1;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_dc_1(i, defAttrVal);
  }
  if (hasf_dc_2) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_dc_2;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_dc_2(i, defAttrVal);
  }

  if (hasscale_0) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kscale_0;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setscale_0(i, defAttrVal);
  }
  if (hasscale_1) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kscale_1;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setscale_1(i, defAttrVal);
  }
  if (hasscale_2) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kscale_2;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setscale_2(i, defAttrVal);
  }

  if (hasrot_0) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::krot_0;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setrot_0(i, defAttrVal);
  }
  if (hasrot_1) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::krot_1;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setrot_1(i, defAttrVal);
  }
  if (hasrot_2) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::krot_2;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setrot_2(i, defAttrVal);
  }
  if (hasrot_3) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::krot_3;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setrot_3(i, defAttrVal);
  }
  if (hasf_rest_0) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_0;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_0(i, defAttrVal);
  }
  if (hasf_rest_1) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_1;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_1(i, defAttrVal);
  }
  if (hasf_rest_2) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_2;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_2(i, defAttrVal);
  }
  if (hasf_rest_3) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_3;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_3(i, defAttrVal);
  }
  if (hasf_rest_4) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_4;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_4(i, defAttrVal);
  }
  if (hasf_rest_5) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_5;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_5(i, defAttrVal);
  }
  if (hasf_rest_6) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_6;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_6(i, defAttrVal);
  }
  if (hasf_rest_7) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_7;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_7(i, defAttrVal);
  }
  if (hasf_rest_8) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_8;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_8(i, defAttrVal);
  }
  if (hasf_rest_9) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_9;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_9(i, defAttrVal);
  }
  if (hasf_rest_10) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_10;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_10(i, defAttrVal);
  }
  if (hasf_rest_11) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_11;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_11(i, defAttrVal);
  }
  if (hasf_rest_12) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_12;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_12(i, defAttrVal);
  }
  if (hasf_rest_13) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_13;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_13(i, defAttrVal);
  }
  if (hasf_rest_14) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_14;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_14(i, defAttrVal);
  }
  if (hasf_rest_15) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_15;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_15(i, defAttrVal);
  }
  if (hasf_rest_16) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_16;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_16(i, defAttrVal);
  }
  if (hasf_rest_17) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_17;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_17(i, defAttrVal);
  }
  if (hasf_rest_18) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_18;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_18(i, defAttrVal);
  }
  if (hasf_rest_19) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_19;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_19(i, defAttrVal);
  }
  if (hasf_rest_20) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_20;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_20(i, defAttrVal);
  }
  if (hasf_rest_21) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_21;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_21(i, defAttrVal);
  }
  if (hasf_rest_22) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_22;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_22(i, defAttrVal);
  }
  if (hasf_rest_23) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_23;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_23(i, defAttrVal);
  }
  if (hasf_rest_24) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_24;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_24(i, defAttrVal);
  }
  if (hasf_rest_25) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_25;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_25(i, defAttrVal);
  }
  if (hasf_rest_26) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_26;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_26(i, defAttrVal);
  }
  if (hasf_rest_27) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_27;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_27(i, defAttrVal);
  }
  if (hasf_rest_28) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_28;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_28(i, defAttrVal);
  }
  if (hasf_rest_29) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_29;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_29(i, defAttrVal);
  }
  if (hasf_rest_30) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_30;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_30(i, defAttrVal);
  }
  if (hasf_rest_31) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_31;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_31(i, defAttrVal);
  }
  if (hasf_rest_32) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_32;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_32(i, defAttrVal);
  }
  if (hasf_rest_33) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_33;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_33(i, defAttrVal);
  }
  if (hasf_rest_34) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_34;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_34(i, defAttrVal);
  }
  if (hasf_rest_35) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_35;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_35(i, defAttrVal);
  }
  if (hasf_rest_36) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_36;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_36(i, defAttrVal);
  }
  if (hasf_rest_37) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_37;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_37(i, defAttrVal);
  }
  if (hasf_rest_38) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_38;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_38(i, defAttrVal);
  }
  if (hasf_rest_39) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_39;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_39(i, defAttrVal);
  }
  if (hasf_rest_40) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_40;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_40(i, defAttrVal);
  }
  if (hasf_rest_41) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_41;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_41(i, defAttrVal);
  }
  if (hasf_rest_42) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_42;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_42(i, defAttrVal);
  }
  if (hasf_rest_43) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_43;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_43(i, defAttrVal);
  }
  if (hasf_rest_44) {
    auto it = std::find_if(
      _outCloud.attrDesc.cbegin(), _outCloud.attrDesc.cend(),
      [](const AttributeDescription& desc) {
        return desc.attributeLabel == KnownAttributeLabel::kf_rest_44;
      });
    attr_t defAttrVal = 1 << (it->bitdepth - 1);
    if (!it->params.attr_default_value.empty())
      defAttrVal = it->params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setf_rest_44(i, defAttrVal);
  }

  // Calculate a tree level at which to stop
  // It should result in at most max points being decoded
  if (_params.decodeMaxPoints && _gps->octree_point_count_list_present_flag) {
    if (_params.decodeMaxPoints > _gbh.footer.geom_num_points_minus1)
      _params.minGeomNodeSizeLog2 = 0;
    else {
      auto it = std::lower_bound(
        std::next(_gbh.footer.octree_lvl_num_points_minus1.begin()),
        _gbh.footer.octree_lvl_num_points_minus1.end(),
        _params.decodeMaxPoints);

      _params.minGeomNodeSizeLog2 =
        std::distance(it, _gbh.footer.octree_lvl_num_points_minus1.end()) + 1;
    }
  }

  EntropyDecoder aec;
  aec.setBuffer(buf.size() - gbhSize - gbfSize, buf.data() + gbhSize);
  aec.enableBypassStream(_sps->cabac_bypass_stream_enabled_flag);
  aec.setBypassBinCodingWithoutProbUpdate(
    _sps->bypass_bin_coding_without_prob_update);
  aec.start();

  if (_gps->predgeom_enabled_flag) {
    _refFrameSph.setInterEnabled(_gbh.interPredictionEnabledFlag);
    biPredDecodeParams._refFrameSph2.setInterEnabled(
      _gbh.interPredictionEnabledFlag && _gbh.biPredictionEnabledFlag);
    if (!_gbh.interPredictionEnabledFlag) {
      _refFrameSph.clearRefFrame();
      biPredDecodeParams._refFrameSph2.clearRefFrame();
    }
    decodePredictiveGeometry(
      *_gps, _gbh, _currentPointCloud, &_posSph, _refFrameSph,
      biPredDecodeParams._refFrameSph2, *_ctxtMemPredGeom, aec);
  } else if (!_gps->trisoup_enabled_flag) {
    if (!_params.minGeomNodeSizeLog2) {
      decodeGeometryOctree(
        *_gps, _gbh, _currentPointCloud, *_ctxtMemOctreeGeom, aec, _refFrame,
        biPredDecodeParams.refPointCloud2, _sps->seqBoundingBoxOrigin);
    } else {
      decodeGeometryOctreeScalable(
        *_gps, _gbh, _params.minGeomNodeSizeLog2, _currentPointCloud,
        *_ctxtMemOctreeGeom, aec, _refFrame,
        biPredDecodeParams.refPointCloud2);
    }
  } else {
    decodeGeometryTrisoup(
      *_gps, _gbh, _currentPointCloud, *_ctxtMemOctreeGeom, aec, _refFrame,
      _sps->seqBoundingBoxOrigin);
  }

  bool currFrameNotCodedAsB =
    (_gps->biPredictionEnabledFlag) && !_gbh.biPredictionEnabledFlag;
  auto& refFrameSph =
    currFrameNotCodedAsB ? biPredDecodeParams._refFrameSph2 : _refFrameSph;
  auto& refPosSph =
    currFrameNotCodedAsB ? biPredDecodeParams._refPosSph2 : _refPosSph;

  if (_gps->interPredictionEnabledFlag && _gps->predgeom_enabled_flag) {
    refFrameSph.insert(_posSph);
    refPosSph = _posSph;
  }

  biPredDecodeParams.preFrameAsBframe = _gbh.biPredictionEnabledFlag;

  if (_gps->biPredictionEnabledFlag) {
    if (!_gbh.biPredictionEnabledFlag) {
      biPredDecodeParams.prePreIPFrame = biPredDecodeParams.preIPFrame;
      biPredDecodeParams.preIPFrame = _outCloud.frameNum;
    }
  }

  // At least the first slice's geometry has been decoded
  _firstSliceInFrame = false;

  clock_user.stop();

  auto total_user =
    std::chrono::duration_cast<std::chrono::milliseconds>(clock_user.count());
  std::cout << "positions processing time (user): "
            << total_user.count() / 1000.0 << " s\n";
  std::cout << std::endl;

  return 0;
}

//--------------------------------------------------------------------------

void
PCCTMC3Decoder3::decodeAttributeBrick(const PayloadBuffer& buf)
{
  std::cout << "PCCTMC3Decoder3::decodeAttributeBrick start" << std::endl;
  assert(buf.type == PayloadType::kAttributeBrick);
  // todo(df): replace assertions with error handling
  assert(_sps);
  assert(_gps);

  // verify that this corresponds to the correct geometry slice
  AttributeBrickHeader abh = parseAbhIds(buf);
  assert(abh.attr_geom_slice_id == _sliceId);

  // todo(df): validate that sps activation is not changed via the APS
  const auto it_attr_aps = _apss.find(abh.attr_attr_parameter_set_id);

  assert(it_attr_aps != _apss.cend());
  const auto& attr_aps = it_attr_aps->second;

  assert(abh.attr_sps_attr_idx < _sps->attributeSets.size());
  const auto& attr_sps = _sps->attributeSets[abh.attr_sps_attr_idx];
  const auto& label = attr_sps.attributeLabel;

  // sanity check for loss detection
  if (_gbh.entropy_continuation_flag)
    assert(_gbh.prev_slice_id == _ctxtMemAttrSliceIds[abh.attr_sps_attr_idx]);

  // Ensure context arrays are allocated context arrays
  // todo(df): move this to sps activation
  _ctxtMemAttrSliceIds.resize(_sps->attributeSets.size());
  _ctxtMemAttrs.resize(_sps->attributeSets.size());

  // In order to determinet hat the attribute decoder is reusable, the abh
  // must be inspected.
  int abhSize;
  abh.RAHTFilterTaps.clear();
  abh = parseAbh(*_sps, attr_aps, buf, &abhSize);

  attrInterPredParams.frameDistance = 1;
  attrInterPredParams.enableAttrInterPred =
    attr_aps.attrInterPredictionEnabled && abh.enableAttrInterPred;

  abh.attrInterPredSearchRange = attr_aps.attrInterPredSearchRange;

  biPredDecodeParams.attrInterPredParams2.enableAttrInterPred =
    _gps->biPredictionEnabledFlag && attr_aps.attrInterPredictionEnabled
    && !abh.disableAttrInterPredForRefFrame2;

  if (_gps->biPredictionEnabledFlag) {
    //FrameMerge for attribute inter prediction
    if (
      _gbh.biPredictionEnabledFlag
      && biPredDecodeParams.attrInterPredParams2.enableAttrInterPred) {
      if (attrInterPredParams.enableAttrInterPred) {
        attrInterPredParams.referencePointCloud.append(
          biPredDecodeParams.attrInterPredParams2.referencePointCloud);

        //The times of neighbor search process is halfed
        abh.attrInterPredSearchRange /= 2;
      } else {
        attrInterPredParams = biPredDecodeParams.attrInterPredParams2;
        attrInterPredParams.frameDistance = 1;
      }
    }
  }

  attrInterPredParams.paramsForInterRAHT.raht_inter_prediction_depth_minus1 =
    attr_aps.raht_inter_prediction_depth_minus1;
  attrInterPredParams.paramsForInterRAHT.enableFilterEstimation =
    attr_aps.raht_send_inter_filters;
  attrInterPredParams.paramsForInterRAHT.skipInitLayersForFiltering =
    attr_aps.raht_inter_skip_layers;
  attrInterPredParams.paramsForInterRAHT.FilterTaps.clear();
  if (attr_aps.raht_send_inter_filters && abh.enableAttrInterPred) {
    attrInterPredParams.paramsForInterRAHT.FilterTaps = abh.RAHTFilterTaps;
  }

  attrInterPredParams.paramsForInterRAHT.raht_enable_inter_intra_layer_RDO =
    attr_aps.raht_enable_code_layer;
  attrInterPredParams.attr_layer_code_mode = abh.raht_attr_layer_code_mode;
  if (
    attr_sps.attr_num_dimensions_minus1 == 0
    && !_gps->geom_angular_mode_enabled_flag)
    attrInterPredParams.enableSkipCode = false;
  else
    attrInterPredParams.enableSkipCode = true;

  pcc::chrono::Stopwatch<pcc::chrono::utime_inc_children_clock> clock_user;

  // replace the attribute decoder if not compatible
  if (!_attrDecoder || !_attrDecoder->isReusable(attr_aps, abh))
    _attrDecoder = makeAttributeDecoder();

  clock_user.start();

  // Convert cartesian positions to spherical for use in attribute coding.
  // NB: this retains the original cartesian positions to restore afterwards
  std::vector<pcc::point_t> altPositions;
  if (attr_aps.spherical_coord_flag) {
    // If predgeom was used, re-use the internal positions rather than
    // calculating afresh.
    Box3<int> bboxRpl;

    pcc::point_t minPos = 0;

    if (_gps->predgeom_enabled_flag) {
      altPositions = _posSph;
      bboxRpl = Box3<int>(altPositions.begin(), altPositions.end());
      minPos = bboxRpl.min;
      if (
        attrInterPredParams.enableAttrInterPred
        || biPredDecodeParams.attrInterPredParams2.enableAttrInterPred) {
        for (auto i = 0; i < 3; i++)
          minPos[i] = minPos[i] < minPos_ref[i] ? minPos[i] : minPos_ref[i];
        auto minPos_shift = minPos_ref - minPos;

        if (minPos_shift[0] || minPos_shift[1] || minPos_shift[2])
          offsetAndScaleShift(
            minPos_shift, attr_aps.attr_coord_scale,
            &attrInterPredParams.referencePointCloud[0],
            &attrInterPredParams.referencePointCloud[0]
              + attrInterPredParams.getPointCount());
      }
      minPos_ref = minPos;
    } else {
      altPositions.resize(_currentPointCloud.getPointCount());

      auto laserOrigin = _gbh.geomAngularOrigin(*_gps);
      bboxRpl = convertXyzToRpl(
        laserOrigin, _gps->angularTheta.data(), _gps->angularTheta.size(),
        &_currentPointCloud[0],
        &_currentPointCloud[0] + _currentPointCloud.getPointCount(),
        altPositions.data());

      if (!attr_aps.attrInterPredictionEnabled) {
        minPos = bboxRpl.min;
      }
    }

    offsetAndScale(
      minPos, attr_aps.attr_coord_scale, altPositions.data(),
      altPositions.data() + altPositions.size());

    _currentPointCloud.swapPoints(altPositions);
  }

  if (!attr_aps.spherical_coord_flag)
    for (auto i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud[i] += _sliceOrigin;

  if (attr_aps.attrInterPredictionEnabled)
    if (attr_aps.attr_encoding != AttributeEncoding::kRAHTransform)
      if (_refFrameAlt) {
        Box3<int> currentFrameBox = _currentPointCloud.computeBoundingBox();
        attrInterPredParams.referencePointCloud = _refFrameAlt->cloud;
        int count = 0;
        for (int i = 0; i < attrInterPredParams.getPointCount(); i++) {
          point_t p = attrInterPredParams.referencePointCloud[i];
          if (currentFrameBox.contains(p)) {
            attrInterPredParams.referencePointCloud[count] = p;
            if (attrInterPredParams.referencePointCloud.hasReflectances())
              attrInterPredParams.referencePointCloud.setReflectance(
                count,
                attrInterPredParams.referencePointCloud.getReflectance(i));
            if (attrInterPredParams.referencePointCloud.hasOpacities())
              attrInterPredParams.referencePointCloud.setOpacity(
                count, attrInterPredParams.referencePointCloud.getOpacity(i));
            if (attrInterPredParams.referencePointCloud.hasf_dc_0())
              attrInterPredParams.referencePointCloud.setf_dc_0(
                count, attrInterPredParams.referencePointCloud.getf_dc_0(i));
            if (attrInterPredParams.referencePointCloud.hasf_dc_1())
              attrInterPredParams.referencePointCloud.setf_dc_1(
                count, attrInterPredParams.referencePointCloud.getf_dc_1(i));
            if (attrInterPredParams.referencePointCloud.hasf_dc_2())
              attrInterPredParams.referencePointCloud.setf_dc_2(
                count, attrInterPredParams.referencePointCloud.getf_dc_2(i));

            if (attrInterPredParams.referencePointCloud.hasscale_0())
              attrInterPredParams.referencePointCloud.setscale_0(
                count, attrInterPredParams.referencePointCloud.getscale_0(i));
            if (attrInterPredParams.referencePointCloud.hasscale_1())
              attrInterPredParams.referencePointCloud.setscale_1(
                count, attrInterPredParams.referencePointCloud.getscale_1(i));
            if (attrInterPredParams.referencePointCloud.hasscale_2())
              attrInterPredParams.referencePointCloud.setscale_2(
                count, attrInterPredParams.referencePointCloud.getscale_2(i));

            if (attrInterPredParams.referencePointCloud.hasrot_0())
              attrInterPredParams.referencePointCloud.setrot_0(
                count, attrInterPredParams.referencePointCloud.getrot_0(i));
            if (attrInterPredParams.referencePointCloud.hasrot_1())
              attrInterPredParams.referencePointCloud.setrot_1(
                count, attrInterPredParams.referencePointCloud.getrot_1(i));
            if (attrInterPredParams.referencePointCloud.hasrot_2())
              attrInterPredParams.referencePointCloud.setrot_2(
                count, attrInterPredParams.referencePointCloud.getrot_2(i));
            if (attrInterPredParams.referencePointCloud.hasrot_3())
              attrInterPredParams.referencePointCloud.setrot_3(
                count, attrInterPredParams.referencePointCloud.getrot_3(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_0())
              attrInterPredParams.referencePointCloud.setf_rest_0(
                count, attrInterPredParams.referencePointCloud.getf_rest_0(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_1())
              attrInterPredParams.referencePointCloud.setf_rest_1(
                count, attrInterPredParams.referencePointCloud.getf_rest_1(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_2())
              attrInterPredParams.referencePointCloud.setf_rest_2(
                count, attrInterPredParams.referencePointCloud.getf_rest_2(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_3())
              attrInterPredParams.referencePointCloud.setf_rest_3(
                count, attrInterPredParams.referencePointCloud.getf_rest_3(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_4())
              attrInterPredParams.referencePointCloud.setf_rest_4(
                count, attrInterPredParams.referencePointCloud.getf_rest_4(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_5())
              attrInterPredParams.referencePointCloud.setf_rest_5(
                count, attrInterPredParams.referencePointCloud.getf_rest_5(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_6())
              attrInterPredParams.referencePointCloud.setf_rest_6(
                count, attrInterPredParams.referencePointCloud.getf_rest_6(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_7())
              attrInterPredParams.referencePointCloud.setf_rest_7(
                count, attrInterPredParams.referencePointCloud.getf_rest_7(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_8())
              attrInterPredParams.referencePointCloud.setf_rest_8(
                count, attrInterPredParams.referencePointCloud.getf_rest_8(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_9())
              attrInterPredParams.referencePointCloud.setf_rest_9(
                count, attrInterPredParams.referencePointCloud.getf_rest_9(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_10())
              attrInterPredParams.referencePointCloud.setf_rest_10(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_10(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_11())
              attrInterPredParams.referencePointCloud.setf_rest_11(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_11(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_12())
              attrInterPredParams.referencePointCloud.setf_rest_12(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_12(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_13())
              attrInterPredParams.referencePointCloud.setf_rest_13(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_13(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_14())
              attrInterPredParams.referencePointCloud.setf_rest_14(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_14(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_15())
              attrInterPredParams.referencePointCloud.setf_rest_15(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_15(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_16())
              attrInterPredParams.referencePointCloud.setf_rest_16(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_16(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_17())
              attrInterPredParams.referencePointCloud.setf_rest_17(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_17(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_18())
              attrInterPredParams.referencePointCloud.setf_rest_18(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_18(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_19())
              attrInterPredParams.referencePointCloud.setf_rest_19(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_19(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_20())
              attrInterPredParams.referencePointCloud.setf_rest_20(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_20(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_21())
              attrInterPredParams.referencePointCloud.setf_rest_21(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_21(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_22())
              attrInterPredParams.referencePointCloud.setf_rest_22(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_22(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_23())
              attrInterPredParams.referencePointCloud.setf_rest_23(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_23(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_24())
              attrInterPredParams.referencePointCloud.setf_rest_24(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_24(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_25())
              attrInterPredParams.referencePointCloud.setf_rest_25(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_25(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_26())
              attrInterPredParams.referencePointCloud.setf_rest_26(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_26(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_27())
              attrInterPredParams.referencePointCloud.setf_rest_27(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_27(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_28())
              attrInterPredParams.referencePointCloud.setf_rest_28(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_28(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_29())
              attrInterPredParams.referencePointCloud.setf_rest_29(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_29(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_30())
              attrInterPredParams.referencePointCloud.setf_rest_30(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_30(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_31())
              attrInterPredParams.referencePointCloud.setf_rest_31(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_31(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_32())
              attrInterPredParams.referencePointCloud.setf_rest_32(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_32(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_33())
              attrInterPredParams.referencePointCloud.setf_rest_33(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_33(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_34())
              attrInterPredParams.referencePointCloud.setf_rest_34(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_34(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_35())
              attrInterPredParams.referencePointCloud.setf_rest_35(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_35(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_36())
              attrInterPredParams.referencePointCloud.setf_rest_36(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_36(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_37())
              attrInterPredParams.referencePointCloud.setf_rest_37(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_37(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_38())
              attrInterPredParams.referencePointCloud.setf_rest_38(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_38(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_39())
              attrInterPredParams.referencePointCloud.setf_rest_39(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_39(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_40())
              attrInterPredParams.referencePointCloud.setf_rest_40(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_40(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_41())
              attrInterPredParams.referencePointCloud.setf_rest_41(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_41(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_42())
              attrInterPredParams.referencePointCloud.setf_rest_42(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_42(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_43())
              attrInterPredParams.referencePointCloud.setf_rest_43(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_43(i));
            if (attrInterPredParams.referencePointCloud.hasf_rest_44())
              attrInterPredParams.referencePointCloud.setf_rest_44(
                count,
                attrInterPredParams.referencePointCloud.getf_rest_44(i));

            if (attrInterPredParams.referencePointCloud.hasColors())
              attrInterPredParams.referencePointCloud.setColor(
                count, attrInterPredParams.referencePointCloud.getColor(i));
            count++;
          }
        }
        attrInterPredParams.referencePointCloud.resize(count);
      }

  auto& ctxtMemAttr = _ctxtMemAttrs.at(abh.attr_sps_attr_idx);
  _attrDecoder->decode(  // AttributeDecoder::decode
    *_sps, attr_sps, attr_aps, abh, _gbh.footer.geom_num_points_minus1,
    _params.minGeomNodeSizeLog2, buf.data() + abhSize, buf.size() - abhSize,
    ctxtMemAttr, _currentPointCloud, attrInterPredParams);

  _reconSliceAltPositions = _currentPointCloud;
  bool currFrameNotCodedAsB =
    (_gps->biPredictionEnabledFlag && !_gbh.biPredictionEnabledFlag);
  auto& refCloud = currFrameNotCodedAsB
    ? biPredDecodeParams.attrInterPredParams2.referencePointCloud
    : attrInterPredParams.referencePointCloud;

  if (attr_aps.spherical_coord_flag) {
    refCloud = _currentPointCloud;
    _currentPointCloud.swapPoints(altPositions);
  } else {
    refCloud = _currentPointCloud;
  }
  if (!attr_aps.spherical_coord_flag)
    for (auto i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud[i] -= _sliceOrigin;

  _accumCloudAltPositions.append(_reconSliceAltPositions);

  // Note the current sliceID for loss detection
  _ctxtMemAttrSliceIds[abh.attr_sps_attr_idx] = _sliceId;

  clock_user.stop();

  std::cout << label << "s bitstream size " << buf.size() << " B\n";

  auto total_user =
    std::chrono::duration_cast<std::chrono::milliseconds>(clock_user.count());
  std::cout << label
            << "s processing time (user): " << total_user.count() / 1000.0
            << " s\n";
  std::cout << std::endl;
}

//--------------------------------------------------------------------------

void
PCCTMC3Decoder3::decodeConstantAttribute(const PayloadBuffer& buf)
{
  std::cout << "PCCTMC3Decoder3::decodeConstantAttribute start" << std::endl;
  assert(buf.type == PayloadType::kConstantAttribute);
  // todo(df): replace assertions with error handling
  assert(_sps);
  assert(_gps);

  ConstantAttributeDataUnit cadu = parseConstantAttribute(*_sps, buf);

  // verify that this corresponds to the correct geometry slice
  assert(cadu.constattr_geom_slice_id == _sliceId);

  assert(cadu.constattr_sps_attr_idx < _sps->attributeSets.size());
  const auto& attrDesc = _sps->attributeSets[cadu.constattr_sps_attr_idx];
  const auto& label = attrDesc.attributeLabel;

  // todo(df): replace with proper attribute mapping
  if (label == KnownAttributeLabel::kColour) {
    Vec3<attr_t> defAttrVal;
    for (int k = 0; k < 3; k++)
      defAttrVal[k] = attrDesc.params.attr_default_value[k];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setColor(i, defAttrVal);
  }

  if (label == KnownAttributeLabel::kReflectance) {
    attr_t defAttrVal = attrDesc.params.attr_default_value[0];
    for (int i = 0; i < _currentPointCloud.getPointCount(); i++)
      _currentPointCloud.setReflectance(i, defAttrVal);
  }
}

//============================================================================

}  // namespace pcc
