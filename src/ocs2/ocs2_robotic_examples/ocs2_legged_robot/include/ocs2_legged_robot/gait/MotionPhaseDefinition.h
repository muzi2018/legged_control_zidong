/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

#include "ocs2_legged_robot/common/Types.h"

namespace ocs2 {
namespace legged_robot {

enum ModeNumber {  // {LF, RF, LH, RH}
  FLY = 0,
  LF_LH = 10,
  RF_RH = 5,
  STANCE = 15,
};
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline contact_flag_t modeNumber2StanceLeg(const size_t& modeNumber) {
  contact_flag_t stanceLegs;  // {LF, RF, LH, RH}

  switch (modeNumber) {
    case 0:
      stanceLegs = contact_flag_t{false, false, false, false};
      break;  // 0:  0-leg-stance
    case 10:
      stanceLegs = contact_flag_t{true, false, true, false};
      break;  // 10: LF, LH
    case 5:
      stanceLegs = contact_flag_t{false, true, false, true};
      break;  // 5:  RF, RH
    case 15:
      stanceLegs = contact_flag_t{true, true, true, true};
      break;  // 15: 4-leg-stance
  }

  return stanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline size_t stanceLeg2ModeNumber(const contact_flag_t& stanceLegs) {
  return static_cast<size_t>(stanceLegs[3]) + 2 * static_cast<size_t>(stanceLegs[2]) + 4 * static_cast<size_t>(stanceLegs[1]) +
         8 * static_cast<size_t>(stanceLegs[0]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline std::string modeNumber2String(const size_t& modeNumber) {
  // build the map from mode number to name
  std::map<size_t, std::string> modeToName;
  modeToName[FLY] = "FLY";
  modeToName[LF_LH] = "LF_LH";
  modeToName[RF_RH] = "RF_RH";
  modeToName[STANCE] = "STANCE";

  return modeToName[modeNumber];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline size_t string2ModeNumber(const std::string& modeString) {
  // build the map from name to mode number
  std::map<std::string, size_t> nameToMode;
  nameToMode["FLY"] = FLY;
  nameToMode["LF_LH"] = LF_LH;
  nameToMode["RF_RH"] = RF_RH;
  nameToMode["STANCE"] = STANCE;


  return nameToMode[modeString];
}

}  // namespace legged_robot
}  // end of namespace ocs2
