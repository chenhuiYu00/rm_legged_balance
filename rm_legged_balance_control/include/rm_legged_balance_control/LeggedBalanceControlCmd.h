//
// Created by yuchen on 23-9-2.
//

#pragma once

namespace rm {
using ocs2::scalar_t;

class LeggedBalanceControlCmd {
 public:
  void setPowerLimit(scalar_t powerLimit) { powerLimit_ = powerLimit; }
  void setMaxPitch(scalar_t maxPitch, scalar_t time) {
    if (abs(maxPitch - maxPitch_) > 1e-9) {
      lastMaxPitch_ = maxPitch_;
      maxPitch_ = maxPitch;
      maxPitchSetTime_ = time;
    }
  }
  void setSitDown(bool sitDown) { sitDown_ = sitDown; }
  void setJump(bool jump) { jump_ = jump; }
  void setForwardVel(scalar_t forwardVel) { forwardVel_ = forwardVel; }
  void setYawError(scalar_t yawError) { yawError_ = yawError; }
  void setPendulumLength(ocs2::vector_t length) { pendulumLength_ = length; }
  void setLegCmd(scalar_t cmd) { legCmd_ = cmd; };
  void setRollCmd(scalar_t cmd) { rollCmd_ = cmd; }

  scalar_t getPowerLimit() const { return powerLimit_; }
  scalar_t getMaxPitch() const { return maxPitch_; }
  scalar_t getLastMaxPitch() const { return lastMaxPitch_; }
  scalar_t getMaxPitchSetTime() const { return maxPitchSetTime_; }
  ocs2::vector_t getPendulumLength() { return pendulumLength_; }
  bool getSitDown() const { return sitDown_; }
  bool getJump() const { return jump_; }
  scalar_t getForwardVel() { return forwardVel_; }
  scalar_t getYawError() { return yawError_; }
  scalar_t getLegCmd() { return legCmd_; }
  scalar_t getRollCmd() { return rollCmd_; }

 private:
  scalar_t powerLimit_, maxPitch_, lastMaxPitch_, maxPitchSetTime_, legCmd_, rollCmd_;
  ocs2::vector_t pendulumLength_;
  bool sitDown_, jump_;
  scalar_t forwardVel_, yawError_;
};

}  // namespace rm
