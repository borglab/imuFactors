/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    Window.cpp
 * @brief   Dataset-backed integration window implementation
 */

#include "Window.h"

#include <algorithm>
#include <stdexcept>

namespace gtsam {

Window::Window(const Dataset& dataset, size_t startIndex, size_t endIndex)
    : dataset_(&dataset), startIndex_(startIndex), endIndex_(endIndex) {
  const size_t sampleCount =
      std::min(dataset_->getStates().size(), dataset_->getImuData().size());
  if (startIndex_ >= endIndex_) {
    throw std::runtime_error(
        "Window must contain at least one integration step.");
  }
  if (endIndex_ >= sampleCount) {
    throw std::runtime_error(
        "Window end index exceeds synchronized sample count.");
  }
}

double Window::startTime() const { return initialStateMeasurement().timestamp; }

double Window::endTime() const { return terminalStateMeasurement().timestamp; }

double Window::duration() const { return endTime() - startTime(); }

double Window::timestep() const { return dataset_->timestep(); }

const Dataset::StateMeasurement& Window::initialStateMeasurement() const {
  return dataset_->getStates()[startIndex_];
}

const Dataset::StateMeasurement& Window::terminalStateMeasurement() const {
  return dataset_->getStates()[endIndex_];
}

const NavState& Window::initialState() const {
  return initialStateMeasurement().navState;
}

const NavState& Window::terminalState() const {
  return terminalStateMeasurement().navState;
}

const imuBias::ConstantBias& Window::initialBias() const {
  return initialStateMeasurement().bias;
}

const imuBias::ConstantBias& Window::terminalBias() const {
  return terminalStateMeasurement().bias;
}

}  // namespace gtsam
