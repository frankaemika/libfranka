// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <algorithm>
#include <array>
#include <cmath>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

namespace {

int sgn(double x) {
  if (x == 0) {
    return 0;
  }
  return (x > 0) ? 1 : -1;
}

std::array<double, 7> add(const std::array<double, 7>& a, const std::array<double, 7>& b) {
  std::array<double, 7> result;
  for (size_t i = 0; i < a.size(); i++) {
    result[i] = a[i] + b[i];
  }
  return result;
}

std::array<double, 7> subtract(const std::array<double, 7>& a, const std::array<double, 7>& b) {
  std::array<double, 7> result;
  for (size_t i = 0; i < a.size(); i++) {
    result[i] = a[i] - b[i];
  }
  return result;
}

}  // anonymous namespace

std::array<double, 7> saturate(const std::array<double, 7>& max_value,
                               const std::array<double, 7>& desired_value,
                               const std::array<double, 7>& last_value) {
  std::array<double, 7> saturated_value{};
  for (size_t i = 0; i < 7; i++) {
    double vel = (desired_value[i] - last_value[i]) / 1e-3;
    saturated_value[i] =
        last_value[i] + std::max(std::min(vel, max_value[i]), -max_value[i]) * 1e-3;
  }
  return saturated_value;
};

MotionGenerator::MotionGenerator(double speed_factor, const std::array<double, 7> q_goal)
    : q_goal_(q_goal) {
  for (size_t i = 0; i < 7; i++) {
    dq_max_[i] *= speed_factor;
    ddq_max_start_[i] *= speed_factor;
    ddq_max_goal_[i] *= speed_factor;
  }
}

bool MotionGenerator::calculateDesiredValues(double t, std::array<double, 7>* delta_q_d) const {
  std::array<int, 7> sign_delta_q;
  std::array<double, 7> t_d = subtract(t_2_sync_, t_1_sync_);
  std::array<double, 7> delta_t_2_sync_ = subtract(t_f_sync_, t_2_sync_);
  std::array<bool, 7> joint_motion_finished{};

  for (size_t i = 0; i < 7; i++) {
    sign_delta_q[i] = sgn(delta_q_[i]);
    if (std::abs(delta_q_[i]) < kDeltaQMotionFinished_) {
      (*delta_q_d)[i] = 0;
      joint_motion_finished[i] = true;
    } else {
      if (t < t_1_sync_[i]) {
        (*delta_q_d)[i] = -1.0 / std::pow(t_1_sync_[i], 3) * dq_max_sync_[i] * sign_delta_q[i] *
                          (0.5 * t - t_1_sync_[i]) * std::pow(t, 3);
      } else if (t >= t_1_sync_[i] && t < t_2_sync_[i]) {
        (*delta_q_d)[i] = q_1_[i] + (t - t_1_sync_[i]) * dq_max_sync_[i] * sign_delta_q[i];
      } else if (t >= t_2_sync_[i] && t < t_f_sync_[i]) {
        (*delta_q_d)[i] = delta_q_[i] +
                          0.5 *
                              (1.0 / std::pow(delta_t_2_sync_[i], 3) *
                                   (t - t_1_sync_[i] - 2 * delta_t_2_sync_[i] - t_d[i]) *
                                   std::pow((t - t_1_sync_[i] - t_d[i]), 3) +
                               (2.0 * t - 2.0 * t_1_sync_[i] - delta_t_2_sync_[i] - 2.0 * t_d[i])) *
                              dq_max_sync_[i] * sign_delta_q[i];
      } else {
        (*delta_q_d)[i] = delta_q_[i];
        joint_motion_finished[i] = true;
      }
    }
  }
  return std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(),
                     [](bool x) { return x; });
}

void MotionGenerator::calculateSynchronizedValues() {
  std::array<double, 7> dq_max_reach = dq_max_;
  std::array<double, 7> t_f{};
  std::array<double, 7> delta_t_2{};
  std::array<double, 7> t_1{};
  std::array<double, 7> delta_t_2_sync{};
  int sign_delta_q_[7];
  for (size_t i = 0; i < 7; i++) {
    sign_delta_q_[i] = sgn(delta_q_[i]);
    if (std::abs(delta_q_[i]) > kDeltaQMotionFinished_) {
      if (std::abs(delta_q_[i]) < (3.0 / 4.0 * (std::pow(dq_max_[i], 2) / ddq_max_start_[i]) +
                                   3.0 / 4.0 * (std::pow(dq_max_[i], 2) / ddq_max_goal_[i]))) {
        dq_max_reach[i] = std::sqrt(4.0 / 3.0 * delta_q_[i] * sign_delta_q_[i] *
                                    (ddq_max_start_[i] * ddq_max_goal_[i]) /
                                    (ddq_max_start_[i] + ddq_max_goal_[i]));
      }
      t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_start_[i];
      delta_t_2[i] = 1.5 * dq_max_reach[i] / ddq_max_goal_[i];
      t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_q_[i]) / dq_max_reach[i];
    }
  }
  double max_t_f = *std::max_element(t_f.begin(), t_f.end());
  for (size_t i = 0; i < 7; i++) {
    if (std::abs(delta_q_[i]) > kDeltaQMotionFinished_) {
      double a = 1.5 / 2.0 * (ddq_max_goal_[i] + ddq_max_start_[i]);
      double b = -1.0 * max_t_f * ddq_max_goal_[i] * ddq_max_start_[i];
      double c = std::abs(delta_q_[i]) * ddq_max_goal_[i] * ddq_max_start_[i];
      double delta = b * b - 4.0 * a * c;
      dq_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
      t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_start_[i];
      delta_t_2_sync[i] = 1.5 * dq_max_sync_[i] / ddq_max_goal_[i];
      t_f_sync_[i] =
          (t_1_sync_)[i] / 2 + delta_t_2_sync[i] / 2 + std::abs(delta_q_[i] / dq_max_sync_[i]);
      t_2_sync_[i] = (t_f_sync_)[i] - delta_t_2_sync[i];
      q_1_[i] = (dq_max_sync_)[i] * sign_delta_q_[i] * (0.5 * (t_1_sync_)[i]);
    }
  }
}

franka::JointPositions MotionGenerator::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration time_step) {
  if (time_ == 0.0) {
    q_start_ = robot_state.q_d;
    delta_q_ = subtract(q_goal_, q_start_);
    calculateSynchronizedValues();
  }

  time_ += time_step.toSec();

  std::array<double, 7> delta_q_d;
  bool motion_finished = calculateDesiredValues(time_, &delta_q_d);

  franka::JointPositions output = add(q_start_, delta_q_d);
  output.motion_finished = motion_finished;
  return output;
}