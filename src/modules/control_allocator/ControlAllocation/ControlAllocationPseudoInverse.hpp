/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ControlAllocationPseudoInverse.hpp
 *
 * Simple Control Allocation Algorithm
 *
 * It computes the pseudo-inverse of the effectiveness matrix
 * Actuator saturation is handled by simple clipping, do not
 * expect good performance in case of actuator saturation.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include "ControlAllocation.hpp"
#include <matrix/matrix/math.hpp>

class ControlAllocationPseudoInverse: public ControlAllocation
{
public:
	ControlAllocationPseudoInverse() = default;
	virtual ~ControlAllocationPseudoInverse() = default;

	void allocate() override;
	void setEffectivenessMatrix(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
				    const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
				    bool update_normalization_scale) override;

protected:
	matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> _mix;

	bool _mix_update_needed{false};

	/**
	 * Recalculate pseudo inverse if required.
	 *
	 */
	void updatePseudoInverse();

private:
	void normalizeControlAllocationMatrix();
	void updateControlAllocationMatrixScale();
	bool _normalization_needs_update{false};
// modeify: new control allocation, define parameters
	double theta1 = 0; // 假设 theta1 = 45 度  M_PI / 4
	double theta2 = 0; // 假设 theta2 = 30 度  M_PI / 6
	double theta3 = 0; // 假设 theta3 = 60 度  M_PI / 3
	double f1 = 1; //
	double f2 = 1; //
	double f3 = 1; //

	double L1 = 0.21;          // 假设 L1 =
	double L2 = 0.79;          // 假设 L2 =
	double L3 = 0.484;          // 假设 L3 =
	// 定义 sin 和 cos 函数的简写
	double sin_theta1 = sin(theta1);
	double cos_theta1 = cos(theta1);
	double sin_theta2 = sin(theta2);
	double cos_theta2 = cos(theta2);
	double sin_theta3 = sin(theta3);
	double cos_theta3 = cos(theta3);

	double scale1 = 0.5;
	double scale2 = 0.5 * L1 / (L1+L2);
	double scale3 = 0.5;
	double scale4 = 0.5;
	double scale5 = 1;
	double scale6 = 0.5 * L1 / (L1+L2);
};
