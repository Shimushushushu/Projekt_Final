/*---------------------------Modified Private Lib-----------------------------*/

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "privatelib/DifferentialDrive_Kai.h"

#include <algorithm>
#include <cmath>

#include <frc/smartdashboard/SendableBuilder.h>
#include <hal/HAL.h>

using namespace frc;

DifferentialDrive_Kai::DifferentialDrive_Kai(SpeedController& leftMotor, SpeedController& rightMotor,
											 SpeedController& frontMotor, SpeedController& rearMotor)
	: m_leftMotor(leftMotor), m_rightMotor(rightMotor), m_frontMotor(frontMotor), m_rearMotor(rearMotor) {
	AddChild(&m_leftMotor);
	AddChild(&m_rightMotor);
	AddChild(&m_frontMotor);
	AddChild(&m_rearMotor);
	static int instances = 0;
	++instances;
	SetName("DifferentialDrive_Kai", instances);
}

void DifferentialDrive_Kai::ArcadeDrive_Kai(double xSpeed, double ySpeed, double zRotation,
									bool squareInputs) {
	static bool reported = false;
	if (!reported) {
	HAL_Report(HALUsageReporting::kResourceType_RobotDrive, 2,
				 HALUsageReporting::kRobotDrive2_DifferentialArcade);
	reported = true;
	}

	xSpeed = Limit(xSpeed);
	xSpeed = ApplyDeadband(xSpeed, m_deadband);

	ySpeed = Limit(ySpeed);
	ySpeed = ApplyDeadband(ySpeed, m_deadband);

	zRotation = Limit(zRotation);
	zRotation = ApplyDeadband(zRotation, m_deadband);

	// Square the inputs (while preserving the sign) to increase fine control
	// while permitting full power.
	if (squareInputs) {
	xSpeed = std::copysign(xSpeed * xSpeed, xSpeed);
	xSpeed = std::copysign(ySpeed * ySpeed, ySpeed);
	zRotation = std::copysign(zRotation * zRotation, zRotation);
	}

	double leftMotorOutput;
	double rightMotorOutput;
	double frontMotorOutput;
	double rearMotorOutput;

	double maxInputx =
		std::copysign(std::max(std::abs(xSpeed), std::abs(zRotation)), xSpeed);
	double maxInputy =
		std::copysign(std::max(std::abs(ySpeed), std::abs(zRotation)), ySpeed);

	if (zRotation >= 0.0) {
		if (xSpeed >= 0.0) {
			leftMotorOutput = maxInputx;
			rightMotorOutput = xSpeed - zRotation;
		} else {
			leftMotorOutput = xSpeed + zRotation;
			rightMotorOutput = maxInputx;
		}
		if (ySpeed >= 0.0) {
			frontMotorOutput = maxInputy;
			rearMotorOutput = ySpeed - zRotation;
		} else {
			frontMotorOutput = ySpeed + zRotation;
			rearMotorOutput = maxInputy;
		}
	} else {
		if (xSpeed >= 0.0) {
			leftMotorOutput = xSpeed + zRotation;
			rightMotorOutput = maxInputx;
		} else {
			leftMotorOutput = maxInputx;
			rightMotorOutput = xSpeed - zRotation;
		}
		if (ySpeed >= 0.0) {
			frontMotorOutput = ySpeed + zRotation;
			rearMotorOutput = maxInputy;
		} else {
			frontMotorOutput = maxInputy;
			rearMotorOutput = ySpeed - zRotation;
		}
	}

	/**
	 * Well, That's a very interesting thing that I have never seen or tried before, but I think it SHOULD work.
	 * Let me show you what's happening.
	 * First, we have a standard Arcade Drive.
	 * But, we want to add another two omni wheels in front and rear of the machine.
	 * Like this:              Their names are:
	 *           ___                 FrontMotor
	 *      |__   |   __|    leftMotor        RightMotor
	 *      |  |||||||  |    (Group)          (Group)
	 *         |||||||
	 *         |||||||
	 *      |__|||||||__|    LeftMotor        RightMotor
	 *      |     |     |    (Group)          (Group)
	 *           ___                 RearMotor
	 * 
	 * And originally the Arcade Drive have a control system like Cartesian coordinate system
	 * 
	 *                         xSpeed
	 *                         |
	 *    Second quadrant      |        First quadrant
	 *                         |----->
	 *                         |    \zRotation(Clockwise) 
	 *                         |     \
	 *     ____________________O______\_______> NaN
	 *                         | 
	 *                         |
	 *    Third quadrant       |        Fourth quadrant
	 *                         |
	 * 
	 * In order to move in the NaN direction above(we call it ySpeed), we tried to rotate the Cartesian clockwise by 90 degrees.
	 * So, like this:
	 * 
	 *                         ySpeed
	 *    Second quadrant      |        First quadrant
	 *    Original First       |        Original Fourth
	 *                         |----->
	 *                         |    \zRotation(Clockwise) 
	 *                         |     \
	 * Original xSpeed<________O______\_______
	 *                         | 
	 *                         |
	 *    Third quadrant       |        Fourth quadrant
	 *    Original Second      |        Original Third
	 *                         |
	 * 
	 * So, we perhaps successfully convert the ySpeed into the xSpeed, which means we only have to copy the xSpeed code to the ySpeed part.
	 * Like this:              Their suppposed names(suppose we want to drive to THIS DIRECTION) are:
	 *           ___                 LeftMotor
	 *      |__   |   __|    RearMotor        FrontMotor
	 *      |  |||||||  |    (Group)          (Group)
	 *         |||||||                                        THIS DIRECTION ---->>>
	 *         |||||||                                        ------------------->>>
	 *      |__|||||||__|    RearMotor        FrontMotor
	 *      |     |     |    (Group)          (Group)
	 *           ___                 RightMotor
	 * 
	 * That's all. 
	 */                        

	m_leftMotor.Set(Limit(leftMotorOutput) * m_maxOutput);
	m_frontMotor.Set(Limit(frontMotorOutput) * m_maxOutput);
	m_rightMotor.Set(Limit(rightMotorOutput) * m_maxOutput *
					 m_rightSideInvertMultiplier);
	m_rearMotor.Set(Limit(rearMotorOutput) * m_maxOutput *
					 m_rearSideInvertMultiplier);

	Feed();
}

void DifferentialDrive_Kai::SetQuickStopThreshold(double threshold) {
	m_quickStopThreshold = threshold;
}

void DifferentialDrive_Kai::SetQuickStopAlpha(double alpha) {
	m_quickStopAlpha = alpha;
}

bool DifferentialDrive_Kai::IsRightSideInverted() const {
	return m_rightSideInvertMultiplier == -1.0;
}

bool DifferentialDrive_Kai::IsRearSideInverted() const {
	return m_rearSideInvertMultiplier == -1.0;
}

void DifferentialDrive_Kai::SetRightSideInverted(bool rightSideInverted) {
	m_rightSideInvertMultiplier = rightSideInverted ? -1.0 : 1.0;
}

void DifferentialDrive_Kai::SetRearSideInverted(bool rearSideInverted) {
	m_rearSideInvertMultiplier = rearSideInverted ? -1.0 : 1.0;
}

void DifferentialDrive_Kai::StopMotor() {
	m_leftMotor.StopMotor();
	m_rightMotor.StopMotor();
	m_frontMotor.StopMotor();
	m_rearMotor.StopMotor();
	Feed();
}

void DifferentialDrive_Kai::GetDescription(wpi::raw_ostream& desc) const {
	desc << "DifferentialDrive_Kai";
}

void DifferentialDrive_Kai::InitSendable(SendableBuilder& builder) {
	builder.SetSmartDashboardType("DifferentialDrive_Kai");
	builder.SetActuator(true);
	builder.SetSafeState([=] { StopMotor(); });
	builder.AddDoubleProperty("Left Motor Speed",
							[=]() { return m_leftMotor.Get(); },
							[=](double value) { m_leftMotor.Set(value); });
	builder.AddDoubleProperty(
		"Right Motor Speed",
		[=]() { return m_rightMotor.Get() * m_rightSideInvertMultiplier; },
		[=](double value) {
		m_rightMotor.Set(value * m_rightSideInvertMultiplier);
		});
	builder.AddDoubleProperty("Front Motor Speed",
							[=]() { return m_frontMotor.Get(); },
							[=](double value) { m_frontMotor.Set(value); });
	builder.AddDoubleProperty(
		"Rear Motor Speed",
		[=]() { return m_rearMotor.Get() * m_rearSideInvertMultiplier; },
		[=](double value) {
		m_rearMotor.Set(value * m_rearSideInvertMultiplier);
		});
}
