/*---------------------------Modified Private Lib-----------------------------*/

#include "privatelib/DifferentialDrive_Kaini.h"
#include "privatelib/RobotDriveBase_Kai.h"

#include <algorithm>
#include <cmath>

#include <frc/smartdashboard/SendableBuilder.h>
#include <hal/HAL.h>

using namespace frc;

DifferentialDrive_Kaini::DifferentialDrive_Kaini(SpeedController& leftMotor, SpeedController& rightMotor,
											 SpeedController& frontMotor, SpeedController& rearMotor)
	: m_leftMotor(leftMotor), m_rightMotor(rightMotor), m_frontMotor(frontMotor), m_rearMotor(rearMotor) {
	AddChild(&m_leftMotor);
	AddChild(&m_rightMotor);
	AddChild(&m_frontMotor);
	AddChild(&m_rearMotor);
	static int instances = 0;
	++instances;
	SetName("DifferentialDrive_Kaini", instances);
}

void DifferentialDrive_Kaini::ArcadeDrive_Kaini(double xSpeed, double ySpeed, double zRotation,
									bool isQuickTurn, bool isHighGear) {
	ApplyDeadband_Kai(xSpeed, m_deadband);
	ApplyDeadband_Kai(ySpeed, m_deadband);
	ApplyDeadband_Kai(zRotation, m_deadband);

	double negInertia = zRotation - m_OldWheel;
	m_OldWheel = zRotation;

	double wheelNonLinearity;
	if (isHighGear) {
		wheelNonLinearity = m_HighWheelNonLinearity;
		double denominator = sin(Pi / 2.0 * wheelNonLinearity);
		// Apply a sin function that's scaled to make it feel better.
		zRotation = sin(Pi / 2.0 * wheelNonLinearity * zRotation) / denominator;
		zRotation = sin(Pi / 2.0 * wheelNonLinearity * zRotation) / denominator;
	} else {
		wheelNonLinearity = m_LowWheelNonLinearity;
		double denominator = sin(Pi / 2.0 * wheelNonLinearity);
		// Apply a sin function that's scaled to make it feel better.
		zRotation = sin(Pi / 2.0 * wheelNonLinearity * zRotation) / denominator;
		zRotation = sin(Pi / 2.0 * wheelNonLinearity * zRotation) / denominator;
		zRotation = sin(Pi / 2.0 * wheelNonLinearity * zRotation) / denominator;
	}

	double leftOutput, rightOutput, frontOutput, rearOutput, overPower;
	double sensitivity;

	double angularPower;
	double linearPower;

	double negInertiaScalar;
		if (isHighGear) {
			negInertiaScalar = m_HighNegInertiaScalar;
			sensitivity = m_HighSensitivity;
		} else {
			if (zRotation * negInertia > 0) {
				// If we are moving away from 0.0, aka, trying to get more wheel.
				negInertiaScalar = m_LowNegInertiaTurnScalar;
			} else {
				// Otherwise, we are attempting to go back to 0.0.
				if (abs(zRotation) > m_LowNegInertiaThreshold) {
					negInertiaScalar = m_LowNegInertiaFarScalar;
				} else {
					negInertiaScalar = m_LowNegInertiaCloseScalar;
				}
			}
			sensitivity = m_LowSensitiity;
		}
	
	double negInertiaPower = negInertia * negInertiaScalar;
	m_NegInertiaAccumlator += negInertiaPower;

	zRotation = zRotation + m_NegInertiaAccumlator;
	if (m_NegInertiaAccumlator > 1) {
	m_NegInertiaAccumlator -= 1;
	} else if (m_NegInertiaAccumlator < -1) {
		m_NegInertiaAccumlator += 1;
	} else {
		m_NegInertiaAccumlator = 0;
	}
	linearPower = xSpeed; //TODO

	if(isQuickTurn) {
		if(abs(linearPower) < m_QuickStopDeadband) {
			double alpha = m_QuickStopWeight;
			m_QuickStopAccumlator = (1 - alpha) * m_QuickStopAccumlator
					+ alpha * Util.limit(zRotation, 1.0) * m_QuickStopScalar;
		}
		overPower = 1.0;
		angularPower = zRotation;
	} else {
		overPower = 0.0;
		angularPower = abs(xSpeed) * zRotation * sensitivity - m_QuickStopAccumlator;
		if (m_QuickStopAccumlator > 1) {
			m_QuickStopAccumlator -= 1;
		} else if (m_QuickStopAccumlator < -1) {
			m_QuickStopAccumlator += 1;
		} else {
			m_QuickStopAccumlator = 0.0;
		}
	}

//	Limit(xSpeed);

	/*
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
	xSpeed = std::copysign(xSpeed * xSpeed, xSpeed);
	xSpeed = std::copysign(ySpeed * ySpeed, ySpeed);
	zRotation = std::copysign(zRotation * zRotation, zRotation);

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

	m_leftMotor.Set(Limit(leftMotorOutput) * m_maxOutput);
	m_frontMotor.Set(Limit(frontMotorOutput) * m_maxOutput);
	m_rightMotor.Set(Limit(rightMotorOutput) * m_maxOutput *
					 m_rightSideInvertMultiplier);
	m_rearMotor.Set(Limit(rearMotorOutput) * m_maxOutput *
					 m_rearSideInvertMultiplier);

	Feed();
	*/
}

void DifferentialDrive_Kaini::SetQuickStopThreshold(double threshold) {
	m_quickStopThreshold = threshold;
}

void DifferentialDrive_Kaini::SetQuickStopAlpha(double alpha) {
	m_quickStopAlpha = alpha;
}

bool DifferentialDrive_Kaini::IsRightSideInverted() const {
	return m_rightSideInvertMultiplier == -1.0;
}

bool DifferentialDrive_Kaini::IsRearSideInverted() const {
	return m_rearSideInvertMultiplier == -1.0;
}

void DifferentialDrive_Kaini::SetRightSideInverted(bool rightSideInverted) {
	m_rightSideInvertMultiplier = rightSideInverted ? -1.0 : 1.0;
}

void DifferentialDrive_Kaini::SetRearSideInverted(bool rearSideInverted) {
	m_rearSideInvertMultiplier = rearSideInverted ? -1.0 : 1.0;
}

void DifferentialDrive_Kaini::StopMotor() {
	m_leftMotor.StopMotor();
	m_rightMotor.StopMotor();
	m_frontMotor.StopMotor();
	m_rearMotor.StopMotor();
	Feed();
}

void DifferentialDrive_Kaini::GetDescription(wpi::raw_ostream& desc) const {
	desc << "DifferentialDrive_Kai";
}

void DifferentialDrive_Kaini::InitSendable(SendableBuilder& builder) {
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
