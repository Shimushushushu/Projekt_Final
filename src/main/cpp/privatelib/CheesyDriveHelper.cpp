/*---------------------------Modified Private Lib-----------------------------*/

#include "privatelib/CheesyDriveHelper.h"

#include <algorithm>
#include <cmath>

#include <frc/smartdashboard/SendableBuilder.h>
#include <hal/HAL.h>

using namespace frc;

CheesyDriveHelper::CheesyDriveHelper(SpeedController& leftMotor, SpeedController& rightMotor)
	: m_leftMotor(leftMotor), m_rightMotor(rightMotor) {
	AddChild(&m_leftMotor);
	AddChild(&m_rightMotor);
	static int instances = 0;
	++instances;
	SetName("CheesyDriveHelper", instances);
}

void CheesyDrive(double throttle, double wheel, bool isQuickTurn, bool isHighGear) {
	wheel = HandleDeadband(wheel, kWheelDeadband);
	throttle = HandleDeadband(throttle, kThrottleDeadband);

	double negInertia = wheel - mOldWheel;
	mOldWheel = wheel;

	double wheelNonLinearity;
	if(isHighGear) {
		wheelNonLinearity = kHighWheelNonLinearity;
		const double denominator = sin(Pi / 2.0 * wheelNonLinearity);
		// Apply a sin function that's scaled to make it feel better.
		wheel = sin(Pi / 2.0 * wheelNonLinearity * wheel) / denominator;
		wheel = sin(Pi / 2.0 * wheelNonLinearity * wheel) / denominator;
	} else {
		wheelNonLinearity = kLowWheelNonLinearity;
		const double denominator = sin(Pi / 2.0 * wheelNonLinearity);
		// Apply a sin function that's scaled to make it feel better.
		wheel = sin(Pi / 2.0 * wheelNonLinearity * wheel) / denominator;
		wheel = sin(Pi / 2.0 * wheelNonLinearity * wheel) / denominator;
		wheel = sin(Pi / 2.0 * wheelNonLinearity * wheel) / denominator;
	}

	double leftPwm, rightPwm, overPower;
	double sensitivity;
	
	double angularPower;
	double linearPower;

	// Negative inertia!
	double negInertiaScalar;
	if(isHighGear) {
		negInertiaScalar = kHighNegInertiaScalar;
		sensitivity = kHighSensitivity;
	} else {
		if(wheel * negInertia > 0) {
			// If we are moving away from 0.0, aka, trying to get more wheel.
			negInertiaScalar = kLowNegInertiaTurnScalar;
		} else {
			// Otherwise, we are attempting to go back to 0.0.
			if(abs(wheel) > kLowNegInertiaThreshold) {
				negInertiaScalar = kLowNegInertiaFarScalar;
			} else {
				negInertiaScalar = kLowNegInertiaCloseScalar;
			}
		}
		sensitivity = kLowSensitiity;
	}

	double negInertiaPower = negInertia * negInertiaScalar;
	mNegInertiaAccumlator += negInertiaPower;

	wheel = wheel + mNegInertiaAccumlator;
	if(mNegInertiaAccumlator > 1) {
		mNegInertiaAccumlator -= 1;
	} else if(mNegInertiaAccumlator < -1) {
		mNegInertiaAccumlator += 1;
	} else {
		mNegInertiaAccumlator = 0;
	}
	linearPower = throttle;

	// Quickturn!
	if(isQuickTurn) {
		if(abs(linearPower) < kQuickStopDeadband) {
			double alpha = kQuickStopWeight;
			mQuickStopAccumlator = (1 - alpha) * mQuickStopAccumlator
					+ alpha * UtilLimit(wheel, 1.0) * kQuickStopScalar;
		}
		overPower = 1.0;
		angularPower = wheel;
	} else {
		overPower = 0.0;
		angularPower = abs(throttle) * wheel * sensitivity - mQuickStopAccumlator;
		if(mQuickStopAccumlator > 1) {
			mQuickStopAccumlator -= 1;
		} else if(mQuickStopAccumlator < -1) {
			mQuickStopAccumlator += 1;
		} else {
			mQuickStopAccumlator = 0.0;
		}
	}

	rightPwm = leftPwm = linearPower;
	leftPwm += angularPower;
	rightPwm -= angularPower;

	if(leftPwm > 1.0) {
		rightPwm -= overPower * (leftPwm - 1.0);
		leftPwm = 1.0;
	} else if(rightPwm > 1.0) {
		leftPwm -= overPower * (rightPwm - 1.0);
		rightPwm = 1.0;
	} else if(leftPwm < -1.0) {
		rightPwm += overPower * (-1.0 - leftPwm);
		leftPwm = -1.0;
	} else if(rightPwm < -1.0) {
		leftPwm += overPower * (-1.0 - rightPwm);
		rightPwm = -1.0;
	}

	m_leftMotor.Set(UtilLimit(leftPwm, 1.0);
	m_rightMotor.Set(UtilLimit(rightPwm * m_rightSideInvertMultiplier), 1.0);

	Feed();
}

double HandleDeadband(double val, double deadband) {
	return (abs(val) > abs(deadband)) ? val : 0.0;
}

double UtilLimit(double v, double min, double max) {
	return min(max, max(min, v));
}

bool CheesyDriveHelper::IsRightSideInverted() const {
	return m_rightSideInvertMultiplier == -1.0;
}

void CheesyDriveHelper::SetRightSideInverted(bool rightSideInverted) {
	m_rightSideInvertMultiplier = rightSideInverted ? -1.0 : 1.0;
}


void CheesyDriveHelper::StopMotor() {
	m_leftMotor.StopMotor();
	m_rightMotor.StopMotor();
	Feed();
}

void CheesyDriveHelper::GetDescription(wpi::raw_ostream& desc) const {
	desc << "CheesyDriveHelper";
}

void CheesyDriveHelper::InitSendable(SendableBuilder& builder) {
	builder.SetSmartDashboardType("CheesyDriveHelper");
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
}
