/*---------------------------Modified Private Lib-----------------------------*/

#pragma once

#include <ctre/Phoenix.h>
#include <frc/drive/RobotDriveBase.h>
#include <wpi/raw_ostream.h>

namespace frc {
	class SpeedController;

	class CheesyDriveHelper : public RobotDriveBase {
		public:
			CheesyDriveHelper(SpeedController& leftMotor, SpeedController& rightMotor);

			~CheesyDriveHelper() override = default;

			CheesyDriveHelper(CheesyDriveHelper&&) = default;
			CheesyDriveHelper& operator=(CheesyDriveHelper&&) = default;

			void CheesyDrive(double throttle, double wheel, bool isQuickTurn, bool isHighGear);

			double HandleDeadband(double val, double deadband);

			double UtilLimit(double v, double maxMagnitude);

			bool IsRightSideInverted() const;

			void SetRightSideInverted(bool rightSideInverted);

			void StopMotor() override;
			void GetDescription(wpi::raw_ostream& desc) const override;

			void InitSendable(SendableBuilder& builder) override;

		private:
			SpeedController& m_leftMotor;
			SpeedController& m_rightMotor;

			double m_rightSideInvertMultiplier = -1.0;

			constexpr double Pi = 3.14159265358979;

			constexpr double kThrottleDeadband = 0.02;
    		constexpr double kWheelDeadband = 0.02;

    		// These factor determine how fast the wheel traverses the "non linear" sine curve.
    		constexpr double kHighWheelNonLinearity = 0.65;
    		constexpr double kLowWheelNonLinearity = 0.5;

    		constexpr double kHighNegInertiaScalar = 4.0;

    		constexpr double kLowNegInertiaThreshold = 0.65;
    		constexpr double kLowNegInertiaTurnScalar = 3.5;
    		constexpr double kLowNegInertiaCloseScalar = 4.0;
    		constexpr double kLowNegInertiaFarScalar = 5.0;

    		constexpr double kHighSensitivity = 0.65;
    		constexpr double kLowSensitiity = 0.65;

 	   		constexpr double kQuickStopDeadband = 0.5;
    		constexpr double kQuickStopWeight = 0.1;
    		constexpr double kQuickStopScalar = 5.0;

    		double mOldWheel = 0.0;
    		double mQuickStopAccumlator = 0.0;
    		double mNegInertiaAccumlator = 0.0;
	};
}
