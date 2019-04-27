/*---------------------------Modified Private Lib-----------------------------*/

#pragma once

#include <ctre/Phoenix.h>
#include <frc/drive/RobotDriveBase.h>
#include <wpi/raw_ostream.h>

namespace frc {
	class SpeedController;

	class DifferentialDrive_Kaini : public RobotDriveBase {
		public:
			static constexpr double kDefaultQuickStopThreshold = 0.2;
			static constexpr double kDefaultQuickStopAlpha = 0.1;

			DifferentialDrive_Kaini(SpeedController& leftMotor, SpeedController& rightMotor,
									SpeedController& frontMotor, SpeedController& rearMotor);

			~DifferentialDrive_Kaini() override = default;

			DifferentialDrive_Kaini(DifferentialDrive_Kaini&&) = default;
			DifferentialDrive_Kaini& operator=(DifferentialDrive_Kaini&&) = default;

			void ArcadeDrive_Kaini(double xSpeed, double ySpeed, double zRotation, bool isQuickTurn, bool isHighGear);

			void SetQuickStopThreshold(double threshold);

			void SetQuickStopAlpha(double alpha);

			bool IsRightSideInverted() const;
			bool IsRearSideInverted() const;

			void SetRightSideInverted(bool rightSideInverted);
			void SetRearSideInverted(bool rearSideInverted);

			void StopMotor() override;
			void GetDescription(wpi::raw_ostream& desc) const override;

			void InitSendable(SendableBuilder& builder) override;

	 	private:
			SpeedController& m_leftMotor;
			SpeedController& m_rightMotor;
			SpeedController& m_frontMotor;
			SpeedController& m_rearMotor;

			double Pi = 3.14159265358979;

			double m_quickStopThreshold = kDefaultQuickStopThreshold;
			double m_quickStopAlpha = kDefaultQuickStopAlpha;

			double m_quickStopAccumulator = 0.0;
			double m_rightSideInvertMultiplier = -1.0;
			double m_rearSideInvertMultiplier = -1.0;

			double kThrottleDeadband = 0.02;
    		double kWheelDeadband = 0.02;

			double m_HighWheelNonLinearity = 0.65;
			double m_LowWheelNonLinearity = 0.5;

			double m_HighNegInertiaScalar = 4.0;

			double m_LowNegInertiaThreshold = 0.65;
   			double m_LowNegInertiaTurnScalar = 3.5;
			double m_LowNegInertiaCloseScalar = 4.0;
			double m_LowNegInertiaFarScalar = 5.0;

			double m_HighSensitivity = 0.65;
			double m_LowSensitiity = 0.65;

			double m_QuickStopDeadband = 0.5;
			double m_QuickStopWeight = 0.1;
			double m_QuickStopScalar = 5.0;

			double m_OldWheel = 0.0;
			double m_QuickStopAccumlator = 0.0;
			double m_NegInertiaAccumlator = 0.0;
	};
}