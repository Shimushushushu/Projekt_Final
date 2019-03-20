/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Encoder.h>
#include <frc/Joystick.h>
#include <frc/commands/Subsystem.h>
#include <WPI_TalonSRX.h>
#include "priavatelib/DifferentialDrive_Kai.h"

class OmniDrive_1plus6 : public frc::Subsystem {
	private:
		frc::TalonSRX m_frontLeft{1};
		frc::TalonSRX m_rearLeft{2};
		frc::SpeedControllerGroup m_left{m_frontLeft, m_rearLeft};

		frc::TalonSRX m_frontRight{3};
		frc::TalonSRX m_rearRight{4};
		frc::SpeedControllerGroup m_right{m_frontRight, m_rearRight};

		frc::TalonSRX m_front{5};
		frc::TalonSRX m_rear{6};

		frc::DifferentialDrive_Kai m_robotDrive{m_left, m_right, m_front, m_rear};

		frc::Encoder m_leftEncoder{1, 2};
		frc::Encoder m_rightEncoder{3, 4};


		// It's desirable that everything possible under private except
		// for methods that implement subsystem capabilities

	public:
		OmniDrive_1plus6();
		void InitDefaultCommand() override;
		void Log();
		void Drive(double x, double y, double z);
};
