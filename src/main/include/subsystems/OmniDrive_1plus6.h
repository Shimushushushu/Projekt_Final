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
#include <frc/SpeedControllerGroup.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/Phoenix.h>
#include <frc/WPILib.h>
#include "privatelib/DifferentialDrive_Kai.h"

class OmniDrive_1plus6 : public frc::Subsystem {
	private:
		WPI_TalonSRX * m_frontLeft = new WPI_TalonSRX(1);
		WPI_TalonSRX * m_rearLeft = new WPI_TalonSRX(2);

		WPI_TalonSRX * m_frontRight = new WPI_TalonSRX(3);
		WPI_TalonSRX * m_rearRight = new WPI_TalonSRX(4);
		
		WPI_TalonSRX * m_front = new WPI_TalonSRX(5);
		WPI_TalonSRX * m_rear = new WPI_TalonSRX(6);

		frc::DifferentialDrive_Kai * m_robotDrive = new frc::DifferentialDrive_Kai(*m_frontLeft, *m_frontRight, *m_front, *m_rear);

		frc::Encoder m_leftEncoder{1, 2};
		frc::Encoder m_rightEncoder{3, 4};

		// It's desirable that everything possible under private except
		// for methods that implement subsystem capabilities

	public:
		OmniDrive_1plus6();
		void InitDefaultCommand() override;
		void Log();
		void Drive(double x, double y, double z);
		void TalonInit();
};
