/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "RobotMap.h"
#include "privatelib/DifferentialDrive_Kai.h"

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/WPILib.h>
#include <frc/Encoder.h>
#include <frc/commands/Subsystem.h>


class OmniDrive_1plus6 : public frc::Subsystem {
	private:
		WPI_TalonSRX * m_frontLeft = new WPI_TalonSRX(frontLeftMap);
		WPI_TalonSRX * m_rearLeft = new WPI_TalonSRX(rearLeftMap);

		WPI_TalonSRX * m_frontRight = new WPI_TalonSRX(frontRightMap);
		WPI_TalonSRX * m_rearRight = new WPI_TalonSRX(rearRightMap);
		
		WPI_TalonSRX * m_front = new WPI_TalonSRX(frontMap);
		WPI_TalonSRX * m_rear = new WPI_TalonSRX(rearMap);

		frc::DifferentialDrive_Kai * m_robotDrive = new frc::DifferentialDrive_Kai(*m_frontLeft, *m_frontRight, *m_front, *m_rear);

		frc::Encoder m_leftEncoder{Encoder_L1, Encoder_L2};
		frc::Encoder m_rightEncoder{Encoder_R1, Encoder_R2};

		// It's desirable that everything possible under private except
		// for methods that implement subsystem capabilities

	public:
		OmniDrive_1plus6();
		void InitDefaultCommand() override;
		void Log();
		void Drive(double x, double y, double z);
		void TalonInit();
};
