/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ArcadeDriveWithDualJoystick.h"
#include "subsystems/OmniDrive_1plus6.h"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>

OmniDrive_1plus6::OmniDrive_1plus6() : Subsystem("OmniDrive_1plus6") {
	AddChild("Left Encoder", m_leftEncoder);
	AddChild("Right Encoder", m_rightEncoder);
}

void OmniDrive_1plus6::InitDefaultCommand() {
	SetDefaultCommand(new ArcadeDriveWithDualJoystick());
}

void OmniDrive_1plus6::Log() {
	frc::SmartDashboard::PutNumber("Left Distance", m_leftEncoder.GetDistance());
	frc::SmartDashboard::PutNumber("Right Distance", m_rightEncoder.GetDistance());
	frc::SmartDashboard::PutNumber("Left Speed", m_leftEncoder.GetRate());
	frc::SmartDashboard::PutNumber("Right Speed", m_rightEncoder.GetRate());
}

void OmniDrive_1plus6::Drive(double x, double y, double z) {
	m_robotDrive->ArcadeDrive_Kai(x, y, z);
}

void OmniDrive_1plus6::TalonInit() {
	m_frontLeft->ConfigFactoryDefault();
	m_rearLeft->ConfigFactoryDefault();
	m_frontRight->ConfigFactoryDefault();
	m_rearRight->ConfigFactoryDefault();
	m_front->ConfigFactoryDefault();
	m_rear->ConfigFactoryDefault();

	m_rearLeft->Follow(*m_frontLeft);
	m_rearRight->Follow(*m_frontRight);

	m_frontLeft->SetInverted(false);
	m_rearLeft->SetInverted(false);
	m_frontRight->SetInverted(false);
	m_rearRight->SetInverted(false);
	m_front->SetInverted(false);
	m_rear->SetInverted(false);

	m_frontLeft->SetSensorPhase(true);
	m_frontRight->SetSensorPhase(true);
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
