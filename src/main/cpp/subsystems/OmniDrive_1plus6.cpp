/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/OmniDrive_1plus6.h"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "commands/ArcadeDriveWithJoystick.h"

OmniDrive_1plus6::OmniDrive_1plus6() : Subsystem("OmniDrive_1plus6") {
	AddChild("Left Encoder", m_leftEncoder);
	AddChild("Right Encoder", m_rightEncoder);
}

void OmniDrive_1plus6::InitDefaultCommand() {
  SetDefaultCommand(new ArcadeDriveWithJoystick());
}



void OmniDrive_1plus6::Log() {
	frc::SmartDashboard::PutNumber("Left Distance", m_leftEncoder.GetDistance());
	frc::SmartDashboard::PutNumber("Right Distance", m_rightEncoder.GetDistance());
	frc::SmartDashboard::PutNumber("Left Speed", m_leftEncoder.GetRate());
	frc::SmartDashboard::PutNumber("Right Speed", m_rightEncoder.GetRate());
}

void OmniDrive_1plus6::Drive(double x, double y, double z) {
	m_robotDrive.ArcadeDrive_Kai(x, y, z);
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
