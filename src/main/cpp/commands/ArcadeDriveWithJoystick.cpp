/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ArcadeDriveWithJoystick.h"

#include "Robot.h"

ArcadeDriveWithJoystick::ArcadeDriveWithJoystick() {
		: frc::Command("ArcadeDriveWithJoystick") {
	Requires(&Robot::OmniDrive_1plus6);
}


// Called repeatedly when this Command is scheduled to run
void ArcadeDriveWithJoystick::Execute() {
	auto& joystick = Robot::oi.GetJoystick();
	Robot::OmniDrive_1plus6.Drive(-joystick.GetY(), -joystick.GetRawAxis(4));
}

// Make this return true when this Command no longer needs to run execute()
bool ArcadeDriveWithJoystick::IsFinished() { return false; }

// Called once after isFinished returns true
void ArcadeDriveWithJoystick::End() { Robot::OmniDrive_1plus6.Drive(0, 0, 0); }
