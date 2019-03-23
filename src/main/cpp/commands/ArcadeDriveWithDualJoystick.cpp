/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "commands/ArcadeDriveWithDualJoystick.h"
#include "privatelib/DualXtremeControl.h"

ArcadeDriveWithDualJoystick::ArcadeDriveWithDualJoystick()
		: frc::Command("ArcadeDriveWithDualJoystick") {
	Requires(&Robot::omnidrive_1plus6);
}

// Called repeatedly when this Command is scheduled to run
void ArcadeDriveWithDualJoystick::Execute() {
	auto& XL = Robot::oi.GetL();
	auto& XR = Robot::oi.GetR();
	Robot::omnidrive_1plus6.Drive(-xCombine(XL.GetX(), XR.GetX()), yCombine(XL.GetY(), XR.GetY()), zCombine(XL.GetX(), XR.GetX()));
}

// Make this return true when this Command no longer needs to run execute()
bool ArcadeDriveWithDualJoystick::IsFinished() { return false; }

// Called once after isFinished returns true
void ArcadeDriveWithDualJoystick::End() { Robot::omnidrive_1plus6.Drive(0, 0, 0); }
