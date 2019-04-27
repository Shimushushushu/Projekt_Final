/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"

#include <frc/WPILib.h>

OI::OI() {
	// Process operator interface input here.
}

frc::Joystick& OI::GetL() { return m_Xtreme_L; }
frc::Joystick& OI::GetR() { return m_Xtreme_R; }
frc::Joystick& OI::GetX() { return m_Map_Joy_Xbox; }