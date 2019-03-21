/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Joystick.h>
#include <frc/buttons/JoystickButton.h>

class OI {
	public:
		OI();
		frc::Joystick& GetL();
		frc::Joystick& GetR();
		frc::Joystick& GetX();
	
	private:
		frc::Joystick m_Xtreme_L{0};
		frc::Joystick m_Xtreme_R{1};
		frc::Joystick m_Xbox{2};

  		frc::JoystickButton m_example{&m_Xbox, 1};
};
