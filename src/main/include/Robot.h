/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "OI.h"
#include "commands/Autonomous.h"
#include "subsystems/OmniDrive_1plus6.h"

#include <frc/TimedRobot.h>
#include <frc/livewindow/LiveWindow.h>


class Robot : public frc::TimedRobot {
	public:
		static OmniDrive_1plus6 omnidrive_1plus6;
		static OI oi;

 	private:
		Autonomous m_autonomousCommand;
		frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();
		frc::PowerDistributionPanel m_pdp;

		void RobotInit() override;
		void RobotPeriodic() override;
		void DisabledInit() override;
		void DisabledPeriodic() override;
		void AutonomousInit() override;
		void AutonomousPeriodic() override;
		void TeleopInit() override;
		void TeleopPeriodic() override;
		void TestPeriodic() override;
};
