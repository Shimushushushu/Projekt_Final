/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/TimedRobot.h>
#include <frc/commands/Command.h>
#include <frc/commands/Scheduler.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "OI.h"
#include "subsystems/OmniDrive_1plus6.h"
#include "subsystems/ExampleSubsystem.h"

class Robot : public frc::TimedRobot {
	public:
		static OmniDrive_1plus6 OmniDrive_1plus6;
		static ExampleSubsystem m_subsystem;

 	private:
		frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();

		void RobotInit() override;
		void RobotPeriodic() override;
		void DisabledInit() override;
		void DisabledPeriodic() override;
		void AutonomousInit() override;
		void AutonomousPeriodic() override;
		void TeleopInit() override;
		void TeleopPeriodic() override;
		void TestPeriodic() override;

		frc::Command* m_autonomousCommand = nullptr;
		frc::SendableChooser<frc::Command*> m_chooser;
};
