/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "RobotMap.h"

#include <iostream>

#include <frc/commands/Command.h>
#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

OmniDrive_1plus6 Robot::omnidrive_1plus6;
OI Robot::oi;

void Robot::RobotInit() {
	Robot::omnidrive_1plus6.TalonInit();

	frc::SmartDashboard::PutData(&omnidrive_1plus6);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
	frc::SmartDashboard::PutNumber("DriveTrain Currrent",
									(m_pdp.GetCurrent(CIM_LF) + m_pdp.GetCurrent(CIM_LR) + m_pdp.GetCurrent(CIM_RF) + m_pdp.GetCurrent(CIM_RR) +
									m_pdp.GetCurrent(Pro775_F1) + m_pdp.GetCurrent(Pro775_F2) + m_pdp.GetCurrent(Pro775_R1) + m_pdp.GetCurrent(Pro775_R2))/8); //Only for EXAMPLE
	frc::SmartDashboard::PutNumber("Voltage", m_pdp.GetVoltage());
    frc::SmartDashboard::PutNumber("Temperature", m_pdp.GetTemperature());
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() { frc::Scheduler::GetInstance()->Run(); }

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString code to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional commands to the
 * chooser code above (like the commented example) or additional comparisons to
 * the if-else structure below with additional strings & commands.
 */
void Robot::AutonomousInit() {
	m_autonomousCommand.Start();
	std::cout << "Starting Auto" << std::endl;
}

void Robot::AutonomousPeriodic() { frc::Scheduler::GetInstance()->Run(); }

void Robot::TeleopInit() {
	// This makes sure that the autonomous stops running when
	// teleop starts running. If you want the autonomous to
	// continue until interrupted by another command, remove
	// this line or comment it out.
	m_autonomousCommand.Cancel();
	std::cout << "Starting Teleop" << std::endl;
}

void Robot::TeleopPeriodic() { frc::Scheduler::GetInstance()->Run(); }

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
