/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

// CAN Wiring
constexpr int frontLeftMap = 1;
constexpr int rearLeftMap = 2;
constexpr int frontRightMap = 3;
constexpr int rearRightMap = 4;
constexpr int frontMap = 5;
constexpr int rearMap = 6;

//DIO Wiring
constexpr int Encoder_L1 = 1;
constexpr int Encoder_L2 = 2;
constexpr int Encoder_R1 = 3;
constexpr int Encoder_R2 = 4;

//Joy Wiring
constexpr int XtremeL = 1;
constexpr int XtremeR = 2;
constexpr int Xbox = 0;

//PDP wiring
constexpr int CIM_LF = 1;
constexpr int CIM_LR = 2;
constexpr int CIM_RF = 3;
constexpr int CIM_RR = 4;
constexpr int Pro775_F1 = 5;
constexpr int Pro775_F2 = 6;
constexpr int Pro775_R1 = 7;
constexpr int Pro775_R2 = 8;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

// For example to map the left and right motors, you could define the
// following variables to use with your drivetrain subsystem.
// constexpr int kLeftMotor = 1;
// constexpr int kRightMotor = 2;

// If you are using multiple modules, make sure to define both the port
// number and the module. For example you with a rangefinder:
// constexpr int kRangeFinderPort = 1;
// constexpr int kRangeFinderModule = 1;
