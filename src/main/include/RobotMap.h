/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

// CAN Wiring
constexpr int Map_CAN_Motor_FL = 1;
constexpr int Map_CAN_Motor_RL = 2;
constexpr int Map_CAN_Motor_FR = 3;
constexpr int Map_CAN_Motor_RR = 4;
constexpr int Map_CAN_Motor_F = 5;
constexpr int Map_CAN_Motor_R = 6;

//DIO Wiring
constexpr int Map_DIO_Encoder_L1 = 1;
constexpr int Map_DIO_Encoder_L2 = 2;
constexpr int Map_DIO_Encoder_R1 = 3;
constexpr int Map_DIO_Encoder_R2 = 4;

//Joy Wiring
constexpr int Map_Joy_XtremeL = 1;
constexpr int Map_Joy_XtremeR = 2;
constexpr int Map_Joy_Xbox = 0;

//PDP wiring
constexpr int Map_PDP_CIM_LF = 1;
constexpr int Map_PDP_CIM_LR = 2;
constexpr int Map_PDP_CIM_RF = 3;
constexpr int Map_PDP_CIM_RR = 4;
constexpr int Map_PDP_775_F1 = 5;
constexpr int Map_PDP_775_F2 = 6;
constexpr int Map_PDP_775_R1 = 7;
constexpr int Map_PDP_775_R2 = 8;

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
