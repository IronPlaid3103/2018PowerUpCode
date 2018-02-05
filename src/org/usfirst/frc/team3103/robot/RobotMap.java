/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3103.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;
	
	//Can Devices
	public static int flMotor = 1;
	public static int frMotor = 2;
	public static int brMotor = 3;
	public static int blMotor = 4;
	public static int wMotor1 = 12;
	public static int wMotor2 = 13; 
	public static int elevatorMotor1 = 30;
	public static int elevatorMotor2 = 31;
	public static int boxCatcherRoller1 = 20;
	public static int actuatingBoxCatcherMototor = 21;
	public static int boxCatcherRoller2 = 22;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
}
