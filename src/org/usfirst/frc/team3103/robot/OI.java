/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3103.robot;

import org.usfirst.frc.team3103.robot.commands.AutoAim;
import org.usfirst.frc.team3103.robot.commands.deliverBox_command;
import org.usfirst.frc.team3103.robot.commands.elevatorDown_command;
import org.usfirst.frc.team3103.robot.commands.elevatorUp_command;
import org.usfirst.frc.team3103.robot.commands.getBox_command;
import org.usfirst.frc.team3103.robot.commands.turnAngle_command;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	Joystick driveControl = new Joystick(0);
	//XboxController driveControl = new XboxController(0);
	//boolean buttonPressed = driveControl.getRawButtonPressed(5);
	
	public OI()
	{
		//Joystick driveControl = new Joystick(0);

		//driveControl.getBumper(Hand.kRight).whenPressed(new AutoAim());
		//boolean rightBumper = driveControl.getBumper(Hand.kLeft); 
		
		//Button limelight = new JoystickButton(driveControl, 1);
		Button turnAngle = new JoystickButton(driveControl, 2);
		//Button intake = new JoystickButton(driveControl, 3); // button x
		//Button outtake = new JoystickButton(driveControl, 4); // button y 
		//Button eleUp = new JoystickButton(driveControl, 5); // left bumper
		//Button eleDown = new JoystickButton(driveControl, 6); // right bumper 
		
		//driveControl.getRawButton(5);

    	/* if (buttonPressed) {
    		
    		System.out.println("test - oi");
    		
    		new AutoAim();
    		
    	} */
		
		//new JoystickButton(driveControl, 6).whenPressed(new AutoAim());
		
		turnAngle.whenPressed(new turnAngle_command(45));
		
		//eleUp.whileHeld(new elevatorUp_command());
		//eleDown.whileHeld(new elevatorDown_command());
		
		//driveControl new turnAngle_command();
		
		//limelight.whenPressed(new AutoAim());
		/*
		if (driveControl.getXButtonPressed()) {
			new getBox_command();
		}
		
		if (driveControl.getYButtonPressed()) {
			new deliverBox_command();
		}
		
		//if (driveControl.getTriggerAxis(Hand.kRight) <= 1) {
		if (driveControl.getAButtonPressed()) {
			new elevatorUp_command();
		}
		
		if (driveControl.getTriggerAxis(Hand.kLeft) <= 1) {
			new elevatorDown_command();
		}
		
		if (driveControl.getBButtonPressed()) {
			new turnAngle_command(45);
		}
		*/
	}
	
	public Joystick getJoystickController() {
		return driveControl;
	}
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
}
