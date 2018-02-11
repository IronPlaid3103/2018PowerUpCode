package org.usfirst.frc.team3103.robot.commands;

import org.usfirst.frc.team3103.robot.Robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class AutoAim extends Command {
	
	XboxController driveControl = new XboxController(0);
	
    public AutoAim() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	requires(Robot.camera);
    	requires(Robot.mainDrive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.camera.table.getEntry("ledMode").setValue(-2);
    	
    	SmartDashboard.putNumber("initialize", 10);

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putNumber("execute", 10);

        Robot.camera.table.getEntry("ledMode").setValue(0);
        Robot.mainDrive.random(Robot.camera.aimLeft(), 0.2);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	
    	Button button6 = new JoystickButton(driveControl,6);
    	
    	SmartDashboard.putNumber("hello finish", 10);
    	
    	if(button6.get() == false) {
            Robot.camera.table.getEntry("ledMode").setValue(1);
    		return true;
    	}
    	
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.camera.table.getEntry("ledMode").setValue(1);

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
