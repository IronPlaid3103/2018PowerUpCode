package org.usfirst.frc.team3103.robot.commands;

import org.usfirst.frc.team3103.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class turnAngle_command extends Command {

    public turnAngle_command() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	requires(Robot.mainDrive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.mainDrive.turnToTargetAngle(45.0, Robot.m_oi.getJoystickController()); 
    	System.out.println("Yaw = " + Robot.mainDrive.getYaw());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(Robot.mainDrive.getYaw() == Math.abs(45.0)) {
    		return true;
    	}
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.mainDrive.getDefaultCommand();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
