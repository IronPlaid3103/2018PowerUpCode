package org.usfirst.frc.team3103.robot.commands;

import org.usfirst.frc.team3103.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class getBox_command extends Command {

    public getBox_command() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.gripper);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.gripper.close_Catcher();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.gripper.intake_Box();    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	//Robot.gripper.close_Catcher();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
