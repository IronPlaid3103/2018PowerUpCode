package org.usfirst.frc.team3103.robot.commands;

import org.usfirst.frc.team3103.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class MoveForwardAuto_command extends CommandGroup {

    public MoveForwardAuto_command(int position) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	requires(Robot.mainDrive);
    	
    	switch (position) {
    	case 1:
    		addSequential(new MoveForward_command(120));
    		addSequential(new ElevatorUpScale_command());
    	case 2:
    		addSequential(new turnAngle_command(30.96));
    		addSequential(new MoveForward_command(130));
    		addSequential(new ElevatorUpScale_command());
    	case 3:
    		addSequential(new MoveForward_command(120));
    		addSequential(new ElevatorUpScale_command());
    	}
    		
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
