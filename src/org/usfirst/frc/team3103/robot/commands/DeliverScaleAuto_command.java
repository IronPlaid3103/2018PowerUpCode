package org.usfirst.frc.team3103.robot.commands;

import org.usfirst.frc.team3103.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DeliverScaleAuto_command extends CommandGroup {

    public DeliverScaleAuto_command(int startPosition, String gameData) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	requires(Robot.mainDrive);
    	requires(Robot.elevator);
    	requires(Robot.gripper);
    	
    	
    	if (gameData.length() > 0) {
    		if (gameData.charAt(0) == 'L') {
    			switch(startPosition) {
    			
    			//deliver box to the left side of Scale from the left position
    			case 1:
    				addSequential(new MoveForward_command(120));
    				addSequential(new ElevatorUpScale_command());
    				addSequential(new deliverBox_command()); 
    				break;
    				
        		//deliver box to the left side of Scale from the middle position
    			case 2:
    				addSequential(new MoveForward_command(130));
    				addSequential(new ElevatorUpScale_command());
    				addSequential(new deliverBox_command());
    				break;
    				
        		//deliver box to the left side of Scale from the right position
    			case 3:
    				addSequential(new MoveForward_command(140));
    				addSequential(new ElevatorUpScale_command());
    				addSequential(new deliverBox_command());
    				break;
    			}
    		}
    		else if (gameData.charAt(0) == 'R'){
    			switch(startPosition) {
    			
    			//deliver box to the right side of Scale from the left position
    			case 1:
    				addSequential(new MoveForward_command(140));
    				addSequential(new ElevatorUpScale_command());
    				addSequential(new deliverBox_command()); 
    				break;
    				
        		//deliver box to the right side of Scale from the middle position
    			case 2:
    				addSequential(new MoveForward_command(130));
    				addSequential(new ElevatorUpScale_command());
    				addSequential(new deliverBox_command());
    				break;
    				
        		//deliver box to the right side of Scale from the right position
    			case 3:
    				addSequential(new MoveForward_command(120));
    				addSequential(new ElevatorUpScale_command());
    				addSequential(new deliverBox_command());
    				break;
    			}
    		}
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
