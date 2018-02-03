package org.usfirst.frc.team3103.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Winch extends Subsystem {
	WPI_TalonSRX wMotor1 = new WPI_TalonSRX(RobotMap.wMotor1); 
	WPI_TalonSRX wMotor2 = new WPI_TalonSRX(RobotMap.wMotor2);
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    public void climb() {
    	wMotor2.set1
    	
    }
    	
    public void WinchInit() {
    	wMotor1.setInverted(false);
    	wMotor2.setInverted(false);
    	
    	wMotor1.follow(wMotor2);   	
    }
}
    
    