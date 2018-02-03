package org.usfirst.frc.team3103.robot.subsystems;

import org.usfirst.frc.team3103.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Box_Catcher extends Subsystem {
	WPI_TalonSRX mover = new WPI_TalonSRX(RobotMap.actuatingBoxCatcherMototor);
	WPI_TalonSRX catcher1 = new WPI_TalonSRX(RobotMap.boxCatcherRoller1);
	WPI_TalonSRX catcher2 = new WPI_TalonSRX(RobotMap.boxCatcherRoller2);
	
	public void Box_CatcherInit() {
		//inversion
		mover.setInverted(false);
		catcher1.setInverted(false);
		catcher2.setInverted(false);
		//follow
		catcher2.follow(catcher1);
	
}
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
    	// actuating stuff forwards and backwards
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    	
    public void open_Catcher() {
    	mover.set(1);
    	
    }
    public void close_Catcher() {
    	mover.set(-1);
    	
    }
    public void intake_Box() {
    	catcher1.set(1);
    	
    }
    public void outake_Box() {
    	catcher1.set(-1);
    }
    
    
    
}

