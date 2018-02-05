package org.usfirst.frc.team3103.robot.subsystems;

import org.usfirst.frc.team3103.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Elevator_Subsystem extends Subsystem {

    WPI_TalonSRX elevatorMotor1 = new WPI_TalonSRX(RobotMap.elevatorMotor1);
    WPI_TalonSRX elevatorMotor2 = new WPI_TalonSRX(RobotMap.elevatorMotor2);

    public void initializeElevator() {
    	//Invert motors
    	
        elevatorMotor1.setInverted(false);
        elevatorMotor2.setInverted(false);
        
        //Follow 
        elevatorMotor2.follow(elevatorMotor1);
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    /*
     * The method makes the elevator go up 
     */
    public void up() {
    	elevatorMotor1.set(0.75);	
    }
    
    /*
     * The method makes the elevator go down 
     */
    public void down() {
    	elevatorMotor1.set(-0.75);
    }
    
}

