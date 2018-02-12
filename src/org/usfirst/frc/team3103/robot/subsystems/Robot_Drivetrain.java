package org.usfirst.frc.team3103.robot.subsystems;

import org.usfirst.frc.team3103.robot.RobotMap;
import org.usfirst.frc.team3103.robot.commands.arcade_Drive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


/**
 *
 */
public class Robot_Drivetrain extends Subsystem {
	WPI_TalonSRX flDrive = new WPI_TalonSRX(RobotMap.flMotor);
	WPI_TalonSRX frDrive = new WPI_TalonSRX(RobotMap.frMotor);
	WPI_TalonSRX blDrive = new WPI_TalonSRX(RobotMap.blMotor);
	WPI_TalonSRX brDrive = new WPI_TalonSRX(RobotMap.brMotor);
	
	DifferentialDrive WCD = new DifferentialDrive(flDrive, frDrive);
	
	public void InitializeDrive() {
		//Inversion
		frDrive.setInverted(false); //right
		flDrive.setInverted(false); //left
		brDrive.setInverted(false); //right
		blDrive.setInverted(false); //left 
		//Follow
		blDrive.follow(flDrive);
		brDrive.follow(frDrive);
	}

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new arcade_Drive());
    }
    
    public void teleopDrive(XboxController xboxController) {
    	WCD.arcadeDrive(xboxController.getRawAxis(1), xboxController.getRawAxis(4), false);
    }
    
    public void random(double left, double right) {
    	flDrive.set(left);
    	frDrive.set(right);
    }
}

