package org.usfirst.frc.team3103.robot.subsystems;

import org.usfirst.frc.team3103.robot.RobotMap;
import org.usfirst.frc.team3103.robot.commands.arcade_Drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.MotorSafety;
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
	
	PigeonIMU gyro = new PigeonIMU(flDrive);
	
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
		
		
    	WCD.setSafetyEnabled(false);
	}

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new arcade_Drive());
    }
        
    
    public void teleopDrive(Joystick joystick) {
    	WCD.arcadeDrive(joystick.getRawAxis(1), joystick.getRawAxis(4), false);
    }
    
    public void random(double left, double right) {
    	flDrive.set(left);
    	frDrive.set(right);
    }
    
   
    
    double kPgain = 0.04;
    double kDgain = 0.04;
    double kMaxCorrectionRatio = 0.30;
    double targetAngle = 0;
    
    double [] ypr = new double[3];
    double [] xyz_dps = new double [3];

    
    //get angle from gyro
    public double getYaw() {
    	return gyro.getFusedHeading();
    }
    
    //deadband
    public double Db(double axisVal) {
    	if (axisVal < -0.10)
    		return axisVal;
    	if (axisVal > +0.10)
    		return axisVal;
    	return 0;
    }
    
    
	double Cap(double value, double peak) {
		if (value < -peak)
			return -peak;
		if (value > +peak)
			return +peak;
		return value;
	}
    
    public void turnToTargetAngle(double target, Joystick joystick) {
    	System.out.println("turntotargetangle - test");
    	
    	double currentAngle = gyro.getFusedHeading();
    	gyro.getRawGyro(xyz_dps);
        double currentAngularRate = xyz_dps[2];
    	double forwardThrottle = joystick.getRawAxis(1) * -1.0;
    	System.out.println("get raw axis 1 = " + joystick.getRawAxis(1));
    	System.out.println("forwardThrottle = " + forwardThrottle);
    	
    	//double turnThrottle = joystick.getRawAxis(4) * -1.0;
    	
		double turnThrottle = (target - currentAngle) * kPgain - (currentAngularRate) * kDgain;

    	System.out.println("get raw axis 4 = " + joystick.getRawAxis(4));
    	System.out.println("turnThrottle = " + turnThrottle);

    	
    	
    	forwardThrottle = Db(forwardThrottle);
    	turnThrottle = Db(turnThrottle);
    	
    	double left = forwardThrottle - turnThrottle;
    	double right = forwardThrottle + turnThrottle;
    	left = Cap(left, 1.0);
    	right = Cap(right, 1.0);
    	
    	System.out.println("Current Angle = " + currentAngle);
    	
    	if (currentAngle != target) {
    		flDrive.set(ControlMode.PercentOutput, left);
    		//flDrive.set(ControlMode.PercentOutput, 0.5);
    		System.out.println("left "+ left); 
    		frDrive.set(ControlMode.PercentOutput, -1 * right);
    		//frDrive.set(ControlMode.PercentOutput, -0.5);
    		System.out.println("right "+ right); 
    	}
    	else {
    		flDrive.set(ControlMode.PercentOutput, 0);
    		frDrive.set(ControlMode.PercentOutput, 0);
    	}
    	
    		
    }
    
}

