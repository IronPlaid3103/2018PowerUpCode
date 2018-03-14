package org.usfirst.frc.team3103.robot.subsystems;

import org.usfirst.frc.team3103.robot.RobotMap;
import org.usfirst.frc.team3103.robot.commands.arcade_Drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.Timer;
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
	
	PigeonIMU gyro = new PigeonIMU(brDrive);
	
	
	DifferentialDrive WCD = new DifferentialDrive(flDrive, frDrive);
	
	public void InitializeDrive() {
		//Inversion
		frDrive.setInverted(false); //2 front right
		flDrive.setInverted(false); //1 front left
		brDrive.setInverted(false); // 3 back left
		blDrive.setInverted(false); //4 back right
		//Follow
		blDrive.follow(flDrive);
		brDrive.follow(frDrive);
		
		//frDrive.follow(brDrive);
		//flDrive.follow(blDrive);
		
		//gyro.setFusedHeading(0.0, 5000);
		flDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		frDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		
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
  //  	double testAngle  = gyro.getFusedHeading();
    	double testAngle = gyro.getAbsoluteCompassHeading();
		System.out.println("Current heading: " + testAngle);		
		
	    System.out.println("encoder value = " + rpm);

		
    	WCD.arcadeDrive(-joystick.getRawAxis(1), joystick.getRawAxis(4), false);
    }
    
    
    public void random(double left, double right) {
    	flDrive.set(left);
    	frDrive.set(right);
    }
    
    double rpm = FeedbackDevice.CTRE_MagEncoder_Relative.value;
    
    
    public void forwardAuto(int targetDistance) {
    	Timer timer = new Timer();
    	timer.reset();
    	timer.start();
    	double currentTime = timer.get();
    	double currentDistance = rpm * currentTime / 60 * 6 * Math.PI;
    	if (currentDistance < targetDistance) {
    		flDrive.set(1);
    		frDrive.set(1);
    	}
    	else if (currentDistance >= targetDistance) {
    		flDrive.set(0);
    		frDrive.set(0);
    	}
    }
   
    
    double kPgain = 0.04;
    double kDgain = 0;  //was 5 before, might e4xplain erratic movement
    double kMaxCorrectionRatio = 0.30;
    double targetAngle = 0;
    
    double [] ypr = new double[3];
    double [] xyz_dps = new double [3];

    
    //get angle from gyro
    public double getYaw() {
    	gyro.getYawPitchRoll(ypr);
    	return ypr[0];
    	//return gyro.getFusedHeading();
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
	
	double MaxCorrection(double forwardThrot, double scalor) {
		/* make it positive */
		if(forwardThrot < 0) {forwardThrot = -forwardThrot;}
		/* max correction is the current forward throttle scaled down */
		forwardThrot *= scalor;
		/* ensure caller is allowed at least 10% throttle,
		 * regardless of forward throttle */
		if(forwardThrot < 0.10)
			return 0.10;
		return forwardThrot;
	}
		
    
    public void turnToTargetAngle(double target, Joystick joystick) {
    	System.out.println("turntotargetangle - test");
    	
    	gyro.getYawPitchRoll(ypr);
    	
    	double currentAngle = ypr[0];
    	
    	System.out.println("current angle = " + currentAngle);
    	gyro.getRawGyro(xyz_dps);
        double currentAngularRate = xyz_dps[2];
    	double forwardThrottle = joystick.getRawAxis(1) * -1.0;
		double maxThrot = MaxCorrection(forwardThrottle, kMaxCorrectionRatio);
    	
    	System.out.println("get raw axis 1 = " + joystick.getRawAxis(1));
    	System.out.println("forwardThrottle = " + forwardThrottle);
    	
    	//double turnThrottle = joystick.getRawAxis(4) * -1.0;
    	
		double turnThrottle = (target - currentAngle) * kPgain - (currentAngularRate); //removed kD gain

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

