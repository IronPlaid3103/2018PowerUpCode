package org.usfirst.frc.team3103.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team3103.robot.Robot;
import org.usfirst.frc.team3103.robot.commands.AutoAim;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 *
 */
public class LimeLight extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	NetworkTableEntry tx = table.getEntry("tx");
	NetworkTableEntry ty = table.getEntry("ty");
	NetworkTableEntry ta = table.getEntry("ta");
	double x = tx.getDouble(0);
	double y = ty.getDouble(0);
	double area = ta.getDouble(0);
	double kP = -0.1;
	
	public double aimLeft() {
		double heading_error = x;
		double steering_adjust = kP * x;
		double leftCommand = 0.75;
		
		return leftCommand += steering_adjust;
	}
	
	public void aimRight() {
		double heading_error = x;
		double steering_adjust = kP * x;
	}
	
	public void lightOn() {
        Robot.camera.table.getEntry("ledMode").setValue(0);
	}
	
	public void lightOff() {
        Robot.camera.table.getEntry("ledMode").setValue(1);
	}
	
	public void lightFlash() {
        Robot.camera.table.getEntry("ledMode").setValue(2);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	
    	//setDefaultCommand(new AutoAim());
    }
}

