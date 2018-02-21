package org.usfirst.frc.team3103.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Compressor_Subsystem extends Subsystem {

Compressor C = new Compressor(0);

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	public void Compressor_on(){
		C.setClosedLoopControl(true);
		
	}
	public void Compressor_off() {
		C.setClosedLoopControl(false);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
}

