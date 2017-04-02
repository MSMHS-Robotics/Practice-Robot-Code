package org.usfirst.frc.team2723.robot.commands;

import org.usfirst.frc.team2723.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import com.kauailabs.navx.frc.AHRS;

/**
 *
 */
public class GearNavX extends Command implements PIDOutput {

    public GearNavX() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	/*final double p = 0.02;
	    final double i = 0.00;
	    final double d = 0.00;
	    final double f = 0.00;
	    AHRS ahrs = new AHRS(SPI.Port.kMXP,(byte)200);
	    
	    PIDController controller1 = new PIDController(p, i, d, f, ahrs, this);
	    */
    	
    	/*if (!controller1.isEnabled())
    	{
    	    controller1.setSetpoint(Robot.ahrs.getYaw());
    	    Robot.rotateToAngleRate = 0;
    	    controller1.enable();
    	} 
    	Robot.myRobot.tankDrive(0.5, 0.5);
    	Timer.delay(3); */
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.myRobot.tankDrive(0,0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.myRobot.tankDrive(0,0);
    }

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		
	}
}
