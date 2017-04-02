
package org.usfirst.frc.team2723.robot;

import org.usfirst.frc.team2723.robot.commands.CrossLine;


import org.usfirst.frc.team2723.robot.commands.DriveForward;
import org.usfirst.frc.team2723.robot.commands.ExampleCommand;
import org.usfirst.frc.team2723.robot.commands.Shooter;
import org.usfirst.frc.team2723.robot.commands.GearNavX;
import org.usfirst.frc.team2723.robot.commands.ShooterAuto;
import org.usfirst.frc.team2723.robot.subsystems.ExampleSubsystem;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot implements PIDOutput {

	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;

    Command autonomousCommand;
    SendableChooser<Command> chooser;
    public static AHRS ahrs;
    
    //Defines the variables as members of our Robot class
    static CANTalon frontLeftMotor = new CANTalon(4);
    static CANTalon rearLeftMotor = new CANTalon(3);
    static CANTalon frontRightMotor = new CANTalon(2);
    static CANTalon rearRightMotor = new CANTalon(1);	
    static CANTalon lift = new CANTalon(5);
    static CANTalon intake = new CANTalon(9);
    public static CANTalon wow = new CANTalon(8);
    public static CANTalon lShooter = new CANTalon(6); 
    public static CANTalon rShooter = new CANTalon(7);
    public static Jaguar mixer = new Jaguar(0);
    boolean shooterOn = false;
    
    public static RobotDrive myRobot;  // class that handles basic drive operations
    Joystick joystick;  // set to ID 1 in DriverStation
    Joystick joystick2;
    
    /*public static PIDController turnController = new PIDController(0.02, 0.00, 0.00, 0.00, ahrs, this);
    turnController.setInputRange(-180.0f,  180.0f);
    turnController.setOutputRange(-0.8, 0.8); */
    
    public static double rotateToAngleRate;
    public static double leftRobotSpeed = 0.5;
    public static double rightRobotSpeed = 0.5;
    public static double magnitude;
    

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
		oi = new OI();
        chooser = new SendableChooser();
        chooser.addDefault("Default Auto", new ExampleCommand());
        //chooser.addObject("My Auto", new MyAutoCommand());
        SmartDashboard.putData("Auto mode", chooser);
        UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture();
		UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture();
        
        myRobot = new RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
       //myRobot.setExpiration(0.01);
        myRobot.setSafetyEnabled(false);
        joystick = new Joystick(0);
        joystick2 = new Joystick(1);
        
        frontLeftMotor.enableControl();
        rearLeftMotor.enableControl();
        frontRightMotor.enableControl();
        rearRightMotor.enableControl();
        lift.enableControl();
        intake.enableControl();
        wow.enableControl();
        lShooter.enableControl();
        rShooter.enableControl();
       
        try {
            /* Communicate w/navX MXP via the MXP SPI Bus.                                     */
            /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
            /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
            ahrs = new AHRS(SPI.Port.kMXP,(byte)200);
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
        
        chooser.addObject("DriveForward", new DriveForward());
        chooser.addObject("ShooterAuto", new ShooterAuto());
        chooser.addObject("CrossLine", new CrossLine());
        
        /*turnController = new PIDController(p, i, d, f, ahrs, (PIDOutput) this);
        turnController.setInputRange(-180.0f, 180.0f);
        turnController.setOutputRange(-1.0, 1.0);
        turnController
        turnController
        turnController */
        
        
    }
	
	/**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
     */
    public void disabledInit(){

    }
	
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString code to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the chooser code above (like the commented example)
	 * or additional comparisons to the switch structure below with additional strings & commands.
	 */
    public void autonomousInit() {
        //autonomousCommand = (Command) chooser.getSelected();
     
		 String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
		switch(autoSelected) {
		case "DriveForward":
			autonomousCommand = new DriveForward();
			break;
		case "ShooterAuto":
			autonomousCommand = new ShooterAuto();
			break;
		case "CrossLine":
			autonomousCommand = new CrossLine();
			break;
		case "Default Auto":
		default:
			autonomousCommand = new ExampleCommand();
			break;
		} 
    	
    	// schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        
        //code goes here
      
    }

    public void teleopInit() {
		// This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        double start_time = Timer.getFPGATimestamp();
        
        while (isOperatorControl() && isEnabled()) {
        	double slowdown = joystick.getRawAxis(2); //2= left trigger
			double left = -joystick.getRawAxis(1); // 1= left stick y
	    	double right = -joystick.getRawAxis(5); //5- right stick y
	    	if (slowdown > 0.5) {
	    		left = left/2;
	    		right = right/2;
        	}
	    	
	    	if(joystick.getRawButton(5)) // Left Bumper Button on the Logitech F310 joystick
            {
            	intake.set(1); //intake out
            } else if(joystick.getRawButton(6)) // Right Bumper Button on joystick
            {
            	intake.set(-1); //intake in
            } else {
            	intake.set(0);
            }

        	myRobot.tankDrive(left, right);
        	
        	Timer.delay(0.005);		// wait for a motor update time
        	
        	//Controller 2 Code
        	lift.set(joystick2.getRawAxis(1));

            if(joystick2.getRawButton(5)) // Left Bumper Button on the Logitech F310 joystick
            {
            	wow.set(-1); //wow up
            	mixer.set(1); //mixer in
            } else if(joystick2.getRawAxis(2) > 0.5) // Left Trigger Button on joystick
            {
            	wow.set(1); //wow down
            	mixer.set(-1); //mixer out
            } else {
            	wow.set(0);
            	mixer.set(0);
            }
            
            if(joystick2.getRawButton(6)){ //right bumper
            	shooterOn = true;
            }
            if(joystick2.getRawAxis(3) > 0.5){ //right trigger
            	shooterOn = false;
            }
            if(shooterOn)
            { 
            	lShooter.set(1);
            	rShooter.set(-1);
            } else {
            	lShooter.set(0);
            	rShooter.set(0);
            }
        	
        	
        	//Timer.delay(0.020);		/* wait for one motor update time period (50Hz)     */
             /*
             boolean zero_yaw_pressed = joystick.getTrigger();
             if ( zero_yaw_pressed ) {
                 ahrs.zeroYaw();
             }

             Calculate/display effective update rate in hz */
             //double delta_time = Timer.getFPGATimestamp() - start_time;
             //double update_count = ahrs.getUpdateCount();
            // if ( update_count > 0 ) {
            // 	double avg_updates_per_sec = delta_time / update_count;
             //	if ( avg_updates_per_sec > 0.0 ) {
             	//	SmartDashboard.putNumber("IMU_EffUpdateRateHz", 1.0 / avg_updates_per_sec);
             	//}
           //  }  
             
             /* Display 6-axis Processed Angle Data                                      */
             SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
             SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
             SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
             SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
             SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
             
             /* Display tilt-corrected, Magnetometer-based heading (requires             */
             /* magnetometer calibration to be useful)                                   */
             
             SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());
             
             /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
             SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());

             /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
             /* path for upgrading from the Kit-of-Parts gyro to the navx MXP            */
             
             SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
             SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());

             /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
             
             SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
             SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
             SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving());
             SmartDashboard.putBoolean(  "IMU_IsRotating",       ahrs.isRotating());

             /* Display estimates of velocity/displacement.  Note that these values are  */
             /* not expected to be accurate enough for estimating robot position on a    */
             /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
             /* of these errors due to single (velocity) integration and especially      */
             /* double (displacement) integration.                                       */
             
             SmartDashboard.putNumber(   "Velocity_X",           ahrs.getVelocityX());
             SmartDashboard.putNumber(   "Velocity_Y",           ahrs.getVelocityY());
             SmartDashboard.putNumber(   "Displacement_X",       ahrs.getDisplacementX());
             SmartDashboard.putNumber(   "Displacement_Y",       ahrs.getDisplacementY());
             
             /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
             /* NOTE:  These values are not normally necessary, but are made available   */
             /* for advanced users.  Before using this data, please consider whether     */
             /* the processed data (see above) will suit your needs.                     */
             
             SmartDashboard.putNumber(   "RawGyro_X",            ahrs.getRawGyroX());
             SmartDashboard.putNumber(   "RawGyro_Y",            ahrs.getRawGyroY());
             SmartDashboard.putNumber(   "RawGyro_Z",            ahrs.getRawGyroZ());
             SmartDashboard.putNumber(   "RawAccel_X",           ahrs.getRawAccelX());
             SmartDashboard.putNumber(   "RawAccel_Y",           ahrs.getRawAccelY());
             SmartDashboard.putNumber(   "RawAccel_Z",           ahrs.getRawAccelZ());
             SmartDashboard.putNumber(   "RawMag_X",             ahrs.getRawMagX());
             SmartDashboard.putNumber(   "RawMag_Y",             ahrs.getRawMagY());
             SmartDashboard.putNumber(   "RawMag_Z",             ahrs.getRawMagZ());
             SmartDashboard.putNumber(   "IMU_Temp_C",           ahrs.getTempC());
             
             /* Omnimount Yaw Axis Information                                           */
             /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
             AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
             SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
             SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
             
             /* Sensor Board Information                                                 */
             SmartDashboard.putString(   "FirmwareVersion",      ahrs.getFirmwareVersion());
             
             /* Quaternion Data                                                          */
             /* Quaternions are fascinating, and are the most compact representation of  */
             /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
             /* from the Quaternions.  If interested in motion processing, knowledge of  */
             /* Quaternions is highly recommended.                                       */
             SmartDashboard.putNumber(   "QuaternionW",          ahrs.getQuaternionW());
             SmartDashboard.putNumber(   "QuaternionX",          ahrs.getQuaternionX());
             SmartDashboard.putNumber(   "QuaternionY",          ahrs.getQuaternionY());
             SmartDashboard.putNumber(   "QuaternionZ",          ahrs.getQuaternionZ());
             
             /* Sensor Data Timestamp */
             SmartDashboard.putNumber(   "SensorTimestamp",		ahrs.getLastSensorTimestamp());
             
             /* Connectivity Debugging Support                                           */
             SmartDashboard.putNumber(   "IMU_Byte_Count",       ahrs.getByteCount());
             SmartDashboard.putNumber(   "IMU_Update_Count",     ahrs.getUpdateCount());
        }
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
	}
	
	//gets the position for our drive-train encoders
	//haven't used these variables yet
	/*
	public static double getFrontLeftEncoder() {
		return frontLeftMotor.getEncPosition();
	}
	
	public static double getRearLeftEncoder() {
		return rearLeftMotor.getEncPosition();
	}
	
	public static double getFrontRightEncoder() {
		return frontRightMotor.getEncPosition();
	}
	
	public static double getRearRightEncoder() {
		return rearRightMotor.getEncPosition();
	} */
}
