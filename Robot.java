package org.usfirst.frc.team97.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import org.usfirst.frc.team97.robot.GripPipeline;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
// Left is on 1
// Right is on 0
public class Robot extends IterativeRobot implements CompetitionVitals {
	RobotDrive myRobot;
	Joystick leftStick;
	Joystick rightStick;
	// Camera
	CameraServer cameserver;
	// Drive motors
	Jaguar[] driveMotors = new Jaguar[4];
	//Aux motor controllers
	Talon armMotorLeft;
	Talon armMotorRight;
	Talon gatherMotor;
	Victor winchMotor;
	Victor liftMotor;
	Servo IansSillyMotorMan;
	// gyroscope
	AnalogGyro gyro;
	
	DigitalInput autoSwitch1;
	DigitalInput autoSwitch2;
	
	double motorSensitivity = 1.0;
	double offset = 0.0f;// how much to the right the robot favors, between -1.0 to 1.0
	double gyroAngle;
	double gyroRate;
	
	//fields for the gatherer arm
	int armPower;
	
	autoMode autoSetting = autoMode.None;
	
	int switchVal;
	
	double servoZero;
	
	
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	// 2
	
	double initialGyroAngle;
	public void robotInit() {
		
		cameserver = CameraServer.getInstance();
		cameserver.startAutomaticCapture();
		
		SmartDashboard.putString("robotInit", "works");
		leftStick = new Joystick(0);
		rightStick = new Joystick(1);
		gyro = new AnalogGyro(0);
		gyro.initGyro();
		gyro.calibrate();
		gyro.reset();
		gyroAngle = gyro.getAngle();
		// initialGyroAngle = gyro.getAngle();
		gyroRate = gyro.getRate();
		SmartDashboard.putNumber("Gyro Angle", gyroAngle);
		SmartDashboard.putNumber("Gyro Rate", gyroRate);
		for (int i = 0; i < 4; i++) {
			// might change depending on were he put's the PWM
			driveMotors[i] = new Jaguar(i);
		}
		myRobot = new RobotDrive(driveMotors[0], driveMotors[1], driveMotors[2], driveMotors[3]);
		//Auxiliary motors
		gatherMotor = new Talon(4);
		armMotorLeft = new Talon(5);
		armMotorRight = new Talon(6);
		winchMotor = new Victor(7);
		liftMotor = new Victor(8);
		IansSillyMotorMan = new Servo(9);
		armPower = 3;
		autoSwitch1 = new DigitalInput(0);
		autoSwitch2 = new DigitalInput(1);
	}
	
	long timeAtStart;
	double timeElapsed = 0.0;//seconds
	
	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	public void autonomousInit() {
		//gyro.calibrate();
		getInTheModeAutoMode();
		SmartDashboard.putString("AutoMode", autoSetting.name());
		timeAtStart = System.currentTimeMillis();
	}
	
	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		SmartDashboard.putString("AutoMode", autoSetting.name());
		SmartDashboard.putBoolean("switch 1", autoSwitch1.get());
		SmartDashboard.putBoolean("switch 2", autoSwitch2.get());
		double turnFactor = 0.5;
		double moveSpeed = 0.8;
		timeElapsed = (((double) (System.currentTimeMillis() - timeAtStart)) / 1000);
		SmartDashboard.putNumber("autotime", timeElapsed);
		if(autoSetting == autoMode.LowBar) {
			
			 //0 - 3: drive forward
			 //3 - 4: turn right
			 //4 - 7: drive forward
			 //7 - 8: place ball
			 //8 - 11: drive backward
			 //11 - 12: turn until facing our own goal
			 //12 - 15: drive backward
			 
			armPower = 3;
			if(timeElapsed < 0.65) lowerArm();
			if(timeElapsed < 8 && timeElapsed > 0.65) myRobot.tankDrive(moveSpeed, moveSpeed);
			//else if(timeElapsed > 8 && timeElapsed < 13) myRobot.tankDrive(-moveSpeed, -moveSpeed);
			
			
			/*
			if (timeElapsed > 13) {
				//drive
				myRobot.tankDrive(moveSpeed, moveSpeed);
			} else if(timeElapsed > 11) {
				//turn back back
				//turnReverseNeutral(2.0);
				myRobot.tankDrive(-turnFactor, turnFactor);
			} else if (timeElapsed > 9) {
				//drive back
				myRobot.tankDrive(-moveSpeed, -moveSpeed);
			} else if (timeElapsed > 7) {
				//place ball
				fire(1.0);
			} else if (timeElapsed > 6) {
				//drive forward
				myRobot.tankDrive(moveSpeed, moveSpeed);
			} else if (timeElapsed > 4) {
				//turn
				myRobot.tankDrive(turnFactor, -turnFactor);
			} else {
				//drive forward through low bar
				myRobot.tankDrive(moveSpeed, moveSpeed);
			}
			if (timeElapsed < 1.0) lowerArm();
			*/
		} else if (autoSetting == autoMode.breaker) {
			myRobot.tankDrive(moveSpeed, moveSpeed);
			armPower = 3;
			if(timeElapsed > 10 && timeElapsed < 10.65) lowerArm();
			
		} else if (autoSetting == autoMode.cheval) {//41 inches	between bot front and defense
			if(timeElapsed < .8) myRobot.tankDrive(.6, .6); 
			//else if (timeElapsed > 1.2) myRobot.tankDrive(moveSpeed, moveSpeed); 
			else myRobot.stopMotor();
			
			if(timeElapsed > .8 && timeElapsed < .84) lowerArm();
			
			
			/*if(timeElapsed < 1.8 || timeElapsed > 3.0) {//drive forward until at cheval
				myRobot.tankDrive(0.6, 0.6);
			}
			
			if(timeElapsed > 1.8 && timeElapsed < 2.45) { //flatten Cheval de Frise if time appropriate
				myRobot.tankDrive(0, 0);
				lowerArm();
			}
			else if(timeElapsed > 8.0 && timeElapsed < 8.5)
				raiseArm();*/
			
		}
	}

	/**
	 * This function is called once each time the robot enters user-operated
	 * mode
	 */
	public void teleopInit() {
		gyro.calibrate();
		SmartDashboard.putString("teleopInit", "works");
		for (Jaguar j : driveMotors) {j.setSafetyEnabled(false);}
		armMotorLeft.setSafetyEnabled(false);
		armMotorRight.setSafetyEnabled(false);
		winchMotor.setSafetyEnabled(false);
		servoZero = IansSillyMotorMan.getAngle();
		
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		
		//SmartDashboard.putNumber("switch", switchVal);
		double thresh = 0.1;
		// This code puts a time counter, gyroscope rate,
		// and gyroscope angle to the Smart Dash-board when the
		// robot is enabled in teleop mode

		//gyro.reset();

		// counter++;
		// SmartDashboard.putNumber("counter x", counter);
		gyroRate = gyro.getRate();
		SmartDashboard.putNumber("Gyro Rate x", gyroRate);
		gyroAngle = gyro.getAngle();
		SmartDashboard.putNumber("Gyro Angle x", gyroAngle);
		SmartDashboard.putNumber("Align Button Pressed", leftStick.getRawButton(3) ? 1.0 : 0.0);

		/*if (leftStick.getRawButton(3)) {
			turnTowardsNeutral(); 
		}*/
		if(leftStick.getMagnitude() > thresh || rightStick.getMagnitude() > thresh) {
			stickDrive();
		}
		else {
			myRobot.stopMotor();
		}
				
		if(rightStick.getTrigger()) {
			gatherMotor.set(-0.5);
		} else if (leftStick.getTrigger()) {
			gatherMotor.set(0.5);
		} else {
			gatherMotor.stopMotor();
		}
		
		//armPower readout
		if (armPower == 3){
			SmartDashboard.putString("Arm Power Level", "High (80%)");
		} else if (armPower == 2) {
			SmartDashboard.putString("Arm Power Level", "Medium (50%)");
		} else if (armPower == 1) {
			SmartDashboard.putString("Arm Power Level", "Low (40%)");
		}
		
		if(rightStick.getRawButton(5)){
			raisePower();
		}
		if(rightStick.getRawButton(4)){
			lowerPower();
		}
		
		SmartDashboard.putBoolean("raise pressed", rightStick.getRawButton(3));
		SmartDashboard.putBoolean("lower pressed", rightStick.getRawButton(2));
		if (rightStick.getRawButton(3)) {
			raiseArm();
		} else if (rightStick.getRawButton(2)) {
			lowerArm();
		} else {
			armMotorLeft.set(armMotorLeft.get()/4);
			armMotorRight.set(armMotorRight.get()/4);
		}
		
		if (leftStick.getRawButton(5)) {
			winchMotor.set(0.9);
		}
		else if (leftStick.getRawButton(4)) {
			winchMotor.set(-0.9);
		}
		else {
			winchMotor.stopMotor();
		}
		if (leftStick.getRawButton(2)) {
			liftMotor.set(-0.4);
		}
		else if (leftStick.getRawButton(3)) {
			liftMotor.set(0.4);
		}
		else {
			liftMotor.stopMotor();
			if (rightStick.getRawButton(7)) {
				IansSillyMotorMan.setAngle(servoZero);
			}
			else if (rightStick.getRawButton(6)) {
				IansSillyMotorMan.setAngle(servoZero+45);
			}
			
		}
	}

	/**
	 * This function is called periodically during test mode
	 */

	public void testPeriodic() {

	}

	@Override
	public void turnTowardsNeutral() {
		int turnDirection = gyroAngle > initialGyroAngle ? -1 : 1; // -1 is
																	// left, 1
																	// is right\
		double turningPower = Math.abs(gyroAngle - initialGyroAngle) / 20;
		SmartDashboard.putNumber("turning power", turningPower);
		myRobot.tankDrive(turnDirection * turningPower, turnDirection * -1 * turningPower);
	}

	@Override
	public void turnTowardsNeutral(double threshold) {
		int turnDirection = Math.abs(gyroAngle - initialGyroAngle) > threshold ? -1 : 1;
		double turningPower = Math.abs(gyroAngle - initialGyroAngle) / 20;
		SmartDashboard.putNumber("turning power", turningPower);
		myRobot.tankDrive(turnDirection * turningPower, turnDirection * -1 * turningPower);
	}
	
	public void turnReverseNeutral (double threshold) {
		int turnDirection = Math.abs(gyroAngle - (initialGyroAngle + 180)) > threshold ? -1 : 1;
		double turningPower = Math.abs(gyroAngle - (initialGyroAngle + 180)) / 20;
		SmartDashboard.putNumber("turning power", turningPower);
		myRobot.tankDrive(turnDirection * turningPower, turnDirection * -1 * turningPower);
	}
	
	public void raisePower()
	{
		if (armPower >= 1 && armPower <= 3)
		{
			if (armPower == 1)
			{
				armPower = 2;
			} else if (armPower == 2) {
				armPower = 3;
			}
		}
		Timer.delay(0.2);
	}
	
	public void lowerPower()
	{
		if (armPower >= 1 && armPower <= 3)
		{
			if (armPower == 3)
			{
				armPower = 2;
			} else if (armPower == 2) {
				armPower = 1;
			}
		}
		Timer.delay(0.2);
	}
	
	@Override
	public void stickDrive() {
		myRobot.tankDrive(Math.pow(-leftStick.getY(), 3), Math.pow(-rightStick.getY(), 3));
	}

	@Override
	public void fire(double normalizedFirePower) {
		gatherMotor.set(-normalizedFirePower);
	}

	@Override
	public void raiseArm() {
		if (armPower == 3)
		{
			armMotorLeft.set(-0.8);
			armMotorRight.set(-0.8);			
		}
		else if (armPower == 2)
		{
			armMotorLeft.set(-0.5);
			armMotorRight.set(-0.5);		}
		else if (armPower == 1)
		{
			armMotorLeft.set(-0.4);
			armMotorRight.set(-0.4);
		}
		SmartDashboard.putString("arm", "lift");
	}

	@Override
	public void lowerArm() {
		if (armPower == 3)
		{
			armMotorLeft.set(0.8);
			armMotorRight.set(0.8);
		}
		else if (armPower == 2)
		{
			armMotorLeft.set(0.5);
			armMotorRight.set(0.5);
		}
		else if (armPower == 1)
		{
			armMotorLeft.set(0.25);
			armMotorRight.set(0.25);
		}
		SmartDashboard.putString("arm", "lower");
	}

	@Override
	public boolean hasBall() {
		//ayyy don't worry about it
		return false;
	}

	@Override
	public double getDistance() {
		// Don't use, we don't have camera feed sending processed data
		return 0;
	}
	
	enum autoMode {
		None,
		LowBar,
		breaker,
		cheval
	}
	
	private byte compositeButtons (int[] params) {
		//adds button inputs in such a way that each combination returns a unique value.
		//bit shifts as it goes, so the first button writes to the ones place, the second writes to the twos, the next button the fours, etc 
		byte sum = 0;
		for(int i = 0; i < params.length; i++) {
			sum += params[i] << i;
		}
		return sum;
	}
	
	public int getInTheModeAutoMode () {//converts button input from the two auto mode switches into four possible modes.  Mode 4 is unassigned
		byte autoModem = compositeButtons(new int[] {autoSwitch1.get() ? 1 : 0, autoSwitch2.get() ? 1 : 0});
		switch (autoModem) {
		case(0):
			autoSetting = autoMode.None;
			break;
		case(1):
			autoSetting = autoMode.breaker;
			break;
		case(2):
			autoSetting = autoMode.LowBar;
			break;
		case(3):
			autoSetting = autoMode.cheval;
			break;
		}
		return 0;
	}
}