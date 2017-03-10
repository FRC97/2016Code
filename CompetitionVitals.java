package org.usfirst.frc.team97.robot;

public interface CompetitionVitals {
	
	//turns the robot until it is the same rotation as at the start, AKA facing the enemy tower directly
	//since a double can never be exactly a value, use a threshold to determine when the angle is acceptable
	//don't make it a looped function, call it every iteration of teleopPeriodic
	public void turnTowardsNeutral ();
	
	//same as the first, but takes a custom threshold
	public void turnTowardsNeutral(double threshold);
	
	
	//drives the robot from the joy sticks
	public void stickDrive ();
	
	//fires with a power between 0 and 1
	public void fire (double normalizedFirePower);
	
	//extends climber arm
	public void raiseArm();
	
	//retracts arm
	public void lowerArm();
	
	//has a ball
	public boolean hasBall ();
	
	//gets distance in front of IR Sensor
	public double getDistance();
	
	
}