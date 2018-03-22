package ca.mcgill.ecse211.project;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public final class Robot {
	public static final int FORWARD_SPEED = 180;
	public static final int ROTATE_SPEED = 60;
	public static final int GEAR_SPEED = 125;
	public static final int ACCELERATION = 2000;
	public static final int GEAR_ACCELERATION = 500;
	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 11.25;
	
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor backMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	
	public Robot() {
		
	}
	
	
	/**
	 * Free sets both wheels forward indefinitely.
	 * 
	 */
	public static void forward() {
		leftMotor.forward();
		rightMotor.forward();
	}
	
	/**
	 * Freely rotates the robot clockwise indefinitely.
	 */
	public static void rotateClockWise() {
		leftMotor.forward();
		rightMotor.backward();
	}
	
	/**
	 * Freely rotates the robot counter-clockwise indefinitely.
	 */
	public static void rotateCounterClockWise() {
		leftMotor.backward();
		rightMotor.forward();
	}
	
	/**
	 * Stop both wheels.
	 */
	public static void stop() {
		leftMotor.stop(true);
		rightMotor.stop(false);
	}
	
	/**
	 * Travel distance dist.
	 * @param dist : distance to travel
	 * @param leftWheelDir : 1 for the left wheel to go forwrd, -1 for backward
	 * @param rightWheelDir : 1 for the right wheel to go forward, -1 for backward
	 */
	public static void rotateByDistance(double dist, int leftWheelDir, int rightWheelDir) {
		leftMotor.rotate(leftWheelDir * convertDistance(WHEEL_RAD, dist), true);
		rightMotor.rotate(rightWheelDir * convertDistance(WHEEL_RAD, dist), false);
		stop();
	}
	
	/**
	 * 
	 * @param degrees
	 * @param leftWheelDir
	 * @param rightWheelDir
	 */
	public static void rotateByAngle(double degrees, int leftWheelDir, int rightWheelDir) {
		leftMotor.rotate(leftWheelDir * convertAngle(WHEEL_RAD, TRACK, degrees), true);
		rightMotor.rotate(rightWheelDir * convertAngle(WHEEL_RAD, TRACK, degrees), false);
		stop();
	}
	
	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method allows the conversion of a angle to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @param angle
	 * @return
	 */
	public static int convertAngle(double radius, double width, double degrees) {
		return convertDistance(radius, Math.PI * width * degrees / 360.0);
	}

	/**
	 * Calculates distance between (x1, y1) and (x2, y2)
	 * @param x1 : x-coord of the current position
	 * @param y1 : y-coord of the current position
	 * @param x2 : x-coord of the destination
	 * @param y2 : y-coord of the destination
	 * @return distance between current position and destination
	 */
	public static double calculateDistance(double x1, double y1, double x2, double y2) {
		return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
	}
	
	/**
	 * Update the robot's acceleration
	 * @param acc : desired acceleration
	 */
	public static void setAcceleration(int acc) {
		leftMotor.setAcceleration(acc);
		rightMotor.setAcceleration(acc);
	}
	
	/**
	 * Update the robot's speed.
	 * @param sp : desired speed
	 */
	public static void setSpeed(int sp) {
		leftMotor.setSpeed(sp);
		rightMotor.setSpeed(sp);
	}
	
	public static void landingGearOn() {
		backMotor.setSpeed(Robot.GEAR_SPEED);
		backMotor.setAcceleration(Robot.GEAR_ACCELERATION);
		
		backMotor.rotate(250);
	}
	
	public static void landingGearOff() {
		backMotor.setSpeed(Robot.GEAR_SPEED);
		backMotor.setAcceleration(Robot.GEAR_ACCELERATION);
		
		backMotor.rotate(-250);
	}
	
}
