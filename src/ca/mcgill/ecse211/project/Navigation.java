package ca.mcgill.ecse211.project;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * Class containing all movement related behaviors.
 * Extends Thread
 * 
 * @author Volen Mihaylov
 * @author Patrick Ghazal
 * @author Bryan Jay
 */
public class Navigation extends Thread {

	private Odometer odometer;
	
	private double deltaX;
	private double deltaY;

	// current location of the vehicle
	private double currX;
	private double currY;
	private double currDegrees;

	private boolean navigating = false;

	public USLocalizer usLoc;

	private static final Port usSidePort = LocalEV3.get().getPort("S4");

	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor backMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	
	private static SensorModes sideUltrasonicSensor;
	private static SampleProvider sideUsDistance;
	private float[] sideUsData;

	private static int distanceSensorToBlock = 2;

	/**
	 * Navigation constructor
	 * 
	 * @param odo Odometer object to keep track of position for coordinate related movements
	 */
	public Navigation(Odometer odo) {
		this.odometer = odo;
		setAcceleration(Robot.ACCELERATION);
		setSpeed(Robot.FORWARD_SPEED);

		// usSensor is the instance
		sideUltrasonicSensor = new EV3UltrasonicSensor(usSidePort);
		// usDistance provides samples from this instance
		sideUsDistance = sideUltrasonicSensor.getMode("Distance");
		sideUsData = new float[sideUsDistance.sampleSize()];
	}

	/**
	 * A method to drive our vehicle to a certain Cartesian coordinate
	 * 
	 * @param x
	 *            X-Coordinate
	 * @param y
	 *            Y-Coordinate
	 */
	public void travelTo(double x, double y, boolean lookForBlocks, SearchAndLocalize search) {
		navigating = true;
		/*
		 * The search instance of SearchAndLocalize in the parameters is only necessary
		 * when lookForBlocks is true. Therefore, in calls where lookForBlocks is false,
		 * we pass null to the search parameter.
		 */

		currX = odometer.getXYT()[0];
		currY = odometer.getXYT()[1];

		deltaX = x - currX;
		deltaY = y - currY;

		// Calculate the angle to turn around
		currDegrees = odometer.getXYT()[2];
		double mDegrees = Math.atan2(deltaX, deltaY)/Math.PI*180 - currDegrees;
		double hypot = Math.hypot(deltaX, deltaY);

		// Turn to the correct angle towards the endpoint
		turnTo(mDegrees);

		setSpeed(Robot.FORWARD_SPEED);
		rotateByDistance(hypot, 1, 1);

		// stopRobot vehicle
		stopRobot();
		navigating = false;
	}

	/**
	 * Goes to a block when it is detected in the field
	 * @param searcher : instance of the SearchAndLocalize class
	 */
	private void goToBlock(SearchAndLocalize searcher) {
		int dist = this.usLoc.fetchUS();
		if (dist > distanceSensorToBlock) {
			rotateByDistance(dist - distanceSensorToBlock, 1, 1);
		}
		searcher.getCC().colourDetection();
		if (searcher.getCC().isBlock()) {
			searcher.setFoundBlock(true);
		}
	}

	/**
	 * Checks for the presence of a block in the sights of the sensor.
	 * @param searcher: instance of the SearchAndLocalize class
	 * @return 0 if no block is detected, 1 if a block is detected
	 * on the side, 2 if a block is detected at the front
	 */
	private int blockDetected(SearchAndLocalize searcher) {
		/*
		 * 0: no block 1: side block 2: front block
		 */
		int sideDistance = fetchSideUS();
		int frontDistance = this.usLoc.fetchUS();
		if (sideDistance < (searcher.lowerLeftX - searcher.upperRightX) / 2 * Robot.TILESIZE + 5) {
			return 1;
		} else if (frontDistance < 5) {
			return 2;
		}
		return 0;
	}

	/**
	 * A method to turn our vehicle to a certain angle
	 * 
	 * @param degrees
	 */
	public void turnTo(double degrees) {
		navigating = true;
		// ensures minimum angle for turning
		if (degrees > 180) {
			degrees -= 360;
		} else if (degrees < -180) {
			degrees += 360;
		}

		// set Speed
		setSpeed(Robot.ROTATE_SPEED);
		
		// if angle is negative, turn to the left
		if (degrees < 0) {
			rotateByAngle(-1 * degrees, -1, 1);

		} else {
			// angle is positive, turn to the right
			rotateByAngle(degrees, 1, -1);
		}
		stopRobot();
		navigating = false;
	}
	/**
	 * Moves robot forward or back by certain amount
	 * @param distance Amount to move by 
	 */
	public static void moveBy(double distance) {
		if (distance>=0) {
		rotateByDistance(distance, 1, 1);
		}
		else {
			rotateByDistance(distance, -1, -1);
			
		}
	}
	
	
	
	/**
	 * Freely sets both wheels forward indefinitely.
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
	public static void stopRobot() {
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
		leftMotor.rotate((int)(leftWheelDir * Robot.convertDistance(Robot.WHEEL_RAD, dist)*(Robot.TILESIZE+0.4)/Robot.TILESIZE), true);
		rightMotor.rotate((int)(rightWheelDir * Robot.convertDistance(Robot.WHEEL_RAD, dist)*(Robot.TILESIZE+0.4)/Robot.TILESIZE), false);
		stopRobot();
	}
	
	/**
	 * Turn by 'degrees' degrees.
	 * @param degrees : angle to turn
	 * @param leftWheelDir : 1 for the left wheel to go forwrd, -1 for backward
	 * @param rightWheelDir : 1 for the right wheel to go forward, -1 for backward
	 */
	public static void rotateByAngle(double degrees, int leftWheelDir, int rightWheelDir) {
		if (leftWheelDir == 1 && rightWheelDir == -1) {
			leftMotor.rotate(leftWheelDir * Robot.convertAngle(Robot.WHEEL_RAD, Robot.TRACK, degrees+2), true);
			rightMotor.rotate(rightWheelDir * Robot.convertAngle(Robot.WHEEL_RAD, Robot.TRACK, degrees+2), false);

		}
		else {
			leftMotor.rotate(leftWheelDir * Robot.convertAngle(Robot.WHEEL_RAD, Robot.TRACK, degrees), true);
			rightMotor.rotate(rightWheelDir * Robot.convertAngle(Robot.WHEEL_RAD, Robot.TRACK, degrees), false);
		}
		stopRobot();
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
	/**
	 * Lowers back wheels
	 */
	public static void landingGearOn() {
		backMotor.setSpeed(Robot.GEAR_SPEED);
		backMotor.setAcceleration(Robot.GEAR_ACCELERATION);
		
		backMotor.rotate(195);
	}
	/**
	 * Lifts back wheels
	 */
	public static void landingGearOff() {
		backMotor.setSpeed(Robot.GEAR_SPEED);
		backMotor.setAcceleration(Robot.GEAR_ACCELERATION);
		
		backMotor.rotate(-195);
	}

	/**
	 * A method to determine whether another thread has called travelTo and turnTo
	 * methods or not
	 * 
	 * @return
	 */
	boolean isNavigating() throws OdometerExceptions {
		return navigating;
	}

	/**
	 * Returns the distance value as seen by the side sensor.
	 * @return
	 */
	public int fetchSideUS() {
		sideUsDistance.fetchSample(sideUsData, 0);
		return (int) (sideUsData[0] * 100);
	}
}
