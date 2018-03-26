package ca.mcgill.ecse211.project;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
<<<<<<< HEAD
 * Class containing all movement related behaviors. Extends Thread
=======
 * Class containing all movement related behaviors.
 * Extends Thread
>>>>>>> b7bc67623d694a277a68dae8bd8f18f9e946ee30
 * 
 * @author Volen Mihaylov
 * @author Patrick Ghazal
 * @author Bryan Jay
 */
public class Navigation extends Thread {

	private Odometer odometer;
<<<<<<< HEAD

=======
	
>>>>>>> b7bc67623d694a277a68dae8bd8f18f9e946ee30
	private double deltaX;
	private double deltaY;

	// current location of the vehicle
	private double currX;
	private double currY;
	private double currDegrees;

	private boolean navigating = false;

	public USLocalizer usLoc;

<<<<<<< HEAD
	// private static final Port usSidePort = LocalEV3.get().getPort("S3");
=======
//	private static final Port usSidePort = LocalEV3.get().getPort("S3");
>>>>>>> b7bc67623d694a277a68dae8bd8f18f9e946ee30

	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor backMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
<<<<<<< HEAD

=======
	
>>>>>>> b7bc67623d694a277a68dae8bd8f18f9e946ee30
	private static SensorModes sideUltrasonicSensor;
	private static SampleProvider sideUsDistance;
	private float[] sideUsData;

	private static int distanceSensorToBlock = 2;

<<<<<<< HEAD
	// Data from Wifi
	private static String currentZone;
	private static int startingCorner;

	/**
	 * Navigation constructor
	 * 
	 * @param odo
	 *            Odometer object to keep track of position for coordinate related
	 *            movements
=======
	/**
	 * Navigation constructor
	 * 
	 * @param odo Odometer object to keep track of position for coordinate related movements
>>>>>>> b7bc67623d694a277a68dae8bd8f18f9e946ee30
	 */
	public Navigation(Odometer odo) {
		this.odometer = odo;
		setAcceleration(Robot.ACCELERATION);
		setSpeed(Robot.FORWARD_SPEED);

		// usSensor is the instance
<<<<<<< HEAD
		// sideUltrasonicSensor = new EV3UltrasonicSensor(usSidePort);
		// // usDistance provides samples from this instance
		// sideUsDistance = sideUltrasonicSensor.getMode("Distance");
		// sideUsData = new float[sideUsDistance.sampleSize()];
=======
//		sideUltrasonicSensor = new EV3UltrasonicSensor(usSidePort);
//		// usDistance provides samples from this instance
//		sideUsDistance = sideUltrasonicSensor.getMode("Distance");
//		sideUsData = new float[sideUsDistance.sampleSize()];
>>>>>>> b7bc67623d694a277a68dae8bd8f18f9e946ee30
	}

	/**
	 * A method to drive our vehicle to a certain Cartesian coordinate
	 * 
	 * @param x
	 *            X-Coordinate
	 * @param y
	 *            Y-Coordinate
	 */
<<<<<<< HEAD
	public void travelTo(double x, double y) {
		navigating = true;
=======
	public void travelTo(double x, double y, boolean lookForBlocks, SearchAndLocalize search) {
		navigating = true;
		/*
		 * The search instance of SearchAndLocalize in the parameters is only necessary
		 * when lookForBlocks is true. Therefore, in calls where lookForBlocks is false,
		 * we pass null to the search parameter.
		 */
>>>>>>> b7bc67623d694a277a68dae8bd8f18f9e946ee30

		currX = odometer.getXYT()[0];
		currY = odometer.getXYT()[1];

		deltaX = x - currX;
		deltaY = y - currY;

		// Calculate the angle to turn around
<<<<<<< HEAD
		double mDegrees = Math.atan2(deltaX, deltaY) / Math.PI * 180;
=======
		double mDegrees = Math.atan2(deltaX, deltaY)/Math.PI*180;
>>>>>>> b7bc67623d694a277a68dae8bd8f18f9e946ee30
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
	 * 
	 * @param searcher
	 *            : instance of the SearchAndLocalize class
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
	 * 
	 * @param searcher:
	 *            instance of the SearchAndLocalize class
	 * @return 0 if no block is detected, 1 if a block is detected on the side, 2 if
	 *         a block is detected at the front
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
<<<<<<< HEAD

=======
		
>>>>>>> b7bc67623d694a277a68dae8bd8f18f9e946ee30
		navigating = true;
		// ensures minimum angle for turning
		degrees = degrees - odometer.getXYT()[2];
		if (degrees > 180) {
			degrees -= 360;
		} else if (degrees < -180) {
			degrees += 360;
		}

		// set Speed
		setSpeed(Robot.ROTATE_SPEED);
<<<<<<< HEAD

=======
		
>>>>>>> b7bc67623d694a277a68dae8bd8f18f9e946ee30
		// if angle is negative, turn to the left
		if (degrees < 0) {
			rotateByAngle(-1 * degrees, -1, 1);

		} else {
			// angle is positive, turn to the right
			rotateByAngle(degrees, 1, -1);
<<<<<<< HEAD
		}
		stopRobot();
		navigating = false;
	}

	/**
	 * Moves robot forward or back by certain amount
	 * 
	 * @param distance
	 *            Amount to move by
	 */
	public static void moveBy(double distance) {
		if (distance >= 0) {
			rotateByDistance(distance, 1, 1);
		} else {
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
=======
		}
		stopRobot();
		navigating = false;
>>>>>>> b7bc67623d694a277a68dae8bd8f18f9e946ee30
	}
	/**
<<<<<<< HEAD
	 * Travel distance dist.
	 * 
	 * @param dist
	 *            : distance to travel
	 * @param leftWheelDir
	 *            : 1 for the left wheel to go forwrd, -1 for backward
	 * @param rightWheelDir
	 *            : 1 for the right wheel to go forward, -1 for backward
	 */
	public static void rotateByDistance(double dist, int leftWheelDir, int rightWheelDir) {
		leftMotor.rotate(leftWheelDir * Robot.convertDistance(Robot.WHEEL_RAD, dist), true);
		rightMotor.rotate(rightWheelDir * Robot.convertDistance(Robot.WHEEL_RAD, dist), false);
		stopRobot();
=======
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
>>>>>>> b7bc67623d694a277a68dae8bd8f18f9e946ee30
	}
	
	
	
	/**
<<<<<<< HEAD
	 * Turn by 'degrees' degrees.
	 * 
	 * @param degrees
	 *            : angle to turn
	 * @param leftWheelDir
	 *            : 1 for the left wheel to go forwrd, -1 for backward
	 * @param rightWheelDir
	 *            : 1 for the right wheel to go forward, -1 for backward
	 */
	public static void rotateByAngle(double degrees, int leftWheelDir, int rightWheelDir) {
		if (leftWheelDir == 1 && rightWheelDir == -1) {
			leftMotor.rotate(leftWheelDir * Robot.convertAngle(Robot.WHEEL_RAD, Robot.TRACK, degrees + 2), true);
			rightMotor.rotate(rightWheelDir * Robot.convertAngle(Robot.WHEEL_RAD, Robot.TRACK, degrees + 2), false);

		} else {
			leftMotor.rotate(leftWheelDir * Robot.convertAngle(Robot.WHEEL_RAD, Robot.TRACK, degrees + 0), true);
			rightMotor.rotate(rightWheelDir * Robot.convertAngle(Robot.WHEEL_RAD, Robot.TRACK, degrees + 0), false);
		}
		stopRobot();
=======
	 * Freely sets both wheels forward indefinitely.
	 * 
	 */
	public static void forward() {
		leftMotor.forward();
		rightMotor.forward();
>>>>>>> b7bc67623d694a277a68dae8bd8f18f9e946ee30
	}
	
	/**
<<<<<<< HEAD
	 * Update the robot's acceleration
	 * 
	 * @param acc
	 *            : desired acceleration
	 */
	public static void setAcceleration(int acc) {
		leftMotor.setAcceleration(acc);
		rightMotor.setAcceleration(acc);
	}

	/**
	 * Update the robot's speed.
	 * 
	 * @param sp
	 *            : desired speed
	 */
	public static void setSpeed(int sp) {
		leftMotor.setSpeed(sp);
		rightMotor.setSpeed(sp);
=======
	 * Freely rotates the robot clockwise indefinitely.
	 */
	public static void rotateClockWise() {
		leftMotor.forward();
		rightMotor.backward();
>>>>>>> b7bc67623d694a277a68dae8bd8f18f9e946ee30
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
<<<<<<< HEAD
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
=======
	 * Travel distance dist.
	 * @param dist : distance to travel
	 * @param leftWheelDir : 1 for the left wheel to go forwrd, -1 for backward
	 * @param rightWheelDir : 1 for the right wheel to go forward, -1 for backward
	 */
	public static void rotateByDistance(double dist, int leftWheelDir, int rightWheelDir) {
		leftMotor.rotate(leftWheelDir * Robot.convertDistance(Robot.WHEEL_RAD, dist), true);
		rightMotor.rotate(rightWheelDir * Robot.convertDistance(Robot.WHEEL_RAD, dist), false);
		stopRobot();
>>>>>>> b7bc67623d694a277a68dae8bd8f18f9e946ee30
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
			leftMotor.rotate(leftWheelDir * Robot.convertAngle(Robot.WHEEL_RAD, Robot.TRACK, degrees+0), true);
			rightMotor.rotate(rightWheelDir * Robot.convertAngle(Robot.WHEEL_RAD, Robot.TRACK, degrees+0), false);
		}
		stopRobot();
	}

	
	/**
<<<<<<< HEAD
	 * Returns the distance value as seen by the side sensor.
	 * 
	 * @return
	 */
	public int fetchSideUS() {
		sideUsDistance.fetchSample(sideUsData, 0);
		return (int) (sideUsData[0] * 100);
	}

	public void travelToTunnelEntrance() {
		// x is the halfway point between both x's, y is the lower-left y because we're
		// coming from green
		double tunnelEntranceX = ((WiFiData.tnURX + WiFiData.tnLLX) / 2.0) * 30.48;
		double tunnelEntranceY = (WiFiData.tnLLY) * 30.48;
		travelTo(tunnelEntranceX, tunnelEntranceY);
	}

	public void travelToBridgeEntrance() {
		// x is the halfway point between both x's, y is the upper-right y because we're
		// coming from red
		double bridgeEntranceX = ((WiFiData.brURX + WiFiData.brLLX) / 2.0) * 30.48;
		double bridgeEntranceY = (WiFiData.brLLY) * 30.48;
		travelTo(bridgeEntranceX, bridgeEntranceY);
	}

	// TODO: finish implementing this method
	public void checkForEndOfArea() {
		int zoneLLX = 0, zoneLLY = 0, zoneURX = 0, zoneURY = 0;
		if (currentZone.equals("green")) {
			zoneLLX = WiFiData.greenLLX;
			zoneLLY = WiFiData.greenLLY;
			zoneURX = WiFiData.greenURX;
			zoneURX = WiFiData.greenURY;
		} else if (currentZone.equals("red")) {
			zoneLLX = WiFiData.redLLX;
			zoneLLY = WiFiData.redLLY;
			zoneURX = WiFiData.redURX;
			zoneURX = WiFiData.redURY;
		}
		// Needs to be implemented as a thread (or find a way to make it run constantly)
		while (true) {
			double currX = odometer.getXYT()[0], currY = odometer.getXYT()[1];
			if (currX - 10 <= zoneLLX || currX + 10 >= zoneURX) {
				stopRobot(); // after stopping, what should it do?
			}
			if (currY - 10 <= zoneLLY || currY + 10 >= zoneURY) {
				stopRobot(); // see above
			}
		}
	}

	public static void setCurrentZone(String colour) {
		currentZone = colour;
	}

	public static void setStartingCorner(int sC) {
		startingCorner = sC;
=======
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
>>>>>>> b7bc67623d694a277a68dae8bd8f18f9e946ee30
	}
}
