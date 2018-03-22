package ca.mcgill.ecse211.project;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

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
	private static SensorModes sideUltrasonicSensor;
	private static SampleProvider sideUsDistance;
	private float[] sideUsData;

	private static int distanceSensorToBlock = 2;

	// constructor for navigation
	public Navigation(Odometer odo) {
		this.odometer = odo;
		Robot.setAcceleration(Robot.ACCELERATION);

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

		Robot.setSpeed(Robot.FORWARD_SPEED);
		Robot.rotateByDistance(hypot, 1, 1);

		// stop vehicle
		Robot.stop();
		navigating = false;
	}

	/**
	 * Goes to a block when it is detected in the field
	 * @param searcher : instance of the SearchAndLocalize class
	 */
	private void goToBlock(SearchAndLocalize searcher) {
		int dist = this.usLoc.fetchUS();
		if (dist > distanceSensorToBlock) {
			Robot.rotateByDistance(dist - distanceSensorToBlock, 1, 1);
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
		if (sideDistance < (searcher.lowerLeftX - searcher.upperRightX) / 2 * USLocalizer.TILESIZE + 5) {
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
		Robot.setSpeed(Robot.ROTATE_SPEED);
		
		// if angle is negative, turn to the left
		if (degrees < 0) {
			Robot.rotateByAngle(-1 * degrees, -1, 1);

		} else {
			// angle is positive, turn to the right
			Robot.rotateByAngle(degrees, 1, -1);
		}
		Robot.stop();
		navigating = false;
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
