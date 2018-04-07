package ca.mcgill.ecse211.project;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;

/**
 * Class containing all robot movement related behaviors. Extends Thread.
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

	private boolean navigating = false;

	public USLocalizer usLoc;

	// private static final Port usSidePort = LocalEV3.get().getPort("S3");

	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor backMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	public static final EV3LargeRegulatedMotor usMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3ColorSensor lightSensorLeft = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	private static final EV3ColorSensor lightSensorRight = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
	private float sampleLeft;
	private float sampleRight;

	private SensorMode idColourLeft;
	private SensorMode idColourRight;
	double[] lineData;

	/**
	 * Navigation constructor
	 */
	public Navigation() {
		this.odometer = Controller.getOdometerInstance();
		setAcceleration(Robot.ACCELERATION);
		setSpeed(Robot.FORWARD_SPEED);

		idColourLeft = lightSensorLeft.getRedMode(); // set the sensor light to red
		idColourRight = lightSensorRight.getRedMode(); // set the sensor light to red
		lineData = new double[2];


	}

	/**
	 * A method to drive our vehicle to a certain Cartesian coordinate
	 * 
	 * @param x
	 *            X-Coordinate of destination
	 * @param y
	 *            Y-Coordinate of destination
	 */
	public void travelTo(double x, double y) {
		

		currX = odometer.getXYT()[0];
		currY = odometer.getXYT()[1];

		deltaX = x - currX;
		deltaY = y - currY;

		// Calculate the angle to turn around
		double mDegrees = Math.atan2(deltaX, deltaY) / Math.PI * 180;
		double hypot = Math.hypot(deltaX, deltaY);

		// Turn to the correct angle towards the endpoint
		turnTo(mDegrees);

		setSpeed(Robot.FORWARD_SPEED);
		rotateByDistance(hypot, 1, 1);

		// stopRobot vehicle
		stopRobot();
		
	}

	/**
	 * A method to turn our vehicle to a certain orientation
	 * 
	 * @param degrees desired orientation
	 */
	public void turnTo(double degrees) {

		
		// ensures minimum angle for turning
		degrees = degrees - odometer.getXYT()[2];
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
		
	}

	/**
	 * Moves robot forward or back by certain amount
	 * 
	 * @param distance
	 *            Distance to move by
	 */
	public void moveBy(double distance) {
		
		setSpeed(Robot.FORWARD_SPEED);
		if (distance >= 0) {
			rotateByDistance(distance, 1, 1);
		} else {
			rotateByDistance(-1*distance, -1, -1);

		}
		
	}

	/**
	 * Freely sets both wheels forward indefinitely.
	 */
	public void forward() {
		navigating = true;
		setSpeed(Robot.FORWARD_SPEED);
		leftMotor.forward();
		rightMotor.forward();
	}

	public void forward(int speed) {
		navigating = true;
		setSpeed(speed);
		leftMotor.forward();
		rightMotor.forward();
	}
	/**
	 * Freely rotates the robot clockwise indefinitely.
	 */
	public void rotateClockWise() {
		
		setSpeed(Robot.FORWARD_SPEED);
		leftMotor.forward();
		rightMotor.backward();
	}

	/**
	 * Freely rotates the robot counter-clockwise indefinitely.
	 */
	public void rotateCounterClockWise() {
		
		setSpeed(Robot.FORWARD_SPEED);
		leftMotor.backward();
		rightMotor.forward();
	}

	/**
	 * Stops both wheels.
	 */
	public void stopRobot() {
		navigating = false;
		leftMotor.stop(true);
		rightMotor.stop(false);
		
	}

	/**
	 * Travel distance dist.
	 * 
	 * @param dist
	 *            distance to travel
	 * @param leftWheelDir
	 *            1 for the left wheel to go forward, -1 for backward
	 * @param rightWheelDir
	 *            1 for the right wheel to go forward, -1 for backward
	 */
	public void rotateByDistance(double dist, int leftWheelDir, int rightWheelDir) {
		
		leftMotor.rotate(leftWheelDir * Robot.convertDistance(Robot.WHEEL_RAD, dist), true);
		rightMotor.rotate(rightWheelDir * Robot.convertDistance(Robot.WHEEL_RAD, dist), false);
		stopRobot();
		
	}

	/**
	 * Turn by a certain angle in a certain direction
	 * 
	 * @param degrees
	 *            angle to turn
	 * @param leftWheelDir
	 *            1 for the left wheel to go forward, -1 for backward
	 * @param rightWheelDir
	 *            1 for the right wheel to go forward, -1 for backward
	 */
	public void rotateByAngle(double degrees, int leftWheelDir, int rightWheelDir) {
		

		if (leftWheelDir == 1 && rightWheelDir == -1) {
			leftMotor.rotate(leftWheelDir * Robot.convertAngle(Robot.WHEEL_RAD, Robot.TRACK, degrees + 0), true);
			rightMotor.rotate(rightWheelDir * Robot.convertAngle(Robot.WHEEL_RAD, Robot.TRACK, degrees + 0), false);

		} else {
			leftMotor.rotate(leftWheelDir * Robot.convertAngle(Robot.WHEEL_RAD, Robot.TRACK, degrees + 0), true);
			rightMotor.rotate(rightWheelDir * Robot.convertAngle(Robot.WHEEL_RAD, Robot.TRACK, degrees + 0), false);
		}
		stopRobot();
		
	}

	/**
	 * Update the robot's acceleration
	 * 
	 * @param acc
	 *            desired acceleration
	 */
	public static void setAcceleration(int acc) {
		leftMotor.setAcceleration(acc);
		rightMotor.setAcceleration(acc);
	}

	public void travelToLoc(double x, double y) {
		double[] xyt = odometer.getXYT();
		double xloc = xyt[0];
		double yloc = xyt[1];
		boolean leftData = false;
		boolean rightData = false;
		double fix = 0;
		double decimal = 0;
//		if (yloc < y) {
//			turnTo(0);
//		}
//		else if (yloc > y ) {
//			turnTo(180);
//		}
		if (xloc < x) {
			turnTo(90);
			while (odometer.getXYT()[0]< x) {
				sampleRight = fetchSampleRight();
				sampleLeft = fetchSampleLeft();
				if (navigating==false) {
					forward();
				}
				if (sampleLeft < 0.28) {
					lineData[0] = odometer.getXYT()[0];
					Sound.beepSequenceUp();
					leftData = true;
//					fix = lineData[0]-Robot.LSTOWHEEL;
//					decimal = (fix/Robot.TILESIZE)%1;
//					if (decimal < 0.5) {
//						odometer.setX(lineData[0]-decimal*Robot.TILESIZE);
//					} else {
//						odometer.setX(lineData[0]+decimal*Robot.TILESIZE);
//					}
				}
				if (sampleRight < 0.28) {
					lineData[1] = odometer.getXYT()[0];
					Sound.beepSequenceUp();
					rightData = true;
					fix = lineData[0]-Robot.LSTOWHEEL;
					decimal = (fix/Robot.TILESIZE)%1;
//					if (decimal < 0.5) {
//						odometer.setX(lineData[0]-decimal*Robot.TILESIZE);
//					} else {
//						odometer.setX(lineData[0]+decimal*Robot.TILESIZE);
//					}
				}
				if (leftData && rightData) {
					fix = lineData[0] - lineData[1];
					stopRobot();
					turnTo(90 +  Math.asin(fix / Robot.LSTOLS) * 180 / Math.PI);
					odometer.setTheta(90);
					leftData = false;
					rightData = false;
				}
			}
			stopRobot();
		}
	}


	/**
	 * This method gets the colour value detected by the left light sensor
	 */
	private float fetchSampleLeft() {
		float[] colorValue = new float[idColourLeft.sampleSize()];
		idColourLeft.fetchSample(colorValue, 0);
		return colorValue[0];
	}

	/**
	 * This method gets the colour value detected by the right light sensor
	 */
	private float fetchSampleRight() {
		float[] colorValue = new float[idColourRight.sampleSize()];
		idColourRight.fetchSample(colorValue, 0);
		return colorValue[0];
	}
	
	/**
	 * Update the robot's speed.
	 * 
	 * @param sp
	 *            desired speed
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
	 * @return navigating boolean value, true if the robot is on the move
	 */
	boolean isNavigating() throws OdometerExceptions {
		return navigating;
	}

	/**
	 * Travel to the entrance of the tunnel.
	 */
	public void travelToTunnelEntrance() {
		// x is the halfway point between both x's, y is the lower-left y because we're
		// coming from green (- 5 for not starting right on edge of tunnel)
		double tunnelEntranceX = ((WiFiData.tnURX + WiFiData.tnLLX) / 2.0) * 30.48;
		double tunnelEntranceY = (WiFiData.tnLLY) * 30.48 - 5;
		travelTo(tunnelEntranceX, tunnelEntranceY);
	}

	/**
	 * Travel to the entrance of the bridge.
	 */
	public void travelToBridgeEntrance() {
		// x is the halfway point between both x's, y is the upper-right y because we're
		// coming from red (+ 5 for not starting right on edge of bridge)
		double bridgeEntranceX = ((WiFiData.brURX + WiFiData.brLLX) / 2.0) * 30.48;
		double bridgeEntranceY = (WiFiData.brURY) * 30.48 + 5;
		travelTo(bridgeEntranceX, bridgeEntranceY);
	}

}
