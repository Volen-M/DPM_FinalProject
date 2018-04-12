package ca.mcgill.ecse211.project;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Class containing all robot movement related behaviors. Extends Thread.
 * 
 * @author Volen Mihaylov
 * @author Patrick Ghazal
 * @author Bryan Jay
 */
public class Navigation extends Thread {

	private Odometer odometer;
	private LightLocalizer lightLocalizer;
	public USLocalizer usLocalizer;

	private double deltaX;
	private double deltaY;

	// current location of the vehicle
	private double currX;
	private double currY;

	private boolean navigating = false;

	// private static final Port usSidePort = LocalEV3.get().getPort("S3");

	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor backMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	public static final EV3LargeRegulatedMotor usMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

	/**
	 * Navigation constructor. Sets odometer, as well as robot speed and
	 * acceleration.
	 */
	public Navigation() {
		this.odometer = Controller.getOdometerInstance();
		setAcceleration(Robot.ACCELERATION);
		setSpeed(Robot.FORWARD_SPEED);
	}

	/**
	 * Sets localizers instances. Called from the Controller class.
	 */
	public void setLocalizers() {
		this.lightLocalizer = Controller.getLightLocalizerInstance();
		this.usLocalizer = Controller.getUSLocalizerInstance();
	}

	/**
	 * Travel to a point of the grid, while localizing at each grid line, by moving
	 * along one axis at a time
	 * 
	 * @param gridX
	 *            x-coordinate of the destination (in grid units)
	 * @param gridY
	 *            y-coordinate of the destination (in grid units)
	 * @param firstY
	 *            travel along the y axis first if true
	 * @throws OdometerExceptions
	 * @throws InterruptedException
	 */
	public void travelToAdvGrid(double gridX, double gridY, boolean firstY)
			throws OdometerExceptions, InterruptedException {
		double[] xyt = odometer.getXYT();
		double currX = xyt[0];
		double currY = xyt[1];
		if (firstY) {
			travelToAdvGridY(gridY, currY);
			Thread.sleep(250);
			travelToAdvGridX(gridX, currX);
		} else {
			travelToAdvGridX(gridX, currX);
			Thread.sleep(250);
			travelToAdvGridY(gridY, currY);
		}
	}

	/**
	 * Travel to a certain x-coordinate, at the same y-coordinate, while localizing
	 * at each grid line
	 * 
	 * @param gridX
	 *            x-coordinate of the destination (in grid units)
	 * @param currX
	 *            current x-position (in real distances)
	 * @throws OdometerExceptions
	 * @throws InterruptedException
	 */
	private void travelToAdvGridX(double gridX, double currX) throws OdometerExceptions, InterruptedException {
		double lineSpot = 0;
		if (currX < gridX * Robot.TILESIZE) {
			turnTo(90);
			Thread.sleep(250);
			while (odometer.getXYT()[0] < (gridX - 1) * Robot.TILESIZE) {
				lightLocalizer.localizeX();
				lineSpot = odometer.getXYT()[0];
				while (Math.abs(odometer.getXYT()[0] - lineSpot) <= 5) {
					if (!navigating) {
						forward(Robot.FORWARD_SPEED);
					}
				}
			}
			while (odometer.getXYT()[0] < gridX * Robot.TILESIZE) {
				if (!navigating) {
					forward(Robot.FORWARD_SPEED);
				}
			}
		} else if (currX > gridX * Robot.TILESIZE) {
			turnTo(270);
			Thread.sleep(250);
			while (odometer.getXYT()[0] > (gridX + 1) * Robot.TILESIZE) {
				lightLocalizer.localizeX();
				lineSpot = odometer.getXYT()[0];
				while (Math.abs(odometer.getXYT()[0] - lineSpot) <= 5) {
					if (!navigating) {
						forward(Robot.FORWARD_SPEED);
					}
				}
			}
			while (odometer.getXYT()[0] > gridX * Robot.TILESIZE) {
				if (!navigating) {
					forward(Robot.FORWARD_SPEED);
				}
			}
		}
		stopRobot();

	}

	/**
	 * Travel to a certain y-coordinate, at the same x-coordinate, while localizing
	 * at each grid line
	 * 
	 * @param gridY
	 *            y-coordinate of the destination (in grid units)
	 * @param currY
	 *            current y-position (in real distances)
	 * @throws OdometerExceptions
	 * @throws InterruptedException
	 */
	private void travelToAdvGridY(double gridY, double currY) throws OdometerExceptions, InterruptedException {
		double lineSpot = 0;
		if (currY < gridY * Robot.TILESIZE) {
			turnTo(0);
			Thread.sleep(250);
			while (odometer.getXYT()[1] < (gridY - 1) * Robot.TILESIZE) {
				lightLocalizer.localizeY();
				lineSpot = odometer.getXYT()[1];
				while (Math.abs(odometer.getXYT()[1] - lineSpot) <= 5) {
					if (!navigating) {
						forward(Robot.FORWARD_SPEED);
					}
				}
				// moveByGrid(0.25, Robot.LOCALIZATION_SPEED);
			}
			while (odometer.getXYT()[1] < gridY * Robot.TILESIZE) {
				if (!navigating) {
					forward(Robot.FORWARD_SPEED);
				}
			}
		} else if (currY > gridY * Robot.TILESIZE) {
			turnTo(180);
			Thread.sleep(250);
			while (odometer.getXYT()[1] > (gridY + 1) * Robot.TILESIZE) {
				lightLocalizer.localizeY();
				lineSpot = odometer.getXYT()[1];
				while (Math.abs(odometer.getXYT()[1] - lineSpot) <= 5) {
					if (!navigating) {
						forward(Robot.FORWARD_SPEED);
					}
				}
				// moveByGrid(0.25, Robot.LOCALIZATION_SPEED);
			}
			while (odometer.getXYT()[1] > gridY * Robot.TILESIZE) {
				if (!navigating) {
					forward(Robot.FORWARD_SPEED);
				}
			}
		}
		stopRobot();
	}

	/**
	 * Travel to a point of the grid, while localizing at each grid line, by moving
	 * along the y and x axis in that order
	 * 
	 * @param gridX
	 *            x-coordinate of the destination (in grid units)
	 * @param gridY
	 *            y-coordinate of the destination (in grid units)
	 * @deprecated Since version 06.07.00
	 * @throws OdometerExceptions
	 * @throws InterruptedException
	 */
	public void travelToAdv(double gridX, double gridY) throws OdometerExceptions, InterruptedException {
		double[] xyt = odometer.getXYT();
		double currX = xyt[0];
		double currY = xyt[1];
		double lineSpot = xyt[2];
		if (currY < gridY * Robot.TILESIZE) {
			turnTo(0);
			Thread.sleep(250);
			while (odometer.getXYT()[1] < (gridY - 1) * Robot.TILESIZE) {
				lightLocalizer.localizeY();
				lineSpot = odometer.getXYT()[1];
				while (Math.abs(odometer.getXYT()[1] - lineSpot) <= 5) {
					if (!navigating) {
						forward(Robot.FORWARD_SPEED);
					}
				}
				// moveByGrid(0.25, Robot.LOCALIZATION_SPEED);
			}
			while (odometer.getXYT()[1] < gridY * Robot.TILESIZE) {
				if (!navigating) {
					forward(Robot.FORWARD_SPEED);
				}
			}
			stopRobot();
		} else if (currY > gridY * Robot.TILESIZE) {
			turnTo(180);
			Thread.sleep(250);
			while (odometer.getXYT()[1] > (gridY + 1) * Robot.TILESIZE) {
				lightLocalizer.localizeY();
				lineSpot = odometer.getXYT()[1];
				while (Math.abs(odometer.getXYT()[1] - lineSpot) <= 5) {
					if (!navigating) {
						forward(Robot.FORWARD_SPEED);
					}
				}
				// moveByGrid(0.25, Robot.LOCALIZATION_SPEED);
			}
			while (odometer.getXYT()[1] > gridY * Robot.TILESIZE) {
				if (!navigating) {
					forward(Robot.FORWARD_SPEED);
				}
			}
		}
		Thread.sleep(250);
		if (currX < gridX * Robot.TILESIZE) {
			turnTo(90);
			Thread.sleep(250);
			while (odometer.getXYT()[0] < (gridX - 1) * Robot.TILESIZE) {
				lightLocalizer.localizeX();
				lineSpot = odometer.getXYT()[0];
				while (Math.abs(odometer.getXYT()[0] - lineSpot) <= 5) {
					if (!navigating) {
						forward(Robot.FORWARD_SPEED);
					}
				}
				// moveByGrid(0.25, Robot.LOCALIZATION_SPEED);
			}
			while (odometer.getXYT()[0] < gridX * Robot.TILESIZE) {
				if (!navigating) {
					forward(Robot.FORWARD_SPEED);
				}
			}
		} else if (currX > gridX * Robot.TILESIZE) {
			turnTo(270);
			Thread.sleep(250);
			while (odometer.getXYT()[0] > (gridX + 1) * Robot.TILESIZE) {
				lightLocalizer.localizeX();
				lineSpot = odometer.getXYT()[0];
				while (Math.abs(odometer.getXYT()[0] - lineSpot) <= 5) {
					if (!navigating) {
						forward(Robot.FORWARD_SPEED);
					}
				}
				// moveByGrid(0.25, Robot.LOCALIZATION_SPEED);
			}
			while (odometer.getXYT()[0] > gridX * Robot.TILESIZE) {
				if (!navigating) {
					forward();
				}
			}
		}
		stopRobot();
	}

	/**
	 * A method to drive our vehicle to a certain Cartesian coordinate by going
	 * directly to it. No localization is done in this method.
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
	 * @param degrees
	 *            desired orientation
	 */
	public void turnTo(double degrees) {
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
	 * Moves robot forward or back by a certain distance
	 * 
	 * @param distance
	 *            Distance to move by
	 */
	public void moveBy(double distance) {
		moveBy(distance, Robot.FORWARD_SPEED);
	}

	/**
	 * Moves robot forward or back by a certain distance, at a specific speed
	 * 
	 * @param distance
	 *            Distance to move by (real distance)
	 * @param speed
	 *            Speed at which to travel
	 */
	public void moveBy(double distance, int speed) {
		setSpeed(speed);
		if (distance >= 0) {
			rotateByDistance(distance, 1, 1);
		} else {
			rotateByDistance(-1 * distance, -1, -1);

		}
	}

	/**
	 * Move robot forward or back by a certain distance
	 * 
	 * @param grids
	 *            Distance to move by (in grid units)
	 */
	public void moveByGrid(double grids) {
		moveBy(grids * Robot.TILESIZE);
	}

	/**
	 * Move robot forward or back by a certain distance, at a specific speed.
	 * 
	 * @param grids
	 *            Distance to move by (in grid units)
	 * @param speed
	 *            Speed at which to move
	 */
	public void moveByGrid(double grids, int speed) {
		moveBy(grids * Robot.TILESIZE, speed);
	}

	/**
	 * Freely sets both wheels forward indefinitely.
	 */
	public void forward() {
		forward(Robot.FORWARD_SPEED);
	}

	/**
	 * Freely sets both wheels forward indefinity, at a certain speed.
	 * 
	 * @param speed
	 */
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
		navigating = true;
		setSpeed(250);
		leftMotor.forward();
		rightMotor.backward();
	}

	/**
	 * Freely rotates the robot counter-clockwise indefinitely.
	 */
	public void rotateCounterClockWise() {
		navigating = true;
		setSpeed(250);
		leftMotor.backward();
		rightMotor.forward();
	}

	/**
	 * Sets the left wheel forward, at a specific speed
	 * 
	 * @param speed
	 *            Speed of the left wheel
	 */
	public void rotateLeftWheel(int speed) {
		navigating = true;
		setSpeed(speed);
		leftMotor.forward();
	}

	/**
	 * Sets the left wheel backward, at a specific speed
	 * 
	 * @param speed
	 *            Speed of the left wheel
	 */
	public void rotateLeftWheelBack(int speed) {
		navigating = true;
		setSpeed(speed);
		leftMotor.backward();

	}

	/**
	 * Sets the right wheel forward, at a specific speed
	 * 
	 * @param speed
	 *            Speed of the right wheel
	 */
	public void rotateRightWheel(int speed) {
		navigating = true;
		setSpeed(speed);
		rightMotor.forward();
	}

	/**
	 * Sets the right wheel backward, at a specific speed
	 * 
	 * @param speed
	 *            Speed of the right wheel.
	 */
	public void rotateRightWheelBack(int speed) {
		navigating = true;
		setSpeed(speed);
		rightMotor.backward();
	}

	/**
	 * Stops both wheels.
	 */
	public void stopRobot() {
		rightMotor.stop(true);
		leftMotor.stop(false);
		navigating = false;
	}

	/**
	 * Travels a certain distance, in a certain direction.
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
	 * Rotate the front US Sensor for color block identification.
	 * 
	 * @param flagSearch
	 *            indicates whether the robot is searching for a flag
	 */
	public static void motorSearch(boolean flagSearch) {
		if (flagSearch) {
			usMotor.rotate(-92);
		} else {
			usMotor.rotate(92);
		}
	}

	/**
	 * Update the robot's acceleration.
	 * 
	 * @param acc
	 *            desired acceleration
	 */
	public static void setAcceleration(int acc) {
		leftMotor.setAcceleration(acc);
		rightMotor.setAcceleration(acc);
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
	 * Lowers back wheels for obstacle traversal.
	 */
	public static void landingGearOn() {
		backMotor.setSpeed(Robot.GEAR_SPEED);
		backMotor.setAcceleration(Robot.GEAR_ACCELERATION);

		backMotor.rotate(195);
	}

	/**
	 * Lifts back wheels after obstacle traversal.
	 */
	public static void landingGearOff() {
		backMotor.setSpeed(Robot.GEAR_SPEED);
		backMotor.setAcceleration(Robot.GEAR_ACCELERATION);

		backMotor.rotate(-195);
	}

	/**
	 * A method to determine whether another thread has called navigation methods.
	 * 
	 * @return boolean true if the robot is on the move
	 */
	boolean isNavigating() throws OdometerExceptions {
		return navigating;
	}

}
