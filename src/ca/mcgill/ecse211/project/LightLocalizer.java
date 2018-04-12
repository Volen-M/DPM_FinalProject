package ca.mcgill.ecse211.project;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

/**
 * Class that corrects odometer data by using tile lines to calculate offset
 * from theoretical position.
 * 
 * @author Volen Mihaylov
 *
 */
@SuppressWarnings("static-access")
public class LightLocalizer {

	// vehicle constants
	private Odometer odometer;
	private Navigation navigation;

	// Instantiate the EV3 Colour Sensor
	private static final EV3ColorSensor lightSensorLeft = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	private static final EV3ColorSensor lightSensorRight = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
	private float sampleLeft;
	private float sampleRight;

	private SensorMode idColourLeft;
	private SensorMode idColourRight;

	double[] lineData;

	/**
	 * LightLocalizer constructor. Retrieves odometer and navigation instances. Sets
	 * the light sensor modes.
	 */
	public LightLocalizer() {

		odometer = Controller.getOdometerInstance();
		navigation = Controller.getNavigationInstance();

		idColourLeft = lightSensorLeft.getRedMode(); // set the sensor light to red
		idColourRight = lightSensorRight.getRedMode(); // set the sensor light to red
		lineData = new double[2];
	}

	public double roundDeci(double x) {
		return Math.round(x * 100) / 100;
	}

	/**
	 * Localize along the x-axis, to correct the x-coordinate of the robot.
	 * 
	 * @throws OdometerExceptions
	 * @throws InterruptedException
	 */
	public void localizeX() throws OdometerExceptions, InterruptedException {
		double oriCoord = odometer.getXYT()[0];
		double oriTheta = odometer.getXYT()[2];
		boolean failedTurn = false;
		double goal = 0;
		if (oriTheta >= 0 && oriTheta <= 180) {
			goal = Math.ceil(((oriCoord - (Robot.LSTOWHEEL)) / Robot.TILESIZE)) * Robot.TILESIZE + Robot.LSTOWHEEL;
		} else {
			goal = Math.floor(((oriCoord + (Robot.LSTOWHEEL)) / Robot.TILESIZE)) * Robot.TILESIZE - Robot.LSTOWHEEL;

		}
		navigation.forward(Robot.FORWARD_SPEED);
		while (true) {
			sampleLeft = fetchSampleLeft();
			sampleRight = fetchSampleRight();
			if (sampleLeft < 0.28) {
				odometer.setX(goal);
				navigation.stopRobot();
				Thread.sleep(250);
				while (true) {
					sampleRight = fetchSampleRight();
					if (sampleRight < 0.28) {
						if (navigation.isNavigating()) {
							navigation.stopRobot();
						}
						odometer.setTheta(oriTheta);
						return;
					}
					if (!navigation.isNavigating()) {
						navigation.rotateRightWheel(Robot.LOCALIZATION_SPEED);
					}
					if (!failedTurn && Math.abs(odometer.getXYT()[2] - oriTheta) > 35
							&& Math.abs(odometer.getXYT()[2] - oriTheta) < 325) {
						Thread.sleep(250);
						navigation.rotateRightWheelBack(Robot.LOCALIZATION_SPEED);
						failedTurn = true;
					}
				}
			}
			if (sampleRight < 0.28) {
				odometer.setX(goal);
				navigation.stopRobot();
				Thread.sleep(250);
				// navigation.moveBy(-1.00);
				while (true) {
					sampleLeft = fetchSampleLeft();
					if (sampleLeft < 0.28) {
						if (navigation.isNavigating()) {
							navigation.stopRobot();
						}
						odometer.setTheta(oriTheta);
						return;
					}
					if (!navigation.isNavigating()) {
						navigation.rotateLeftWheel(Robot.LOCALIZATION_SPEED);
					}
					if (!failedTurn && Math.abs(odometer.getXYT()[2] - oriTheta) > 35
							&& Math.abs(odometer.getXYT()[2] - oriTheta) < 325) {
						Thread.sleep(250);
						navigation.rotateLeftWheelBack(Robot.LOCALIZATION_SPEED);
						failedTurn = true;
					}
				}
			}
		}
	}

	/**
	 * Localize along the y-axis, to correct the y-coordinate of the robot.
	 * 
	 * @throws OdometerExceptions
	 * @throws InterruptedException
	 */
	public void localizeY() throws OdometerExceptions, InterruptedException {
		double oriCoord = odometer.getXYT()[1];
		double oriTheta = odometer.getXYT()[2];
		boolean failedTurn = false;
		double goal = 0;
		if (oriTheta >= 270 || oriTheta <= 90) {
			goal = Math.ceil(((oriCoord - (Robot.LSTOWHEEL)) / Robot.TILESIZE)) * Robot.TILESIZE + Robot.LSTOWHEEL;
		} else {
			goal = Math.floor(((oriCoord + (Robot.LSTOWHEEL)) / Robot.TILESIZE)) * Robot.TILESIZE - Robot.LSTOWHEEL;

		}
		navigation.forward(Robot.FORWARD_SPEED);
		while (true) {
			sampleLeft = fetchSampleLeft();
			sampleRight = fetchSampleRight();
			if (sampleLeft < 0.28) {
				odometer.setY(goal);
				navigation.stopRobot();
				Thread.sleep(250);
				// navigation.moveBy(-1.00);
				while (true) {
					sampleRight = fetchSampleRight();
					if (sampleRight < 0.28) {
						if (navigation.isNavigating()) {
							navigation.stopRobot();
						}
						odometer.setTheta(oriTheta);
						return;
					}
					if (!navigation.isNavigating()) {
						navigation.rotateRightWheel(Robot.LOCALIZATION_SPEED);
					}
					if (!failedTurn && Math.abs(odometer.getXYT()[2] - oriTheta) > 35
							&& Math.abs(odometer.getXYT()[2] - oriTheta) < 325) {
						Thread.sleep(250);
						navigation.rotateRightWheelBack(Robot.LOCALIZATION_SPEED);
						failedTurn = true;
					}
				}
			}
			if (sampleRight < 0.28) {
				odometer.setY(goal);
				navigation.stopRobot();
				Thread.sleep(250);
				// navigation.moveBy(-1.00);
				while (true) {
					sampleLeft = fetchSampleLeft();
					if (sampleLeft < 0.28) {
						if (navigation.isNavigating()) {
							navigation.stopRobot();
						}
						odometer.setTheta(oriTheta);
						return;
					}
					if (!navigation.isNavigating()) {
						navigation.rotateLeftWheel(Robot.LOCALIZATION_SPEED);
					}
					if (!failedTurn && Math.abs(odometer.getXYT()[2] - oriTheta) > 35
							&& Math.abs(odometer.getXYT()[2] - oriTheta) < 325) {
						Thread.sleep(250);
						navigation.rotateLeftWheelBack(Robot.LOCALIZATION_SPEED);
						failedTurn = true;
					}
				}
			}
		}
	}

	/**
	 * Fetches the value read by the left color sensor, for localization on the left
	 * wheel.
	 * 
	 * @return the value read
	 */
	private float fetchSampleLeft() {
		float[] colorValue = new float[idColourLeft.sampleSize()];
		idColourLeft.fetchSample(colorValue, 0);
		return colorValue[0];
	}

	/**
	 * Fetches the value read by the right color sensor, for localization on the
	 * right wheel.
	 * 
	 * @return the value read
	 */
	private float fetchSampleRight() {
		float[] colorValue = new float[idColourRight.sampleSize()];
		idColourRight.fetchSample(colorValue, 0);
		return colorValue[0];
	}
}
