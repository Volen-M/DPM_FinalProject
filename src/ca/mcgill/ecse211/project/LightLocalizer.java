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
	 * LightLocalizer constructor. Sets the light sensor modes.
	 */
	public LightLocalizer() {

		odometer = Controller.getOdometerInstance();
		navigation = Controller.getNavigationInstance();

		idColourLeft = lightSensorLeft.getRedMode(); // set the sensor light to red
		idColourRight = lightSensorRight.getRedMode(); // set the sensor light to red
		lineData = new double[2];
	}

	/**
	 * Method to fully localize its position
	 * 
	 * @param type
	 *            integer to determine the type of localization to be done depending
	 *            on robots location
	 */
	public void fullLocalize(int corner) {
		boolean leftCheck = true;
		boolean rightCheck = true;

		navigation.forward(Robot.LOCALIZATION_SPEED);
		while (leftCheck || rightCheck) {
			sampleLeft = fetchSampleLeft();
			sampleRight = fetchSampleRight();
			if (leftCheck && sampleLeft < 0.28) {
				navigation.leftMotor.stop();
				Sound.beepSequenceUp();
				leftCheck = false;
			}
			if (rightCheck && sampleRight < 0.28) {
				navigation.rightMotor.stop();
				Sound.beepSequenceUp();
				rightCheck = false;
			}
		}
		if (corner == 0 || corner == 1) {
			odometer.setX(Robot.TILESIZE);
		} else if (corner == 2 || corner == 3) {
			odometer.setX(7 * Robot.TILESIZE);

		}

		if (corner == 0 || corner == 3) {
			navigation.turnTo(90);
			navigation.forward(Robot.LOCALIZATION_SPEED);

			while (leftCheck || rightCheck) {
				sampleLeft = fetchSampleLeft();
				sampleRight = fetchSampleRight();
				if (leftCheck && sampleLeft < 0.28) {
					navigation.leftMotor.stop();
					Sound.beepSequenceUp();
					leftCheck = false;
				}
				if (rightCheck && sampleRight < 0.28) {
					navigation.rightMotor.stop();
					Sound.beepSequenceUp();
					rightCheck = false;
				}
			}

			odometer.setY(Robot.TILESIZE);
		} else if (corner == 1 || corner == 2) {
			navigation.turnTo(270);
			navigation.forward(Robot.LOCALIZATION_SPEED);

			while (leftCheck || rightCheck) {
				sampleLeft = fetchSampleLeft();
				sampleRight = fetchSampleRight();
				if (leftCheck && sampleLeft < 0.28) {
					navigation.leftMotor.stop();
					Sound.beepSequenceUp();
					leftCheck = false;
				}
				if (rightCheck && sampleRight < 0.28) {
					navigation.rightMotor.stop();
					Sound.beepSequenceUp();
					rightCheck = false;
				}
			}
		}
	}

	public double roundDeci(double x) {
		return Math.round(x * 100) / 100;
	}

	/**
	 * Localizes along the x-axis, making sure to end up in the center of the tile.
	 */
	public void localizeXMid() {
		localizeX();
		double xLoc = odometer.getXYT()[0];
		double deg = odometer.getXYT()[2];
		if (deg >= 0 && deg <= 180) {
			navigation.moveBy((Math.floor(xLoc / Robot.TILESIZE) * Robot.TILESIZE + 0.5 * Robot.TILESIZE) - xLoc);
		} else {
			navigation.moveBy(xLoc - (Math.ceil(xLoc / Robot.TILESIZE) * Robot.TILESIZE - 0.5 * Robot.TILESIZE));
		}
	}

	/**
	 * Localizes along the y-axis, making sure to end up in the center of the tile.
	 */
	public void localizeYMid() {
		localizeY();
		double yLoc = odometer.getXYT()[1];
		double deg = odometer.getXYT()[2];
		if (deg >= 270 || deg <= 90) {
			navigation.moveBy((Math.floor(yLoc / Robot.TILESIZE) * Robot.TILESIZE + 0.5 * Robot.TILESIZE) - yLoc);
		} else {
			navigation.moveBy(yLoc - (Math.ceil(yLoc / Robot.TILESIZE) * Robot.TILESIZE - 0.5 * Robot.TILESIZE));
		}
	}

	/**
	 * X Coordinate correction method. Must be called when robot's Light Sensors are
	 * at a distance from the line of at maximum 2.5 times the distance between the
	 * light sensors and the front wheel axle
	 */
	public void localizeX() {
		boolean leftCheck = true;
		boolean rightCheck = true;
		double oriCoord = odometer.getXYT()[0];
		double oriTheta = odometer.getXYT()[2];
		lineData = new double[2];
		navigation.forward(Robot.LOCALIZATION_SPEED);
		while (leftCheck || rightCheck) {
			sampleLeft = fetchSampleLeft();
			sampleRight = fetchSampleRight();
			if (leftCheck && sampleLeft < 0.28) {
				lineData[0] = odometer.getXYT()[0];
				Sound.beepSequenceUp();
				leftCheck = false;
			}
			if (rightCheck && sampleRight < 0.28) {
				lineData[1] = odometer.getXYT()[0];
				Sound.beepSequenceUp();
				rightCheck = false;
			}
			if (Math.abs(oriCoord - odometer.getXYT()[0]) > Robot.TILESIZE * 1.1) {
				double fix = rightCheck == false ? lineData[1] : lineData[0];
				double xLoc = odometer.getXYT()[0];
				if (oriTheta >= 0 && oriTheta <= 180) {
					odometer.setX(Math.floor(xLoc / Robot.TILESIZE) + (xLoc - fix) + Robot.LSTOWHEEL);
					Controller.logger();
				} else {
					odometer.setX(Math.ceil(xLoc / Robot.TILESIZE) + (fix - xLoc) - Robot.LSTOWHEEL);
				}
				navigation.stopRobot();
				Sound.beepSequence();
				return;
			}
		}
		navigation.stopRobot();
		double deltaOdo = oriTheta <= 180 ? lineData[0] - lineData[1] : lineData[1] - lineData[0];
		double deltaDeg = Math.asin(deltaOdo / Robot.LSTOLS) * 180 / Math.PI;
		double xLoc = odometer.getXYT()[0];
		navigation.turnTo(odometer.getXYT()[2] + deltaDeg);
		odometer.setTheta(oriTheta);
		if (oriTheta >= 0 && oriTheta <= 180) {
			odometer.setX(Math.floor(xLoc / Robot.TILESIZE) * Robot.TILESIZE
					+ Math.abs(deltaOdo) / 2 * Robot.TRACKOVERLS + Robot.LSTOWHEEL);
		} else {
			odometer.setX(Math.ceil(xLoc / Robot.TILESIZE) * Robot.TILESIZE - Math.abs(deltaOdo) / 2 * Robot.TRACKOVERLS
					- Robot.LSTOWHEEL);
		}
	}

	/**
	 * Y Coordinate correction method. Must be called when robot's Light Sensors are
	 * at a distance from the line of at maximum 1.1 times the Tilesize
	 */
	public void localizeY() {
		boolean leftCheck = true;
		boolean rightCheck = true;
		double oriCoord = odometer.getXYT()[1];
		double oriTheta = odometer.getXYT()[2];
		lineData = new double[2];
		navigation.forward(Robot.LOCALIZATION_SPEED);
		while (leftCheck || rightCheck) {
			sampleLeft = fetchSampleLeft();
			sampleRight = fetchSampleRight();
			if (leftCheck && sampleLeft < 0.28) {
				lineData[0] = odometer.getXYT()[1];
				Sound.beepSequenceUp();
				leftCheck = false;
			}
			if (rightCheck && sampleRight < 0.28) {
				lineData[1] = odometer.getXYT()[1];
				Sound.beepSequenceUp();
				rightCheck = false;
			}
			if (Math.abs(oriCoord - odometer.getXYT()[1]) > Robot.TILESIZE * 1.1) {
				double fix = rightCheck == false ? lineData[1] : lineData[0];
				double yLoc = odometer.getXYT()[1];
				if (oriTheta >= 270 && oriTheta <= 90) {
					odometer.setX(Math.floor(yLoc / Robot.TILESIZE) + (yLoc - fix) + Robot.LSTOWHEEL);
				} else {
					odometer.setX(Math.ceil(yLoc / Robot.TILESIZE) + (fix - yLoc) - Robot.LSTOWHEEL);
				}
				navigation.stopRobot();
				Sound.beepSequence();
				return;
			}
		}

		navigation.stopRobot();
		double deltaOdo = oriTheta <= 90 || oriTheta >= 270 ? lineData[0] - lineData[1] : lineData[1] - lineData[0];
		double deltaDeg = Math.asin(deltaOdo / Robot.LSTOLS) * 180 / Math.PI;
		double yLoc = odometer.getXYT()[1];
		navigation.turnTo(odometer.getXYT()[2] + deltaDeg);
		odometer.setTheta(oriTheta);
		if (oriTheta >= 270 || oriTheta <= 90) {
			odometer.setY(
					Math.floor(yLoc / Robot.TILESIZE) * Robot.TILESIZE + Math.abs(deltaOdo) / 2 + Robot.LSTOWHEEL);
		} else {
			odometer.setY(Math.ceil(yLoc / Robot.TILESIZE) * Robot.TILESIZE - Math.abs(deltaOdo) / 2 - Robot.LSTOWHEEL);
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
}
