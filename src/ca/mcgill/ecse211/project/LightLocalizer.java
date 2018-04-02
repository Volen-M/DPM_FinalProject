package ca.mcgill.ecse211.project;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

/**
 * Class that allows odometer Correction by using tile lines to calculate offset
 * from theoretical position.
 * 
 * @author Volen Mihaylov
 *
 */
@SuppressWarnings("static-access")
public class LightLocalizer {

	// vehicle constants

	private Odometer odometer;
	public Navigation navigation;
	// Instantiate the EV3 Colour Sensor
	private static final EV3ColorSensor lightSensorLeft = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	private static final EV3ColorSensor lightSensorRight = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
	private float sampleLeft;
	private float sampleRight;

	private SensorMode idColourLeft;
	private SensorMode idColourRight;

	double[] lineData;

	/**
	 * LightLocalizer constructor
	 * 
	 * @param odometer
	 *            Odometer object to keep track of position for coordinate related
	 *            movements
	 * @param nav
	 *            Navigation object to be able to induce movement into the robot
	 */
	public LightLocalizer(Odometer odometer, Navigation nav) {

		this.odometer = odometer;
		idColourLeft = lightSensorLeft.getRedMode(); // set the sensor light to red
		this.navigation = nav;
		idColourRight = lightSensorRight.getRedMode(); // set the sensor light to red
		lineData = new double[4];
	}

	/**
	 * Method to fully localise its position
	 * 
	 * @param int
	 *            integer to determine the type of localisation to be done depending on robots location
	 */
	public void fullLocalize(int type) {
	}

	/**
	 * X Coordinate correction method. Must be called when robot's Light Sensors are at a distance from the line 
	 * of at maximum 2.5 times the distance between the light sensors and the front wheel axle
	 */
	public void localizeX() {
		Navigation.setSpeed(Robot.LOCALIZATION_SPEED);
		boolean leftCheck = true;
		boolean rightCheck = true;
		double oriCoord = odometer.getXYT()[0];
		double oriTheta = odometer.getXYT()[2];
		navigation.forward();
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
			if (Math.abs(oriCoord - odometer.getXYT()[0]) > Robot.LSTOWHEEL * 2.5) {
				navigation.stopRobot();
				Sound.beepSequence();
				return;
			}
		}
		navigation.stopRobot();
		double deltaOdo = lineData[0] - lineData[1];
		double deltaDeg = Math.asin(deltaOdo / Robot.LSTOLS) * 180 / Math.PI;
		if (deltaDeg <= 15) {
			navigation.turnTo(odometer.getXYT()[2] + deltaDeg);
			odometer.setTheta(oriTheta); 
			odometer.setX(oriCoord + deltaOdo / 2 + Robot.LSTOWHEEL);
		}
		else {
			Sound.beepSequenceUp();
			Sound.beepSequence();
			Sound.beepSequenceUp();
		}
	}

	/**
	 * Y Coordinate correction method. Must be called when robot's Light Sensors are at a distance from the line 
	 * of at maximum 2.5 times the distance between the light sensors and the front wheel axle
	 */
	public void localizeY() {
		navigation.setSpeed(Robot.LOCALIZATION_SPEED);
		boolean leftCheck = true;
		boolean rightCheck = true;
		double oriCoord = odometer.getXYT()[1];
		double oriTheta = odometer.getXYT()[2];
		navigation.forward();
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
			if (Math.abs(oriCoord - odometer.getXYT()[1]) > Robot.LSTOWHEEL * 2.5) {
				navigation.stopRobot();
				Sound.beepSequence();
				return;
			}
		}
		navigation.stopRobot();
		double deltaOdo = lineData[0] - lineData[1];
		double deltaDeg = Math.asin(deltaOdo / Robot.LSTOLS) * 180 / Math.PI;
		if (deltaDeg <= 15) {
			navigation.turnTo(odometer.getXYT()[2] + deltaDeg);
			odometer.setTheta(oriTheta);
			odometer.setY(oriCoord + deltaOdo / 2 + Robot.LSTOWHEEL);
		}
		else {
			Sound.beepSequenceUp();
			Sound.beepSequence();
			Sound.beepSequenceUp();
		}
	}


	/**
	 * This method gets the colour value of the left light sensor
	 * 
	 */
	private float fetchSampleLeft() {
		float[] colorValue = new float[idColourLeft.sampleSize()];
		idColourLeft.fetchSample(colorValue, 0);
		return colorValue[0];
	}

	/**
	 * This method gets the colour value of the right light sensor
	 * 
	 */
	private float fetchSampleRight() {
		float[] colorValue = new float[idColourRight.sampleSize()];
		idColourRight.fetchSample(colorValue, 0);
		return colorValue[0];
	}
}
