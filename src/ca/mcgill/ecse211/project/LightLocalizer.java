package ca.mcgill.ecse211.project;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

/**
 * Class that allows odometer Correction by using tile lines to calculate offset from 
 * theoretical position.
 * @author Volen Mihaylov
 * @author Byran Jay
 * @author Patrick Ghazal
 *
 */
public class LightLocalizer {

	// vehicle constants
	public static int ROTATION_SPEED = 100;
	private double SENSOR_LENGTH = 11.9;

	private Odometer odometer;
	public Navigation navigation;
	// Instantiate the EV3 Color Sensor
	private static final EV3ColorSensor lightSensorLeft = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	private static final EV3ColorSensor lightSensorRight = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
	private float sampleLeft;
	private float sampleRight;

	private SensorMode idColourLeft;
	private SensorMode idColourRight;


	double[] lineData;
/**
 * LightLocalizet constructor
 * @param odometer Odometer object to keep track of position for coordinate related movements
 * @param nav Navigation object to be able to induce movement into the robot
 */
	public LightLocalizer(Odometer odometer, Navigation nav) {

		this.odometer = odometer;

		idColourLeft = lightSensorLeft.getRedMode(); // set the sensor light to red
		idColourRight = lightSensorLeft.getRedMode(); // set the sensor light to red
		lineData = new double[4];
		this.navigation = nav;
	}

	/**
	 * 
	 * This method localizes the robot using the light sensor to precisely move to
	 * the right location
	 * 
	 * @param finalX Final X coordinate that the robot will set
	 * @param finalY Final X coordinate that the robot will set
	 * @param finalTheta Final theta coordinate that the robot will set
	 */
	public void localize() {
		int index = 0;
		if (odometer.getXYT()[3] > 0.25 || odometer.getXYT()[3] < 359.75) {
			navigation.turnTo(0);
		}
		navigation.setSpeed(Robot.LOCALIZATION_SPEED);
		navigation.forward();
		index = 0;
		boolean leftCheck = false;
		boolean rightCheck = true;
		while (leftCheck || rightCheck) {
			sampleLeft = fetchSampleLeft();
			sampleRight = fetchSampleRight();
			if (leftCheck && sampleLeft < 0.38) {
				lineData[0] = odometer.getXYT()[1];
				Sound.beepSequenceUp();
				leftCheck = false;
			}
			if (!rightCheck && sampleRight < 0.38) {
				lineData[1] = odometer.getXYT()[1];
				Sound.beepSequenceUp();
				rightCheck = false;
			}
			Sound.beepSequenceUp();
			index++;
		}
		navigation.stopRobot();
		double deltaOdo = lineData[1]-lineData[2];
		navigation.turnTo(odometer.getXYT()[2]+Math.sin(deltaOdo/Robot.ODOODO));
		odometer.setY(odometer.getXYT()[1]+Robot.ODOWHEEL/2);
		

		index = 0;
		leftCheck = false;
		rightCheck = true;
		while (leftCheck || rightCheck) {
			sampleLeft = fetchSampleLeft();
			sampleRight = fetchSampleRight();
			if (leftCheck && sampleLeft < 0.38) {
				lineData[0] = odometer.getXYT()[0];
				Sound.beepSequenceUp();
				leftCheck = false;
			}
			if (!rightCheck && sampleRight < 0.38) {
				lineData[1] = odometer.getXYT()[0];
				Sound.beepSequenceUp();
				rightCheck = false;
			}
			Sound.beepSequenceUp();
			index++;
		}
		navigation.stopRobot();
		deltaOdo = lineData[1]-lineData[2];
		navigation.turnTo(odometer.getXYT()[2]+Math.sin(deltaOdo/Robot.ODOODO));
		odometer.setY(odometer.getXYT()[0]+Robot.ODOWHEEL/2);
		
		
		
	}

	/**
	 * This method gets the color value of the light sensor
	 * 
	 */
	private float fetchSampleLeft() {
		float[] colorValue = new float[idColourLeft.sampleSize()];
		idColourLeft.fetchSample(colorValue, 0);
		return colorValue[0];
	}


	/**
	 * This method gets the color value of the light sensor
	 * 
	 */
	private float fetchSampleRight() {
		float[] colorValue = new float[idColourRight.sampleSize()];
		idColourRight.fetchSample(colorValue, 0);
		return colorValue[0];
	}
}
