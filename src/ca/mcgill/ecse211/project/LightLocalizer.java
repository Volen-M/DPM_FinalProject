package ca.mcgill.ecse211.project;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

public class LightLocalizer {

	// vehicle constants
	public static int ROTATION_SPEED = 100;
	private double SENSOR_LENGTH = 11.9;

	private Odometer odometer;
	public Navigation navigation;
	// Instantiate the EV3 Color Sensor
	private static final EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	private float sample;
	private float prevSample;
	private float deltaSample;

	private SensorMode idColour;

	double[] lineData;

	public LightLocalizer(Odometer odometer, Navigation nav) {

		this.odometer = odometer;
		prevSample = 0;

		idColour = lightSensor.getRedMode(); // set the sensor light to red
		lineData = new double[4];
		this.navigation = nav;
	}

	/**
	 * This method localizes the robot using the light sensor to precisely move to
	 * the right location
	 */
	public void localize(double finalX, double finalY, double finalTheta) {

		int index = 0;
		Robot.setSpeed(ROTATION_SPEED);

		// ensure that we are close to origin before rotating
		moveToIntersection();

		// Scan all four lines and record our angle
		while (index < 4) {

			Robot.rotateClockWise();

			sample = fetchSample();

			if (sample < 0.38) {
				lineData[index] = odometer.getXYT()[2];
				Sound.beepSequenceUp();
				index++;
			}
		}

		Robot.stop();

		double deltax, deltay, thetax, thetay;

		// calculate our location from 0 using the calculated angles
		thetay = lineData[3] - lineData[1];
		thetax = lineData[2] - lineData[0];

		deltax = -1 * SENSOR_LENGTH * Math.cos(Math.toRadians(thetay / 2));
		deltay = -1 * SENSOR_LENGTH * Math.cos(Math.toRadians(thetax / 2));

		// travel to one-one to correct position
		odometer.setXYT(deltax, deltay, odometer.getXYT()[2]);
		if (Robot.calculateDistance(odometer.getXYT()[2], odometer.getXYT()[1], 0, 0) > 1.5) {
			this.navigation.travelTo(0, 0, false, null);
		}

		this.navigation.turnTo(0.0);

		Robot.setSpeed(ROTATION_SPEED / 2);

		// if we are not facing 0.0 then turn ourselves so that we are
		if (odometer.getXYT()[2] <= 357 && odometer.getXYT()[2] >= 3) {
			Sound.beep();
			Robot.rotateByAngle(-odometer.getXYT()[2] + 9, 1, -1);
		}

		Robot.stop();
		odometer.setXYT(finalX * USLocalizer.TILESIZE, finalY * USLocalizer.TILESIZE, finalTheta - 5);
		lightSensor.close();

	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
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
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/**
	 * This method gets the color value of the light sensor
	 * 
	 */
	private float fetchSample() {
		float[] colorValue = new float[idColour.sampleSize()];
		idColour.fetchSample(colorValue, 0);
		return colorValue[0];
	}

	/**
	 * Move to the first intersection on the North East of the initial square
	 */
	public void moveToIntersection() {

		navigation.turnTo(Math.PI / 4);

		Robot.setSpeed(ROTATION_SPEED);

		// get sample
		sample = fetchSample();

		// move forward past the origin until light sensor sees the line
		while (sample > 0.38) {
			sample = fetchSample();
			Robot.forward();

		}
		Robot.stop();
		Sound.beep();

		// Move backwards so our origin is close to origin
		Robot.rotateByDistance(-10, 1, 1);

	}
}
