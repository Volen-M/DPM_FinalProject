package ca.mcgill.ecse211.project;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Controller {

	// Motor Objects, and Robot related parameters
	public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port usPort = LocalEV3.get().getPort("S3");

	// Single navigation instance used by all classes
	private static Navigation navigation;

	// Constants for part 2
	private static double lowerLeftX = 2 * Robot.TILESIZE;
	private static double lowerLeftY = 3 * Robot.TILESIZE;
	private static double upperRightX = 6 * Robot.TILESIZE;
	private static double upperRightY = 7 * Robot.TILESIZE;
	private static int targetBlock = 3;

	
	/**
	 * Main method. Initial entry point of the code for this lab.
	 * @param args
	 * @throws OdometerExceptions
	 * @throws InterruptedException
	 */
	@SuppressWarnings("deprecation")
	public static void main(String[] args) throws OdometerExceptions, InterruptedException {

		ColourCalibration colourCalibration = new ColourCalibration();
		Thread colourCalibrationThread = new Thread(colourCalibration);
		// Odometer related objects
		Odometer odometer = Odometer.getOrCreateOdometer(Robot.TRACK, Robot.WHEEL_RAD);

		// usSensor is the instance
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);
		// usDistance provides samples from this instance
		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");

		navigation = new Navigation(odometer);

		// Start odometer and display threads and correction Threads.
		Thread odoThread = new Thread(odometer);
		odoThread.start();

		// Create ultrasonic and light localizer objects.
		USLocalizer USLocalizer = new USLocalizer(odometer, usDistance, 0, navigation);
		navigation.usLoc = USLocalizer;
		LightLocalizer lightLocatizer = new LightLocalizer(odometer, navigation);

		while (Button.waitForAnyPress() != Button.ID_DOWN)
			;

		navigation.moveBy(1*Robot.TILESIZE);

		System.out.println(odometer.getXYT()[0] + " " + odometer.getXYT()[1]+ " "  + odometer.getXYT()[2] );
		
		while (Button.waitForAnyPress() != Button.ID_DOWN)
			;

		navigation.moveBy(1*Robot.TILESIZE);

		System.out.println(odometer.getXYT()[0] + " " + odometer.getXYT()[1]+ " "  + odometer.getXYT()[2] );
		

		while (Button.waitForAnyPress() != Button.ID_DOWN)
			;

		navigation.moveBy(2*Robot.TILESIZE);

		System.out.println(odometer.getXYT()[0] + " " + odometer.getXYT()[1]+ " "  + odometer.getXYT()[2] );
		

		while (Button.waitForAnyPress() != Button.ID_DOWN)
			;

		navigation.moveBy(2*Robot.TILESIZE);

		System.out.println(odometer.getXYT()[0] + " " + odometer.getXYT()[1]+ " "  + odometer.getXYT()[2] );
		

		while (Button.waitForAnyPress() != Button.ID_DOWN)
			;

		navigation.moveBy(6*Robot.TILESIZE);

		System.out.println(odometer.getXYT()[0] + " " + odometer.getXYT()[1]+ " "  + odometer.getXYT()[2] );
		

		while (Button.waitForAnyPress() != Button.ID_DOWN)
			;

		navigation.moveBy(6*Robot.TILESIZE);

		System.out.println(odometer.getXYT()[0] + " " + odometer.getXYT()[1]+ " "  + odometer.getXYT()[2] );
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

}