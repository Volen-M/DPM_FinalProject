package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Controller {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3LargeRegulatedMotor backMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port usPort = LocalEV3.get().getPort("S3");

	// Single navigation instance used by all classes
	private static Navigation navigation;

	// Set vehicle constants
	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 11.25;
	private static int startingCorner = 0;

	// Constants for part 2
	private static double lowerLeftX = 2 * USLocalizer.TILESIZE;
	private static double lowerLeftY = 3 * USLocalizer.TILESIZE;
	private static double upperRightX = 6 * USLocalizer.TILESIZE;
	private static double upperRightY = 7 * USLocalizer.TILESIZE;
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
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);

		// usSensor is the instance
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);
		// usDistance provides samples from this instance
		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");

		navigation = new Navigation(odometer, leftMotor, rightMotor, backMotor);

		// Start odometer and display threads and correction Threads.
		Thread odoThread = new Thread(odometer);
		odoThread.start();

		// Create ultrasonic and light localizer objects.
		USLocalizer USLocalizer = new USLocalizer(odometer, leftMotor, rightMotor, usDistance, startingCorner,
				navigation);
		navigation.usLoc = USLocalizer;
		LightLocalizer lightLocatizer = new LightLocalizer(odometer, leftMotor, rightMotor, navigation);

		// perform the ultrasonic localization
		USLocalizer.localize();

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

}