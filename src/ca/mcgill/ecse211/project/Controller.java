package ca.mcgill.ecse211.project;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * Main Class of the Robot; dictates robot behaviour.
 * Contains Main function
 * @author Volen Mihaylov
 * @author Patrick Ghazal
 * @author Bryan Jay
 *
 */
public class Controller {

	// Motor Objects, and Robot related parameters
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
		USLocalizer usLocalizer = new USLocalizer(odometer, usDistance, 0, navigation);
		navigation.usLoc = usLocalizer;
		LightLocalizer lightLocalizer = new LightLocalizer(odometer, navigation);


		odometer.setXYT(0, 0, 0);
		int test = 0;
		if (test == 0 ) {		//Test for the back wheel to go up or down need to set the angle by how much they have to rotate (you dont want to over rotate or under
			//You know you over wroted due to screeching sound.... Constant to set:Angle in navigation.landingGearOn() and navigation.landingGearoff() 

			while (Button.waitForAnyPress() != Button.ID_DOWN);
			navigation.landingGearOn();
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			navigation.landingGearOff();


		}
		else if (test == 1) {	//Test for wheel radius (get by what percentage it is off on average so (Distancetravelled+offset)/Distancetravelled
			//and change the WHEEL_RAD by that.... Constant to set: Robot.WHEEL_RAD

			while (Button.waitForAnyPress() != Button.ID_DOWN);
			navigation.rotateByDistance(1*Robot.TILESIZE,1,1);
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			navigation.rotateByDistance(2*Robot.TILESIZE,1,1);
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			navigation.rotateByDistance(4*Robot.TILESIZE,1,1);
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			navigation.rotateByDistance(6*Robot.TILESIZE,1,1);

		}
		else if (test == 2) { //Test for track length by checking turning. Get it as close as possible then just add constant to turning as
			//chances are it wont turn as nicely both ways compare angles to black lines for ease (make sure to replace exactly the cross in the middle of the axis of both wheels after each trial)
			//.... Constants to change: Robot.TRACK, then Navigation constants on line 233-234, 238, 239 (for now they are +2, +2 , +0. +0)

			while (Button.waitForAnyPress() != Button.ID_DOWN);
			navigation.turnTo(90);
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			navigation.turnTo(-90);
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			navigation.turnTo(180);
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			navigation.turnTo(-180);
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			navigation.turnTo(270);
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			navigation.turnTo(-270);


		}
		else if (test == 3) { //This is just to check Test 1 and 2 are successful together first makes the robot go to (5,3) then (2,2) then (3,2) and then (4,1)
			//Please note since at this point the odometer has not been tested it resets it

			while (Button.waitForAnyPress() != Button.ID_DOWN);
			odometer.setXYT(0, 0, 0);
			navigation.travelTo(5*Robot.TILESIZE, 3*Robot.TILESIZE);
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			odometer.setXYT(0, 0, 0);
			navigation.travelTo(2*Robot.TILESIZE, 2*Robot.TILESIZE);
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			odometer.setXYT(0, 0, 0);
			navigation.travelTo(3*Robot.TILESIZE, 2*Robot.TILESIZE);
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			odometer.setXYT(0, 0, 0);
			navigation.travelTo(4*Robot.TILESIZE, 1*Robot.TILESIZE);


		}
		else if (test == 4) { //This is test 3 but with Odometer and without reseting anything to check if Odometer works....Constants: None -> (for Coding)

			while (Button.waitForAnyPress() != Button.ID_DOWN);
			odometer.setXYT(0, 0, 0);
			navigation.travelTo(5*Robot.TILESIZE, 3*Robot.TILESIZE);
			System.out.println("X: "+ odometer.getXYT()[0]);
			System.out.println("Y: "+ odometer.getXYT()[1]);
			System.out.println("Theta: "+ odometer.getXYT()[2]);
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			navigation.travelTo(2*Robot.TILESIZE, 2*Robot.TILESIZE);
			System.out.println("X: "+ odometer.getXYT()[0]);
			System.out.println("Y: "+ odometer.getXYT()[1]);
			System.out.println("Theta: "+ odometer.getXYT()[2]);
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			navigation.travelTo(3*Robot.TILESIZE, 2*Robot.TILESIZE);
			System.out.println("X: "+ odometer.getXYT()[0]);
			System.out.println("Y: "+ odometer.getXYT()[1]);
			System.out.println("Theta: "+ odometer.getXYT()[2]);
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			navigation.travelTo(4*Robot.TILESIZE, 1*Robot.TILESIZE);
			System.out.println("X: "+ odometer.getXYT()[0]);
			System.out.println("Y: "+ odometer.getXYT()[1]);
			System.out.println("Theta: "+ odometer.getXYT()[2]);
		}
		else if (test == 5) { //Light Sensor test, put it on a 3 line cross and see if it works
			//Constants to change: Robot.LSTOLS (distance between both Light sensors) and Robot.LSTOWHEEL (distance between wheel and odo
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			odometer.setXYT(0, 0, 90);
			lightLocalizer.localizeX();
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			odometer.setXYT(0, 0, 90);
			lightLocalizer.localizeX();
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			odometer.setXYT(0, 0, 90);
			lightLocalizer.localizeX();
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			odometer.setXYT(0, 0, 90);
			lightLocalizer.localizeY();
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			odometer.setXYT(0, 0, 0);
			lightLocalizer.localizeY();
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			odometer.setXYT(0, 0, 0);
			lightLocalizer.localizeY();
		}
		else if (test == 6) {//US Sensor Test check if it works
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			usLocalizer.localize();

		}


	}
}