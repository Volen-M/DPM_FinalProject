package ca.mcgill.ecse211.project;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * Main Class of the Robot; dictates robot behaviour. Contains Main method.
 * 
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

	// Other class instances necessary in the main and/or runTests
	private static Odometer odometer;
	private static USLocalizer usLocalizer;
	private static LightLocalizer lightLocalizer;
	private static ColourCalibration colourCalibration;
	private static SearchAndLocalize searchAndLocalize;

	// Constants for part 2
	private static int targetBlock = 3;
	private static String currentTeam;

	// Set to true when testing, allows for beta demo behaviour not to happen
	private static boolean testing = true;
	public static boolean betaDemo = false;

	/**
	 * Main method. Initial entry point of the code for this lab. Instantiates
	 * necessary objects and runs testing or demo behaviour.
	 * 
	 * @param args Standard main method input
	 * @throws OdometerExceptions
	 * @throws InterruptedException
	 */
	@SuppressWarnings({ "static-access", "resource" })
	public static void main(String[] args) throws OdometerExceptions, InterruptedException {

		// Odometer related objects
		odometer = Odometer.getOrCreateOdometer();

		// usSensor is the instance
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);
		// usDistance provides samples from this instance
		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");

		navigation = new Navigation();

		Thread odoThread = new Thread(odometer); // Start odometer thread.
		odoThread.start();

		// Create other necessary class instances.
		usLocalizer = new USLocalizer(usDistance, 0);
		navigation.usLoc = usLocalizer;
		lightLocalizer = new LightLocalizer();
		colourCalibration = new ColourCalibration(targetBlock);
		searchAndLocalize = new SearchAndLocalize(usDistance, 5, 3, 1, 1, 0);

		if (!testing) {
			WiFiData.processData();
		} else {
			runTests();
		}

		if (betaDemo && !testing) {
			if (currentTeam.equals("Green")) {
				usLocalizer.localize();
				odometer.setXYT(7.25 * Robot.TILESIZE, 0.75 * Robot.TILESIZE, 270);
				navigation.turnTo(-45);
				navigation.moveBy(Math.sqrt(2) * Robot.TILESIZE / 2);
				navigation.turnTo(270);
				navigation.moveBy(10);
				lightLocalizer.localizeX();
				navigation.turnTo(0);
				navigation.moveBy(10);
				lightLocalizer.localizeY();
				navigation.travelTo(WiFiData.tnLLX + Robot.TILESIZE / 2.0 + Robot.TILESIZE,
						WiFiData.tnLLY - Robot.TILESIZE - Robot.TILESIZE / 2);
				navigation.turnTo(270);
				navigation.moveBy(10);
				lightLocalizer.localizeX();
				navigation.turnTo(0);
				navigation.moveBy(10);
				lightLocalizer.localizeY();
				navigation.travelTo(WiFiData.tnLLX + Robot.TILESIZE / 2.0, WiFiData.tnLLY - Robot.TILESIZE / 2);
				navigation.travelTo(WiFiData.tnURX - Robot.TILESIZE / 2.0, WiFiData.tnURY + Robot.TILESIZE / 2);
				navigation.turnTo(90);
				navigation.moveBy(10);
				lightLocalizer.localizeX();
				navigation.turnTo(0);
				navigation.moveBy(10);
				lightLocalizer.localizeY();
				navigation.landingGearOn();
				navigation.travelTo(WiFiData.brURX - Robot.TILESIZE / 2.0, WiFiData.brURY + Robot.TILESIZE / 2);
				navigation.travelTo(WiFiData.brURX + Robot.TILESIZE / 2.0, WiFiData.brURY - Robot.TILESIZE / 2);
				navigation.landingGearOff();
				navigation.turnTo(90);
				navigation.moveBy(10);
				lightLocalizer.localizeX();
				navigation.turnTo(180);
				navigation.moveBy(10);
				lightLocalizer.localizeY();
				navigation.travelTo(7.5 * Robot.TILESIZE, 0.5 * Robot.TILESIZE);
			} else {
				usLocalizer.localize();
				odometer.setXYT(0.75 * Robot.TILESIZE, 0.75 * Robot.TILESIZE, 270);
				navigation.turnTo(-45);
				navigation.moveBy(Math.sqrt(2) * Robot.TILESIZE / 2);
				navigation.turnTo(270);
				navigation.moveBy(10);
				lightLocalizer.localizeX();
				navigation.turnTo(0);
				navigation.moveBy(10);
				lightLocalizer.localizeY();
				navigation.travelTo(WiFiData.tnLLX + Robot.TILESIZE / 2.0 + Robot.TILESIZE,
						WiFiData.tnLLY - Robot.TILESIZE - Robot.TILESIZE / 2);
				navigation.turnTo(270);
				navigation.moveBy(10);
				lightLocalizer.localizeX();
				navigation.turnTo(0);
				navigation.moveBy(10);
				lightLocalizer.localizeY();
				navigation.travelTo(WiFiData.tnLLX + Robot.TILESIZE / 2.0, WiFiData.tnLLY - Robot.TILESIZE / 2);
				navigation.travelTo(WiFiData.tnURX - Robot.TILESIZE / 2.0, WiFiData.tnURY + Robot.TILESIZE / 2);
				navigation.turnTo(90);
				navigation.moveBy(10);
				lightLocalizer.localizeX();
				navigation.turnTo(0);
				navigation.moveBy(10);
				lightLocalizer.localizeY();
				navigation.travelTo(WiFiData.brURX - Robot.TILESIZE / 2.0, WiFiData.brURY + Robot.TILESIZE / 2);
				navigation.travelTo(WiFiData.brURX + Robot.TILESIZE / 2.0, WiFiData.brURY - Robot.TILESIZE / 2);
				navigation.turnTo(90);
				navigation.moveBy(10);
				lightLocalizer.localizeX();
				navigation.turnTo(180);
				navigation.moveBy(10);
				lightLocalizer.localizeY();
				navigation.travelTo(7.5 * Robot.TILESIZE, 0.5 * Robot.TILESIZE);
			}
		}
	}

	/**
	 * Used to run various tests.
	 * @throws OdometerExceptions
	 */
	@SuppressWarnings("static-access")
	public static void runTests() throws OdometerExceptions {
		odometer.setXYT(0, 0, 0);
		int test = 4;
		if (test == 0) { // Test for the back wheel to go up or down need to set the angle by how much
			// they have to rotate (you dont want to over rotate or under
			// You know you over wroted due to screeching sound.... Constant to set:Angle in
			// navigation.landingGearOn() and navigation.landingGearoff()

			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			navigation.landingGearOn();
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			navigation.landingGearOff();

		} else if (test == 1) { // Test for wheel radius (get by what percentage it is off on average so
			// (Distancetravelled+offset)/Distancetravelled
			// and change the WHEEL_RAD by that.... Constant to set: Robot.WHEEL_RAD
			//
			//

			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			navigation.moveBy(2 * Robot.TILESIZE);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			navigation.moveBy(2 * Robot.TILESIZE);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			navigation.moveBy(2 * Robot.TILESIZE);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			navigation.moveBy(4 * Robot.TILESIZE);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			navigation.moveBy(4 * Robot.TILESIZE);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			navigation.moveBy(4 * Robot.TILESIZE);

		} else if (test == 2) { // Test for track length by checking turning. Get it as close as possible then
			// just add constant to turning as
			// chances are it wont turn as nicely both ways compare angles to black lines
			// for ease (make sure to replace exactly the cross in the middle of the axis of
			// both wheels after each trial)
			// .... Constants to change: Robot.TRACK, then Navigation constants on line
			// 233-234, 238, 239 (for now they are +2, +2 , +0. +0)

//			while (Button.waitForAnyPress() != Button.ID_DOWN)
//				;
//			odometer.setXYT(0, 0, 0);
//			navigation.turnTo(90);
//			while (Button.waitForAnyPress() != Button.ID_DOWN)
//				;
//			odometer.setXYT(0, 0, 0);
//			navigation.turnTo(-90);
//			while (Button.waitForAnyPress() != Button.ID_DOWN)
//				;
//			odometer.setXYT(0, 0, 0);
//			navigation.turnTo(90);
//			while (Button.waitForAnyPress() != Button.ID_DOWN)
//				;
//			odometer.setXYT(0, 0, 0);
//			navigation.turnTo(-90);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			navigation.turnTo(180);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			navigation.turnTo(-180);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			navigation.turnTo(180);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			navigation.turnTo(-179.9);
			 while (Button.waitForAnyPress() != Button.ID_DOWN)
			 ;
			 navigation.turnTo(180);
			 while (Button.waitForAnyPress() != Button.ID_DOWN)
			 ;
			 navigation.turnTo(-180);
			 while (Button.waitForAnyPress() != Button.ID_DOWN)
			 ;
			 navigation.turnTo(270);
			 while (Button.waitForAnyPress() != Button.ID_DOWN)
			 ;
			 navigation.turnTo(-270);

		} else if (test == 3) { // This is just to check Test 1 and 2 are successful together first makes the
			// robot go to (5,3) then (2,2) then (3,2) and then (4,1)
			// Please note since at this point the odometer has not been tested it resets it

			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			
			navigation.moveBy(Robot.TILESIZE*2);
			navigation.travelTo(5 * Robot.TILESIZE, 3 * Robot.TILESIZE);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			navigation.travelTo(2 * Robot.TILESIZE, 2 * Robot.TILESIZE);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			navigation.travelTo(3 * Robot.TILESIZE, 2 * Robot.TILESIZE);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			navigation.travelTo(4 * Robot.TILESIZE, 1 * Robot.TILESIZE);

		} else if (test == 4) { // This is test 3 but with Odometer and without reseting anything to check if
			// Odometer works....Constants: None -> (for Coding)
			
			
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			
			navigation.moveBy(Robot.TILESIZE*2);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			
			navigation.moveBy(Robot.TILESIZE*3);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			
			navigation.moveBy(Robot.TILESIZE*2);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			
			navigation.moveBy(Robot.TILESIZE*3);

			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			navigation.travelTo(0 * Robot.TILESIZE, 2 * Robot.TILESIZE);
			System.out.println("X: " + odometer.getXYT()[0]);
			System.out.println("Y: " + odometer.getXYT()[1]);
			System.out.println("Theta: " + odometer.getXYT()[2]);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			navigation.travelTo(2 * Robot.TILESIZE, 3 * Robot.TILESIZE);
			System.out.println("X: " + odometer.getXYT()[0]);
			System.out.println("Y: " + odometer.getXYT()[1]);
			System.out.println("Theta: " + odometer.getXYT()[2]);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			navigation.travelTo(3 * Robot.TILESIZE, 2 * Robot.TILESIZE);
			System.out.println("X: " + odometer.getXYT()[0]);
			System.out.println("Y: " + odometer.getXYT()[1]);
			System.out.println("Theta: " + odometer.getXYT()[2]);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			navigation.travelTo(4 * Robot.TILESIZE, 1 * Robot.TILESIZE);
			System.out.println("X: " + odometer.getXYT()[0]);
			System.out.println("Y: " + odometer.getXYT()[1]);
			System.out.println("Theta: " + odometer.getXYT()[2]);
		} else if (test == 5) { // Light Sensor test, put it on a 3 line cross and see if it works
			// Constants to change: Robot.LSTOLS (distance between both Light sensors) and
			// Robot.LSTOWHEEL (distance between wheel and odo
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 90);
			lightLocalizer.localizeX();
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 90);
			lightLocalizer.localizeX();
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 90);
			lightLocalizer.localizeX();
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			lightLocalizer.localizeY();
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			lightLocalizer.localizeY();
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			lightLocalizer.localizeY();
		} else if (test == 6) {// US Sensor Test check if it works
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			usLocalizer.localize();
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			usLocalizer.localize();
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			usLocalizer.localize();
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			usLocalizer.localize();

		} else if (test == 7) {
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			navigation.turnTo(90);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			navigation.turnTo(-90);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			navigation.turnTo(90);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			navigation.turnTo(-90);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			navigation.rotateByDistance(2 * Robot.TILESIZE, 1, 1);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				navigation.rotateByDistance(2 * Robot.TILESIZE, 1, 1);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				navigation.rotateByDistance(2 * Robot.TILESIZE, 1, 1);
		} else if (test == 8) {
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			usLocalizer.localize();
			System.out.println("X: " + odometer.getXYT()[0]);
			System.out.println("Y: " + odometer.getXYT()[1]);
			System.out.println("Theta: " + odometer.getXYT()[2]);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			usLocalizer.localize();
			System.out.println("X: " + odometer.getXYT()[0]);
			System.out.println("Y: " + odometer.getXYT()[1]);
			System.out.println("Theta: " + odometer.getXYT()[2]);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 0);
			lightLocalizer.localizeY();
			System.out.println("X: " + odometer.getXYT()[0]);
			System.out.println("Y: " + odometer.getXYT()[1]);
			System.out.println("Theta: " + odometer.getXYT()[2]);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0, 0, 90);
			lightLocalizer.localizeX();
			System.out.println("X: " + odometer.getXYT()[0]);
			System.out.println("Y: " + odometer.getXYT()[1]);
			System.out.println("Theta: " + odometer.getXYT()[2]);

		} else if (test == 9) {

			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;

			odometer.setXYT(0, 0, 0);
			navigation.landingGearOn();
			navigation.moveBy(4 * Robot.TILESIZE);
			navigation.landingGearOff();

			lightLocalizer.localizeY();

			navigation.turnTo(270);
			lightLocalizer.localizeX();

			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;

			odometer.setXYT(0, 0, 0);
			navigation.landingGearOn();
			navigation.moveBy(4 * Robot.TILESIZE);
			navigation.landingGearOff();

			lightLocalizer.localizeY();

			navigation.turnTo(270);
			lightLocalizer.localizeX();

			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;

			odometer.setXYT(0, 0, 0);
			navigation.landingGearOn();

			navigation.moveBy(4 * Robot.TILESIZE);
			navigation.landingGearOff();

			lightLocalizer.localizeY();

			navigation.turnTo(270);
			lightLocalizer.localizeX();

			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;

			odometer.setXYT(0, 0, 0);
			navigation.landingGearOn();
			navigation.moveBy(4 * Robot.TILESIZE);
			navigation.landingGearOff();

			lightLocalizer.localizeY();

			navigation.turnTo(270);
			lightLocalizer.localizeX();
			
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;

			odometer.setXYT(0, 0, 0);
			navigation.landingGearOn();

			navigation.moveBy(4 * Robot.TILESIZE);
			navigation.landingGearOff();

			lightLocalizer.localizeY();

			navigation.turnTo(270);
			lightLocalizer.localizeX();
			
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;

			odometer.setXYT(0, 0, 0);
			navigation.landingGearOn();

			navigation.moveBy(4 * Robot.TILESIZE);
			navigation.landingGearOff();

			lightLocalizer.localizeY();

			navigation.turnTo(270);
			lightLocalizer.localizeX();

		} else if (test == 10) {

			// should make beep.upsequence if it is the target block
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			System.out.println(colourCalibration.colourDetection());

			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			System.out.println(colourCalibration.colourDetection());

			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			System.out.println(colourCalibration.colourDetection());

			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			System.out.println(colourCalibration.colourDetection());

			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			System.out.println(colourCalibration.colourDetection());
		} else if (test == 11) {
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			searchAndLocalize.testMethod(test - 11);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			searchAndLocalize.testMethod(test - 11);
		} else if (test == 12) {
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			searchAndLocalize.testMethod(test - 11);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			searchAndLocalize.testMethod(test - 11);
		} else if (test == 13) {
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			searchAndLocalize.testMethod(test - 11);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			searchAndLocalize.testMethod(test - 11);
		} else if (test == 14) {
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			searchAndLocalize.testMethod(test - 11);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			searchAndLocalize.testMethod(test - 11);
		} else if (test == 15) {

			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(2.5 * Robot.TILESIZE, 3.5 * Robot.TILESIZE, 90);
			lightLocalizer.localizeXMid();

			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(2.5 * Robot.TILESIZE, 3.5 * Robot.TILESIZE, 90);
			lightLocalizer.localizeXMid();

			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(2.5 * Robot.TILESIZE, 3.5 * Robot.TILESIZE, 270);
			lightLocalizer.localizeXMid();

			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(2.5 * Robot.TILESIZE, 3.5 * Robot.TILESIZE, 270);
			lightLocalizer.localizeXMid();

			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(2.5 * Robot.TILESIZE, 3.5 * Robot.TILESIZE, 0);
			lightLocalizer.localizeYMid();

			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(2.5 * Robot.TILESIZE, 3.5 * Robot.TILESIZE, 0);
			lightLocalizer.localizeYMid();

			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(2.5 * Robot.TILESIZE, 3.5 * Robot.TILESIZE, 180);
			lightLocalizer.localizeYMid();

			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(2.5 * Robot.TILESIZE, 3.5 * Robot.TILESIZE, 180);
			lightLocalizer.localizeYMid();
		} else if(test == 16) {
			
			while(Button.waitForAnyPress() != Button.ID_DOWN);
			logger();
			
			usLocalizer.localize();
			lightLocalizer.fullLocalize(0);
			logger();
			
			while(Button.waitForAnyPress() != Button.ID_DOWN);
			logger();
			
			usLocalizer.localize();
			lightLocalizer.fullLocalize(1);
			logger();
			
			while(Button.waitForAnyPress() != Button.ID_DOWN);
			logger();
			
			usLocalizer.localize();
			lightLocalizer.fullLocalize(2);
			logger();
			
			while(Button.waitForAnyPress() != Button.ID_DOWN);
			logger();
			
			usLocalizer.localize();
			lightLocalizer.fullLocalize(3);
			logger();
			
			
		}
	}

	public static void logger() {
		double[] xyt = new double[3];
		xyt = odometer.getXYT();
		System.out.println("X: " + Math.round(xyt[0]));
		System.out.println("Y: " + Math.round(xyt[1]));
		System.out.println("Deg: " + Math.round(xyt[2]));
	}

	
	/**
	 * Sets the team colour for behaviour purposes. Called from the WifiData class.
	 * 
	 * @param colour
	 *            the colour of the team as per the retrieved wifi data
	 */
	public static void setCurrentTeam(String colour) {
		currentTeam = colour;
	}

	public static Odometer getOdometerInstance() {
		return odometer;
	}

	public static Navigation getNavigationInstance() {
		return navigation;
	}

	public static USLocalizer getUSLocalizerInstance() {
		return usLocalizer;
	}

	public static LightLocalizer getLightLocalizerInstance() {
		return lightLocalizer;
	}

	public static ColourCalibration getColourCalibrationInstance() {
		return colourCalibration;
	}

	public static SearchAndLocalize getSearchAndLocalizeInstance() {
		return searchAndLocalize;
	}
}