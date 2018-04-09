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
	private static int targetBlock = 0;
	private static int[] searchAreaCoords;
	private static String currentTeam;

	// Set to true when testing, allows for beta demo behaviour not to happen
	private static boolean testing = true;
	public static boolean betaDemo = false;

	/**
	 * Main method. Initial entry point of the code for this lab. Instantiates
	 * necessary objects and runs testing or demo behaviour.
	 * 
	 * @param args
	 *            Standard main method input
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
		navigation.intializeLL();
		searchAndLocalize = new SearchAndLocalize(usDistance, 5, 3, 1, 1, 0);
		//searchAndLocalize = new SearchAndLocalize(usDistance, searchAreaCoords[0], searchAreaCoords[1],
//				searchAreaCoords[2], searchAreaCoords[3], 0);

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
	 * 
	 * @throws OdometerExceptions
	 * @throws InterruptedException 
	 */
	@SuppressWarnings("static-access")
	public static void runTests() throws OdometerExceptions, InterruptedException {
		int test = 2;
		if (test == 0) { 
			while (Button.waitForAnyPress() != Button.ID_DOWN);
				startLocalization();
			while (Button.waitForAnyPress() != Button.ID_DOWN);
				startLocalization();
			while (Button.waitForAnyPress() != Button.ID_DOWN);
				startLocalization();
			
		} else if (test == 1) {
			odometer.setXYT(0.5, 0.5, 0);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			navigation.travelToAdv(6, 5);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(0.5, 0.5, 0);
			navigation.travelToAdv(6, 5);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			navigation.travelToAdv(3, 3);
			
		}
		else if (test == 2) {
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			searchAndLocalize.testMethod(4);

			while (Button.waitForAnyPress() != Button.ID_DOWN);
			searchAndLocalize.testMethod(4);
		}
	}

	public static void logger() {
		double[] xyt = new double[3];
		xyt = odometer.getXYT();
		System.out.println("X: " + Math.round(xyt[0]));
		System.out.println("Y: " + Math.round(xyt[1]));
		System.out.println("Deg: " + Math.round(xyt[2]));
	}
	
	@SuppressWarnings("static-access")
	public static void startLocalization() throws OdometerExceptions, InterruptedException {
		navigation.setAcceleration(500);
		usLocalizer.localize();
		navigation.setAcceleration(2000);
		Thread.sleep(500);
		navigation.moveByGrid(0.5);
		Thread.sleep(500);
		navigation.turnTo(90);
		Thread.sleep(500);
		lightLocalizer.localizeXBryan();
		Thread.sleep(500);
		navigation.moveBy(-1*Robot.LSTOWHEEL, 200);
		Thread.sleep(500);
		navigation.turnTo(0);
		Thread.sleep(500);
		lightLocalizer.localizeYBryan();
		Thread.sleep(500);
		navigation.moveBy(-1*Robot.LSTOWHEEL, 200);
		Thread.sleep(500);
		cornerSet(0);
		logger();
		
	}

	public static void cornerSet(int corner) {
		switch (corner) {
		case 0:
			odometer.setXYT(Robot.TILESIZE*1, Robot.TILESIZE*1, odometer.getXYT()[2]);
			break;
		case 1:
			odometer.setXYT(Robot.TILESIZE*11,Robot.TILESIZE*1, odometer.getXYT()[2]);
			break;
		case 2:
			odometer.setXYT(Robot.TILESIZE*11,Robot.TILESIZE*11, odometer.getXYT()[2]);
			break;
		case 3:
			odometer.setXYT(Robot.TILESIZE*1,Robot.TILESIZE*11, odometer.getXYT()[2]);
			break;
		}
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

	public static void setTargetBlock(int block) {
		targetBlock = block;
	}

	public static void setSearchAreaCoords(int[] coords) {
		searchAreaCoords = coords;
	}
}