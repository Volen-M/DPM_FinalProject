package ca.mcgill.ecse211.project;

import lejos.hardware.Button;
import lejos.hardware.Sound;
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
	private static boolean testing = false;
	private static boolean flagSearch = false;
	private static boolean smallGrid = false;

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
		usLocalizer = new USLocalizer(usDistance);
		navigation.usLoc = usLocalizer;
		lightLocalizer = new LightLocalizer();
		colourCalibration = new ColourCalibration();
		navigation.intializeLL();
		searchAndLocalize = new SearchAndLocalize(usDistance);

		if (!testing) {
			WiFiData.processData();
			fullSystemRun();
		} else {
			runTests();
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
		int test = 0;
		if (test == 0) {
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			navigation.landingGearOn();
			navigation.moveBy(4*Robot.TILESIZE);
			navigation.landingGearOff();
			
			while (Button.waitForAnyPress() != Button.ID_DOWN);
			navigation.landingGearOn();
			navigation.moveBy(4*Robot.TILESIZE);
			navigation.landingGearOff();

		} else if (test == 1) {
			odometer.setXYT(1 * Robot.TILESIZE, 1 * Robot.TILESIZE, 0);
			Thread.sleep(500);
			navigation.travelToAdvGrid(6, 5, true);
			logger();
			Thread.sleep(500);
			logger();
			navigation.travelToAdvGrid(3, 3, true);
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			odometer.setXYT(1 * Robot.TILESIZE, 1 * Robot.TILESIZE, 0);
			navigation.travelToAdvGrid(6, 5, true);

		} else if (test == 2) {
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			searchAndLocalize.testMethod(4, 1, 1, 5, 3, 0);

			while (Button.waitForAnyPress() != Button.ID_DOWN)
				;
			searchAndLocalize.testMethod(4, 1, 1, 5, 3, 0);
		}
	}

	@SuppressWarnings("static-access")
	public static void fullSystemRun() throws OdometerExceptions, InterruptedException {
		boolean isUpwards = courseIsUpwards(); // True is upwards, false is sideways field setup (in relation to full
												// course)
		colourCalibration.setFlag(targetBlock);
		if (isUpwards) {
			if (currentTeam.equals("green")) {
				if (WiFiData.greenCorner == 0) {
					startLocalization(WiFiData.greenCorner);
					Thread.sleep(500);
					navigation.travelToAdvGrid(0.5 * (WiFiData.tnLLX + WiFiData.tnURX), WiFiData.tnLLY - 0.5, true);
					Thread.sleep(500);
					navigation.turnTo(0);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(0.25+ 1.0 + WiFiData.tnURY - WiFiData.tnLLY);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeYBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.srURX, WiFiData.srLLY, true);
					Thread.sleep(500);
					navigation.motorSearch(true); Sound.beep();
					if (flagSearch) {
						searchAndLocalize.findFlag(WiFiData.srLLX, WiFiData.srLLY, WiFiData.srURX, WiFiData.srURY, 1);
					} else {
						navigation.travelToAdvGrid(WiFiData.srLLX, WiFiData.srLLY, true);
						for (int i = 0; i < 6; i++)
							Sound.beep();
					}
					Thread.sleep(500);
					navigation.travelToAdvGrid(0.5 * (WiFiData.brLLX + WiFiData.brURX), WiFiData.brURY + 0.5, true);
					Thread.sleep(500);
					navigation.turnTo(180);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(0.25 + 1.0 + WiFiData.brURY - WiFiData.brLLY);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeYBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					if (smallGrid) {
						navigation.travelToAdvGrid(1, 1, false);
					} else {
						navigation.travelToAdvGrid(1, 1, false);

					}
				} else if (WiFiData.greenCorner == 1) {
					startLocalization(WiFiData.greenCorner);
					Thread.sleep(500);
					navigation.travelToAdvGrid(0.5 * (WiFiData.tnLLX + WiFiData.tnURX), WiFiData.tnLLY - 0.5, true);
					Thread.sleep(500);
					navigation.turnTo(0);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(0.25+ 1.0 + WiFiData.tnURY - WiFiData.tnLLY);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeYBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.srURX, WiFiData.srLLY, true);
					Thread.sleep(500);
					navigation.motorSearch(true); Sound.beep();
					if (flagSearch) {
						searchAndLocalize.findFlag(WiFiData.srLLX, WiFiData.srLLY, WiFiData.srURX, WiFiData.srURY, 1);
					} else {
						navigation.travelToAdvGrid(WiFiData.srLLX, WiFiData.srLLY, true);
						for (int i = 0; i < 6; i++)
							Sound.beep();
					}
					Thread.sleep(500);
					navigation.travelToAdvGrid(0.5 * (WiFiData.brLLX + WiFiData.brURX), WiFiData.brURY + 0.5, true);
					Thread.sleep(500);
					navigation.turnTo(180);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(0.25+ 1.0 + WiFiData.brURY - WiFiData.brLLY);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeYBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					if (smallGrid) {
						navigation.travelToAdvGrid(7, 1, false);
					} else {
						navigation.travelToAdvGrid(11, 1, false);

					}

				} else if (WiFiData.greenCorner == 2) {
					startLocalization(WiFiData.greenCorner);
					Thread.sleep(500);
					navigation.travelToAdvGrid(0.5 * (WiFiData.tnLLX + WiFiData.tnURX), WiFiData.tnURY + 0.5, true);
					Thread.sleep(500);
					navigation.turnTo(180);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(0.25 + 1.0 + WiFiData.tnURY - WiFiData.tnLLY);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeYBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.srLLX, WiFiData.srURY, true);
					Thread.sleep(500);
					navigation.motorSearch(true); Sound.beep();
					if (flagSearch) {
						searchAndLocalize.findFlag(WiFiData.srLLX, WiFiData.srLLY, WiFiData.srURX, WiFiData.srURY, 3);
					} else {
						navigation.travelToAdvGrid(WiFiData.srURX, WiFiData.srURY, true);
						for (int i = 0; i < 6; i++)
							Sound.beep();
					}
					Thread.sleep(500);
					navigation.travelToAdvGrid(0.5 * (WiFiData.brLLX + WiFiData.brURX), WiFiData.brLLY - 0.5, true);
					Thread.sleep(500);
					navigation.turnTo(0);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(0.25 + 1.0 + WiFiData.brURY - WiFiData.brLLY);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeYBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					if (smallGrid) {
						navigation.travelToAdvGrid(7, 7, false);
					} else {
						navigation.travelToAdvGrid(11, 11, false);
					}
				} else if (WiFiData.greenCorner == 3) {
					startLocalization(WiFiData.greenCorner);
					Thread.sleep(500);
					navigation.travelToAdvGrid(0.5 * (WiFiData.tnLLX + WiFiData.tnURX), WiFiData.tnURY + 0.5, true);
					Thread.sleep(500);
					navigation.turnTo(180);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(0.25 + 1.0 + WiFiData.tnURY - WiFiData.tnLLY);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeYBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.srLLX, WiFiData.srURY, true);
					Thread.sleep(500);
					navigation.motorSearch(true); Sound.beep();
					if (flagSearch) {
						searchAndLocalize.findFlag(WiFiData.srLLX, WiFiData.srLLY, WiFiData.srURX, WiFiData.srURY, 3);
					} else {
						navigation.travelToAdvGrid(WiFiData.srURX, WiFiData.srURY, true);
						for (int i = 0; i < 6; i++)
							Sound.beep();
					}
					Thread.sleep(500);
					navigation.travelToAdvGrid(0.5 * (WiFiData.brLLX + WiFiData.brURX), WiFiData.brLLY - 0.5, true);
					Thread.sleep(500);
					navigation.turnTo(0);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(0.25 + 1.0 + WiFiData.brURY - WiFiData.brLLY);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeYBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					if (smallGrid) {
						navigation.travelToAdvGrid(1, 7, false);
					} else {
						navigation.travelToAdvGrid(1, 11, false);
					}
				}

			} else {
				if (WiFiData.redCorner == 0) {
					startLocalization(WiFiData.redCorner);
					Thread.sleep(500);
					navigation.travelToAdvGrid(0.5 * (WiFiData.brLLX + WiFiData.brURX), WiFiData.brLLY - 0.5, true);
					Thread.sleep(500);
					navigation.turnTo(0);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(0.25 + 1.0 + WiFiData.brURY - WiFiData.brLLY);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeYBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.sgURX, WiFiData.sgLLY, true);
					Thread.sleep(500);
					navigation.motorSearch(true); Sound.beep();
					if (flagSearch) {
						searchAndLocalize.findFlag(WiFiData.sgLLX, WiFiData.sgLLY, WiFiData.sgURX, WiFiData.sgURY, 1);
					} else {
						navigation.travelToAdvGrid(WiFiData.sgLLX, WiFiData.sgLLY, true);
						for (int i = 0; i < 6; i++)
							Sound.beep();
					}
					Thread.sleep(500);
					navigation.travelToAdvGrid(0.5 * (WiFiData.tnLLX + WiFiData.tnURX), WiFiData.tnURY + 0.5, true);
					Thread.sleep(500);
					navigation.turnTo(180);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(0.25 + 1.0 + WiFiData.tnURY - WiFiData.tnLLY);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeYBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					if (smallGrid) {
						navigation.travelToAdvGrid(1, 1, false);
					} else {
						navigation.travelToAdvGrid(1, 1, false);

					}

				} else if (WiFiData.redCorner == 1) {
					startLocalization(WiFiData.redCorner);
					Thread.sleep(500);
					navigation.travelToAdvGrid(0.5 * (WiFiData.brLLX + WiFiData.brURX), WiFiData.brLLY - 0.5, true);
					Thread.sleep(500);
					navigation.turnTo(0);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(0.25 + 1.0 + WiFiData.brURY - WiFiData.brLLY);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeYBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.sgURX, WiFiData.sgLLY, true);
					Thread.sleep(500);
					navigation.motorSearch(true); Sound.beep();
					if (flagSearch) {
						searchAndLocalize.findFlag(WiFiData.sgLLX, WiFiData.sgLLY, WiFiData.sgURX, WiFiData.sgURY, 1);
					} else {
						navigation.travelToAdvGrid(WiFiData.sgLLX, WiFiData.sgLLY, true);
						for (int i = 0; i < 6; i++)
							Sound.beep();
					}
					Thread.sleep(500);
					navigation.travelToAdvGrid(0.5 * (WiFiData.tnLLX + WiFiData.tnURX), WiFiData.tnURY + 0.5, true);
					Thread.sleep(500);
					navigation.turnTo(180);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(0.25 + 1.0 + WiFiData.tnURY - WiFiData.tnLLY);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeYBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					if (smallGrid) {
						navigation.travelToAdvGrid(7, 1, false);
					} else {
						navigation.travelToAdvGrid(11, 1, false);

					}

				} else if (WiFiData.redCorner == 2) {
					startLocalization(WiFiData.redCorner);
					Thread.sleep(500);
					navigation.travelToAdvGrid(0.5 * (WiFiData.brLLX + WiFiData.brURX), WiFiData.brURY + 0.5, true);
					Thread.sleep(500);
					navigation.turnTo(180);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(0.25 + 1.0 + WiFiData.brURY - WiFiData.brLLY);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeYBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.sgLLX, WiFiData.sgURY, true);
					Thread.sleep(500);
					navigation.motorSearch(true); Sound.beep();
					if (flagSearch) {
						searchAndLocalize.findFlag(WiFiData.sgLLX, WiFiData.sgLLY, WiFiData.sgURX, WiFiData.sgURY, 3);
					} else {
						navigation.travelToAdvGrid(WiFiData.sgURX, WiFiData.sgURY, true);
						for (int i = 0; i < 6; i++)
							Sound.beep();
					}
					Thread.sleep(500);
					navigation.travelToAdvGrid(0.5 * (WiFiData.tnLLX + WiFiData.tnURX), WiFiData.tnLLY - 0.5, true);
					Thread.sleep(500);
					navigation.turnTo(0);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(0.25 + 1 + WiFiData.tnURY - WiFiData.tnLLY);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeYBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					if (smallGrid) {
						navigation.travelToAdvGrid(7, 7, false);
					} else {
						navigation.travelToAdvGrid(11, 11, false);
					}

				} else if (WiFiData.redCorner == 3) {
					startLocalization(WiFiData.redCorner);
					Thread.sleep(500);
					navigation.travelToAdvGrid(0.5 * (WiFiData.brLLX + WiFiData.brURX), WiFiData.brURY + 0.5, true);
					Thread.sleep(500);
					navigation.turnTo(180);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(0.25 + 1 + WiFiData.brURY - WiFiData.brLLY);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeYBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.sgLLX, WiFiData.sgURY, true);
					Thread.sleep(500);
					navigation.motorSearch(true); Sound.beep();
					if (flagSearch) {
						searchAndLocalize.findFlag(WiFiData.sgLLX, WiFiData.sgLLY, WiFiData.sgURX, WiFiData.sgURY, 3);
					} else {
						navigation.travelToAdvGrid(WiFiData.sgURX, WiFiData.sgURY, true);
						for (int i = 0; i < 6; i++)
							Sound.beep();
					}
					Thread.sleep(500);
					navigation.travelToAdvGrid(0.5 * (WiFiData.tnLLX + WiFiData.tnURX), WiFiData.tnLLY - 0.5, true);
					Thread.sleep(500);
					navigation.turnTo(0);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(0.25 + 1 + WiFiData.tnURY - WiFiData.tnLLY);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeYBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					if (smallGrid) {
						navigation.travelToAdvGrid(1, 7, false);
					} else {
						navigation.travelToAdvGrid(1, 11, false);
					}

				}

			}
		} else {
			if (currentTeam.equals("green")) {
				if (WiFiData.greenCorner == 0) {
					startLocalization(WiFiData.greenCorner);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.tnLLX - 0.5, 0.5 * (WiFiData.tnLLY + WiFiData.tnURY), false);
					Thread.sleep(500);
					navigation.turnTo(90);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(0.25 + 1.0 + WiFiData.tnURX - WiFiData.tnLLX);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeXBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.srLLX, WiFiData.srLLY, false);
					Thread.sleep(500);
					navigation.motorSearch(true); Sound.beep();
					if (flagSearch) {
						searchAndLocalize.findFlag(WiFiData.srLLX, WiFiData.srLLY, WiFiData.srURX, WiFiData.srURY, 1);
					} else {
						navigation.travelToAdvGrid(WiFiData.srLLX, WiFiData.srURY, false);
						for (int i = 0; i < 6; i++)
							Sound.beep();
					}
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.brURX + 0.5, 0.5 * (WiFiData.brLLY + WiFiData.brURY), false);
					Thread.sleep(500);
					navigation.turnTo(270);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(0.25 + 1 + WiFiData.brURX - WiFiData.brLLX);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeXBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					if (smallGrid) {
						navigation.travelToAdvGrid(1, 1, true);
					} else {
						navigation.travelToAdvGrid(1, 1, true);

					}

				} else if (WiFiData.greenCorner == 1) {
					startLocalization(WiFiData.greenCorner);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.tnURX + 0.5, 0.5 * (WiFiData.tnLLY + WiFiData.tnURY), false);
					Thread.sleep(500);
					navigation.turnTo(270);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(0.25 + 1 + WiFiData.tnURX - WiFiData.tnLLX);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeXBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.srURX, WiFiData.srURY, false);
					Thread.sleep(500);
					navigation.motorSearch(true); Sound.beep();
					if (flagSearch) {
						searchAndLocalize.findFlag(WiFiData.srLLX, WiFiData.srLLY, WiFiData.srURX, WiFiData.srURY, 1);
					} else {
						navigation.travelToAdvGrid(WiFiData.srURX, WiFiData.srLLY, false);
						for (int i = 0; i < 6; i++)
							Sound.beep();
					}
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.brLLX - 0.5, 0.5 * (WiFiData.brLLY + WiFiData.brURY), false);
					Thread.sleep(500);
					navigation.turnTo(90);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(1.0 + 0.25 + WiFiData.brURX - WiFiData.brLLX);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeXBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					if (smallGrid) {
						navigation.travelToAdvGrid(7, 1, true);
					} else {
						navigation.travelToAdvGrid(11, 1, true);

					}

				} else if (WiFiData.greenCorner == 2) {
					startLocalization(WiFiData.greenCorner);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.tnURX + 0.5, 0.5 * (WiFiData.tnLLY + WiFiData.tnURY), false);
					Thread.sleep(500);
					navigation.turnTo(270);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(0.25 + 1 + WiFiData.tnURX - WiFiData.tnLLX);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeXBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.srURX, WiFiData.srURY, false);
					Thread.sleep(500);
					navigation.motorSearch(true); Sound.beep();
					if (flagSearch) {
						searchAndLocalize.findFlag(WiFiData.srLLX, WiFiData.srLLY, WiFiData.srURX, WiFiData.srURY, 1);
					} else {
						navigation.travelToAdvGrid(WiFiData.srURX, WiFiData.srLLY, false);
						for (int i = 0; i < 6; i++)
							Sound.beep();
					}
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.brLLX - 0.5, 0.5 * (WiFiData.brLLY + WiFiData.brURY), false);
					Thread.sleep(500);
					navigation.turnTo(90);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(0.25 + 1 + WiFiData.brURX - WiFiData.brLLX);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeXBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					if (smallGrid) {
						navigation.travelToAdvGrid(7, 7, true);
					} else {
						navigation.travelToAdvGrid(11, 11, true);

					}


				} else if (WiFiData.greenCorner == 3) {
					startLocalization(WiFiData.greenCorner);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.tnLLX - 0.5, 0.5 * (WiFiData.tnLLY + WiFiData.tnURY), false);
					Thread.sleep(500);
					navigation.turnTo(90);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(1 + 0.25 + WiFiData.tnURX - WiFiData.tnLLX);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeXBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.srLLX, WiFiData.srLLY, false);
					Thread.sleep(500);
					navigation.motorSearch(true); Sound.beep();
					if (flagSearch) {
						searchAndLocalize.findFlag(WiFiData.srLLX, WiFiData.srLLY, WiFiData.srURX, WiFiData.srURY, 1);
					} else {
						navigation.travelToAdvGrid(WiFiData.srLLX, WiFiData.srURY, false);
						for (int i = 0; i < 6; i++)
							Sound.beep();
					}
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.brURX + 0.5, 0.5 * (WiFiData.brLLY + WiFiData.brURY), false);
					Thread.sleep(500);
					navigation.turnTo(270);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(1.0 + 0.25 + WiFiData.brURX - WiFiData.brLLX);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeXBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					if (smallGrid) {
						navigation.travelToAdvGrid(1, 7, true);
					} else {
						navigation.travelToAdvGrid(1, 11, true);

					}

				}

			} else {
				if (WiFiData.redCorner == 0) {
					startLocalization(WiFiData.redCorner);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.brLLX - 0.5, 0.5 * (WiFiData.brLLY + WiFiData.brURY), false);
					Thread.sleep(500);
					navigation.turnTo(90);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(1.0 + 0.25 + WiFiData.brURX - WiFiData.brLLX);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeXBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.sgLLX, WiFiData.sgLLY, false);
					Thread.sleep(500);
					navigation.motorSearch(true); Sound.beep();
					if (flagSearch) {
						searchAndLocalize.findFlag(WiFiData.sgLLX, WiFiData.sgLLY, WiFiData.sgURX, WiFiData.sgURY, 1);
					} else {
						navigation.travelToAdvGrid(WiFiData.sgLLX, WiFiData.sgURY, false);
						for (int i = 0; i < 6; i++)
							Sound.beep();
					}
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.tnURX + 0.5, 0.5 * (WiFiData.tnLLY + WiFiData.tnURY), false);
					Thread.sleep(500);
					navigation.turnTo(270);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(1+ 0.25 + WiFiData.tnURX - WiFiData.tnLLX);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeXBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					if (smallGrid) {
						navigation.travelToAdvGrid(1, 1, true);
					} else {
						navigation.travelToAdvGrid(1, 1, true);

					}

				} else if (WiFiData.redCorner == 1) {
					startLocalization(WiFiData.redCorner);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.brURX + 0.5, 0.5 * (WiFiData.brLLY + WiFiData.brURY), false);
					Thread.sleep(500);
					navigation.turnTo(270);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(1 + 0.25 + WiFiData.brURX - WiFiData.brLLX);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeXBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.sgURX, WiFiData.sgURY, false);
					Thread.sleep(500);
					navigation.motorSearch(true); Sound.beep();
					if (flagSearch) {
						searchAndLocalize.findFlag(WiFiData.sgLLX, WiFiData.sgLLY, WiFiData.sgURX, WiFiData.sgURY, 1);
					} else {
						navigation.travelToAdvGrid(WiFiData.sgURX, WiFiData.sgLLY, false);
						for (int i = 0; i < 6; i++)
							Sound.beep();
					}
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.tnLLX - 0.5, 0.5 * (WiFiData.tnLLY + WiFiData.tnURY), false);
					Thread.sleep(500);
					navigation.turnTo(90);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(1 + 0.25 + WiFiData.tnURX - WiFiData.tnLLX);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeXBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					if (smallGrid) {
						navigation.travelToAdvGrid(7, 1, true);
					} else {
						navigation.travelToAdvGrid(11, 1, true);

					}

				} else if (WiFiData.redCorner == 2) {
					startLocalization(WiFiData.redCorner);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.brURX + 0.5, 0.5 * (WiFiData.brLLY + WiFiData.brURY), false);
					Thread.sleep(500);
					navigation.turnTo(270);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(1.0 + 0.25 + WiFiData.brURX - WiFiData.brLLX);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeXBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.sgURX, WiFiData.sgURY, false);
					Thread.sleep(500);
					navigation.motorSearch(true); Sound.beep();
					if (flagSearch) {
						searchAndLocalize.findFlag(WiFiData.sgLLX, WiFiData.sgLLY, WiFiData.sgURX, WiFiData.sgURY, 1);
					} else {
						navigation.travelToAdvGrid(WiFiData.sgURX, WiFiData.sgLLY, false);
						for (int i = 0; i < 6; i++)
							Sound.beep();
					}
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.tnLLX - 0.5, 0.5 * (WiFiData.tnLLY + WiFiData.tnURY), false);
					Thread.sleep(500);
					navigation.turnTo(90);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(1.0 + 0.25 + WiFiData.tnURX - WiFiData.tnLLX);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeXBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					if (smallGrid) {
						navigation.travelToAdvGrid(7, 7, true);
					} else {
						navigation.travelToAdvGrid(11, 11, true);

					}



				} else if (WiFiData.redCorner == 3) {
					startLocalization(WiFiData.redCorner);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.brLLX - 0.5, 0.5 * (WiFiData.brLLY + WiFiData.brURY), false);
					Thread.sleep(500);
					navigation.turnTo(90);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(1 + 0.25 + WiFiData.brURX - WiFiData.brLLX);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeXBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.sgLLX, WiFiData.sgLLY, false);
					Thread.sleep(500);
					navigation.motorSearch(true); Sound.beep();
					if (flagSearch) {
						searchAndLocalize.findFlag(WiFiData.sgLLX, WiFiData.sgLLY, WiFiData.sgURX, WiFiData.sgURY, 1);
					} else {
						navigation.travelToAdvGrid(WiFiData.sgLLX, WiFiData.sgURY, false);
						for (int i = 0; i < 6; i++)
							Sound.beep();
					}
					Thread.sleep(500);
					navigation.travelToAdvGrid(WiFiData.tnURX + 0.5, 0.5 * (WiFiData.tnLLY + WiFiData.tnURY), false);
					Thread.sleep(500);
					navigation.turnTo(270);
					Thread.sleep(500);
					navigation.landingGearOn();
					Thread.sleep(500);
					navigation.moveByGrid(1.0 + 0.25 + WiFiData.tnURX - WiFiData.tnLLX);
					Thread.sleep(500);
					navigation.landingGearOff();
					Thread.sleep(500);
					lightLocalizer.localizeXBryan();
					Thread.sleep(500);
					navigation.moveBy(-1 * Robot.LSTOWHEEL-0.5*Robot.TILESIZE);
					Thread.sleep(500);
					if (smallGrid) {
						navigation.travelToAdvGrid(1, 7, true);
					} else {
						navigation.travelToAdvGrid(1, 11, true);
					}

				}

			}
		}
	}

	/**
	 * Returns orientation in relation to bridge width and tunnel width. True is
	 * upwards, false is sideways field setup (in relation to full course)
	 */
	public static boolean courseIsUpwards() {
		if (WiFiData.brURX-WiFiData.brLLX ==1  && WiFiData.brURY - WiFiData.brLLY == 1) {
			if (Math.abs(WiFiData.redURY-WiFiData.greenLLY) == 1 || Math.abs(WiFiData.greenURY-WiFiData.redLLY) == 1){
				return true;
			}
			else {
				return false;
			}
		}
		else if (WiFiData.brURX - WiFiData.brLLX > 1 ) {
			return false;
		}
		return true;
	}

	public static void logger() {
		double[] xyt = new double[3];
		xyt = odometer.getXYT();
		System.out.println("X: " + Math.round(xyt[0]));
		System.out.println("Y: " + Math.round(xyt[1]));
		System.out.println("Deg: " + Math.round(xyt[2]));
	}

	@SuppressWarnings("static-access")
	public static void startLocalization(int corner) throws OdometerExceptions, InterruptedException {
		navigation.setAcceleration(500);
		usLocalizer.localize(corner);
		navigation.setAcceleration(2000);
		Thread.sleep(250);
		navigation.moveByGrid(0.4);
		Thread.sleep(250);
		if (corner == 0 || corner == 3) {
			navigation.turnTo(90);
		} else {
			navigation.turnTo(270);
		}
		Thread.sleep(250);
		lightLocalizer.localizeXBryan();
		Thread.sleep(250);
		navigation.moveBy(-1 * Robot.LSTOWHEEL, 200);
		Thread.sleep(250);
		if (corner == 0 || corner == 1) {
			navigation.turnTo(0);
		} else {
			navigation.turnTo(180);
		}
		Thread.sleep(250);
		lightLocalizer.localizeYBryan();
		Thread.sleep(250);
		navigation.moveBy(-1 * Robot.LSTOWHEEL, 200);
		Sound.beepSequenceUp();
		Thread.sleep(250);
		cornerSet(corner);
		

	}

	public static void cornerSet(int corner) {
		switch (corner) {
		case 0:
			odometer.setXYT(Robot.TILESIZE * 1, Robot.TILESIZE * 1, odometer.getXYT()[2]);
			break;
		case 1:
			odometer.setXYT(Robot.TILESIZE * 11, Robot.TILESIZE * 1, odometer.getXYT()[2]);
			break;
		case 2:
			odometer.setXYT(Robot.TILESIZE * 11, Robot.TILESIZE * 11, odometer.getXYT()[2]);
			break;
		case 3:
			odometer.setXYT(Robot.TILESIZE * 1, Robot.TILESIZE * 11, odometer.getXYT()[2]);
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