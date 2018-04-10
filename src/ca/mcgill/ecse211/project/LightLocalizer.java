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

		localizeYMid();
		Controller.logger();
		navigation.moveBy(-(odometer.getXYT()[0]%Robot.TILESIZE));
		if (corner == 0 || corner == 1) {
			odometer.setY(Robot.TILESIZE);
		} else if (corner == 2 || corner == 3) {
			odometer.setY(7 * Robot.TILESIZE);

		}

		navigation.turnTo(45);
		navigation.moveBy(Robot.LSTOLS/2);


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
		Controller.logger();
		boolean leftCheck = true;
		boolean rightCheck = true;
		double oriCoord = odometer.getXYT()[0];
		double oriTheta = odometer.getXYT()[2];
		double goal = 0;
		if (oriTheta >= 0 && oriTheta <=180) {
			goal = Math.ceil((oriCoord/Robot.TILESIZE))*Robot.TILESIZE+Robot.LSTOWHEEL;
		}
		else {
			goal = Math.floor((oriCoord/Robot.TILESIZE))*Robot.TILESIZE+Robot.LSTOWHEEL;

		}
		lineData = new double[2];
		navigation.forward(Robot.LOCALIZATION_SPEED);
		while (leftCheck || rightCheck) {
			sampleLeft = fetchSampleLeft();
			sampleRight = fetchSampleRight();
			if (leftCheck && sampleLeft < 0.28) {
				lineData[0] = odometer.getXYT()[0];
				Sound.beepSequenceUp();
				leftCheck = false;
				odometer.setX(goal);
			}
			if (rightCheck && sampleRight < 0.28) {
				lineData[1] = odometer.getXYT()[0];
				Sound.beepSequenceUp();
				rightCheck = false;
				odometer.setX(goal);
			}
			if ((rightCheck ^ leftCheck) && Math.abs(odometer.getXYT()[0]-goal) >= Robot.TILESIZE/2) {
				navigation.stopRobot();
				Sound.beepSequence();
				Controller.logger();
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
			odometer.setX(xLoc-Math.abs(deltaOdo/2));
		} else {
			odometer.setX(xLoc + Math.abs(deltaOdo/2));
		}
		Controller.logger();
	}

	public void localizeXBryan() throws OdometerExceptions, InterruptedException {
		double oriCoord = odometer.getXYT()[0];
		double oriTheta = odometer.getXYT()[2];
		double goal = 0;
		if (oriTheta >= 0 && oriTheta <=180) {
			goal = Math.ceil(((oriCoord-(Robot.LSTOWHEEL))/Robot.TILESIZE))*Robot.TILESIZE+Robot.LSTOWHEEL;
		}
		else {
			goal = Math.floor(((oriCoord+(Robot.LSTOWHEEL))/Robot.TILESIZE))*Robot.TILESIZE-Robot.LSTOWHEEL;

		}
		navigation.forward(Robot.FORWARD_SPEED);
		while(true){
			sampleLeft = fetchSampleLeft();
			sampleRight = fetchSampleRight();
			if (sampleLeft < 0.28) {
				odometer.setX(goal);
				navigation.stopRobot();
				//navigation.moveBy(-1.00);
				while (true) {
					sampleRight = fetchSampleRight();
					if (sampleRight < 0.28) {
						if (navigation.isNavigating()) {
							navigation.stopRobot();
						}
						odometer.setTheta(oriTheta);
						return;
					}
					if (!navigation.isNavigating()) {
						navigation.rotateRightWheel(Robot.LOCALIZATION_SPEED);
					}
					if (Math.abs(odometer.getXYT()[2]-oriTheta)> 35 && Math.abs(odometer.getXYT()[2]-oriTheta)< 325) {
						navigation.rotateRightWheelBack(Robot.LOCALIZATION_SPEED);
					}
				}
			}
			if (sampleRight < 0.28) {
				odometer.setX(goal);
				navigation.stopRobot();
				//navigation.moveBy(-1.00);
				while (true) {
					sampleLeft = fetchSampleLeft();
					if (sampleLeft < 0.28) {
						if (navigation.isNavigating()) {
							navigation.stopRobot();
						}
						odometer.setTheta(oriTheta);
						return;
					}
					if (!navigation.isNavigating()) {
						navigation.stopRobot();
						navigation.rotateLeftWheel(Robot.LOCALIZATION_SPEED);
					}
					if (Math.abs(odometer.getXYT()[2]-oriTheta)> 35 && Math.abs(odometer.getXYT()[2]-oriTheta)< 325) {
						navigation.stopRobot();
						navigation.rotateLeftWheelBack(Robot.LOCALIZATION_SPEED);

					}
				}
			}
		}
	}

	public void localizeYBryan() throws OdometerExceptions, InterruptedException {
		double oriCoord = odometer.getXYT()[1];
		double oriTheta = odometer.getXYT()[2];
		double goal = 0;
		if (oriTheta >= 270 || oriTheta <=90) {
			goal = Math.ceil(((oriCoord-(Robot.LSTOWHEEL))/Robot.TILESIZE))*Robot.TILESIZE+Robot.LSTOWHEEL;
		}
		else {
			goal = Math.floor(((oriCoord+(Robot.LSTOWHEEL))/Robot.TILESIZE))*Robot.TILESIZE-Robot.LSTOWHEEL;

		}
		navigation.forward(Robot.FORWARD_SPEED);
		while(true){
			sampleLeft = fetchSampleLeft();
			sampleRight = fetchSampleRight();
			if (sampleLeft < 0.28) {
				odometer.setY(goal);
				navigation.stopRobot();
				//				navigation.moveBy(-1.00);
				while (true) {
					sampleRight = fetchSampleRight();
					if (sampleRight < 0.28) {
						if (navigation.isNavigating()) {
							navigation.stopRobot();
						}
						odometer.setTheta(oriTheta);
						return;
					}
					if (!navigation.isNavigating()) {
						navigation.rotateRightWheel(Robot.LOCALIZATION_SPEED);
					}
					if (Math.abs(odometer.getXYT()[2]-oriTheta)> 35 && Math.abs(odometer.getXYT()[2]-oriTheta)< 325) {
						navigation.stopRobot();
						navigation.rotateRightWheelBack(Robot.LOCALIZATION_SPEED);

					}
				}
			}
			if (sampleRight < 0.28) {
				odometer.setY(goal);
				navigation.stopRobot();
				//				navigation.moveBy(-1.00);
				while (true) {
					sampleLeft = fetchSampleLeft();
					if (sampleLeft < 0.28) {
						if (navigation.isNavigating()) {
							navigation.stopRobot();
						}
						odometer.setTheta(oriTheta);
						return;
					}
					if (!navigation.isNavigating()) {
						navigation.rotateLeftWheel(Robot.LOCALIZATION_SPEED);
					}
					if (Math.abs(odometer.getXYT()[2]-oriTheta)> 35 && Math.abs(odometer.getXYT()[2]-oriTheta)< 325) {
						navigation.stopRobot();
						navigation.rotateLeftWheelBack(Robot.LOCALIZATION_SPEED);

					}
				}
			}
		}
	}

	/**
	 * Y Coordinate correction method. Must be called when robot's Light Sensors are
	 * at a distance from the line of at maximum 1.1 times the Tilesize
	 */
	public void localizeY() {
		Controller.logger();
		boolean leftCheck = true;
		boolean rightCheck = true;
		double oriCoord = odometer.getXYT()[1];
		double oriTheta = odometer.getXYT()[2];
		double goal = 0;
		if (oriTheta >= 270 || oriTheta <=90) {
			goal = Math.ceil((oriCoord/Robot.TILESIZE))*Robot.TILESIZE+Robot.LSTOWHEEL;
		}
		else {
			goal = Math.floor((oriCoord/Robot.TILESIZE))*Robot.TILESIZE+Robot.LSTOWHEEL;
		}
		lineData = new double[2]; 
		navigation.forward(Robot.LOCALIZATION_SPEED);
		while (leftCheck || rightCheck) {
			sampleLeft = fetchSampleLeft();
			sampleRight = fetchSampleRight();
			if (leftCheck && sampleLeft < 0.28) {
				lineData[0] = odometer.getXYT()[1];
				odometer.setY(goal);
				Sound.beepSequenceUp();
				leftCheck = false;
			}
			if (rightCheck && sampleRight < 0.28) {
				lineData[1] = odometer.getXYT()[1];
				odometer.setY(goal);
				Sound.beepSequenceUp();
				rightCheck = false;
			}
			if ((rightCheck ^ leftCheck) && Math.abs(odometer.getXYT()[1]-goal) >= Robot.TILESIZE/2) {
				navigation.stopRobot();
				Sound.beepSequence();
				Controller.logger();
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
			odometer.setX(yLoc-Math.abs(deltaOdo/2));
		} else {
			odometer.setX(yLoc + Math.abs(deltaOdo/2));
		}
		Controller.logger();
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
