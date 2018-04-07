package ca.mcgill.ecse211.project;

import java.util.ArrayList;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * Class that finds the needed cube in a certain rectangular area amongst other
 * cubes
 * 
 * @author Volen Mihaylov
 */
@SuppressWarnings("static-access")
public class SearchAndLocalize {
	public double lowerLeftX, lowerLeftY;
	public double upperRightX, upperRightY;
	private Navigation navigation;
	private ColourCalibration colourCalib;
	private LightLocalizer lightLocalizer;
	private Odometer odometer;
	private int corner;
	private double[] limiter;
	private float[] usData;
	private SampleProvider usDistance;
	private double constant = 4;
	private ArrayList<Area> foundCubes;

	public SearchAndLocalize(SampleProvider usDistance, double ur_x, double ur_y, double ll_x, double ll_y,
			int cornerOfZone) {
		this.navigation = Controller.getNavigationInstance();
		this.colourCalib = Controller.getColourCalibrationInstance();
		//this.lightLocalizer = Controller.getLightLocalizerInstance();
		this.lowerLeftX = ll_x;
		this.lowerLeftY = ll_y;
		this.upperRightX = ur_x;
		this.upperRightX = ur_y;
		this.limiter = new double[4];
		this.corner = cornerOfZone;
		this.odometer = Controller.getOdometerInstance();
		this.usDistance = usDistance;
		this.usData = new float[this.usDistance.sampleSize()];
		this.foundCubes = new ArrayList<Area>();
		switch (cornerOfZone) {
		case 0:
			limiter[0] = ur_y * Robot.TILESIZE;
			limiter[1] = ur_x * Robot.TILESIZE;
			limiter[2] = ll_y * Robot.TILESIZE;
			limiter[3] = ll_x * Robot.TILESIZE;
			break;
		case 1:
			limiter[1] = ur_y * Robot.TILESIZE + Robot.TILESIZE * 0.5;
			limiter[2] = ur_x * Robot.TILESIZE + Robot.TILESIZE * 0.5;
			limiter[3] = ll_y * Robot.TILESIZE - Robot.TILESIZE * 0.5;
			limiter[0] = ll_x * Robot.TILESIZE - Robot.TILESIZE * 0.5;
			break;
		case 2:
			limiter[2] = ur_y * Robot.TILESIZE + Robot.TILESIZE * 0.5;
			limiter[3] = ur_x * Robot.TILESIZE + Robot.TILESIZE * 0.5;
			limiter[0] = ll_y * Robot.TILESIZE - Robot.TILESIZE * 0.5;
			limiter[1] = ll_x * Robot.TILESIZE - Robot.TILESIZE * 0.5;
			break;
		case 3:
			limiter[3] = ur_y * Robot.TILESIZE + Robot.TILESIZE * 0.5;
			limiter[0] = ur_x * Robot.TILESIZE + Robot.TILESIZE * 0.5;
			limiter[1] = ll_y * Robot.TILESIZE - Robot.TILESIZE * 0.5;
			limiter[2] = ll_x * Robot.TILESIZE - Robot.TILESIZE * 0.5;
			break;
		}
	}

	public int findFlag() { // Int return is where robot ended, so 0=North, 1=East, 2=South, 3=West
		double xDist;
		double yDist;
		double distToCube;
		boolean checkCube;
		Area tempArea;
		navigation.setSpeed(Robot.FORWARD_SPEED);
		if (corner == 0 || corner == 2) {
			yDist = Math.abs(limiter[1] - limiter[3]);
			xDist = Math.abs(limiter[0] - limiter[2]);
		} else {
			xDist = Math.abs(limiter[1] - limiter[3]);
			yDist = Math.abs(limiter[0] - limiter[2]);
		}

		if (corner == 0) {
			yDist = Math.abs(limiter[1] - limiter[3]);
			xDist = Math.abs(limiter[0] - limiter[2]);

			while (odometer.getXYT()[1] <= limiter[0]) {
				navigation.forward();
				distToCube = fetchUS();
				if (distToCube < xDist) {
					checkCube = true;
					tempArea = new Area(limiter[3] + distToCube, odometer.getXYT()[1], 0);
					for (Area area : foundCubes) {
						if (area.isIn(tempArea)) {
							checkCube = false;
							break;
						}
					}
					if (checkCube) {
						foundCubes.add(tempArea);
						navigation.moveBy(4);
						navigation.turnTo(90);
						navigation.moveBy(distToCube - constant);
						if (colourCalib.colourDetection()) {
							for (int i = 0; i < 3; i++) {
								Sound.beep();
							}
							navigation.moveBy(-1 * distToCube);
							navigation.turnTo(0);
							return 3;
						} else {
							navigation.moveBy(-1 * distToCube);
							navigation.turnTo(0);
						}
					}
				}

			}
			navigation.stopRobot();
			navigation.turnTo(90);
			lightLocalizer.localizeXMid();
			while (odometer.getXYT()[0] <= limiter[1]) {
				navigation.forward();
				distToCube = fetchUS();
				if (distToCube < xDist) {
					checkCube = true;
					tempArea = new Area(odometer.getXYT()[0], limiter[0] - distToCube, 1);
					for (Area area : foundCubes) {
						if (area.isIn(tempArea)) {
							checkCube = false;
							break;
						}
					}
					if (checkCube) {
						foundCubes.add(tempArea);
						navigation.moveBy(4);
						navigation.turnTo(180);
						navigation.moveBy(distToCube - constant);
						if (colourCalib.colourDetection()) {
							for (int i = 0; i < 3; i++) {
								Sound.beep();
							}
							navigation.moveBy(-1 * distToCube);
							navigation.turnTo(90);
							return 0;
						} else {
							navigation.moveBy(-1 * distToCube);
							navigation.turnTo(90);
						}
					}
				}

			}
			navigation.stopRobot();
			navigation.turnTo(180);
			lightLocalizer.localizeYMid();
			while (odometer.getXYT()[1] >= limiter[2]) {
				navigation.forward();
				distToCube = fetchUS();
				if (distToCube < xDist) {
					checkCube = true;
					tempArea = new Area(limiter[1] - distToCube, odometer.getXYT()[1], 2);
					for (Area area : foundCubes) {
						if (area.isIn(tempArea)) {
							checkCube = false;
							break;
						}
					}
					if (checkCube) {
						foundCubes.add(tempArea);
						navigation.moveBy(4);
						navigation.turnTo(270);
						navigation.moveBy(distToCube - constant);
						if (colourCalib.colourDetection()) {
							for (int i = 0; i < 3; i++) {
								Sound.beep();
							}
							navigation.moveBy(-1 * distToCube);
							navigation.turnTo(180);
							return 1;
						} else {
							navigation.moveBy(-1 * distToCube);
							navigation.turnTo(180);
						}
					}
				}
			}
			navigation.stopRobot();
			navigation.turnTo(270);
			lightLocalizer.localizeXMid();
			while (odometer.getXYT()[0] >= limiter[3]) {
				navigation.forward();
				distToCube = fetchUS();
				if (distToCube < xDist) {
					checkCube = true;
					tempArea = new Area(odometer.getXYT()[0], limiter[2] + distToCube, 3);
					for (Area area : foundCubes) {
						if (area.isIn(tempArea)) {
							checkCube = false;
							break;
						}
					}
					if (checkCube) {
						foundCubes.add(tempArea);
						navigation.moveBy(4);
						navigation.turnTo(0);
						navigation.moveBy(distToCube - constant);
						if (colourCalib.colourDetection()) {
							for (int i = 0; i < 3; i++) {
								Sound.beep();
							}
							navigation.moveBy(-1 * distToCube);
							navigation.turnTo(270);
							return 2;
						} else {
							navigation.moveBy(-1 * distToCube);
							navigation.turnTo(270);
						}
					}
				}

			}

			for (int i = 0; i < 6; i++) {
				Sound.beep();
			}
		}
		// else if (corner == 1) {
		//
		// }
		// else if (corner == 2) {
		//
		// }
		// else if (corner == 3) {
		//
		// }
		return -1;
	}

	public void logger() {
		double[] xyt = new double[3];
		xyt = odometer.getXYT();
		System.out.println("X: " + Math.round(xyt[0]));
		System.out.println("Y: " + Math.round(xyt[1]));
		System.out.println("Deg: " + Math.round(xyt[2]));
	}

	public void testMethod(int test) throws OdometerExceptions {
		double xDist;
		double yDist;
		double distToCube;
		boolean checkCube;
		Area tempArea;
		xDist = Math.abs(limiter[1] - limiter[3]) / 2 - 4;
		yDist = Math.abs(limiter[0] - limiter[2]) / 2 - 4;
		if (test == 0) {
			odometer.setXYT(limiter[3], limiter[2], 0);
			logger();
			while (odometer.getXYT()[1] <= limiter[0]) {
				if (navigation.isNavigating() == false) {
					navigation.forward();
				}
			}
			navigation.stopRobot();
			navigation.turnTo(90);
			while (navigation.isNavigating()) {
			}
			;
			logger();
			lightLocalizer.localizeXMid();
			while (navigation.isNavigating()) {
			}
			;
			while (odometer.getXYT()[0] <= limiter[1]) {
				if (navigation.isNavigating() == false) {
					navigation.forward();
				}
			}
			navigation.stopRobot();
			navigation.turnTo(180);
			while (navigation.isNavigating()) {
			}
			;
			logger();
			lightLocalizer.localizeYMid();
			while (navigation.isNavigating()) {
			}
			;
			while (odometer.getXYT()[1] >= limiter[2]) {
				if (navigation.isNavigating() == false) {
					navigation.forward();
				}
			}
			navigation.stopRobot();
			navigation.turnTo(270);
			while (navigation.isNavigating()) {
			}
			;
			logger();
			lightLocalizer.localizeXMid();
			while (navigation.isNavigating()) {
			}
			;
			while (odometer.getXYT()[0] >= limiter[3]) {
				if (navigation.isNavigating() == false) {
					navigation.forward();
				}
			}
			navigation.stopRobot();

		} else if (test == 1) {
			odometer.setXYT(limiter[3], limiter[2], 0);
			while (odometer.getXYT()[1] <= limiter[0]) {
				navigation.forward();
				distToCube = fetchUS();
				if (distToCube < xDist) {
					Sound.beep();
				}
			}
			System.out.println();
			System.out.println();
			navigation.stopRobot();
			navigation.turnTo(90);
			lightLocalizer.localizeXMid();
			while (odometer.getXYT()[0] <= limiter[1]) {
				navigation.forward();
				distToCube = fetchUS();
				if (distToCube < yDist) {
					Sound.beep();
				}
			}
			System.out.println();
			System.out.println();
			navigation.stopRobot();
			navigation.turnTo(180);
			lightLocalizer.localizeYMid();
			while (odometer.getXYT()[1] >= limiter[2]) {
				navigation.forward();
				distToCube = fetchUS();
				if (distToCube < xDist) {
					Sound.beep();
					System.out.print("Beep  ");
				}
			}
			System.out.println();
			System.out.println();
			navigation.stopRobot();
			navigation.turnTo(270);
			lightLocalizer.localizeXMid();
			while (odometer.getXYT()[0] >= limiter[3]) {
				navigation.forward();
				distToCube = fetchUS();
				if (distToCube < yDist) {
					Sound.beep();
					System.out.print("Beep  ");
				}
			}
			System.out.println();
			System.out.println();

		}

	}

	/**
	 * A method to get the distance from our sensor
	 * 
	 * @return
	 */
	public int fetchUS() {
		float[] data = new float[10];
		float distance;
		for (int i = 0; i < data.length; i++) {
			usDistance.fetchSample(usData, 0);
			distance = (int) ((usData[0] * 100));
			data[i] = distance;
		}

		float average = 0;
		for (int i = 1; i < data.length; i++) {
			average = average + data[i];
		}
		average = average / (data.length - 1);
		return (int) average;
	}

	/**
	 * Class Area stores the coordinates of an identified cube
	 * 
	 * @author Volen Mihaylov
	 *
	 */
	private class Area {

		private double x;
		private double y;

		public Area(double x, double y) {
			this.x = x;
			this.y = y;
		}

		public Area(double detectedX, double detectedY, int moving) {
			// Moving Up =0, moving right = 1, moving down = 2, moving left =3
			switch (moving) {
			case 0:
				this.x = detectedX + 5;
				this.y = detectedY - 1 + 5;
				break;
			case 1:
				this.x = detectedX - 1 + 5;
				this.y = detectedY - 5;
				break;
			case 2:
				this.x = detectedX - 5;
				this.y = detectedY + 1 - 5;
				break;
			case 3:
				this.x = detectedX + 1 - 5;
				this.y = detectedY + 5;
				break;
			}
		}

		public boolean isIn(Area area) {
			return Math.hypot(area.x - this.x, area.y - this.y) <= 14.14; // Sum of radii has to be less than distance
																			// between centers
		}
	}
}
