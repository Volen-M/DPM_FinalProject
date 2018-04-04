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
	private int targetBlock;
	private Navigation navigation;
	private boolean foundBlock = false;
	private ColourCalibration colourCalib;
	private Odometer odometer;
	private int corner;
	private double[] limiter;
	private float[] usData;
	private SampleProvider usDistance;
	private double constant = 4;
	private ArrayList<Area> foundCubes;

	public SearchAndLocalize(Navigation navigation, ColourCalibration colourCalibration, SampleProvider usDistance, Odometer odometer,
			double ur_x, double ur_y, double ll_x, double ll_y, int cornerOfZone) {
		this.navigation = navigation;
		this.colourCalib = colourCalibration;
		this.lowerLeftX = ll_x;
		this.lowerLeftY = ll_y;
		this.upperRightX = ur_x;
		this.upperRightX = ur_y;
		this.limiter = new double[4];
		this.corner = cornerOfZone;
		this.odometer = odometer;
		this.usDistance = usDistance;
		this.usData = new float[this.usDistance.sampleSize()];
		this.foundCubes = new ArrayList<Area>();
		switch (cornerOfZone) {
		case 0:
			limiter[0]=ur_y;
			limiter[1]=ur_x;
			limiter[2]=ll_y;
			limiter[3]=ll_x;
			break;
		case 1:
			limiter[1]=ur_y;
			limiter[2]=ur_x;
			limiter[3]=ll_y;
			limiter[0]=ll_x;
			break;
		case 2:
			limiter[2]=ur_y;
			limiter[3]=ur_x;
			limiter[0]=ll_y;
			limiter[1]=ll_x;
			break;
		case 3:
			limiter[3]=ur_y;
			limiter[0]=ur_x;
			limiter[1]=ll_y;
			limiter[2]=ll_x;
			break;
		}
	}

	public int findFlag() { //Int return is where robot ended, so 0=North, 1=East, 2=South, 3=West
		double xDist;
		double yDist;
		double distToCube;
		boolean checkCube;
		Area tempArea;
		navigation.setSpeed(Robot.FORWARD_SPEED);
		if (corner == 0 || corner == 2) {
			yDist = Math.abs(limiter[1]-limiter[3]);
			xDist = Math.abs(limiter[0]-limiter[2]);
		} else {
			xDist = Math.abs(limiter[1]-limiter[3]);
			yDist = Math.abs(limiter[0]-limiter[2]);
		}


		if (corner == 0) {
			yDist = Math.abs(limiter[1]-limiter[3]);
			xDist = Math.abs(limiter[0]-limiter[2]);

			while (odometer.getXYT()[1]<=limiter[0]) {
				navigation.forward();
				distToCube = fetchUS();
				if (distToCube < xDist) {
					checkCube = true;
					tempArea = new Area(limiter[3]+distToCube,odometer.getXYT()[1],0);
					for (Area area: foundCubes) {
						if(area.isIn(tempArea)) {
							checkCube = false;
							break;
						}
					}
					if (checkCube) {
						foundCubes.add(tempArea);
						navigation.moveBy(4);
						navigation.turnTo(90);
						navigation.moveBy(distToCube-constant);
						if(colourCalib.colourDetection()) {
							for(int i = 0; i < 3; i++) {
								Sound.beep();
							}
							navigation.moveBy(-1*distToCube);
							navigation.turnTo(0);
							return 3;
						}
						else {
							navigation.moveBy(-1*distToCube);
							navigation.turnTo(0);
						}
					}
				}

			}
			navigation.turnTo(90);
			while (odometer.getXYT()[0]<=limiter[1]) {
				navigation.forward();
				distToCube = fetchUS();
				if (distToCube < xDist) {
					checkCube = true;
					tempArea = new Area(odometer.getXYT()[0],limiter[0]-distToCube,1);
					for (Area area: foundCubes) {
						if(area.isIn(tempArea)) {
							checkCube = false;
							break;
						}
					}
					if (checkCube) {
						foundCubes.add(tempArea);
						navigation.moveBy(4);
						navigation.turnTo(180);
						navigation.moveBy(distToCube-constant);
						if(colourCalib.colourDetection()) {
							for(int i = 0; i < 3; i++) {
								Sound.beep();
							}
							navigation.moveBy(-1*distToCube);
							navigation.turnTo(90);
							return 0;
						}
						else {
							navigation.moveBy(-1*distToCube);
							navigation.turnTo(90);
						}
					}
				}


			}
			navigation.turnTo(180);
			while (odometer.getXYT()[1]>=limiter[2]) {
				navigation.forward();
				distToCube = fetchUS();
				if (distToCube < xDist) {
					checkCube = true;
					tempArea = new Area(limiter[1]-distToCube,odometer.getXYT()[1],2);
					for (Area area: foundCubes) {
						if(area.isIn(tempArea)) {
							checkCube = false;
							break;
						}
					}
					if (checkCube) {
						foundCubes.add(tempArea);
						navigation.moveBy(4);
						navigation.turnTo(270);
						navigation.moveBy(distToCube-constant);
						if(colourCalib.colourDetection()) {
							for(int i = 0; i < 3; i++) {
								Sound.beep();
							}
							navigation.moveBy(-1*distToCube);
							navigation.turnTo(180);
							return 1;
						}
						else {
							navigation.moveBy(-1*distToCube);
							navigation.turnTo(180);
						}
					}
				}
			}
			navigation.turnTo(270);
			while (odometer.getXYT()[0]>=limiter[3]) {
				navigation.forward();
				distToCube = fetchUS();
				if (distToCube < xDist) {
					checkCube = true;
					tempArea = new Area(odometer.getXYT()[0],limiter[2]+distToCube,3);
					for (Area area: foundCubes) {
						if(area.isIn(tempArea)) {
							checkCube = false;
							break;
						}
					}
					if (checkCube) {
						foundCubes.add(tempArea);
						navigation.moveBy(4);
						navigation.turnTo(0);
						navigation.moveBy(distToCube-constant);
						if(colourCalib.colourDetection()) {
							for(int i = 0; i < 3; i++) {
								Sound.beep();
							}
							navigation.moveBy(-1*distToCube);
							navigation.turnTo(270);
							return 2;
						}
						else {
							navigation.moveBy(-1*distToCube);
							navigation.turnTo(270);
						}
					}
				}

			}

			for(int i = 0; i < 6; i++) {
				Sound.beep();
			}
		}
		//		else if (corner == 1) {
		//
		//		}
		//		else if (corner == 2) {
		//
		//		}
		//		else if (corner == 3) {
		//
		//		}
		return -1;
	}

	public void testMethod(int test) {	
		double xDist;
		double yDist;
		double distToCube;
		boolean checkCube;
		Area tempArea;
		yDist = Math.abs(limiter[1]-limiter[3]);
		xDist = Math.abs(limiter[0]-limiter[2]);
		navigation.setSpeed(Robot.FORWARD_SPEED);
		if (test==0) {
			odometer.setXYT(limiter[3], limiter[2], 0);
			while (odometer.getXYT()[1]>=limiter[0]);
			navigation.turnTo(90);
			while (odometer.getXYT()[0]>=limiter[1]);
			navigation.turnTo(180);
			while (odometer.getXYT()[1]>=limiter[2]);
			navigation.turnTo(270);
			while (odometer.getXYT()[0]>=limiter[3]);

		}
		else if (test == 1) {
			odometer.setXYT(limiter[3], limiter[2], 0);
			while (odometer.getXYT()[1]<=limiter[0]) {
				navigation.forward();
				distToCube = fetchUS();
				if (distToCube < xDist) {
					Sound.beep();
					Sound.beep();
				}
			}
			navigation.turnTo(90);
			while (odometer.getXYT()[0]>=limiter[1]) {
				navigation.forward();
				distToCube = fetchUS();
				if (distToCube < xDist) {
					Sound.beep();
					Sound.beep();
				}

			}
			navigation.turnTo(180);
			while (odometer.getXYT()[1]>=limiter[2]) {
				navigation.forward();
				distToCube = fetchUS();
				if (distToCube < xDist) {
					Sound.beep();
					Sound.beep();
				}

			}
			navigation.turnTo(270);
			while (odometer.getXYT()[0]>=limiter[3]) {
				navigation.forward();
				distToCube = fetchUS();
				if (distToCube < xDist) {
					Sound.beep();
					Sound.beep();
				}

			}
		}
		else if (test == 2) {
			odometer.setXYT(limiter[3], limiter[2], 0);
			if (corner == 0) {
				while (odometer.getXYT()[1]<=limiter[0]) {
					navigation.forward();
					distToCube = fetchUS();
					if (distToCube < xDist) {
						checkCube = true;
						if (checkCube) {
							navigation.moveBy(4);
							navigation.turnTo(90);
							navigation.moveBy(distToCube-constant);
							if(colourCalib.colourDetection()) {
								for(int i = 0; i < 3; i++) {
									Sound.beep();
								}
								navigation.moveBy(-1*distToCube);
								navigation.turnTo(0);
								return;
							}
							else {
								navigation.moveBy(-1*distToCube);
								navigation.turnTo(0);
							}
						}
					}

				}
				navigation.turnTo(90);
				while (odometer.getXYT()[0]<=limiter[1]) {
					navigation.forward();
					distToCube = fetchUS();
					if (distToCube < xDist) {
						checkCube = true;
						if (checkCube) {
							navigation.moveBy(4);
							navigation.turnTo(180);
							navigation.moveBy(distToCube-constant);
							if(colourCalib.colourDetection()) {
								for(int i = 0; i < 3; i++) {
									Sound.beep();
								}
								navigation.moveBy(-1*distToCube);
								navigation.turnTo(90);
								return;
							}
							else {
								navigation.moveBy(-1*distToCube);
								navigation.turnTo(90);
							}
						}
					}


				}
				navigation.turnTo(180);
				while (odometer.getXYT()[1]>=limiter[2]) {
					navigation.forward();
					distToCube = fetchUS();
					if (distToCube < xDist) {
						checkCube = true;
						if (checkCube) {
							navigation.moveBy(4);
							navigation.turnTo(270);
							navigation.moveBy(distToCube-constant);
							if(colourCalib.colourDetection()) {
								for(int i = 0; i < 3; i++) {
									Sound.beep();
								}
								navigation.moveBy(-1*distToCube);
								navigation.turnTo(180);
								return;
							}
							else {
								navigation.moveBy(-1*distToCube);
								navigation.turnTo(180);
							}
						}
					}
				}
				navigation.turnTo(270);
				while (odometer.getXYT()[0]>=limiter[3]) {
					navigation.forward();
					distToCube = fetchUS();
					if (distToCube < xDist) {
						checkCube = true;
						if (checkCube) {
							navigation.moveBy(4);
							navigation.turnTo(0);
							navigation.moveBy(distToCube-constant);
							if(colourCalib.colourDetection()) {
								for(int i = 0; i < 3; i++) {
									Sound.beep();
								}
								navigation.moveBy(-1*distToCube);
								navigation.turnTo(270);
								return;
							}
							else {
								navigation.moveBy(-1*distToCube);
								navigation.turnTo(270);
							}
						}
					}

				}

			}
		}
		else if (test == 3) {
			findFlag();
		}
	}
	/**
	 * A method to get the distance from our sensor
	 * 
	 * @return
	 */
	public int fetchUS() {
		usDistance.fetchSample(usData, 0);
		return (int) (usData[0] * 100);
	}

	private class Area {

		private double x;
		private double y;

		public Area(double x, double y) {
			this.x = x;
			this.y = y;
		}

		public Area(double detectedX, double detectedY, int moving) {
			//Moving Up =0, moving right = 1, moving down = 2, moving left =3
			switch (moving){
			case 0:
				this.x= detectedX + 5;
				this.y=detectedY-1+5;
				break;
			case 1:
				this.x= detectedX-1 + 5;
				this.y=detectedY-5;
				break;
			case 2:
				this.x= detectedX-5;
				this.y=detectedY+1-5;
				break;
			case 3:
				this.x= detectedX+1-5;
				this.y=detectedY+5;
				break;
			}
		}
		public boolean isIn(Area area) {
			return Math.hypot(area.x-this.x,area.y-this.y) <= 14.14; //Sum of radii has to be less than distance between centers
		}
	}
}
