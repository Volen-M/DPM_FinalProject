package ca.mcgill.ecse211.project;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * Class that finds the needed cube in a certain rectangular area amongst other
 * cubes
 * 
 * @author Volen Mihaylov
 */
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

	public SearchAndLocalize(Navigation navigation, ColourCalibration colourCalibration, SampleProvider usDistance, Odometer odometer, int targetBlock, double ur_x, double ur_y, double ll_x, double ll_y, int cornerOfZone) {
		this.navigation = navigation;
		this.colourCalib = colourCalibration;
		this.lowerLeftX = ll_x;
		this.lowerLeftY = ll_y;
		this.upperRightX = ur_x;
		this.upperRightX = ur_y;
		this.targetBlock = targetBlock;
		this.limiter = new double[4];
		this.corner = cornerOfZone;
		this.odometer = odometer;
		this.usDistance = usDistance;
		this.usData = new float[this.usDistance.sampleSize()];
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

	public void findFlag() {
		if (corner == 0 || corner == 2) {
			while (odometer.getXYT()[1]<=limiter[0]) {
				navigation.forward();
				
			}
			while (odometer.getXYT()[0]<=limiter[1]) {

			}
			while (odometer.getXYT()[1]<=limiter[2]) {

			}
			while (odometer.getXYT()[0]<=limiter[3]) {

			}
		}
		else {

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

}
