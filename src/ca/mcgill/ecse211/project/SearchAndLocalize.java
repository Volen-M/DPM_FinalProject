package ca.mcgill.ecse211.project;

import lejos.hardware.Sound;

/**
 * Class that finds the needed cube in a certain rectangular area amongst other
 * cubes
 * 
 * 
 * @author Volen Mihaylov
 * @author Patrick Ghazal
 * @author Bryan Jay
 */
public class SearchAndLocalize {
	public double lowerLeftX, lowerLeftY;
	public double upperRightX, upperRightY;
	private int targetBlock;
	private Navigation navigation;
	private boolean foundBlock = false;
	private ColourCalibration colourCalib;
	private double[] limiter;

	public SearchAndLocalize(Navigation nav, ColourCalibration cc, int targetBlock, double ur_x, double ur_y, double ll_x, double ll_y, int corner) {
		this.navigation = nav;
		this.colourCalib = cc;
		this.lowerLeftX = ll_x;
		this.lowerLeftY = ll_y;
		this.upperRightX = ur_x;
		this.upperRightX = ur_y;
		this.targetBlock = targetBlock;
		limiter = new double[4];

		switch (corner) {
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

	}
}
