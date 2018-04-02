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
	private double[][] destinations;

	public SearchAndLocalize(Navigation nav, ColourCalibration cc) {
		this.navigation = nav;

		this.colourCalib = cc;

	}
}
