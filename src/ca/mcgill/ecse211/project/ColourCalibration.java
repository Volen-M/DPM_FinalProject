package ca.mcgill.ecse211.project;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * Class for everything color-identification related. Makes use of the Gaussian
 * distribution to identify the detected color.
 * 
 * @author Bryan Jay
 * @author Volen Mihaylov
 * @author Patrick Ghazal
 *
 */
public class ColourCalibration {

	private static final EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
	private static final double[] RGB_blue = { 0.023333, 0.043156, 0.0590942 };
	private static final double[] RGB_red = { 0.122568, 0.019627, 0.013738 };
	private static final double[] RGB_white = { 0.197392, 0.20392, 0.125490 };
	private static final double[] RGB_yellow = { 0.233333, 0.132352, 0.021470 };

	private static final double[] std_blue = { 0.0183301, 0.0157346, 0.0154033 };
	private static final double[] std_red = { 0.0492499, 0.0093156, 0.0090942 };
	private static final double[] std_yellow = { 0.093333, 0.063156, 0.00990942 };
	private static final double[] std_white = { 0.0953342, 0.089224356, 0.05930942 };
	private colour currentBlock;
	private int flag;

	// Used to know whether the calibration is used for part 1 or 2 of the demo
	public boolean isFieldSearching = false;

	/*
	 * 0 == red, 1 == blue, 2 == yellow, 3 == white
	 */
	public static enum colour {
		RED, BLUE, YELLOW, WHITE
	}

	/**
	 * ColourCalibration constructor. Sets the light sensor mode for the sensor.
	 */
	public ColourCalibration() {
		lightSensor.setCurrentMode("Red");
	}
	
	/**
	 * Set the value of the target flag.
	 * @param flag integer representation of the target flag (0 to 3)
	 */
	public void setFlag(int flag) {
		if (flag >= 0 && flag <= 3) {
			this.flag = flag;
		}
	}

	/**
	 * Compares the current block and the flag
	 * 
	 * @return boolean current block is the same color as the flag
	 */
	public boolean isBlock() {

		if (flag == currentBlock.ordinal()) {
			Sound.beepSequenceUp();
			return true;
		} else {
			return false;
		}
	}

	/**
	 * Reads sensor sample, sets currentBlock attribute if a colour is identified.
	 * 
	 * @return boolean whether the newly-identified block is the target
	 */
	public boolean colourDetection() {

		float[] RGB = new float[3];
		RGB = getRGB();

		currentBlock = null;

		if (Math.abs(RGB[0] - RGB_blue[0]) <= 2 * std_blue[0] && Math.abs(RGB[1] - RGB_blue[1]) <= 2 * std_blue[1]
				&& Math.abs(RGB[2] - RGB_blue[2]) <= 2 * std_blue[2]) {
			currentBlock = colour.BLUE;

		} else if (Math.abs(RGB[0] - RGB_red[0]) <= 2 * std_red[0] && Math.abs(RGB[1] - RGB_red[1]) <= 2 * std_red[1]
				&& Math.abs(RGB[2] - RGB_red[2]) <= 2 * std_red[2]) {
			currentBlock = colour.RED;
		} else if (Math.abs(RGB[0] - RGB_yellow[0]) <= 2 * std_yellow[0]
				&& Math.abs(RGB[1] - RGB_yellow[1]) <= 2 * std_yellow[1]
				&& Math.abs(RGB[2] - RGB_yellow[2]) <= 2 * std_yellow[2]) {
			currentBlock = colour.YELLOW;

		} else if (Math.abs(RGB[0] - RGB_white[0]) <= 2 * std_white[0]
				&& Math.abs(RGB[1] - RGB_white[1]) <= 2 * std_white[1]
				&& Math.abs(RGB[2] - RGB_white[2]) <= 2 * std_white[2]) {
			currentBlock = colour.WHITE;
		}

		return isBlock();
	}

	/**
	 * Gets the RGB values from the colour sensor.
	 * 
	 * @return RGB array of sensor data
	 */
	public float[] getRGB() {
		lightSensor.setCurrentMode("RGB");
		SampleProvider colorSensor = lightSensor.getRGBMode();
		float[] RGB = new float[colorSensor.sampleSize()];
		colorSensor.fetchSample(RGB, 0);
		return RGB;
	}

	/**
	 * Returns the colour of the last-detected block
	 * 
	 * @return
	 */
	public colour getCurrentBlock() {
		return currentBlock;
	}

	/**
	 * Sets currentBlock to null, erasing the data on the last-detected block
	 */
	public void resetBlock() {
		currentBlock = null;
	}

}
