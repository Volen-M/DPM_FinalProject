package ca.mcgill.ecse211.project;

/**
 * Class that contain robot constants and uses them to convert distance/turn to
 * wheel rotation
 * 
 * @author Volen Mihaylov
 * @author Patrick Ghazal
 *
 */
public final class Robot {
	public static final int FORWARD_SPEED = 160; //250
	public static final int ROTATE_SPEED = 100;
	public static final int LOCALIZATION_SPEED = 160; //135
	public static final int GEAR_SPEED = 125;
	public static final int ACCELERATION = 500;
	public static final int GEAR_ACCELERATION = 500;
	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 13.447;
	public static final double D = 40.0;
	public static final double K = 2;
	public static final double LSTOWHEEL = 7.5;
	public static final double LSTOLS = 14.65;
	public static final double TRACKOVERLS = TRACK / LSTOLS;
	public static final double TILESIZE = 30.48;

	/**
	 * Constructor that initializes robot object
	 */
	public Robot() {

	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 *            wheel radius
	 * @param distance
	 *            distance to be traveled
	 * @return number of rotations to travel distance
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method allows the conversion of a angle to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 *            wheel radius
	 * @param width
	 *            width of vehicle
	 * @param degrees
	 *            angle to travel
	 * @return number of rotations to rotate by angle
	 */
	public static int convertAngle(double radius, double width, double degrees) {
		return convertDistance(radius, Math.PI * width * degrees / 360.0);
	}

	/**
	 * Calculates distance between (x1, y1) and (x2, y2)
	 * 
	 * @param x1
	 *            x-coord of the current position
	 * @param y1
	 *            y-coord of the current position
	 * @param x2
	 *            x-coord of the destination
	 * @param y2
	 *            y-coord of the destination
	 * @return distance distance between current position and destination
	 */
	public static double calculateDistance(double x1, double y1, double x2, double y2) {
		return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
	}

}
