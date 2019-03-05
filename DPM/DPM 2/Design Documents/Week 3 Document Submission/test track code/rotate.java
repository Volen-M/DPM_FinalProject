package testtrack;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
public class rotate {

	
	  private static final int ROTATE_SPEED = 150;
	
	  public static void drive(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
		      double leftRadius, double rightRadius, double track) {
		    // reset the motors
		    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		      motor.stop();
		      motor.setAcceleration(3000);
		    }

		    // Sleep for 2 seconds
		    try {
		      Thread.sleep(2000);
		    } catch (InterruptedException e) {
		     
		    }
		    leftMotor.setSpeed(ROTATE_SPEED);
		    rightMotor.setSpeed(ROTATE_SPEED);

		    leftMotor.rotate(convertAngle(leftRadius, track, 360.0), true);
		    rightMotor.rotate(-convertAngle(rightRadius, track, 360.0), false);
		  }
	  private static int convertDistance(double radius, double distance) {
		    return (int) ((180.0 * distance) / (Math.PI * radius));
		  }

		  private static int convertAngle(double radius, double width, double angle) {
		    return convertDistance(radius, Math.PI * width * angle / 360.0);
		  }
}
