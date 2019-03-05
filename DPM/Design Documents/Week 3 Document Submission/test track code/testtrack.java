package testtrack;


import lejos.hardware.ev3.LocalEV3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class testtrack {
	 private static final EV3LargeRegulatedMotor leftMotor =
		      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
		  private static final EV3LargeRegulatedMotor rightMotor =
		      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
		 
		  public static final double WHEEL_RAD = 2.1;
		  public static final double TRACK = 12.3;
		  
		  public static void main(String[] args) {
			  
			  (new Thread() {
			        public void run() {
			          rotate.drive(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK);
			        }
			      }).start();
		  }
}
