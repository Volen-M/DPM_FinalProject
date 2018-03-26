package ca.mcgill.ecse211.project;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;

/**
<<<<<<< HEAD
 * Example class using WifiConnection to communicate with a server and receive
 * data concerning the competition such as the starting corner the robot is
 * placed in.
 * 
 * Keep in mind that this class is an **example** of how to use the WiFi code;
 * you must use the WifiConnection class yourself in your own code as
 * appropriate. In this example, we simply show how to get and process different
 * types of data.
 * 
 * There are two variables you **MUST** set manually before trying to use this
 * code.
 * 
 * 1. SERVER_IP: The IP address of the computer running the server application.
 * This will be your own laptop, until the beta beta demo or competition where
 * this is the TA or professor's laptop. In that case, set the IP to
 * 192.168.2.3.
 * 
 * 2. TEAM_NUMBER: your project team number
 * 
 * Note: We System.out.println() instead of LCD printing so that full debug
 * output (e.g. the very long string containing the transmission) can be read on
 * the screen OR a remote console such as the EV3Control program via Bluetooth
 * or WiFi. You can disable printing from the WiFi code via
=======
 * Example class using WifiConnection to communicate with a server and receive data concerning the
 * competition such as the starting corner the robot is placed in.
 * 
 * Keep in mind that this class is an **example** of how to use the WiFi code; you must use the
 * WifiConnection class yourself in your own code as appropriate. In this example, we simply show
 * how to get and process different types of data.
 * 
 * There are two variables you **MUST** set manually before trying to use this code.
 * 
 * 1. SERVER_IP: The IP address of the computer running the server application. This will be your
 * own laptop, until the beta beta demo or competition where this is the TA or professor's laptop.
 * In that case, set the IP to 192.168.2.3.
 * 
 * 2. TEAM_NUMBER: your project team number
 * 
 * Note: We System.out.println() instead of LCD printing so that full debug output (e.g. the very
 * long string containing the transmission) can be read on the screen OR a remote console such as
 * the EV3Control program via Bluetooth or WiFi. You can disable printing from the WiFi code via
>>>>>>> b7bc67623d694a277a68dae8bd8f18f9e946ee30
 * ENABLE_DEBUG_WIFI_PRINT (below).
 * 
 * @author Michael Smith, Tharsan Ponnampalam
 *
 */
public class WiFiData {

<<<<<<< HEAD
	// ** Set these as appropriate for your team and current situation **
	private static final String SERVER_IP = "192.168.2.3";
	private static final int TEAM_NUMBER = 7;

	// Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

	// Received data
	// For the beta demo
	public static int greenTeam, greenCorner, redLLX, redLLY, redURX, redURY, greenLLX, greenLLY, greenURX, greenURY;
	public static int tnLLX, tnLLY, tnURX, tnURY, brLLX, brLLY, brURX, brURY;

	// Other data for the final project
	public static int og, or, redTeam, redCorner, srLLX, srLLY, srURX, srURY, sgLLX, sgLLY, sgURX, sgURY;

	// End of received data

	private static boolean betaDemo = true;

	@SuppressWarnings("rawtypes")
	public static void processData() {

		System.out.println("Running..");

		// Initialize WifiConnection class
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

		// Connect to server and get the data, catching any errors that might occur
		try {
			Map data = conn.getData();

			greenTeam = ((Long) data.get("GreenTeam")).intValue();
			greenCorner = ((Long) data.get("GreenCorner")).intValue();
			redLLX = ((Long) data.get("Red_LL_X")).intValue();
			redLLY = ((Long) data.get("Red_LL_Y")).intValue();
			redURX = ((Long) data.get("Red_UR_X")).intValue();
			redURY = ((Long) data.get("Red_UR_Y")).intValue();
			greenLLX = ((Long) data.get("Green_LL_X")).intValue();
			greenLLY = ((Long) data.get("Green_LL_Y")).intValue();
			greenURX = ((Long) data.get("Green_UR_X")).intValue();
			greenURY = ((Long) data.get("Green_UR_Y")).intValue();
			tnLLX = ((Long) data.get("TN_LL_X")).intValue();
			tnLLY = ((Long) data.get("TN_LL_Y")).intValue();
			tnURX = ((Long) data.get("TN_UR_X")).intValue();
			tnURY = ((Long) data.get("TN_UR_Y")).intValue();
			brLLX = ((Long) data.get("BR_LL_X")).intValue();
			brLLY = ((Long) data.get("BR_LL_Y")).intValue();
			brURX = ((Long) data.get("BR_UR_X")).intValue();
			brURY = ((Long) data.get("BR_UR_Y")).intValue();

			if (!betaDemo) {
				redTeam = ((Long) data.get("RedTeam")).intValue();
				redCorner = ((Long) data.get("RedCorner")).intValue();

				// ----------------------------------------------
				// Values to be used after the beta demo
				og = ((Long) data.get("OG")).intValue();
				or = ((Long) data.get("OR")).intValue();
				srLLX = ((Long) data.get("SR_LL_X")).intValue();
				srLLY = ((Long) data.get("SR_LL_Y")).intValue();
				srURX = ((Long) data.get("SR_UR_X")).intValue();
				srURY = ((Long) data.get("SR_UR_Y")).intValue();
				sgLLX = ((Long) data.get("SG_LL_X")).intValue();
				sgLLY = ((Long) data.get("SG_LL_Y")).intValue();
				sgURX = ((Long) data.get("SG_UR_X")).intValue();
				sgURY = ((Long) data.get("SG_UR_Y")).intValue();
				// ----------------------------------------------
			}

			if (TEAM_NUMBER == redTeam) {
				Navigation.setCurrentZone("red");
				Navigation.setStartingCorner(redCorner);
			} else if (TEAM_NUMBER == greenTeam) {
				Navigation.setCurrentZone("green");
				Navigation.setStartingCorner(greenCorner);
			}
		} catch (Exception e) {
			System.err.println("Error: " + e.getMessage());
		}

		// Wait until user decides to end program
		Button.waitForAnyPress();
	}
=======
  // ** Set these as appropriate for your team and current situation **
  private static final String SERVER_IP = "192.168.2.3";
  private static final int TEAM_NUMBER = 7;

  // Enable/disable printing of debug info from the WiFi class
  private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

  @SuppressWarnings("rawtypes")
  public static void main(String[] args) {

    System.out.println("Running..");

    // Initialize WifiConnection class
    WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

    // Connect to server and get the data, catching any errors that might occur
    try {
      /*
       * getData() will connect to the server and wait until the user/TA presses the "Start" button
       * in the GUI on their laptop with the data filled in. Once it's waiting, you can kill it by
       * pressing the upper left hand corner button (back/escape) on the EV3. getData() will throw
       * exceptions if it can't connect to the server (e.g. wrong IP address, server not running on
       * laptop, not connected to WiFi router, etc.). It will also throw an exception if it connects
       * but receives corrupted data or a message from the server saying something went wrong. For
       * example, if TEAM_NUMBER is set to 1 above but the server expects teams 17 and 5, this robot
       * will receive a message saying an invalid team number was specified and getData() will throw
       * an exception letting you know.
       */
      Map data = conn.getData();

      // Example 1: Print out all received data
      System.out.println("Map:\n" + data);

      // Example 2 : Print out specific values
      int redTeam = ((Long) data.get("RedTeam")).intValue();
      System.out.println("Red Team: " + redTeam);

      int og = ((Long) data.get("OG")).intValue();
      System.out.println("Green opponent flag: " + og);

      // Example 3: Compare value
      int tn_ll_x =  ((Long) data.get("TN_LL_x")).intValue();
      if (tn_ll_x < 5) {
        System.out.println("Tunnel LL corner X < 5");
      }
      else {
        System.out.println("Tunnel LL corner X >= 5");
      }

    } catch (Exception e) {
      System.err.println("Error: " + e.getMessage());
    }

    // Wait until user decides to end program
    Button.waitForAnyPress();
  }
>>>>>>> b7bc67623d694a277a68dae8bd8f18f9e946ee30
}
