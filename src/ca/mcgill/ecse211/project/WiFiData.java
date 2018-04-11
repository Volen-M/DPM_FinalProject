package ca.mcgill.ecse211.project;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;

/**
 * WifiData class, used to retrieve the necessary demo data from the server.
 * processData() is called from the Controller class when necessary.
 * 
 * @author Michael Smith, Tharsan Ponnampalam
 * @author Patrick Ghazal
 * @author Volen Mihaylov
 *
 */
public class WiFiData {

	// ** Set these as appropriate for your team and current situation **
	private static final String SERVER_IP = "192.168.2.27";
	private static final int TEAM_NUMBER = 11;

	// Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

	// Received data
	// For the beta demo
	public static int greenTeam, greenCorner, redLLX, redLLY, redURX, redURY, greenLLX, greenLLY, greenURX, greenURY;
	public static int tnLLX, tnLLY, tnURX, tnURY, brLLX, brLLY, brURX, brURY;

	// Other data for the final project
	public static int og, or, redTeam, redCorner, srLLX, srLLY, srURX, srURY, sgLLX, sgLLY, sgURX, sgURY;

	// End of received data

	/**
	 * Retrieve the data from the server. Sets the attributes to their values.
	 */
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
			redTeam = ((Long) data.get("RedTeam")).intValue();
			redCorner = ((Long) data.get("RedCorner")).intValue();
			redLLX = ((Long) data.get("Red_LL_x")).intValue();
			redLLY = ((Long) data.get("Red_LL_y")).intValue();
			redURX = ((Long) data.get("Red_UR_x")).intValue();
			redURY = ((Long) data.get("Red_UR_y")).intValue();
			greenLLX = ((Long) data.get("Green_LL_x")).intValue();
			greenLLY = ((Long) data.get("Green_LL_y")).intValue();
			greenURX = ((Long) data.get("Green_UR_x")).intValue();
			greenURY = ((Long) data.get("Green_UR_y")).intValue();
			tnLLX = ((Long) data.get("TN_LL_x")).intValue();
			tnLLY = ((Long) data.get("TN_LL_y")).intValue();
			tnURX = ((Long) data.get("TN_UR_x")).intValue();
			tnURY = ((Long) data.get("TN_UR_y")).intValue();
			brLLX = ((Long) data.get("BR_LL_x")).intValue();
			brLLY = ((Long) data.get("BR_LL_y")).intValue();
			brURX = ((Long) data.get("BR_UR_x")).intValue();
			brURY = ((Long) data.get("BR_UR_y")).intValue();
			og = ((Long) data.get("OG")).intValue();
			or = ((Long) data.get("OR")).intValue();
			srLLX = ((Long) data.get("SR_LL_x")).intValue();
			srLLY = ((Long) data.get("SR_LL_y")).intValue();
			srURX = ((Long) data.get("SR_UR_x")).intValue();
			srURY = ((Long) data.get("SR_UR_y")).intValue();
			sgLLX = ((Long) data.get("SG_LL_x")).intValue();
			sgLLY = ((Long) data.get("SG_LL_y")).intValue();
			sgURX = ((Long) data.get("SG_UR_x")).intValue();
			sgURY = ((Long) data.get("SG_UR_y")).intValue();

			if (TEAM_NUMBER == redTeam) {
				Controller.setCurrentTeam("red");
				Controller.setTargetBlock(og);
				int[] targetSearchArea = {sgLLX, sgLLY, sgURX, sgURY};
				Controller.setSearchAreaCoords(targetSearchArea);
			} else if (TEAM_NUMBER == greenTeam) {
				Controller.setCurrentTeam("green");
				Controller.setTargetBlock(or);
				int[] targetSearchArea = {srLLX, srLLY, srURX, srURY};
				Controller.setSearchAreaCoords(targetSearchArea);
			}

			// ----------------------------------------------
		} catch (Exception e) {
			System.err.println("Error: " + e.getMessage());
		}

		// Wait until user decides to end program
	}
}
