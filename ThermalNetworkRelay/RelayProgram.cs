using System;
using System.Threading;
using System.Collections;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using SecretLabs.NETMF.Hardware;
using SecretLabs.NETMF.Hardware.Netduino;
using NETMF.OpenSource.XBee;
using NETMF.OpenSource.XBee.Api;
using NETMF.OpenSource.XBee.Api.Zigbee;
using RuleDays = ThermalNetworkRelay.TemperatureRule.RuleDays;
using AnalogChannels = SecretLabs.NETMF.Hardware.Netduino.AnalogChannels;
using System.IO.Ports;

namespace ThermalNetworkRelay {

	public class Program {
		//=====================================================================
		// PORT SETUP
		//=====================================================================
		// Analog input ports
		private static AnalogInput pwrInput = new AnalogInput(AnalogChannels.ANALOG_PIN_A0);	// Analog input to read thermostat power status

		// Digital output ports
		private static OutputPort relayStatusOutput = new OutputPort(Pins.GPIO_PIN_D5, false);		// Output port for the relay status LED and/or the relay control
		private static OutputPort powerStatusOutput = new OutputPort(Pins.GPIO_PIN_D10, true);		// Output port for the power status LED
		private static OutputPort onboardLED = new OutputPort(Pins.ONBOARD_LED, false);				// Turn off the onboard LED

		// Relay control ports
		private static OutputPort relayPinOn = new OutputPort(Pins.GPIO_PIN_D6, false);		// Turning port high will close the relay, for latching relay
		private static OutputPort relayPinOff = new OutputPort(Pins.GPIO_PIN_D7, false);	// Turning port high will open the relay, for latching relay
		private static OutputPort relayControlOutput = new OutputPort(Pins.GPIO_PIN_D11, false);	// Output port to control the relay, for non-latching relay

		//=====================================================================
		// THERMOSTAT CONTROL MEMBERS
		//=====================================================================
		// Basic status members
		private static bool thermoOn = true;	// Keeps track of whether the thermostat is on or off
		private static bool relayOn = false;	// Keeps track of whether the relay is on or off
		private static bool overrideOn = false;	// Keeps track of whether the programming override mode is on or off
		private static double overrideTemp = 0;	// Contains the override temperature the thermostat is targetting

		// Measurement members
		private static float temperature;		// Contains the most recent measured temperature
		private static float humidity;			// Contains the most recent measured humidity
		private static float luminosity;		// Contains the most recent measured luminosity

		// Timing variables
		private const int CONTROL_INTERVAL = 60000;		// The number of microseconds between control evaluations
		private const int SENSOR_PERIODS = 5;			// DEBUGGING - The number of control periods before a sensor evaluation
//		private const int SENSOR_PERIODS = 10;			// The number of control periods before a sensor evaluation
		private static int controlLoops = 0;			// Tracks the current number of control loops without a sensor loop
		private static bool sensorSent = false;			// Tracks whether the controller is waiting for a sensor acknowledgement
		private const int RELAY_DELAY = 10;				// The number of milliseconds to power the relay state to ensure a transition

		// Thermostat rule variables
		private static ArrayList rules;	// Array holding the thermostat rules
		private const double MIN_TEMPERATURE = 16.0;	// Below this temperature, the relay opens no matter the programming
		private const double MAX_TEMPERATURE = 25.0;	// Above this temperature, the relay closes no matter the programming
		private const double TEMPERATURE_BUFFER = 0.15;	// The buffer to apply to the target temperature in evaluation relay status

		// Constants
		private const double TEMP_UNDEFINED = 200.0;	// High temperature values signifies it has not been set

		//=====================================================================
		// XBEE SETUP
		//=====================================================================
		// XBee sensor codes
		private enum XBeePortData { Temperature, Luminosity, Pressure, Humidity, LuminosityLux, HeatingOn, ThermoOn, Power }

		// XBee data codes
		const byte TEMPERATURE_CODE	=   1;
		const byte LUMINOSITY_CODE	=   2;
		const byte PRESSURE_CODE	=   3;
		const byte HUMIDITY_CODE	=   4;
		const byte POWER_CODE		=   5;
		const byte LUX_CODE			=   6;
		const byte HEATING_CODE		=   7;
		const byte THERMOSTAT_CODE	=   8;
		const byte TEMP_12BYTE_CODE	=   9;

		// XBee command codes
		const byte CMD_THERMO_POWER	= 1;
		const byte CMD_OVERRIDE		= 2;
		const byte CMD_RULE_CHANGE	= 3;
		const byte CMD_SENSOR_DATA	= 4;
		const byte CMD_TIME_REQUEST	= 5;
		const byte CMD_STATUS		= 6;

		// XBee subcommand codes
		const byte CMD_NACK			= 0;
		const byte CMD_ACK			= 1;
		const byte STATUS_OFF		= 2;
		const byte STATUS_ON		= 3;
		const byte STATUS_GET		= 4;
		const byte STATUS_ADD		= 5;
		const byte STATUS_DELETE	= 6;
		const byte STATUS_MOVE		= 7;
		const byte STATUS_UPDATE	= 8;

		// XBee Connection Members
		private static XBeeApi xBee;				// The object controlling the interface to the XBee radio
		private static bool xbeeConnected = false;	// A flag to indicate there is a connection to the XBee (true) or not (false)
		const string COORD_ADDRESS = "00 00 00 00 00 00 00 00";	// The address of the coordinator

		//=====================================================================
		// DATA LOGGER SETUP
		//=====================================================================
		private static SerialPort dataLogger = null;		// The port of the OpenLogger device
		private static LogCode LogLevel = LogCode.Status;	// Identifies the lowest level messages to log

		// Log types enum
		private enum LogCode { Data, Error, System, Warning, Status }

		//=====================================================================
		// SENSOR SETUP
		//=====================================================================
		private static HTU21DBusSensor tempSensor = new HTU21DBusSensor();
		private static TSL2561BusSensor luxSensor = new TSL2561BusSensor();
		private static DS1307BusSensor timeKeeper = new DS1307BusSensor();

		//=====================================================================
		// MAIN PROGRAM
		//=====================================================================
		public static void Main() {
			try {
				//-----------------------------------------------------------------
				// INITIALIZE THE TIME, RADIOS, TIMERS AND RULES
				//-----------------------------------------------------------------
				// Set the time on the netduino on startup from the DS1307 clock
				Utility.SetLocalTime(timeKeeper.GetTime().getDateTime());

				// Initialize the relay status
				SetRelay(false);

				// Initialize the logger
				dataLogger = new SerialPort("COM2", 9600);	// RX and TX lines connected to digital pins 2 and 3 for COM2
				dataLogger.Open();

				// Log the startup time
				LogMessage(LogCode.System, "Device restarted with relay OFF");

				// Initialize the XBee
				LogMessage(LogCode.Status, "Initializing XBee");
				xBee = new XBeeApi("COM1", 9600);	// RX and TX lines connected to digital pins 0 and 1 for COM1
				xBee.EnableDataReceivedEvent();
				xBee.EnableAddressLookup();
				xBee.EnableModemStatusEvent();
				xBee.DataReceived += xBee_RequestReceived;
				NETMF.OpenSource.XBee.Util.Logger.Initialize(Debug.Print, NETMF.OpenSource.XBee.Util.LogLevel.All);

				// Connect to the XBee
				ConnectToXBee();

				// Create the default rules
				rules = new ArrayList();
				rules.Add(new TemperatureRule(RuleDays.Weekdays, 23.5, 19.0));
				rules.Add(new TemperatureRule(RuleDays.Weekdays, 16.5, 22.0));
				rules.Add(new TemperatureRule(RuleDays.Weekdays, 8.0, 18.0));
				rules.Add(new TemperatureRule(RuleDays.Weekdays, 6.5, 22.0));
				rules.Add(new TemperatureRule(RuleDays.Weekends, 23.5, 19.0));
				rules.Add(new TemperatureRule(RuleDays.Weekends, 7.5, 22.0));

				// Setup and start the timer
				Timer dataPoll = new Timer(new TimerCallback(OnTimer), null, 5000, CONTROL_INTERVAL);	// Timer delays for 5 seconds first time around, then every control interval

				//-----------------------------------------------------------------
				// INFINTE LOOP TO CHECK POWER STATUS
				//-----------------------------------------------------------------
				while(true) {
					// Check the status of the thermostat based on power from on/off switch (high = on; low = off)
					double powerLevel = 3.3*pwrInput.Read();	// The .Read() method return the fraction of the full pin voltage (3.3 V), with some offset which isn't important for this basic switch

					// Evaluate the thermostat and relay control based on the current voltage level
					if((powerLevel > 1.5) && !thermoOn) SetPowerMode(true);			// Turn on the thermostat if previously off
					else if((powerLevel < 1.5) && thermoOn) SetPowerMode(false);	// Turn off the thermostat if previously on
				}
			} catch(Exception ex) {
				// Log any unhandled exceptions before the code freezes - may not work if the expection is thrown before the datalogger is initialized
				if(dataLogger != null) LogMessage(LogCode.Error, "Unhandled exception found with message => " + ex.Message);
			}
		}

		//=====================================================================
		// ConnectToXBee
		//=====================================================================
		/// <summary>
		/// The method to connect to the XBee through the API
		/// </summary>
		/// <returns>Whether the connection was successfull</returns>
		private static bool ConnectToXBee() {
			//-----------------------------------------------------------------
			// ONLY CONNECT IF NOT ALREADY CONNECTED
			//-----------------------------------------------------------------
			if(!xbeeConnected) {
				try {
					// Connect to the XBee
					xBee.Open();
					xbeeConnected = true;	// Set connected status
					LogMessage(LogCode.Status, "XBee connected");
				} catch(Exception xbeeIssue) {	// This assumes only xBee.Open command throws exceptions
					LogMessage(LogCode.Error, "Caught the following trying to open the XBee connection: " + xbeeIssue.Message);
					return false;	// Signal that the connection failed
				}
			}

			// If the code gets here, the xbee is connected
			return true;
		}

		//=====================================================================
		// XBEE DATA RECEIVED EVENT HANDLER (xBee_RequestReceived)
		//=====================================================================
		/// <summary>
		/// Function that handles the XBee data received event (TxRequest type)
		/// </summary>
		/// <param name="receiver">The API for the receiving XBee</param>
		/// <param name="data">The payload data from the request</param>
		/// <param name="sender">The address of the sending XBee</param>
		static void xBee_RequestReceived(XBeeApi receiver, byte[] data, XBeeAddress sender) {
			// Format the data packet to correct the escape charaters
			byte[] request = FormatApiMode(data, true);

			// Print out the received request
			string message = "Received the following message from " + sender.ToString() + ": ";
			for(int i = 0; i < request.Length; i++) message += request[i].ToString("X") + (i == (request.Length - 1) ? "" : "-");
			Debug.Print(message);

			// Process the request and get the response data
			byte[] response = ProcessRequest(request);

			// Send the response
			if(response != null) SendXBeeTransmission(FormatApiMode(response, false), sender);
		}

		//=====================================================================
		// ProcessRequest
		//=====================================================================
		/// <summary>
		/// Processes a request received through the XBee interface
		/// </summary>
		/// <param name="command">The data packet recieved in the XBee transmission</param>
		/// <returns>The response to send back to the sender of the transmission</returns>
		private static byte[] ProcessRequest(byte[] command) {
			// Setup the response packet
			byte[] dataPacket = null;

			//-----------------------------------------------------------------
			// DETERMINE THE TYPE OF PACKET RECEIVED AND ACT ACCORDINGLY 
			//-----------------------------------------------------------------
			switch(command[0]) {
				//-------------------------------------------------------------
				case CMD_THERMO_POWER:	// Command sent to power on/off the thermostat
					LogMessage(LogCode.Warning, "Received command to change thermostat power status to " + (command[1] == STATUS_ON ? "ON" : "OFF") + " - NOT IMPLEMENTED IN HARDWARE");
					dataPacket = new byte[] { CMD_THERMO_POWER, CMD_ACK };	// Acknowledge the command
					break;
				//-------------------------------------------------------------
				case CMD_OVERRIDE:	// Command to turn on/off the override and set the target temperature
					dataPacket = new byte[] { CMD_OVERRIDE, CMD_ACK };	// By default, set the response to an acknowledgement

					// Check status flag
					switch(command[1]) {
						case STATUS_OFF:	// Turn off override mode
							overrideOn = false;	// Turn off override status
							LogMessage(LogCode.Status, "Received command to turn off override mode");
							break;
						case STATUS_ON:	// Turn on override mode
							// Convert the byte array to a float (1st byte is the command, 2nd to 5th bytes are the float)
							byte[] tempArray = new byte[4];
							for(int i = 0; i < 4; i++) tempArray[i] = command[i+2];	// Copy the byte array for the float
							overrideTemp = (double) ByteToFloat(tempArray);	// Set the target override temperature

							// Change override status
							overrideOn = true;	// Turn on override status
							LogMessage(LogCode.Status, "Received command to turn on override mode with a target temperature of " + overrideTemp);
							break;
						default:	// Command not defined
							LogMessage(LogCode.Warning, "Received command to override mode (" + command[1] + ") not implemented");
							dataPacket[1] = CMD_NACK;	// Indicate that the command is not understood
							break;
					}
					break;
				//-------------------------------------------------------------
				case CMD_RULE_CHANGE:	// A command to change/view the thermostat rules has been made
					// Take action based on the issued command
					switch(command[1]) {
						//-----------------------
						case STATUS_GET:
							LogMessage(LogCode.Status, "Received request to get thermostat rules");
							dataPacket = ProcessGetRuleCMD();	// Get the rules and incorporate them into the response packet
							break;
						//-----------------------
						case STATUS_ADD:
							// Create default return packet
							LogMessage(LogCode.Status, "Received request to add new thermostat rule");
							dataPacket = new byte[] { CMD_RULE_CHANGE, STATUS_ADD, CMD_NACK };

							// Check that the index makes sense
							if(command[2] <= rules.Count) {
								// Create the rule floats
								byte[] tempArray = new byte[4];
								byte[] timeArray = new byte[4];
								for(int i = 0; i < 4; i++) {
									timeArray[i] = command[4+i];
									tempArray[i] = command[8+i];
								}
								float time = ByteToFloat(timeArray);
								float temp = ByteToFloat(tempArray);

								// Add the rule
								TemperatureRule newRule = new TemperatureRule((RuleDays) command[3], time, temp);
								rules.Insert(command[2], newRule);
								dataPacket[2] = CMD_ACK;
							}
							break;
						//-----------------------
						case STATUS_DELETE:
							// Create the default return packet
							LogMessage(LogCode.Status, "Received request to delete thermostat rule");
							dataPacket = new byte[] { CMD_RULE_CHANGE, STATUS_DELETE, CMD_NACK };

							// Delete the entry if it makes sense
							if(command[2] < rules.Count) {
								rules.RemoveAt(command[2]);
								dataPacket[2] = CMD_ACK;
							}
							break;
						//-----------------------
						case STATUS_MOVE:
							// Create the default return packet
							LogMessage(LogCode.Status, "Received request to move thermostat rule");
							dataPacket = new byte[] { CMD_RULE_CHANGE, STATUS_MOVE, CMD_NACK };

							// Check that the indicies are valid
							if((command[2] < rules.Count) && (command[3] < rules.Count)) {
								// Copy the rule
								object moveRule = rules[command[2]];
								rules.RemoveAt(command[2]);
								rules.Insert(command[3], moveRule);

								dataPacket[2] = CMD_ACK;
							}
							break;
						//-----------------------
						case STATUS_UPDATE:
							// Create default return packet
							LogMessage(LogCode.Status, "Received request to update an existing thermostat rule");
							dataPacket = new byte[] { CMD_RULE_CHANGE, STATUS_UPDATE, CMD_NACK };

							// Check that the index makes sense
							if(command[2] < rules.Count) {
								// Create the rule floats
								byte[] tempArray = new byte[4];
								byte[] timeArray = new byte[4];
								for(int i = 0; i < 4; i++) {
									timeArray[i] = command[4+i];
									tempArray[i] = command[8+i];
								}
								float time = ByteToFloat(timeArray);
								float temp = ByteToFloat(tempArray);

								// Add the updated rule and delete the old one
								TemperatureRule updateRule = new TemperatureRule((RuleDays) command[3], time, temp);
								rules.Insert(command[2], updateRule);
								rules.RemoveAt(command[2] + 1);
								dataPacket[2] = CMD_ACK;
							}
							break;
						//-----------------------
						default:
							LogMessage(LogCode.Warning, "Received command for thermostat rule change (" + command[1] + ") that has not been implemented yet");
							dataPacket = new byte[] { CMD_RULE_CHANGE, CMD_NACK };
							break;
					}
					break;
				//-------------------------------------------------------------
				case CMD_SENSOR_DATA:	// Should only receive this for a sensor data acknoledgement packet
					// Check to see if a sensor data packet was sent recently
					if(sensorSent) {
						switch(command[1]) {
							case CMD_NACK:
								// Write the data to the logger
								LogMessage(LogCode.Data, temperature + "," + luminosity + "," + humidity + ",3.3," + (thermoOn ? "1.0," : "0.0,") + (relayOn ? "1.0" : "0.0"));
								sensorSent = false;	// No need to try and send the data anymore
								break;
							case CMD_ACK:
								sensorSent = false;	// Identify that there isn't a sensor reading not acknowledged
								LogMessage(LogCode.Status, "\tSensor data acknowledged!");
								break;
							default:
								LogMessage(LogCode.Warning, "Received command to sensor data mode (" + command[1] + ") not implemented");
								break;
						}
					} else LogMessage(LogCode.Error, "Receiving sensor command without having sent unacknowledged sensor data - not sure how this happened!");
					break;
				//-------------------------------------------------------------
				case CMD_TIME_REQUEST:	// Interact with the DS1307 on the I2C bus
					// Take action based on issued command
					switch(command[1]) {
						case STATUS_GET:	// Get the current time on the DS1307
							// Get the time from the RTC and create the packet
							LogMessage(LogCode.Status, "Received request to get the current time");
							DS1307BusSensor.RTCTime curTime = timeKeeper.GetTime();
							dataPacket = new byte[] { CMD_TIME_REQUEST, STATUS_GET, curTime.second, curTime.minute, curTime.hour, curTime.weekday, curTime.day, curTime.month, curTime.year };
							break;
						case STATUS_UPDATE:	// Set the time on the DS1307
							// Check that the data is there
							if(command.Length == 9) {
								// Convert to a time structure and send to DS1307
								LogMessage(LogCode.Status, "Received request to set the current time");
								DS1307BusSensor.RTCTime setTime = new DS1307BusSensor.RTCTime(command[2], command[3], command[4], command[6], command[7], command[8], (DS1307BusSensor.DayOfWeek) command[5]);
								timeKeeper.SetTime(setTime);
								dataPacket = new byte[] { CMD_TIME_REQUEST, STATUS_UPDATE, CMD_ACK };
							} else {
								// Return an NACK
								LogMessage(LogCode.Status, "Received command to set the time with incorrect number of command elements (" + command.Length + ")!");
								dataPacket = new byte[] { CMD_TIME_REQUEST, STATUS_UPDATE, CMD_NACK };
							}
							break;
						default:	// Command not implemented
							LogMessage(LogCode.Warning, "Received command to time request mode (" + command[1] + ") not implemented");
							dataPacket = new byte[] { CMD_TIME_REQUEST, command[1], CMD_NACK };
							break;
					}
					break;
				//-------------------------------------------------------------
				case CMD_STATUS:
					LogMessage(LogCode.Status, "Received request to get the current thermostat status");

					// Setup the byte array with thermostat and relay status
					dataPacket = new byte[11];
					dataPacket[0] = CMD_STATUS;
					dataPacket[1] = BitConverter.GetBytes(thermoOn)[0];
					dataPacket[2] = BitConverter.GetBytes(relayOn)[0];

					// Get the tempeature reading
					double curTemp = tempSensor.readTemperature();
					byte[] tArray = BitConverter.GetBytes((float) curTemp);
					for(int i = 0; i < 4; i++) dataPacket[3+i] = tArray[i];

					// Get the time and weekday for evaluating the rules
					double evalTime = DateTime.Now.Hour + DateTime.Now.Minute/60.0 + DateTime.Now.Second/3600.0;
					RuleDays curWeekday = (RuleDays) ((int) DateTime.Now.DayOfWeek);	// Cast the returned DayOfWeek enum into the custome DayType enum

					// Determine the target temperature
					bool ruleFound = false;	// Flags that a rule has been found
					while(!ruleFound) {
						// Iterate through the rules until the active one is found
						for(int i = 0; i < rules.Count; i++) {
							// Check to see if current rule applies
							TemperatureRule curRule = rules[i] as TemperatureRule;
							if(RuleApplies(curRule, curWeekday, evalTime)) {
								// Rule applies, so get the target temperature
								tArray = BitConverter.GetBytes((float) curRule.Temperature);
								for(int j = 0; j < 4; j++) dataPacket[7+j] = tArray[j];

								// Rule found, so break from the loops
								ruleFound = true;
								break;
							}
						}

						// No rule was found to apply, so move the day back before checking against rules again
						if(!ruleFound) {
							// Decrease the indicated day, but increase the time
							if(curWeekday == RuleDays.Sunday) curWeekday = RuleDays.Saturday;
							else curWeekday = (RuleDays) ((int) curWeekday - 1);
							evalTime += 24.0;
						}
					}
					break;
				//-------------------------------------------------------------
				default:// Acknowledge the command, even though it doesn't exist
					LogMessage(LogCode.Warning, "TxRequest type has not been implemented yet");
					dataPacket = new byte[] { CMD_OVERRIDE, CMD_NACK };
					break;
			}

			// Return the response data
			return dataPacket;
		}

		//=====================================================================
		// DataCheckThread
		//=====================================================================
		/// <summary>
		/// The method that becomes a thread to check that the message was
		/// successfully sent to the coordinator, or save in the log
		/// </summary>
		static void DataCheckThread() {
			// Wait some time and then check to see if a response was received
			Thread.Sleep(3000);	// Sleep for 3 seconds
			if(sensorSent) {
				// This means that the message was not cleared, so save it to the log
				LogMessage(LogCode.Data, temperature + "," + luminosity + "," + humidity + ",3.3," + (thermoOn ? "1.0," : "0.0,") + (relayOn ? "1.0" : "0.0"));
				sensorSent = false;	// No need to try and send the data anymore
			}
		}

		//=====================================================================
		// ProcessGetRuleCMD
		//=====================================================================
		/// <summary>
		/// This method creates a XBee packet containing the rules and issues to the coordinator
		/// </summary>
		/// <returns>The response to send back to the sender of the transmission</returns>
		private static byte[] ProcessGetRuleCMD() {
			//-----------------------------------------------------------------
			// CREATE THE DATA PACKET
			//-----------------------------------------------------------------
			// Initialize the packet
			int numBytes = 9*rules.Count + 3;	// Get the size of the data packet
			byte[] packet = new byte[numBytes];	// Will contain the data
			packet[0] = CMD_RULE_CHANGE;
			packet[1] = STATUS_GET;
			packet[2] = (byte) rules.Count;

			// Add the rules
			for(int i = 0; i < rules.Count; i++) {
				// Convert the floating point values to byte arrays
				TemperatureRule curRule = rules[i] as TemperatureRule;	// Get the current rule as the TemperatureRule type
				byte[] timeArray, tempArray;
				timeArray = FloatToByte((float) curRule.Time);			// Convert the time
				tempArray = FloatToByte((float) curRule.Temperature);	// Convert the temperature

				// Copy byte arrays to the packet
				packet[9*i + 3] = (byte) curRule.Days;
				for(int j = 0; j < 4; j++) {
					packet[9*i + j + 4] = timeArray[j];
					packet[9*i + j + 8] = tempArray[j];
				}
			}

			//-----------------------------------------------------------------
			// Return the response packet
			//-----------------------------------------------------------------
			return packet;
		}

		//=====================================================================
		// SendXBeeResponse
		//=====================================================================
		/// <summary>
		/// Sends a TxRequest over the XBee network
		/// </summary>
		/// <param name="payload">The data payload for the transmission</param>
		/// <param name="destination">The XBee radio to send the data to</param>
		/// <returns>Where the transmission was successful</returns>
		private static bool SendXBeeTransmission(byte[] payload, XBeeAddress destination) {
			//-----------------------------------------------------------------
			// SEND PACKET TO DESTINATION
			//-----------------------------------------------------------------
			// Create the transmission object to the specified destination
			TxRequest response = new TxRequest(destination, payload);
			response.Option = TxRequest.Options.DisableAck;

			// Create debug console message
			string message = "Sending message to " + destination.ToString() + " (";
			for(int i = 0; i < payload.Length; i++) message += payload[i].ToString("X") + (i == (payload.Length - 1) ? "" : "-");
			message += ")";
			LogMessage(LogCode.Status, message);

			// Connect to the XBee
			bool sentMessage = false;
			if(ConnectToXBee()) {
				try {
					// Send the response
					xBee.Send(response).NoResponse();	// Send packet
					sentMessage = true;
					LogMessage(LogCode.Status, "XBee transmission sent");
				} catch(XBeeTimeoutException) {
					message += "Timeout";
					LogMessage(LogCode.Error, "XBee timed out trying to send message: " + message);
				}  // OTHER EXCEPTION TYPES TO INCLUDE?
			} else LogMessage(LogCode.Error, "XBee not connected and cannot send message");

			return sentMessage;
		}

		//=====================================================================
		// SendSensorData
		//=====================================================================
		/// <summary>
		/// Send a sensor data package to the system logger.
		/// </summary>
		/// <param name="externalTemperature">The measured temperature to send</param>
		private static void SendSensorData(double externalTemperature) {
			//-----------------------------------------------------------------
			// GET ANY DATA FOR THE TRANSMISSION
			//-----------------------------------------------------------------
			// Get temperature
			temperature = (externalTemperature == TEMP_UNDEFINED) ? (float) tempSensor.readTemperature() : (float) externalTemperature;	// Convert double to float
			Debug.Print("\tMeasured temperature = " + temperature);

			// Get luminosity
			luxSensor.SetTiming(TSL2561BusSensor.GainOptions.Low, TSL2561BusSensor.IntegrationOptions.Medium);
			luminosity = (float) luxSensor.readOptimizedLuminosity();
			Debug.Print("\tMeasured luminosity = " + luminosity);

			// Get humidity
			humidity = (float) tempSensor.readHumidity();
			Debug.Print("\tMeasured humidity = " + humidity);

			// Get status indicators
			float power = 3.3f;
			float thermoStatus = thermoOn ? 1f : 0f;
			float relayStatus = relayOn ? 1f : 0f;

			//-----------------------------------------------------------------
			// CREATE THE BYTE ARRAYS AND TRANSMISSION PACKAGE
			//-----------------------------------------------------------------
			// Convert the floats to byte arrays
			byte[] tempBytes, luxBytes, humidityBytes, powerBytes, thermoBytes, relayBytes;
			tempBytes = FloatToByte(temperature);
			luxBytes = FloatToByte(luminosity);
			humidityBytes = FloatToByte(humidity);
			powerBytes = FloatToByte(power);
			thermoBytes = FloatToByte(thermoStatus);
			relayBytes = FloatToByte(relayStatus);

			// Allocate the data package
			int floatSize = sizeof(float);
			Debug.Assert(floatSize == 4);
			byte[] package = new byte[6*(floatSize+1) + 1];	// Allocate memory for the package

			// Create the package of data
			package[0] = CMD_SENSOR_DATA;	// Indicate the package contains sensor data
			package[1] = TEMPERATURE_CODE;
			package[(floatSize+1)+1] = LUX_CODE;
			package[2*(floatSize+1)+1] = HUMIDITY_CODE;
			package[3*(floatSize+1)+1] = POWER_CODE;
			package[4*(floatSize+1)+1] = HEATING_CODE;
			package[5*(floatSize+1)+1] = THERMOSTAT_CODE;
			for(int i = 0; i < floatSize; i++) {
				package[i+2] = tempBytes[i];
				package[(floatSize+1)+(i+2)] = luxBytes[i];
				package[2*(floatSize+1)+(i+2)] = humidityBytes[i];
				package[3*(floatSize+1)+(i+2)] = powerBytes[i];
				package[4*(floatSize+1)+(i+2)] = relayBytes[i];
				package[5*(floatSize+1)+(i+2)] = thermoBytes[i];
			}

			//-----------------------------------------------------------------
			// TRANSMIT THE SENSOR DATA
			//-----------------------------------------------------------------			
			// Create the TxRequest packet and send the data
			XBeeAddress64 loggerAddress = new XBeeAddress64(COORD_ADDRESS);
			sensorSent = SendXBeeTransmission(package, loggerAddress);

			// Start the thread to check the success of the transmission
			Thread trackerThread = new Thread(DataCheckThread);
			trackerThread.Start();
		}

		//=====================================================================
		// TIMER EVENT METHOD (OnTimer)
		//=====================================================================
		/// <summary>
		/// Called every time the timer goes off.  Determines whether to update relay status and/or pass on sensor data
		/// </summary>
		/// <param name="dataObj">Not sure</param>
		private static void OnTimer(Object dataObj) {
			// Determine what to evaluate and send by XBee, depending on thermostat status and type of loop
			controlLoops++;	// Increment the loop counter
			if(thermoOn) EvaluateProgramming(controlLoops >= SENSOR_PERIODS);	// Thermostat is on, so evaluate the relay status through programming rules, only force XBee data if a sensor loop
			else if(controlLoops >= SENSOR_PERIODS) SendSensorData(TEMP_UNDEFINED);	// Thermostat is off, so only send XBee data if a sensor loop

			// Reset the counter, if needed
			if(controlLoops >= SENSOR_PERIODS) controlLoops = 0;
		}

		//=====================================================================
		// EvaluateProgramming
		//=====================================================================
		/// <summary>
		/// Based on the current day, time and temperature, determine the relay status, and then if sensor data is to be sent to the logger
		/// </summary>
		/// <param name="forceUpdate">Force a sensor data update</param>
		private static void EvaluateProgramming(bool forceUpdate) {
			//-----------------------------------------------------------------
			// COLLECT CONTROL CONDITIONS
			//-----------------------------------------------------------------
			// Get the tempeature reading
			double temperature = tempSensor.readTemperature();

			// Get the time and weekday for evaluating the rules
			double curTime = DateTime.Now.Hour + DateTime.Now.Minute/60.0 + DateTime.Now.Second/3600.0;
			RuleDays curWeekday = (RuleDays) ((int) DateTime.Now.DayOfWeek);	// Cast the returned DayOfWeek enum into the custome DayType enum
			LogMessage(LogCode.Status, "Evaluating relay status on day " + curWeekday + " (" + DateTime.Now.ToString("dddd") + ") at " + curTime.ToString("F4") + " with measured temperature at " + temperature.ToString("F"));

			//-----------------------------------------------------------------
			// TEMPERATURE LIMITS CHECK
			//-----------------------------------------------------------------
			bool updatePacket = forceUpdate;	// Default value for update data packet is from a parameter for a forced update
			if(temperature < MIN_TEMPERATURE) {	// Temperature too low
				if(!relayOn) {
					LogMessage(LogCode.Status, "\tRelay turned on due to temperature below minimum limit");
					SetRelay(true);	// Turn on relay
					updatePacket = true;	// Indicate to dispatch change of relay state
				}
			} else if(temperature >= MAX_TEMPERATURE) {	// Temperature above limit
				if(relayOn) {
					LogMessage(LogCode.Status, "\tRelay turned off due to temperature above temperature limit");
					SetRelay(false);	// Turn off relay
					updatePacket = true;	// Indicate to dispatch change of relay state
				}
			} else if(overrideOn) {	// Override is on
				//-------------------------------------------------------------
				// EVALUATE RELAY STATUS AGAINST OVERRIDE TEMPERATURE
				//-------------------------------------------------------------
				if(relayOn && (temperature > (overrideTemp + TEMPERATURE_BUFFER))) {
					// Turn off relay
					SetRelay(false);
					updatePacket = true;
					LogMessage(LogCode.Status, "\tOVERRIDE MODE: Relay turned OFF since temperature (" + temperature.ToString("F") + ") is greater than unbuffered override temperature (" + overrideTemp.ToString("F") + ")");
				} else if(!relayOn && (temperature < (overrideTemp - TEMPERATURE_BUFFER))) {
					// Turn on relay
					SetRelay(true);
					updatePacket = true;
					LogMessage(LogCode.Status, "\tOVERRIDE MODE: Relay turned ON since temperature (" + temperature.ToString("F") + ") is less than unbuffered override temperature (" + overrideTemp.ToString("F") + ")");
				}
			} else {	// Temperature is within limits, so evaluate relay status based on rules in effect
				//-------------------------------------------------------------
				// EVALUATE RELAY STATUS AGAINST PROGRAMMING
				//-------------------------------------------------------------
				// Iterate through the rules
				bool ruleFound = false;	// Flags that a rule has been found
				while(!ruleFound) {
					// Iterate through the rules until the active one is found
					for(int i = 0; i < rules.Count; i++) {
						// Check to see if current rule applies
						TemperatureRule curRule = rules[i] as TemperatureRule;
						if(RuleApplies(curRule, curWeekday, curTime)) {
							// Rule applies, now determine how to control the relay
							if(relayOn && (temperature > (curRule.Temperature + TEMPERATURE_BUFFER))) {
								// Temperature exceeding rule, turn off relay
								SetRelay(false);
								updatePacket = true;	// Indicate to send the updated status
								LogMessage(LogCode.Status, "\tRelay turned OFF since temperature (" + temperature.ToString("F") + ") is greater than the unbuffered rule temperature (" + curRule.Temperature.ToString("F") + ")");
							} else if(!relayOn && (temperature < (curRule.Temperature - TEMPERATURE_BUFFER))) {
								// Temperature below rule, turn on relay
								SetRelay(true);
								updatePacket = true;	// Indicate to send the updated status
								LogMessage(LogCode.Status, "\tRelay turned ON since temperature (" + temperature.ToString("F") + ") is less than the unbuffered rule temperature (" + curRule.Temperature.ToString("F") + ")");
							} else {
								// No relay status change needed, but check for a forced status update
								//Debug.Print("\tRelay remains " + (relayOn ? "ON" : "OFF"));
							}

							// Rule found, so break from the loops
							ruleFound = true;
							break;
						}
					}

					// No rule was found to apply, so move the day back before checking against rules again
					if(!ruleFound) {
						// Decrease the indicated day, but increase the time
						if(curWeekday == RuleDays.Sunday) curWeekday = RuleDays.Saturday;
						else curWeekday = (RuleDays) ((int) curWeekday - 1);
						curTime += 24.0;
					}
				}
			}

			//-----------------------------------------------------------------
			// SEND THE DATA
			//-----------------------------------------------------------------
			if(updatePacket) SendSensorData(temperature);
		}

		//=====================================================================
		// RuleApplies
		//=====================================================================
		/// <summary>
		/// Checks to see if the day and time match the rule
		/// </summary>
		/// <param name="rule">Rule to evaluate</param>
		/// <param name="checkDay">Current day</param>
		/// <param name="checkTime">Current time</param>
		/// <returns>Whether the rule is in effect for the day and time</returns>
		private static bool RuleApplies(TemperatureRule rule, RuleDays checkDay, double checkTime) {
			// First check: time is later than the rule time
			if(checkTime >= rule.Time) {
				if(rule.Days == RuleDays.Everyday) return true; // The specific day doesn't matter in this case
				if(checkDay == rule.Days) return true;	// The day of the rule has been met
				if((rule.Days == RuleDays.Weekdays) && (checkDay >= RuleDays.Monday) && (checkDay <= RuleDays.Friday)) return true;	// The rule is for weekdays and this is met
				if((rule.Days == RuleDays.Weekends) && ((checkDay == RuleDays.Saturday) || (checkDay == RuleDays.Sunday))) return true;	// The rule is for weekend and this is met
			}

			return false;	// If a match hasn't been found, this rule doesn't apply and return false
		}

		//=====================================================================
		// ByteToFloat
		//=====================================================================
		/// <summary>
		/// Converts a 4-byte array into a single precision floating point value
		/// </summary>
		/// <param name="byte_array">The byte array to convert</param>
		/// <returns>The float value of the byte array</returns>
		private static unsafe float ByteToFloat(byte[] byte_array) {
			uint ret = (uint) (byte_array[0] << 0 | byte_array[1] << 8 | byte_array[2] << 16 | byte_array[3] << 24);
			float r = *((float*) &ret);
			return r;
		}

		//=====================================================================
		// FloatToByte
		//=====================================================================
		/// <summary>
		/// Convert a float into a 4-byte array
		/// </summary>
		/// <param name="value">The float to convert</param>
		/// <returns>The 4-byte representation of the float</returns>
		private static unsafe byte[] FloatToByte(float value) {
			Debug.Assert(sizeof(uint) == 4);	// Confirm that the int is a 4-byte variable

			uint asInt = *((uint*) &value);
			byte[] byte_array = new byte[sizeof(uint)];

			byte_array[0] = (byte) (asInt & 0xFF);
			byte_array[1] = (byte) ((asInt >> 8) & 0xFF);
			byte_array[2] = (byte) ((asInt >> 16) & 0xFF);
			byte_array[3] = (byte) ((asInt >> 24) & 0xFF);

			return byte_array;
		}

		//=====================================================================
		// SetPowerMode
		//=====================================================================
		/// <summary>
		/// Sets the power mode of the thermostat control
		/// </summary>
		/// <param name="turnOn">Indicates if the thermostat should be turned on</param>
		private static void SetPowerMode(bool turnOn) {
			if(turnOn) {
				// Update the thermostat status indicators
				thermoOn = true;	// Set the master flag
				powerStatusOutput.Write(true);	// Turn on the LED
				LogMessage(LogCode.System, "Thermostat turned ON");

				// Determine the relay status
				SetRelay(false);	// Turn off the relay by default as the programming logic will evaluate its status
				EvaluateProgramming(true);	// Force a data update since the thermostat status changed
			} else {
				// Update the thermostat status indicators
				thermoOn = false;	// Set the master flag
				powerStatusOutput.Write(false);	// Turn off the LED
				LogMessage(LogCode.System, "Thermostat turned OFF");

				// Open the relay for external control
				SetRelay(true);	// Open the relay
				SendSensorData(TEMP_UNDEFINED);	// Programming rules don't apply, but still need to send data update for thermostat and relay status change
			}
		}

		//=====================================================================
		// FormatApiMode
		//=====================================================================
		/// <summary>
		/// Formats escape characters in the XBee payload data
		/// </summary>
		/// <param name="packet">The payload data</param>
		/// <param name="filterIncoming">Payload from an incoming tranmission with escape characters</param>
		/// <returns>The filtered payload data</returns>
		private static byte[] FormatApiMode(byte[] packet, bool filterIncoming) {
			// Local variables and constants
			byte[] escapeChars = { 0x7d, 0x7e, 0x11, 0x13 };	// The bytes requiring escaping
			const byte filter = 0x20;	// The XOR filter
			byte[] output;	// Contains the formatted packet
			int outSize = packet.Length;	// Contains the size of the outgoing packet

			if(filterIncoming) {	// Removed any escaping sequences
				//-------------------------------------------------------------
				// REMOVE ESCAPING CHARACTERS FROM PACKET FROM XBEE
				//-------------------------------------------------------------
				// Count the outgoing packet size
				foreach(byte b in packet) if(b == escapeChars[0]) outSize--;

				// Iterate through each byte and adjust
				output = new byte[outSize];
				int pos = 0;
				for(int i = 0; i < packet.Length; i++) {
					if(packet[i] == escapeChars[0]) output[pos++] = (byte) (packet[++i]^filter);	// Cast needed as XOR works on ints
					else output[pos++] = packet[i];
				}
			} else {
				//-------------------------------------------------------------
				// ADD ESCAPING CHARACTERS TO PACKET SENT FROM XBEE
				//-------------------------------------------------------------
				// Determine the new size
				foreach(byte b in packet) if(Array.IndexOf(escapeChars, b) > -1) outSize++;

				// Iterate through each byte and adjust
				output = new byte[outSize];
				int pos = 0;
				for(int i = 0; i < packet.Length; i++) {
					if(Array.IndexOf(escapeChars, packet[i]) > -1) {
						output[pos++] = escapeChars[0];
						output[pos++] = (byte) (packet[i]^filter);
					} else output[pos++] = packet[i];
				}
			}

			return output;
		}

		//=====================================================================
		// SetRelay
		//=====================================================================
		/// <summary>
		/// Operate the relay
		/// </summary>
		/// <param name="openRelay">Turn on the relay</param>
		private static void SetRelay(bool openRelay) {
			if(openRelay && !relayOn) {	// Turn on relay only when it's off
				// Turn on relay
/*				relayPinOn.Write(true);
				Thread.Sleep(RELAY_DELAY);
				relayPinOn.Write(false);*/
				relayControlOutput.Write(true);

//				relayStatusOutput.Write(true);	// Turn on LED
				relayOn = true;	// Set master flag
				LogMessage(LogCode.System, "Relay turned ON");
			} else if(!openRelay && relayOn) {
				// Turn off relay
/*				relayPinOff.Write(true);
				Thread.Sleep(RELAY_DELAY);
				relayPinOff.Write(false);*/
				relayControlOutput.Write(false);

//				relayStatusOutput.Write(false);	// Turn off LED
				relayOn = false;	// Set master flag
				LogMessage(LogCode.System, "Relay turned OFF");
			}
		}

		//=====================================================================
		// LogMessage
		//=====================================================================
		/// <summary>
		/// Method to write messages to the log and debug screen
		/// </summary>
		/// <param name="type">The type of the message</param>
		/// <param name="message">The message to be written</param>
		/// <param name="debug">Flag to write to the debug screen (default is true)</param>
		/// <param name="timestamp">Flag to write out the time of the message (default is true)</param>
		private static void LogMessage(LogCode type, string message, bool debug = true, bool timestamp = true) {
			// Get the timestamp, if required
			string time_str = timestamp ? DateTime.Now.ToString() : "";

			// Determine the log code
			string header = "";
			switch(type) {
				case LogCode.Data:
					header = "[DATA]";
					break;
				case LogCode.Error:
					header = "[ERROR]";
					break;
				case LogCode.System:
					header = "[SYSTEM]";
					break;
				case LogCode.Warning:
					header = "[WARNING]";
					break;
				case LogCode.Status:
					header = "[STATUS]";
					break;
			}

			// Create the message and send to the logger
			string log_msg = header + " " + time_str + " -> " + message;
			if(debug) Debug.Print(log_msg);	// Print to the debug console
			log_msg += "\n";
			if(dataLogger.IsOpen && (type <= LogLevel)) dataLogger.Write(System.Text.Encoding.UTF8.GetBytes(log_msg), 0, log_msg.Length);
		}
	}
}
