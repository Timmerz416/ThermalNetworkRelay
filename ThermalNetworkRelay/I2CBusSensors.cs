using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using System.Threading;

namespace ThermalNetworkRelay {
	//=========================================================================
	// BasicI2CBusSensor Class
	//=========================================================================
	class BasicI2CBusSensor : I2CBus {
		//=====================================================================
		// CLASS MEMBERS
		//=====================================================================
		// I2C device configuration properties
		public I2CDevice.Configuration _config;

		//=====================================================================
		// Class Constructor
		//=====================================================================
		public BasicI2CBusSensor(ushort address, int clockSpeed) : base() {
			_config = new I2CDevice.Configuration(address, clockSpeed);
		}

		//=====================================================================
		// Write Override
		//=====================================================================
		protected void Write(byte[] writeBuffer, int transactionTimeout = DEFAULT_TIMEOUT) {
			base.Write(_config, writeBuffer, transactionTimeout);
		}

		//=====================================================================
		// Read Override
		//=====================================================================
		protected void Read(byte[] readBuffer, int transactionTimeout = DEFAULT_TIMEOUT) {
			base.Read(_config, readBuffer, transactionTimeout);
		}

		//=====================================================================
		// ReadRegister Override
		//=====================================================================
		protected void ReadRegister(byte register, byte[] readBuffer, int transactionTimeout = DEFAULT_TIMEOUT) {
			base.ReadRegister(_config, register, readBuffer, transactionTimeout);
		}

		//=====================================================================
		// WriteRegister - Array of Bytes
		//=====================================================================
		protected void WriteRegister(byte register, byte[] writeBuffer, int transactionTimeout = DEFAULT_TIMEOUT) {
			base.WriteRegister(_config, register, writeBuffer, transactionTimeout);
		}

		//=====================================================================
		// WriteRegister - Single Byte
		//=====================================================================
		protected void WriteRegister(byte register, byte writeBuffer, int transactionTimeout = DEFAULT_TIMEOUT) {
			base.WriteRegister(_config, register, writeBuffer, transactionTimeout);
		}
	}

	//=========================================================================
	// HTU21DBusSensor
	//=========================================================================
	class HTU21DBusSensor : BasicI2CBusSensor {
		//=====================================================================
		// CLASS CONSTANTS
		//=====================================================================
		// Set bus device properties
		private const ushort BUS_ADDRESS = 0x40;
		private const int CLOCK_SPEED = 400;

		// The HTU21D Commands
		private const byte MEASURE_TEMPERATURE_HOLD		= 0xE3;
		private const byte MEASURE_TEMPERATURE_NOHOLD	= 0xF3;
		private const byte MEASURE_HUMIDITY_HOLD		= 0xE5;
		private const byte MEASURE_HUMIDITY_NOHOLD		= 0xF5;
		private const byte WRITE_USER_REGISTER			= 0xE6;
		private const byte READ_USER_REGISTER			= 0xE7;
		private const byte SOFT_RESET					= 0xFE;

		//=====================================================================
		// Class Constructor
		//=====================================================================
		/// <summary>
		/// Set the address of the humidity sensor and set the clock speed
		/// </summary>
		public HTU21DBusSensor() : base(BUS_ADDRESS, CLOCK_SPEED) { }

		//=====================================================================
		// readTemperature
		//=====================================================================
		/// <summary>
		/// Read the temperature from the humidity sensor
		/// </summary>
		/// <returns>The measured temperature in Celsius</returns>
		public double readTemperature() {
			//-----------------------------------------------------------------
			// Implement the No Hold approach
			//-----------------------------------------------------------------
			// Signal for a measurement of the temperature
			Write(new byte[] { MEASURE_TEMPERATURE_NOHOLD });

			// Dealy for 60 ms while the sensor takes the measurement
			Thread.Sleep(60);	// Longest read time is 50 ms based on spec sheet, but add extra time

			// Read the resultant measurements - after delay, read 3 bytes
			byte[] buffer = new byte[3];
			Read(buffer);

			// TODO - CONFIRM CHECKSUM

			// Create raw measurement, minus the status bits
			uint rawTemperature = ((uint) buffer[0] << 8) | (uint) buffer[1];	// Combine the two measurement bytes
			uint statusBits = rawTemperature & 0x0003;	// Get the status bits
			rawTemperature &= 0xFFFC;	// Strip off the status bits

			// Confirm we have temperature data
			if(statusBits == 0) return 175.72*((double) rawTemperature)/65536.0 - 46.85;
			else throw new I2CException("Humidity measurement returns when requesting temperature measurement");
		}

		//=====================================================================
		// readHumidity
		//=====================================================================
		/// <summary>
		/// Read the humidity from the sensor
		/// </summary>
		/// <returns>The measured humidity in relative percent</returns>
		public double readHumidity() {
			//-----------------------------------------------------------------
			// Implement the No Hold approach
			//-----------------------------------------------------------------
			// Signal for a measurement of the temperature
			Write(new byte[] { MEASURE_HUMIDITY_NOHOLD });

			// Dealy for 60 ms while the sensor takes the measurement
			Thread.Sleep(60);	// Longest read time is 50 ms based on spec sheet, but add extra time

			// Read the resultant measurements - after delay, read 3 bytes
			byte[] buffer = new byte[3];
			Read(buffer);

			// TODO - CONFIRM CHECKSUM

			// Create raw measurement, minus the status bits
			uint rawHumidity = ((uint) buffer[0] << 8) | (uint) buffer[1];	// Combine the two measurement bytes
			uint statusBits = rawHumidity & 0x0003;	// Get the status bits
			rawHumidity &= 0xFFFC;	// Strip off the status bits

			// Confirm we have humidity data
			if(statusBits == 2) return 125.0*((double) rawHumidity)/65536.0 - 6.0;
			else throw new I2CException("Temperature measurement returns when requesting humidity measurement");
		}
	}

	//=========================================================================
	// TSL2561BusSensor
	//=========================================================================
	class TSL2561BusSensor : BasicI2CBusSensor {
		//=====================================================================
		// CLASS CONSTANTS
		//=====================================================================
		// Set bus device properties
		private const ushort BUS_ADDRESS = 0x39;
		private const int CLOCK_SPEED = 100;

		// Measurement erros
		public const double LUX_ERROR = -1.0;

		//=====================================================================
		// CLASS ENUMERATIONS
		//=====================================================================
		// Registers for TSL2561
		private enum Registers {
			Control				= 0x0,	// Control of basic functions
			Timing				= 0x1,	// Integration time/gain control
			ThresholdLowLow		= 0x2,	// Low byte of low interrupt threshold
			ThresholdLowHigh	= 0x3,	// High byte of low interrupt threshold
			ThresholdHighLow	= 0x4,	// Low byte of high interrupt threshold
			ThresholdHighHigh	= 0x5,	// High byte of high interrupt threshold
			Interrupt			= 0x6,	// Interrupt control
			ID					= 0xA,	// Part number / revision ID
			Data0Low			= 0xC,	// Low byte of ADC Channel 0
			Data0High			= 0xD,	// High byte of ADC Channel 0
			Data1Low			= 0xE,	// Low byte of ADC Channel 1
			Data1High			= 0xF	// High byte of ADC Channel 1
		}

		// Command options
		private enum CommandOptions {
			CommandBit	= 0x80,	// Identify the transaction as a command
			ClearBit	= 0x40,	// Clears pending interrupts
			WordBit		= 0x20,	// Indicates if a work (two bytes) are to be read/written to the device
			BlockBit	= 0x10	// Turn on blocking (1) or off (0)
		}

		// Device power options
		private enum PowerOptions {
			Off	= 0x00,	// Power down the device
			On	= 0x03	// Power up the device
		}

		// Gain options
		public enum GainOptions {
			Low		= 0x00,	// Low (x1) gain setting
			High	= 0x10	// High (x16) gain setting
		}

		// Integration time options
		public enum IntegrationOptions {
			Short	= 0x0,	// Shortest, 13.7 ms integration window
			Medium	= 0x1,	// Middle, 101 ms integration window
			Long	= 0x2	// Longest 402 ms integration window
		}

		// Data channel options
		private enum Channels {
			Channel0,	// The visible + infrared sensor
			Channel1	// The infrared sensor
		}

		//=====================================================================
		// Internal Class
		//=====================================================================
		private class SensorException : Exception {
			// Enums
			public enum SignalError {
				Low,
				Saturated
			}

			// Members
			public SignalError Error { get; set; }

			// Constructor
			public SensorException(SignalError error) : base("Luminosity sensor signal error") {
				// Copy error type
				Error = error;
			}
		}

		//=====================================================================
		// Class Members
		//=====================================================================
		GainOptions _gain;
		IntegrationOptions _intPeriod;

		//=====================================================================
		// Class Constructor
		//=====================================================================
		/// <summary>
		/// Set the address of the luminosity sensor and set the clock speed
		/// </summary>
		public TSL2561BusSensor() : base(BUS_ADDRESS, CLOCK_SPEED) {
			// Determine the current timing settings
			byte command = (byte) CommandOptions.CommandBit | (byte) Registers.Timing;
			byte[] options = new byte[1];	// Will hold registry values
			ReadRegister(command, options);

			// Determine the timing settings
			_gain = (GainOptions) (options[0] & (byte) GainOptions.High);
			_intPeriod = (IntegrationOptions) (options[0] & 0x03);
		}

		//=====================================================================
		// ReadWord
		//=====================================================================
		private UInt16 ReadWord(Registers source) {
			// Create the command and response data variables
			byte command = (byte) ((byte) CommandOptions.CommandBit | (byte) CommandOptions.WordBit | (byte) source);
			byte[] response = new byte[2];	// Contains the word as 2 bytes
			UInt16 word = 0;	// The final word

			// Get the word and convert to an integer
			ReadRegister(command, response);
			word = (UInt16) ((UInt16) response[1] << 8 | (UInt16) response[0]);

			return word;
		}

		//=====================================================================
		// GetChannelData - only for preset integration times
		//=====================================================================
		private UInt16 GetChannelData(Channels channel) {
			// Enable the sensor, and wait for integration time
			EnableSensor();
			switch(_intPeriod) {
				case IntegrationOptions.Short:
					Thread.Sleep(20);	// Sleep for 20 ms	(6 ms extra)
					break;
				case IntegrationOptions.Medium:
					Thread.Sleep(110);	// Sleep for 110 ms (9 ms extra)
					break;
				default:
					Thread.Sleep(410);	// Sleep for 410 ms (8 ms extra)
					break;
			}

			// Read the data and disable the sensor
			UInt16 signal = 0;
			if(channel == Channels.Channel0) signal = ReadWord(Registers.Data0Low);
			else signal = ReadWord(Registers.Data1Low);
			DisableSensor();

			return signal;
		}

		//=====================================================================
		// PowerSensor
		//=====================================================================
		private void EnableSensor() {
			// Power up the sensor
			byte command = (byte) CommandOptions.CommandBit | (byte) Registers.Control;
			WriteRegister(command, (byte) PowerOptions.On);
		}

		//=====================================================================
		// HibernateSensor
		//=====================================================================
		private void DisableSensor() {
			// Turn off sensor power
			byte command = (byte) CommandOptions.CommandBit | (byte) Registers.Control;
			WriteRegister(command, (byte) PowerOptions.Off);
		}

		//=====================================================================
		// SetTiming
		//=====================================================================
		public void SetTiming(GainOptions gain, IntegrationOptions integration) {
			// Set the commands
			byte command = (byte) CommandOptions.CommandBit | (byte) Registers.Timing;
			byte options = (byte) ((byte) gain | (byte) integration);

			// Write the timing information
			EnableSensor();
			WriteRegister(command, options);
			DisableSensor();

			// Update the object
			_gain = gain;
			_intPeriod = integration;
		}

		//=====================================================================
		// readLuminosity
		//=====================================================================
		public double readLuminosity() {
			//-----------------------------------------------------------------
			// Read the sensor measurements
			//-----------------------------------------------------------------
			// Initialize values
			double luminosity = LUX_ERROR;

			// Get the measured luminosity in both channels
			UInt16 chan0 = GetChannelData(Channels.Channel0);
			UInt16 chan1 = GetChannelData(Channels.Channel1);

			//-----------------------------------------------------------------
			// Determine scaling
			//-----------------------------------------------------------------
			double scale;

			// First, account for integration time
			switch(_intPeriod) {
				case IntegrationOptions.Short:
					scale = 402.0/13.7;
					break;
				case IntegrationOptions.Medium:
					scale = 402.0/101.0;
					break;
				default:
					scale = 1.0;
					break;
			}

			// Adjust for gain
			if(_gain == GainOptions.Low) scale *= 16.0;

			//-----------------------------------------------------------------
			// Calculate luminosity
			//-----------------------------------------------------------------
			// Scale the readings
			double d0 = scale*chan0;
			double d1 = scale*chan1;

			// Calculation from the TSL2561 datasheet
			double ratio = (double) chan1 / (double) chan0;
			if(ratio <= 0.5) luminosity = 0.0304*d0 - 0.062*System.Math.Pow(ratio, 1.4)*d0;
			else if(ratio <= 0.61) luminosity = 0.0224*d0 - 0.031*d1;
			else if(ratio <= 0.8) luminosity = 0.0128*d0 - 0.0153*d1;
			else if(ratio <= 1.3) luminosity = 0.00146*d0 - 0.00112*d1;
			else luminosity = 0.0;

			return luminosity;
		}

		//=====================================================================
		// readOptimizedLuminosity
		//=====================================================================
		public double readOptimizedLuminosity() {
			//-----------------------------------------------------------------
			// Read the sensor measurements
			//-----------------------------------------------------------------
			// Initialize values
			double luminosity = LUX_ERROR;
			bool luxCaptured = false;
			UInt16 chan0 = 0;
			UInt16 chan1 = 0;

			// Sensor signal checks
			while(!luxCaptured) {
				// Get the measured luminosity in both channels
				chan0 = GetChannelData(Channels.Channel0);
				chan1 = GetChannelData(Channels.Channel1);

				// Evaluate the signal strength
				if((chan0 == 0xFFFF) || (chan1 == 0xFFFF)) {	// Saturated signal
					// Sensor saturated, so adjust timing
					if(_gain == GainOptions.High) _gain = GainOptions.Low;	// Lower the gain
					else if((int) _intPeriod > 0) --_intPeriod;	// Reduce the integration time
					else luxCaptured = true;	// Can't make any further adjustments
				} else if((chan0 < 10) || (chan1 < 10)) {	// Low signal
					// Low signal, so again try adjusting timing
					if((int) _intPeriod < 2) ++_intPeriod;	// Increase the integration time
					else if(_gain == GainOptions.Low) _gain = GainOptions.High;	// Increase the gain
					else {
						luminosity = readLuminosity();	// The measured value is the best one
						luxCaptured = true;
					}
				} else luxCaptured = true;	// Signal is fine, return from loop and calculate

				// Make adjustments if needed
				if(!luxCaptured) SetTiming(_gain, _intPeriod);	// A bit crude, but fine for now
			}

			//-----------------------------------------------------------------
			// Determine scaling
			//-----------------------------------------------------------------
			double scale;

			// First, account for integration time
			switch(_intPeriod) {
				case IntegrationOptions.Short:
					scale = 402.0/13.7;
					break;
				case IntegrationOptions.Medium:
					scale = 402.0/101.0;
					break;
				default:
					scale = 1.0;
					break;
			}

			// Adjust for gain
			if(_gain == GainOptions.Low) scale *= 16.0;

			//-----------------------------------------------------------------
			// Calculate luminosity
			//-----------------------------------------------------------------
			// Scale the readings
			double d0 = scale*chan0;
			double d1 = scale*chan1;

			// Calculation from the TSL2561 datasheet
			double ratio = (double) chan1 / (double) chan0;
			if(ratio <= 0.5) luminosity = 0.0304*d0 - 0.062*System.Math.Pow(ratio, 1.4)*d0;
			else if(ratio <= 0.61) luminosity = 0.0224*d0 - 0.031*d1;
			else if(ratio <= 0.8) luminosity = 0.0128*d0 - 0.0153*d1;
			else if(ratio <= 1.3) luminosity = 0.00146*d0 - 0.00112*d1;
			else luminosity = 0.0;

			return luminosity;
		}
	}

	//=========================================================================
	// DS1307BusSensor
	//=========================================================================
	class DS1307BusSensor : BasicI2CBusSensor {
		//=====================================================================
		// CLASS CONSTANTS
		//=====================================================================
		// Set bus device properties
		private const ushort BUS_ADDRESS = 0x68;
		private const int CLOCK_SPEED = 100;

		//=====================================================================
		// CLASS ENUMERATIONS
		//=====================================================================
		public enum DayOfWeek { Sunday, Monday, Tuesday, Wednesday, Thursday, Friday, Saturday };
		
		//=====================================================================
		// Class Structures
		//=====================================================================
		public struct RTCTime {
			public byte second;
			public byte minute;
			public byte hour;
			public byte day;
			public byte month;
			public byte year;
			public byte weekday;

			public RTCTime(byte ss, byte mm, byte hh, byte dd, byte MM, byte yy, DayOfWeek wd) {
				second = ss;
				minute = mm;
				hour = hh;
				day = dd;
				month = MM;
				year = yy;
				weekday = (byte) wd;
			}

			public DateTime getDateTime() {
				return new DateTime(2000 + year, month, day, hour, minute, second);
			}
		}

		//=====================================================================
		// Class Constructor
		//=====================================================================
		/// <summary>
		/// Set the address of the real time clock and set the clock speed
		/// </summary>
		public DS1307BusSensor() : base(BUS_ADDRESS, CLOCK_SPEED) { }

		//=====================================================================
		// ToBCD
		//=====================================================================
		private static byte ToBCD(byte value) {
			return (byte) ((value/10 << 4) + value % 10);
		}

		//=====================================================================
		// FromBCD
		//=====================================================================
		private static byte FromBCD(byte value) {
			// Get the components of the value
			int low = value & 0x0F;
			int high = (value & 0x70) >> 4;
			return (byte) (10*high + low);
		}

		//=====================================================================
		// SetTime
		//=====================================================================
		public void SetTime(RTCTime timeStruct) {
			// Create array to set the time
			byte[] timeArray = new byte[] {
				0x00,	// Stop oscillator
				ToBCD(timeStruct.second),
				ToBCD(timeStruct.minute),
				ToBCD(timeStruct.hour),
				ToBCD((byte) (timeStruct.weekday + 1)),
				ToBCD(timeStruct.day),
				ToBCD(timeStruct.month),
				ToBCD(timeStruct.year),
				0x00	// Restart oscillator
			};

			// Write the time
			WriteRegister(0x00, timeArray);
		}

		//=====================================================================
		// GetTime
		//=====================================================================
		public RTCTime GetTime() {
			// Create array to receive the time, and get the time
			byte[] timeArray = new byte[7];
			ReadRegister(0x00, timeArray);

			// Copy the array to the return object
			return new RTCTime(FromBCD((byte) (timeArray[0] & 0x7F)), FromBCD(timeArray[1]), FromBCD((byte) (timeArray[2] & 0x3F)), FromBCD(timeArray[4]), FromBCD(timeArray[5]), FromBCD(timeArray[6]), (DayOfWeek) FromBCD((byte) (timeArray[3] - 1)));
		}
	}
}
