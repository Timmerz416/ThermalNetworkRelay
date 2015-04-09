using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

namespace ThermalNetworkRelay {

	//=========================================================================
	// I2CBus Class
	//=========================================================================
	/// <summary>
	/// Class that controls multiple I2C devices on a bus, taken from http://forums.netduino.com/index.php?/topic/563-i2cbus/
	/// </summary>
//	public class I2CBus : IDisposable {
	public class I2CBus {
		//=====================================================================
		// CLASS CONSTANTS
		//=====================================================================
		public const int DEFAULT_TIMEOUT = 1000;

		//=====================================================================
		// CLASS MEMBERS
		//=====================================================================
		private static readonly object LockObject = new object();	// Used for locking static creation of the bus instance
		protected static I2CDevice _slaveDevice = new I2CDevice(new I2CDevice.Configuration(0, 0));	// The target device

		//=====================================================================
		// Constructor
		//=====================================================================
        public I2CBus() {
        }

		//=====================================================================
		// Write
		//=====================================================================
        /// <summary>
        /// Generic write operation to I2C slave device.
        /// </summary>
        /// <param name="config">I2C slave device configuration.</param>
        /// <param name="writeBuffer">The array of bytes that will be sent to the device.</param>
        /// <param name="transactionTimeout">The amount of time the system will wait before resuming execution of the transaction.</param>
        protected void Write(I2CDevice.Configuration config, byte[] writeBuffer, int transactionTimeout = DEFAULT_TIMEOUT) {
            // create an i2c write transaction to be sent to the device.
            I2CDevice.I2CTransaction[] writeXAction = new I2CDevice.I2CTransaction[] { I2CDevice.CreateWriteTransaction(writeBuffer) };

            lock(_slaveDevice) {
				// Set i2c device configuration.
				_slaveDevice.Config = config;

				// the i2c data is sent here to the device.
                int transferred = _slaveDevice.Execute(writeXAction, transactionTimeout);

                // make sure the data was sent.
                if(transferred != writeBuffer.Length) throw new Exception("Could not write to device.");
            }
        }

		//=====================================================================
		// Read
		//=====================================================================
        /// <summary>
        /// Generic read operation from I2C slave device.
        /// </summary>
        /// <param name="config">I2C slave device configuration.</param>
        /// <param name="readBuffer">The array of bytes that will contain the data read from the device.</param>
        /// <param name="transactionTimeout">The amount of time the system will wait before resuming execution of the transaction.</param>
        protected void Read(I2CDevice.Configuration config, byte[] readBuffer, int transactionTimeout = DEFAULT_TIMEOUT) {
            // create an i2c read transaction to be sent to the device.
            I2CDevice.I2CTransaction[] readXAction = new I2CDevice.I2CTransaction[] { I2CDevice.CreateReadTransaction(readBuffer) };

            lock(_slaveDevice) {
				// Set i2c device configuration.
				_slaveDevice.Config = config;

				// the i2c data is received here from the device.
                int transferred = _slaveDevice.Execute(readXAction, transactionTimeout);

                // make sure the data was received.
                if(transferred != readBuffer.Length) throw new Exception("Could not read from device.");
            }
        }

		//=====================================================================
		// ReadRegister
		//=====================================================================
        /// <summary>
        /// Read array of bytes at specific register from the I2C slave device.
        /// </summary>
        /// <param name="config">I2C slave device configuration.</param>
        /// <param name="register">The register to read bytes from.</param>
        /// <param name="readBuffer">The array of bytes that will contain the data read from the device.</param>
        /// <param name="transactionTimeout">The amount of time the system will wait before resuming execution of the transaction.</param>
        protected void ReadRegister(I2CDevice.Configuration config, byte register, byte[] readBuffer, int transactionTimeout = DEFAULT_TIMEOUT) {
            byte[] registerBuffer = { register };
            Write(config, registerBuffer, transactionTimeout);
            Read(config, readBuffer, transactionTimeout);
        }

		//=====================================================================
		// WriteRegister - Array of Bytes
		//=====================================================================
        /// <summary>
        /// Write array of bytes value to a specific register on the I2C slave device.
        /// </summary>
        /// <param name="config">I2C slave device configuration.</param>
        /// <param name="register">The register to send bytes to.</param>
        /// <param name="writeBuffer">The array of bytes that will be sent to the device.</param>
        /// <param name="transactionTimeout">The amount of time the system will wait before resuming execution of the transaction.</param>
        protected void WriteRegister(I2CDevice.Configuration config, byte register, byte[] writeBuffer, int transactionTimeout = DEFAULT_TIMEOUT) {
            byte[] registerBuffer = { register };
            Write(config, registerBuffer, transactionTimeout);
            Write(config, writeBuffer, transactionTimeout);
        }

		//=====================================================================
		// WriteRegister - Single Byte
		//=====================================================================
        /// <summary>
        /// Write a byte value to a specific register on the I2C slave device.
        /// </summary>
        /// <param name="config">I2C slave device configuration.</param>
        /// <param name="register">The register to send bytes to.</param>
        /// <param name="value">The byte that will be sent to the device.</param>
        /// <param name="transactionTimeout">The amount of time the system will wait before resuming execution of the transaction.</param>
        protected void WriteRegister(I2CDevice.Configuration config, byte register, byte value, int transactionTimeout = DEFAULT_TIMEOUT) {
            byte[] writeBuffer = { register, value };
            Write(config, writeBuffer, transactionTimeout);
        }
    }

	//=========================================================================
	// I2CException Class
	//=========================================================================
	public class I2CException : Exception {
		//=====================================================================
		// Basic Constructor
		//=====================================================================
		public I2CException(string message) : base(message) { }
	}
}
