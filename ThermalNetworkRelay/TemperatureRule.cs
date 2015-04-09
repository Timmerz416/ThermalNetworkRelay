using System;
using Microsoft.SPOT;

namespace ThermalNetworkRelay {

	class TemperatureRule {
		// Enums
		public enum RuleDays { Sunday, Monday, Tuesday, Wednesday, Thursday, Friday, Saturday, Weekdays, Weekends, Everyday };

		// Members
		private RuleDays _days;
		private double _time;
		private double _temperature;

		// Constructor
		public TemperatureRule(RuleDays Day, double Time, double Temperature) {
			_days = Day;
			_time = Time;
			_temperature = Temperature;
		}

		// Parameters
		public RuleDays Days {
			get { return _days; }
		}

		public double Time {
			get { return _time; }
		}

		public double Temperature {
			get { return _temperature; }
		}
	}
}
