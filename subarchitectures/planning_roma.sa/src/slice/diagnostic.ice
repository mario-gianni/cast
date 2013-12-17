#ifndef _DIAGNOSTIC_ICE
#define _DIAGNOSTIC_ICE

#include <cast/slice/CDL.ice>

module eu {
	module nifti {
		module env {
			module diagnostic {
			
				enum LinkQuality {GOOD, MODERATE, WEAK, LOST};
				enum PowerLevel {BMEDIUM, BHIGH, BLOW};
				
				class BatteryStatus{
					PowerLevel level;
				};
				
				class WiFiStatus{
					LinkQuality quality;
				};
			
			};
		};
	};
};

#endif
