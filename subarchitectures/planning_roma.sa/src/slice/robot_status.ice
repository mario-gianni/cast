#ifndef _EU_NIFTI_ENV_ROBOT_STATUS_ICE
#define _EU_NIFTI_ENV_ROBOT_STATUS_ICE

module eu {
	module nifti {
		module env {
			module status{			
				
				class ControllersStatus{
					int core;
					int trackLeft;
					int trackRight;
					int flipperFrontLeft;
					int flipperFrontRight;
					int flipperRearLeft;
					int flipperRearRight;
				};
				
				class RobotStatus{
					double batteryLevel;
					int batteryStatus;
					bool brakeOn;
					double scanningSpeed;
					ControllersStatus status;
					ControllersStatus error;
				};
				
			};
		};
	};
};

#endif
