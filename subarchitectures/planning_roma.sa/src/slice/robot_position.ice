#ifndef _ROBOT_POSITION_ICE
#define _ROBOT_POSITION_ICE

#include <topograph.ice>

module eu {
	module nifti {
		module env {
			module position{
				class CurrentPos
				{
					eu::nifti::env::topograph::Node node;
				};
				class BasePos
				{
				    eu::nifti::env::topograph::Node node;
				};
				class Pose
				{
				    double x;
				    double y;
				    double z;
				    double theta;
				};
			};
		};
	};
};

#endif
