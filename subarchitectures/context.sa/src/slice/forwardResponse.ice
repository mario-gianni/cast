#ifndef EU_NIFTI_CONTEXT_FORWARD_RESPONSE_ICE
#define EU_NIFTI_CONTEXT_FORWARD_RESPONSE_ICE

module eu
{
	module nifti
	{
		module context
		{
			class rosPose
			{
				double x;
				double y;
				double z;
				double qx;
				double qy;
				double qz; 
				double qw; 
			};
			sequence<rosPose> rosPoses;
			class forwardResponse
			{
				int requestID;
				rosPose targetPose;
				rosPoses intermediatePoses;
				bool responseFlag;
				bool repeatFlag;
			};
		};
	};
};



#endif
