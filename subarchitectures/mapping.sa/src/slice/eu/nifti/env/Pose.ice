// Benoit 2011-04-27

#ifndef EU_NIFTI_ENV_POSE_ICE
#define EU_NIFTI_ENV_POSE_ICE

module eu 
{
    module nifti 
    {
        module env
        {
            struct Pose
            {
                double x;
                double y;
		double z;
		double theta; // In RADIANS
            };

        };

    };
};

#endif
