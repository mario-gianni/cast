// Benoit 2011-04-14

#ifndef EU_NIFTI_ENV_POINT_3D_ICE
#define EU_NIFTI_ENV_POINT_3D_ICE

module eu 
{
    module nifti 
    {
        module env
        {
            struct Point3D
            {
                double x;
                double y;
		double z;
            };

            sequence<eu::nifti::env::Point3D> ListOfPoints3D;

        };

    };
};

#endif
