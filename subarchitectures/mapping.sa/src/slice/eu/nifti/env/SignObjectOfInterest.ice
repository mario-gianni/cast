// Benoit 2011-04-27

#ifndef EU_NIFTI_ENV_SIGN_OBJECT_OF_INTEREST_ICE
#define EU_NIFTI_ENV_SIGN_OBJECT_OF_INTEREST_ICE

#include <eu/nifti/env/ObjectOfInterest.ice>


module eu 
{
    module nifti 
    {

        module env
        { 
	    enum SignObjectType {FLAMMABLE, RADIOACTIVE, DANGER};

            // Represents a sign in the environment.
            class SignObjectOfInterest extends ObjectOfInterest
            {
                SignObjectType signType;
            };

        };
    };
};

#endif
