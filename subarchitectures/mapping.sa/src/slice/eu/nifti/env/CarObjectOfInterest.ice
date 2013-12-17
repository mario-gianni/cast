// Benoit 2011-04-27

#ifndef EU_NIFTI_ENV_CAR_OBJECT_OF_INTEREST_ICE
#define EU_NIFTI_ENV_CAR_OBJECT_OF_INTEREST_ICE

#include <cast/slice/CDL.ice>

#include <eu/nifti/env/ObjectOfInterest.ice>
#include <eu/nifti/env/FunctionalArea.ice>
#include <eu/nifti/env/Window.ice>
#include <eu/nifti/env/VantagePoint.ice>


module eu 
{
    module nifti 
    {

        module env
        { 
            sequence<cast::cdl::WorkingMemoryPointer> ListOfFunctionalAreaWMPs;

            // Represents a car in the environment.
            class CarObjectOfInterest extends ObjectOfInterest
            {
                string carClass;
                ListOfFunctionalAreaWMPs functionalAreaWMPs;
                ListOfFunctionalAreas functionalAreas;
		ListOfVantagePoints vantagePoints;
                ListOfWindows windows;
            };

        };
    };
};

#endif

