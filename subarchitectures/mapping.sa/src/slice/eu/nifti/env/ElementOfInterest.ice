// Benoit 2011-04-14

#ifndef EU_NIFTI_ENV_ELEMENT_OF_INTEREST_ICE
#define EU_NIFTI_ENV_ELEMENT_OF_INTEREST_ICE

module eu 
{
    module nifti 
    {

        module env
        { 

            // Comments at the bottom of the file
            // I need to escape object because it's a reserved word in ICE
            enum ElementOfInterestType {ELEMENT, LOCATION, AREA, \OBJECT, CAR, SIGN, VICTIM};
            enum ElementOfInterestSourceType {VISION, USER}; 
			enum ElementOfInterestStatus {UNCONFIRMED, CONFIRMED, REJECTED};

            // Represents the most abstract class for elements in the environment.
            // Sub-classes are LocationOfInterest, AreaOfInterest, and ObjectOfInterestST
            // Tells when an element is selected
            class ElementOfInterest
            {
                int uuid;
                string name;
                ElementOfInterestType type;
                ElementOfInterestSourceType sourceType;
				double confidence; // Confidence of a detection (range 0 -> 1)
				ElementOfInterestStatus status; // Denotes the status of the element decided by the user
            };

        };
    };
};

#endif



//int8 TYPE_ELEMENT =  	0	# The object is of type element, but we don't know more
//int8 TYPE_LOCATION =  1	# The object is a location
//int8 TYPE_AREA =	2	# The object is an area
//int8 TYPE_OBJECT = 	3	# The object is an actual real-life object, but we don't know more
//int8 TYPE_CAR = 	4	# The object is a car
//int8 TYPE_SIGN = 	5	# The object is a sign
//int8 TYPE_VICTIM = 	6	# The object is a victim

//int8 SOURCE_TYPE_VISION = 0	# Vision detected this object
//int8 SOURCE_TYPE_USER =  1	# The user detected this object


