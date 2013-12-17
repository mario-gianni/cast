/* 
 * File:   Conversions.hpp
 * Author: shanker
 *
 * Created on October 12, 2011, 4:39 PM
 */

#ifndef CONVERSIONS_HPP
#define	CONVERSIONS_HPP

#include "FEMAreasGenerator.hpp"

float FEMAreasGenerator::convertXsdFloatToFloat(std::string &_str)
    {
        _str.erase(_str.begin()); // remove the first "
        std::size_t _pos;
        _pos = _str.find_first_of("\"");
        if(_pos==std::string::npos)
        {
            ROS_INFO("The charecter \" was expected in conversion but not found, so we return a value of 0.0");
            return(0);
        }
        std::string relevantPart = _str.substr(0,_pos);
        char *_chrcopy = new char [relevantPart.size()+1];
        strcpy (_chrcopy, relevantPart.c_str());
        return(std::atof(_chrcopy));
    }


#endif	/* CONVERSIONS_HPP */

