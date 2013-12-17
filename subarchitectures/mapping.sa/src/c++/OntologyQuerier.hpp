/* 
 * File:   OntologyQuerier.hpp
 * Author: shanker
 *
 * Created on October 12, 2011, 5:03 PM
 */

#ifndef ONTOLOGYQUERIER_HPP
#define	ONTOLOGYQUERIER_HPP

#include "FEMAreasGenerator.hpp"
#include "Conversions.hpp"

float FEMAreasGenerator::queryOntologyFloat(char* query)
{
    if(query == NULL)
    {
        ROS_ERROR("An empty query was received, not processing");
    }
    std::string queryString(query);
    eu::nifti::mapping::QueryResults qResult = this->_hfcserver->querySelect(query);
    float result = FEMAreasGenerator::convertXsdFloatToFloat(qResult.bt[0][0]);
    return(result);
}


#endif	/* ONTOLOGYQUERIER_HPP */

