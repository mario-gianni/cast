// Benoit & Shanker 2011-07-12

#ifndef EU_NIFTI_CONVERTER_UTIL_HPP_
#define EU_NIFTI_CONVERTER_UTIL_HPP_

#include "std_msgs/Header.h"

#include <eu_nifti_cast/WorkingMemoryPointer.h>

#include "eu/nifti/ros/Header.hpp"
#include "eu/nifti/ros/Time.hpp"


namespace eu
{
    namespace nifti
    {

        class ConverterUtil
        {
        public:

            // Todo this should take the argument by reference

            static inline eu::nifti::ros::TimePtr convertTimeToCAST(const ::ros::Time time)
            {
                return new eu::nifti::ros::Time(time.sec, time.nsec);
            }

            static inline ::ros::Time convertTimeToROS(const eu::nifti::ros::TimePtr time)
            {
                return ::ros::Time(time->sec, time->nsec);
            }

            // Todo this should take the argument by reference

            static inline eu::nifti::ros::HeaderPtr convertHeaderToCAST(const std_msgs::Header header)
            {
                return new eu::nifti::ros::Header(header.seq, convertTimeToCAST(header.stamp), header.frame_id);
            }

            static inline std_msgs::Header convertHeaderToROS(const eu::nifti::ros::HeaderPtr header)
            {
                std_msgs::Header newHeader;
                newHeader.seq = header->seq;
                if (header->stamp != 0)
                {
                    newHeader.stamp = convertTimeToROS(header->stamp);
                }
                newHeader.frame_id = header->frameID;
                return newHeader;
            }

            static inline cast::cdl::WorkingMemoryPointerPtr convertWorkingMemoryPointerToCAST(const eu_nifti_cast::WorkingMemoryPointer wmp)
            {
                cast::cdl::WorkingMemoryAddress wma_CAST;
                wma_CAST.id = wmp.address.id;
                wma_CAST.subarchitecture = wmp.address.subarchitecture;
                return new cast::cdl::WorkingMemoryPointer(wma_CAST, wmp.type);
            }

            static inline eu_nifti_cast::WorkingMemoryPointer convertWorkingMemoryPointerToROS(const cast::cdl::WorkingMemoryPointer wmp)
            {
                eu_nifti_cast::WorkingMemoryAddress wma_ROS;
                wma_ROS.id = wmp.address.id;
                wma_ROS.subarchitecture = wmp.address.subarchitecture;

                eu_nifti_cast::WorkingMemoryPointer wmp_ROS;
                wmp_ROS.address = wma_ROS;
                wmp_ROS.type = wmp.type;

                return wmp_ROS;
            }

        };

    };

};

#endif // EU_NIFTI_CONVERTER_UTIL_HPP_

