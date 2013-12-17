/* 
 * File:   GeometricalComputation.hpp
 * Author: shanker
 *
 * Created on October 12, 2011, 3:26 PM
 */

#ifndef GEOMETRICALCOMPUTATION_HPP
#define	GEOMETRICALCOMPUTATION_HPP

#include "FEMAreasGenerator.hpp"

#define ANGLE_INCREMENT_CAMERA_VIEW_CONE 2.0 // In degrees

int FEMAreasGenerator::pointInPolygonHelper(geometry_msgs::Point testPoint, eu_nifti_env::Polygon _polygon)
{
    float testPointX = testPoint.x;
    float testPointY = testPoint.y;
    int numberOfPointsInPolygon = _polygon.points.size();
    float *polygonPointX = new float[numberOfPointsInPolygon];
    float *polygonPointY = new float[numberOfPointsInPolygon];
    for(int ctr=0;ctr<numberOfPointsInPolygon;ctr++)
    {
        polygonPointX[ctr] = _polygon.points[ctr].x;
        polygonPointY[ctr] = _polygon.points[ctr].y;
    };
    int inclusion = pnpoly(numberOfPointsInPolygon,polygonPointX,polygonPointY,testPointX,testPointY);
    delete []polygonPointX;
    delete []polygonPointY;
    return(inclusion);
}

float FEMAreasGenerator::computeDistanceBetweenPoints(geometry_msgs::Point point1, geometry_msgs::Point point2)
 {
     float distance = pow( pow(point1.x-point2.x,2) + pow(point1.y-point2.y,2) + pow(point1.z-point2.z,2) , 0.5);
     return(distance);
 }

geometry_msgs::Point FEMAreasGenerator::rotateAndTranslatePoint(geometry_msgs::Point origPoint, geometry_msgs::Vector3 translation, float rotX, float rotY, float rotZ)
{
    KDL::Vector origVec(origPoint.x, origPoint.y, origPoint.z);
    KDL::Rotation transMat = KDL::Rotation::EulerZYX(rotZ,rotY,rotX);
    KDL::Vector transfVec = transMat*origVec;
    geometry_msgs::Point resPoint = constructPoint(transfVec.x() + translation.x, transfVec.y() + translation.y, transfVec.z() + translation.z) ;
    return(resPoint);
}

geometry_msgs::Point FEMAreasGenerator::translateAndRotatePoint(geometry_msgs::Point origPoint, geometry_msgs::Vector3 translation, float rotX, float rotY, float rotZ)
{
    geometry_msgs::Point translPoint = constructPoint(origPoint.x + translation.x, origPoint.y + translation.y, origPoint.z + translation.z);
    KDL::Vector origVec(translPoint.x, translPoint.y, translPoint.z);
    KDL::Rotation transMat = KDL::Rotation::EulerZYX(rotZ,rotY,rotX);
    KDL::Vector transfVec = transMat*origVec;
    geometry_msgs::Point resPoint = constructPoint(transfVec.x(), transfVec.y(), transfVec.z());
    return(resPoint);
}

eu_nifti_env::Polygon FEMAreasGenerator::rotateAndTranslateROSPolygon(eu_nifti_env::Polygon origPolygon, geometry_msgs::Vector3 translation, float rotX, float rotY, float rotZ)
{
    eu_nifti_env::Polygon resultPolygon;
    geometry_msgs::Point resultPoint;
    for(unsigned int ctr = 0;ctr<origPolygon.points.size();ctr++)
    {
        geometry_msgs::Point newPoint = FEMAreasGenerator::constructPoint(origPolygon.points[ctr].x,origPolygon.points[ctr].y,origPolygon.points[ctr].z);
        newPoint = FEMAreasGenerator::rotateAndTranslatePoint(newPoint,translation,rotX,rotY,rotZ);
        resultPoint.x = newPoint.x; resultPoint.y = newPoint.y; resultPoint.z = newPoint.z;
        resultPolygon.points.push_back(resultPoint);
    }
    return(resultPolygon);
}

eu::nifti::env::Polygon FEMAreasGenerator::rotateAndTranslateCASTPolygon(eu::nifti::env::Polygon origPolygon, geometry_msgs::Vector3 translation, float rotX, float rotY, float rotZ)
{
    eu_nifti_env::Polygon rosPolygon= eu::nifti::ConverterUtil_Mapping::convertPolygonToROS(origPolygon);
    rosPolygon = FEMAreasGenerator::rotateAndTranslateROSPolygon(rosPolygon, translation, rotX, rotY, rotZ);
    eu::nifti::env::Polygon resultPolygon = eu::nifti::ConverterUtil_Mapping::convertPolygonToCAST(rosPolygon);
    return(resultPolygon);
}

std::vector<float> FEMAreasGenerator::computePlaneFromPolygon(eu_nifti_env::Polygon _Polygon)
{// Compute plane equation constants from Polygon points
    std::vector<float> _planeConstants;
    if(_Polygon.points.size() < 3)   // We need 3 points from the polygon to compute plane
    {
        log("The provided polygon has less than 3 points, cannot compute plane");
        _planeConstants.push_back(0); _planeConstants.push_back(0); _planeConstants.push_back(0); _planeConstants.push_back(0);
        return(_planeConstants);
    }

    float A = _Polygon.points[0].y*(_Polygon.points[1].z-_Polygon.points[2].z) + _Polygon.points[1].y*(_Polygon.points[2].z-_Polygon.points[0].z) + _Polygon.points[2].y*(_Polygon.points[0].z-_Polygon.points[1].z);
    float B = _Polygon.points[0].z*(_Polygon.points[1].x-_Polygon.points[2].x) + _Polygon.points[1].z*(_Polygon.points[2].x-_Polygon.points[0].x) + _Polygon.points[2].z*(_Polygon.points[0].x-_Polygon.points[1].x);
    float C = _Polygon.points[0].x*(_Polygon.points[1].y-_Polygon.points[2].y) + _Polygon.points[1].x*(_Polygon.points[2].y-_Polygon.points[0].y) + _Polygon.points[2].x*(_Polygon.points[0].y-_Polygon.points[1].y);
    float D = - _Polygon.points[0].x*((_Polygon.points[1].y*_Polygon.points[2].z)-(_Polygon.points[2].y*_Polygon.points[1].z))
                - _Polygon.points[1].x*((_Polygon.points[2].y*_Polygon.points[0].z)-(_Polygon.points[0].y*_Polygon.points[2].z))
                    - _Polygon.points[2].x*((_Polygon.points[0].y*_Polygon.points[1].z)-(_Polygon.points[1].y*_Polygon.points[0].z));
    _planeConstants.push_back(A);
    _planeConstants.push_back(B);
    _planeConstants.push_back(C);
    _planeConstants.push_back(D);
    return(_planeConstants);
}

float FEMAreasGenerator::computeIntersectionOfVizConeAndPolygon(geometry_msgs::Point _conePeak, float _coneDirection ,eu_nifti_env::Polygon _polygon, float _vertAngleCone, float _horAngleCone, float _maxDist) // TODO: Make _coneDirection from an angle around Z axis to a unit vector
 {
     // First transform points to axis with center as conePeak and positive X axis turned towards coneDirection
     geometry_msgs::Vector3 _translationOfConePeakToOrigin = FEMAreasGenerator::constructVector3(-_conePeak.x,-_conePeak.y,-_conePeak.z );    
     geometry_msgs::Point _trConePeak = translateAndRotatePoint(_conePeak, _translationOfConePeakToOrigin, 0, 0, -_coneDirection); // This now becomes 0,0,0
     eu_nifti_env::Polygon _trPolygon;
     for(unsigned int ctr = 0; ctr<_polygon.points.size();ctr++)
     {
         _trPolygon.points.push_back(translateAndRotatePoint(_polygon.points[ctr],_translationOfConePeakToOrigin,0,0,-_coneDirection));
     }
   
     std::vector<float> _planeOfWindow = computePlaneFromPolygon(_trPolygon); //Equation of the plane "A*x + B*y + C*z + D = 0", [A B C] is normal vector to the plane and D is distance from origin.
     //To check pointinPolygon, transform polygon to XY plane of car CS. First, rotate (_coneDirection - M_PI) to YZ plane, then -M_PI/2 to XY plane. Transform origin to first point of polygon
      geometry_msgs::Vector3 _translateOriginToWindow = FEMAreasGenerator::constructVector3(-_polygon.points[0].x,-_polygon.points[0].y,-_polygon.points[0].z);
      geometry_msgs::Vector3 _emptyVec = FEMAreasGenerator::constructVector3(0,0,0);
      for(unsigned int ctr = 0; ctr<_polygon.points.size();ctr++)
        {
         _polygon.points[ctr] = translateAndRotatePoint(_polygon.points[ctr],_translateOriginToWindow, 0,0 ,-(_coneDirection-M_PI)); // First rotate about Z axis to to bring to YZ plane
         _polygon.points[ctr] = translateAndRotatePoint(_polygon.points[ctr],_emptyVec, 0,-M_PI_2 ,0);
        }

     int allPointsCtr = 0;
     int inclPointsCtr = 0;
     int cumulativeDistanceOfInclPoints = 0;
     for(float vctr = -_vertAngleCone/2; vctr<= _vertAngleCone/2; vctr+=ANGLE_INCREMENT_CAMERA_VIEW_CONE)
     {
         for(float hctr =  _horAngleCone/2 ; hctr >= -_horAngleCone/2; hctr-=ANGLE_INCREMENT_CAMERA_VIEW_CONE)
         {
             // Any point on a ray is "R0 + Rd*t" , where R0 is origin, Rd is ray vector, t is distance. To get Rd, take unit vector on X axis and rotate about Z (+ve) and Y (-ve)
             geometry_msgs::Point Rd = rotateAndTranslatePoint(constructPoint(1,0,0),_emptyVec,0, vctr*M_PI/180, 0);
             Rd = rotateAndTranslatePoint(Rd,_emptyVec,0, 0 , hctr*M_PI/180 );
             // Equation of plane A*x + B*y + C*z + D = 0 and ray R0 + Rd*t , so t = -( ((A * R0.x) + (B * R0.y) + (C * R0.z) + D) / ((A * Rd.x) + (B * Rd.y) + (C * Rd.z)) )
             float t = -( ((_planeOfWindow[0] * _trConePeak.x) + (_planeOfWindow[1] * _trConePeak.y) + (_planeOfWindow[2] * _trConePeak.z) + _planeOfWindow[3]) / ((_planeOfWindow[0] * Rd.x) + (_planeOfWindow[1] * Rd.y) + (_planeOfWindow[2] * Rd.z)) );
             if((t<0)||(t>_maxDist))
               {
                  continue;
               }
             geometry_msgs::Point _pointOfXn = constructPoint(_trConePeak.x+(Rd.x*t),_trConePeak.y+(Rd.y*t),_trConePeak.z+(Rd.z*t)); //point of intersection is R0 + Rd * t
             geometry_msgs::Vector3 _translationOfOriginToConePeak = FEMAreasGenerator::constructVector3(_conePeak.x,_conePeak.y,_conePeak.z );
             _pointOfXn = rotateAndTranslatePoint(_pointOfXn,_translationOfOriginToConePeak,0,0,_coneDirection);

             // Now to check if this point is inside our polygon, Transform both the test point and the polygon to XY plane and verify, Let us make the first corner of the polygon as the new origin
             geometry_msgs::Point _trPointOfXn;
             _trPointOfXn = translateAndRotatePoint(_pointOfXn,_translateOriginToWindow, 0,0 ,-(_coneDirection-M_PI)); // _coneDirection - M_PI is the direction the window faces, negative of that is angle which we have to turn it
             _trPointOfXn = translateAndRotatePoint(_trPointOfXn,_emptyVec, 0,-M_PI_2 ,0);

             int inclusion = pointInPolygonHelper(_trPointOfXn,_polygon);
             if(inclusion==1)
             {
                 cumulativeDistanceOfInclPoints += t;
                 inclPointsCtr+=1;
             }
             allPointsCtr+=1;
         }
     }

     // We have cumulative distance of Included points , so
     float avgDistOfInclPoints = float(cumulativeDistanceOfInclPoints)/inclPointsCtr;
     //Given the angle increment of _angleIncr, the distance at this angle is
     float incrDistance = avgDistOfInclPoints*(ANGLE_INCREMENT_CAMERA_VIEW_CONE*M_PI/180);
     // Given no. of incl points is n, the patch size would be pow(pow(n,0.5) - 1)),2 * (incrDistance)^2
     float patchSize = pow( pow(float(inclPointsCtr),0.5)-1 , 2) * pow(incrDistance,2);

    return(patchSize);
 }



#endif	/* GEOMETRICALCOMPUTATION_HPP */

