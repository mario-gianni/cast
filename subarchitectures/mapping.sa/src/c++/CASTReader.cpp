#include "CASTReader.hpp"

#include "autogen/eu/nifti/env/CarObjectOfInterest.hpp"

extern "C" {
	cast::CASTComponentPtr newComponent() {
		return new CASTReader();
	}
}


void CASTReader::start() {
    println("starting up");
	addChangeFilter(cast::createLocalTypeFilter<eu::nifti::env::CarObjectOfInterest>(cast::cdl::ADD),new cast::MemberFunctionChangeReceiver<CASTReader>(this, &CASTReader::CarAdded));
    addChangeFilter(cast::createLocalTypeFilter<eu::nifti::env::CarObjectOfInterest>(cast::cdl::OVERWRITE),new cast::MemberFunctionChangeReceiver<CASTReader>(this, &CASTReader::CarModified));
}

CASTReader::~CASTReader() {
    println("aaaaaaaaa i'm dead");
}

void CASTReader::CarAdded(const cast::cdl::WorkingMemoryChange & _wmc){
	//eu::nifti::env::CarObjectOfInterestPtr readCar = getMemoryEntry<eu::nifti::env::CarObjectOfInterest>(_wmc.address);
	//println("A new car %s has been added by %s", readCar->carClass.c_str(),_wmc.src.c_str());
	return;
}

void CASTReader::CarModified(const cast::cdl::WorkingMemoryChange & _wmc){
	eu::nifti::env::CarObjectOfInterestPtr readCar = getMemoryEntry<eu::nifti::env::CarObjectOfInterest>(_wmc.address);
	println("An existing Car %d , has been modified by %s", readCar->uuid,_wmc.src.c_str());
	println("Printing the vantage points");
	for(unsigned int ctr = 0;ctr<readCar->vantagePoints.size();ctr++)
	{
		println("Vantage point number %d",ctr);
		println("x : %6.4f y : %6.4f z : %6.4f theta : %6.4f",readCar->vantagePoints[ctr]->pose.x,readCar->vantagePoints[ctr]->pose.y,readCar->vantagePoints[ctr]->pose.z,readCar->vantagePoints[ctr]->pose.theta);
	}
	return;
}
