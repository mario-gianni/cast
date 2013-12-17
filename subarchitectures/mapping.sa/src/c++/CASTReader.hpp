#ifndef CAST_READER_HPP_
#define CAST_READER_HPP_

#include <cast/architecture.hpp>
//#include "autogen/mapping.hpp"

class CASTReader: public cast::ManagedComponent {

	protected: virtual void start();
	public: virtual ~CASTReader();
	protected: virtual void CarAdded(const cast::cdl::WorkingMemoryChange&);
	protected: virtual void CarModified(const cast::cdl::WorkingMemoryChange&);

};



#endif
