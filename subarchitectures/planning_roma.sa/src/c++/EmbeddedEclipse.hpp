#ifndef EMBEDDED_ECLIPSE_HPP_
#define EMBEDDED_ECLIPSE_HPP_

#include "eclipseclass.h"

class EmbeddedEclipse
{

	public:
	
		EmbeddedEclipse();
		~EmbeddedEclipse();
		
		void compile(std::string);
		void post_string(std::string);
		void posta_goal(EC_word);
		void my_assert(EC_word);
		void my_retract(EC_word);
		int resume();
		void message_output(int);

};

#endif
