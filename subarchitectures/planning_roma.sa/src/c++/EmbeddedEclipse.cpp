#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include "stdlib.h"
#include "EmbeddedEclipse.hpp"

EmbeddedEclipse::EmbeddedEclipse()
{
	
	ec_set_option_ptr(EC_OPTION_ECLIPSEDIR,getenv("ECLIPSE_ROOT"));
	
	ec_set_option_int(EC_OPTION_IO, MEMORY_IO);
	ec_set_option_long(EC_OPTION_PARALLEL_WORKER, 0);
	ec_set_option_long(EC_OPTION_ALLOCATION, ALLOC_PRE);
	ec_set_option_ptr(EC_OPTION_MAPFILE, NULL);

	
	ec_init();

}

EmbeddedEclipse::~EmbeddedEclipse()
{
	ec_cleanup();
}

int EmbeddedEclipse::resume()
{
	return EC_resume();
}


void EmbeddedEclipse::compile(std::string filename)
{
	std::string compile = "compile('"+filename+"')";
	ec_post_string((char*)compile.c_str());
}

void EmbeddedEclipse::post_string(std::string query)
{
	ec_post_string((char*)query.c_str());
}

void EmbeddedEclipse::posta_goal(EC_word goal)
{
	post_goal(goal);
}

void EmbeddedEclipse::my_assert(EC_word predicate)
{
	EC_functor command_name = EC_functor((char*)"assert",1);
	EC_word command = term(command_name,predicate);
	posta_goal(command);
	resume();
	message_output(1);
	message_output(2);
}

void EmbeddedEclipse::my_retract(EC_word predicate)
{
	EC_functor command_name = EC_functor((char*)"retract",1);
	EC_word command = term(command_name,predicate);
	posta_goal(command);
	resume();
	message_output(1);
	message_output(2);
}

void EmbeddedEclipse::message_output(int queue)
{
	int n;
	char buf[1024];
	n = ec_queue_read(queue,buf,1024);
	buf[n] = 0;
	std::cout << "from eclipse: " << buf << std::endl;
}
