#ifndef _TASK_WRITER_HPP_
#define _TASK_WRITER_HPP_

#include <cast/architecture.hpp>
#include <planning_roma.hpp>


class TaskWriter : public cast::ManagedComponent
{
    protected:
        virtual void start();
        virtual void runComponent();


};

#endif
