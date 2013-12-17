/*
 * Parser.cpp
 *
 *  Created on: Dec 29, 2011
 *      Author: alcor
 */

#include <Parser.hpp>
#include "tf/transform_listener.h"
#include <geometry_msgs/Quaternion.h>

Parser::Parser() {
	// TODO Auto-generated constructor stub

}

Parser::~Parser() {
	// TODO Auto-generated destructor stub
}

TopoGraphWriterActionPtr Parser::eclipse2IceTopoGraphWriterAction(EC_word action)
{
    TopoGraphWriterActionPtr _action = new TopoGraphWriterAction();

    EC_functor action_name;
    action.functor(&action_name);
    std::string temp = action_name.name();

    if(temp.compare("start_read_topo") == 0)
    {
    	_action->name = "start_read_topo";
        _action->op = START;
        EC_word time;
        action.arg(1,time);
        if(time.is_var() == EC_succeed)
        {
            _action->time = -1;
        }
        else
        {
            double _time;
		    time.is_double(&_time);
		    _action->time = _time;
        }
    }
    else if(temp.compare("end_read_topo") == 0)
    {
    	_action->name = "end_read_topo";
        _action->op = END;
        EC_word time;
        action.arg(1,time);
        if(time.is_var() == EC_succeed)
        {
            _action->time = -1;
        }
        else
        {
            double _time;
		    time.is_double(&_time);
		    _action->time = _time;
        }
    }
    else
    {
        std::cout << "From Eclipse -> ICE: Unknown action" << std::endl;
    }

    return _action;
}

FunctionalMappingActionPtr Parser::eclipse2IceFuncMappingAction(EC_word action)
{
    FunctionalMappingActionPtr _action = new FunctionalMappingAction();

    EC_functor action_name;
    action.functor(&action_name);
    std::string temp = action_name.name();

     if(temp.compare("start_func_map") == 0)
    {
    	_action->name = "start_func_map";
        _action->op = START;
        EC_word time;
        action.arg(1,time);
        if(time.is_var() == EC_succeed)
        {
            _action->time = -1;
        }
        else
        {
            double _time;
		    time.is_double(&_time);
		    _action->time = _time;
        }
    }
    else if(temp.compare("end_func_map") == 0)
    {
    	_action->name = "end_func_map";
        _action->op = END;
        EC_word time;
        action.arg(1,time);
        if(time.is_var() == EC_succeed)
        {
            _action->time = -1;
        }
        else
        {
            double _time;
		    time.is_double(&_time);
		    _action->time = _time;
        }
    }
    else
    {
        std::cout <<  "From Eclipse -> ICE: Unknown action" << std::endl;
    }


    return _action;
}

TogoGraphBuilderActionPtr Parser::eclipse2IceTopoGraphBuilderAction(EC_word action)
{
    TogoGraphBuilderActionPtr _action = new TogoGraphBuilderAction();

    EC_functor action_name;
    action.functor(&action_name);
    std::string temp = action_name.name();

    if(temp.compare("start_build_topo") == 0)
    {
    	_action->name = "start_build_topo";
        _action->op = START;
        EC_word time;
        action.arg(1,time);
        if(time.is_var() == EC_succeed)
        {
            _action->time = -1;
        }
        else
        {
            double _time;
		    time.is_double(&_time);
		    _action->time = _time;
        }
    }
    else if(temp.compare("end_build_topo") == 0)
    {
    	_action->name = "end_build_topo";
        _action->op = END;
        EC_word time;
        action.arg(1,time);
        if(time.is_var() == EC_succeed)
        {
            _action->time = -1;
        }
        else
        {
            double _time;
		    time.is_double(&_time);
		    _action->time = _time;
        }
    }
    else
    {
        std::cout << "From Eclipse -> ICE: Unknown action" << std::endl;
    }
    return _action;
}

FlipperActionPtr Parser::eclipse2IceFlipperAction(EC_word action)
{
    FlipperActionPtr _action = new FlipperAction();

    EC_functor action_name;
    action.functor(&action_name);
    _action->name = action_name.name();

    EC_word comp;
    action.arg(1,comp);
    EC_atom component;
    comp.is_atom(&component);
    _action->component = component.name();

    EC_word angle;
    action.arg(2,angle);
    double alfa;
    angle.is_double(&alfa);
    _action->alfa = alfa;

    EC_word time;
    action.arg(3,time);
    if(time.is_var() == EC_succeed)
    {
        _action->time = -1;
    }
    else
    {
        double _time;
	    time.is_double(&_time);
		_action->time = _time;
    }

    return _action;
}

GapDetectionActionPtr Parser::eclipse2IceGapDetectionAction(EC_word action)
{
    GapDetectionActionPtr _action = new GapDetectionAction();

    EC_functor action_name;
	action.functor(&action_name);
	std::string type = action_name.name();

	if(type.compare("start_gap_detection") == 0)
	{
		_action->name = "start_gap_detection";
	    _action->op = START;
        EC_word time;
        action.arg(1,time);
        if(time.is_var() == EC_succeed)
        {
            _action->time = -1;
        }
        else
        {
            double _time;
		    time.is_double(&_time);
		    _action->time = _time;
        }
	}
	else if(type.compare("end_gap_detection") == 0)
	{
		_action->name = "end_gap_detection";
	    _action->op = END;
        EC_word time;
        action.arg(1,time);
        if(time.is_var() == EC_succeed)
        {
            _action->time = -1;
        }
        else
        {
            double _time;
		    time.is_double(&_time);
		    _action->time = _time;
        }
	}
	else
	{
	    std::cout <<  "From Eclipse -> ICE: Action Unknown" << std::endl;
	}

    return _action;
}

GoToNodeActionPtr Parser::eclipse2IceGoToNodeAction(EC_word action)
{
	GoToNodeActionPtr _action = new GoToNodeAction();
	NodePtr my_node = new Node();

	EC_functor action_name;
	action.functor(&action_name);
	_action->name = action_name.name();

	EC_word node;
	action.arg(1,node);

	EC_word label;
	node.arg(1,label);
	EC_atom _label;
	label.is_atom(&_label);
	my_node->label = _label.name();

	EC_word x;
	node.arg(2,x);
	double _x;
	x.is_double(&_x);
	my_node->x = _x;

	EC_word y;
	node.arg(3,y);
	double _y;
	y.is_double(&_y);
	my_node->y = _y;

	EC_word flag;
	node.arg(4,flag);
	double _flag;
	flag.is_double(&_flag);
	my_node->flag = _flag;

	EC_word theta;
	action.arg(2,theta);
	if(theta.is_var() == EC_succeed)
	{
		_action->theta = 0;
	}
	else
	{
		double _theta;
		theta.is_double(&_theta);
		_action->theta = _theta;
	}

	EC_word time;
	action.arg(3,time);
	if(time.is_var() == EC_succeed)
	{
		_action->time = -1;
	}
	else
	{
		double _time;
		time.is_double(&_time);
		_action->time = _time;
	}

	_action->node = my_node;

	return _action;
}

RotatingLaserActionPtr Parser::eclipse2IceRotatingLaserAction(EC_word action)
{
    RotatingLaserActionPtr _action = new RotatingLaserAction();

    EC_functor action_name;
    action.functor(&action_name);
    _action->name = action_name.name();

    EC_word speed;
    action.arg(1,speed);
    double _speed;
    speed.is_double(&_speed);
    _action->speed = _speed;

    EC_word time;
    action.arg(2,time);
    if(time.is_var() == EC_succeed)
	{
		_action->time = -1;
	}
	else
	{
		double _time;
		time.is_double(&_time);
		_action->time = _time;
	}

    return _action;
}

CenterLaserActionPtr Parser::eclipse2IceCenterLaserAction(EC_word action)
{
    CenterLaserActionPtr _action = new CenterLaserAction();

    EC_functor action_name;
    action.functor(&action_name);
    _action->name = action_name.name();

    EC_word time;
    action.arg(1,time);
    if(time.is_var() == EC_succeed)
	{
		_action->time = -1;
	}
	else
	{
		double _time;
		time.is_double(&_time);
		_action->time = _time;
	}

    return _action;
}

DifferentialActionPtr Parser::eclipse2IceDifferentialAction(EC_word action)
{
    DifferentialActionPtr _action = new DifferentialAction();

    EC_functor action_name;
	action.functor(&action_name);
	std::string type = action_name.name();

	if(type.compare("lock") == 0)
    {
		_action->name = "lock";
        _action->flag = ON;
        EC_word time;
        action.arg(1,time);
        if(time.is_var() == EC_succeed)
        {
            _action->time = -1;
        }
        else
        {
            double _time;
		    time.is_double(&_time);
		    _action->time = _time;
        }
    }
    else if(type.compare("unlock") == 0)
    {
    	_action->name = "unlock";
        _action->flag = OFF;
        EC_word time;
        action.arg(1,time);
        if(time.is_var() == EC_succeed)
        {
            _action->time = -1;
        }
        else
        {
            double _time;
		    time.is_double(&_time);
		    _action->time = _time;
        }
    }
    else
    {
        std::cout <<  "From Eclipse -> ICE: Unknown action" << std::endl;
    }

    return _action;
}

MoveBaseActionPtr Parser::eclipse2IceMoveBaseAction(EC_word action)
{
    MoveBaseActionPtr _action = new MoveBaseAction();

    EC_functor action_name;
	action.functor(&action_name);
	std::string motion = action_name.name();

	if(motion.compare("move_forward") == 0)
	{
		_action->name = "move_forward";
        _action->command = MOVEFORWARD;
	}
	else if(motion.compare("move_left") == 0)
	{
		_action->name = "move_left";
	    _action->command = MOVELEFT;
	}
	else if(motion.compare("move_right") == 0)
	{
		_action->name = "move_right";
	    _action->command = MOVERIGHT;
	}
	else if(motion.compare("move_back") == 0)
	{
		_action->name = "move_back";
	    _action->command = MOVEBACK;
	}
	else if(motion.compare("turn_left") == 0)
	{
		_action->name = "turn_left";
	    _action->command = TURNLEFT;
	}
	else if(motion.compare("turn_right") == 0)
	{
		_action->name = "turn_right";
	    _action->command = TURNRIGHT;
	}
	else
	{
	    std::cout << "From Eclipse -> ICE: Unknown action" << std::endl;
	}

	 EC_word time;
     action.arg(1,time);
     if(time.is_var() == EC_succeed)
     {
        _action->time = -1;
     }
     else
     {
        double _time;
	    time.is_double(&_time);
		_action->time = _time;
     }

    return _action;
}

AutoModeActionPtr Parser::eclipse2IceAutoModeAction(EC_word w)
{
	AutoModeActionPtr result = new AutoModeAction();
	EC_atom atom;
	if(!(w.is_atom(&atom) && strcmp("auto_mode", atom.name()) == 0)) {
		 std::cout << "From Eclipse -> ICE: bad AutoModeAction" << std::endl;
	}
	return result;
}

EC_word Parser::artefact(ArtefactPtr obj)
{
	EC_word result;
	EC_functor _artefact = EC_functor((char*)"artefact",11);
	EC_atom label = EC_atom((char*)obj->label.c_str());
	EC_word doc = EC_word(obj->confidenceDegree);
	EC_word x = EC_word(obj->mapPose.pos.x);
	EC_word y = EC_word(obj->mapPose.pos.y);
	EC_word z = EC_word(obj->mapPose.pos.z);

	geometry_msgs::Quaternion q;
	q.x = obj->mapPose.orient.x;
	q.y = obj->mapPose.orient.y;
	q.z = obj->mapPose.orient.z;
	q.w = obj->mapPose.orient.w;
	double yaw = tf::getYaw(q);
	EC_word theta = EC_word(yaw);

	EC_word bx = EC_word(obj->bbox.x);
	EC_word by = EC_word(obj->bbox.y);
	EC_word bz = EC_word(obj->bbox.z);
	EC_word timestamp = EC_word(obj->timestamp);

	EC_atom type;

	switch(obj->type)
	{
	case eu::nifti::env::are::CAR:
		type = EC_atom((char*)"car");
		break;
	case eu::nifti::env::are::MALE:
		type = EC_atom((char*)"male");
		break;
	case eu::nifti::env::are::FEMALE:
		type = EC_atom((char*)"female");
		break;
	case eu::nifti::env::are::ROBOT:
		type = EC_atom((char*)"robot");
		break;
	default:
		std::cout<<"****** ERROR - Artefact Type not found *********"<<std::endl;
		break;
	}

	EC_word args[11];
	args[0] = timestamp;
	args[1] = type;
	args[2] = label;
	args[3] = x;
	args[4] = y;
	args[5] = z;
	args[6] = theta;
	args[7] = bx;
	args[8] = by;
	args[9] = bz;
	args[10] = doc;

	result = term(_artefact,args);
	return result;
}

EC_word Parser::node(NodePtr node)
{
	EC_word result;
	EC_functor _node = EC_functor((char*)"node",4);
	EC_atom label = EC_atom((char*)node->label.c_str());
	EC_word x = EC_word(node->x);
	EC_word y = EC_word(node->y);
	EC_word flag = EC_word(node->flag);
	result = term(_node,label,x,y,flag);
	return result;
}

EC_word Parser::edge(EdgePtr edge)
{
    EC_word result;
	EC_functor _edge = EC_functor((char*)"edge",3);
	EC_word a = node(edge->a);
	EC_word b = node(edge->b);
	EC_word property = EC_word(1);
	result = term(_edge,a,b,property);
	return result;
}

EC_word Parser::in(CurrentPosPtr cp)
{
    EC_word result;

	EC_functor holds = EC_functor((char*)"holds",3);
	EC_atom nav = EC_atom((char*)"navigation");

	EC_functor _in = EC_functor((char*)"in",1);
	EC_word _node = node(cp->node);
	EC_word _in_ = term(_in,_node);

	EC_word s = newvar();

	result = term(holds,nav,_in_,s);

	return result;
}

EC_word Parser::origin(BasePosPtr base)
{
    EC_word result;
    EC_functor _base = EC_functor((char*)"base",3);
    EC_atom nav = EC_atom((char*)"navigation");
    EC_word pos = node(base->node);
    EC_word s = nil();
    result = term(_base,nav,pos,s);
    return result;
}

EC_word Parser::at(PosePtr pose)
{
    EC_word result;
    EC_functor holds = EC_functor((char*)"holds",3);
    EC_atom nav = EC_atom((char*)"navigation");

    EC_functor _at = EC_functor((char*)"at",4);
    EC_word _x = EC_word(pose->x);
    EC_word _y = EC_word(pose->y);
    EC_word _z = EC_word(pose->z);
    EC_word _theta = EC_word(pose->theta);
    EC_word pos = term(_at,_x,_y,_z,_theta);

    EC_word s = newvar();

    result = term(holds,nav,pos,s);
    return result;
}

EC_word Parser::battery(BatteryStatusPtr status)
{
	EC_word result;
	EC_functor holds = EC_functor((char*)"holds",3);
    EC_atom diag = EC_atom((char*)"diagnostic");

    EC_functor battery = EC_functor((char*)"battery",1);
    EC_word _battery = newvar();

    switch(status->level)
    {
    case eu::nifti::env::diagnostic::BHIGH:
    	_battery = term(battery,EC_atom((char*)"high"));
    	break;
    case eu::nifti::env::diagnostic::BMEDIUM:
    	_battery = term(battery,EC_atom((char*)"medium"));
    	break;
    case eu::nifti::env::diagnostic::BLOW:
    	_battery = term(battery,EC_atom((char*)"low"));
    	break;
    default:
    	std::cout<<"****** ERROR - Battery status not found *********"<<std::endl;
    	break;
    }

    EC_word s = newvar();
    result = term(holds,diag,_battery,s);

	return result;
}
EC_word Parser::wifi(WiFiStatusPtr status)
{
	EC_word result;
	EC_functor holds = EC_functor((char*)"holds",3);
    EC_atom diag = EC_atom((char*)"diagnostic");

    EC_functor wifi = EC_functor((char*)"wifi",1);
    EC_word _wifi = newvar();

    switch(status->quality)
    {
    case eu::nifti::env::diagnostic::GOOD:
    	_wifi = term(wifi,EC_atom((char*)"good"));
    	break;
    case eu::nifti::env::diagnostic::MODERATE:
    	_wifi = term(wifi,EC_atom((char*)"moderate"));
    	break;
    case eu::nifti::env::diagnostic::WEAK:
    	_wifi = term(wifi,EC_atom((char*)"weak"));
    	break;
    case eu::nifti::env::diagnostic::LOST:
    	_wifi = term(wifi,EC_atom((char*)"lost"));
    	break;
    default:
    	std::cout<<"****** ERROR - Wifi status not found *********"<<std::endl;
    	break;
    }
    EC_word s = newvar();
    result = term(holds,diag,_wifi,s);
	return result;
}

EC_word Parser::wifiStrength(CurrentPosPtr position, WiFiStatusPtr status)
{
	EC_word result;
	result = newvar();

	EC_functor wifi_strenght = EC_functor((char*)"wifi_strenght",2);
	EC_atom label = EC_atom((char*)position->node->label.c_str());

	switch(status->quality)
	{
	case eu::nifti::env::diagnostic::GOOD:
		result = term(wifi_strenght,label,EC_atom((char*)"good"));
		break;
	case eu::nifti::env::diagnostic::MODERATE:
		result = term(wifi_strenght,label,EC_atom((char*)"moderate"));
		break;
	case eu::nifti::env::diagnostic::WEAK:
		result = term(wifi_strenght,label,EC_atom((char*)"weak"));
		break;
	case eu::nifti::env::diagnostic::LOST:
		result = term(wifi_strenght,label,EC_atom((char*)"lost"));
		break;
	default:
		std::cout<<"****** ERROR - Wifi status not found *********"<<std::endl;
		break;
	}

	return result;
}

EC_word Parser::ice2EclipseGotoNodeAction(GoToNodeActionPtr action)
{
	EC_word result;
	EC_functor _action = EC_functor((char*)action->name.c_str(),3);
	EC_word _node = node(action->node);

	EC_word _theta;
    _theta = EC_word(action->theta);

	EC_word _time = EC_word(action->time);
	result = term(_action,_node,_theta,_time);
	return result;
}

EC_word Parser::ice2EclipseFlipperAction(FlipperActionPtr action)
{
    EC_word result;
    EC_functor move = EC_functor((char*)"move",3);
    EC_atom flipper = EC_atom((char*)action->component.c_str());
    EC_word angle = EC_word(action->alfa);
    EC_word time = EC_word(action->time);
    result = term(move,flipper,angle,time);
    return result;
}

EC_word Parser::ice2EclipseTopoGraphBuilderAction(TogoGraphBuilderActionPtr action)
{
    EC_word result;

    if(action->op == START)
    {
        EC_functor _action = EC_functor((char*)"start_build_topo",1);
        EC_word _time = EC_word(action->time);
        result = term(_action,_time);
    }
    else if(action->op == END)
    {
        EC_functor _action = EC_functor((char*)"end_build_topo",1);
        EC_word _time = EC_word(action->time);
        result = term(_action,_time);
    }
    else
    {
        std::cout << "From ICE -> Eclipse: Unknown action" << std::endl;
    }

    return result;
}

EC_word Parser::ice2EclipseGapDetectionAction(GapDetectionActionPtr action)
{
    EC_word result;

    if(action->op == START)
    {
        EC_functor _action = EC_functor((char*)"start_gap_detection",1);
        EC_word _time = EC_word(action->time);
        result = term(_action,_time);
    }
    else if(action->op == END)
    {
        EC_functor _action = EC_functor((char*)"end_gap_detection",1);
        EC_word _time = EC_word(action->time);
        result = term(_action,_time);
    }
    else
    {
        std::cout << "From ICE -> Eclipse: Unknown action" << std::endl;
    }

    return result;
}

EC_word Parser::ice2EclipseTopoGraphWriterAction(TopoGraphWriterActionPtr action)
{
    EC_word result;

    if(action->op == START)
    {
        EC_functor _action = EC_functor((char*)"start_read_topo",1);
        EC_word _time = EC_word(action->time);
        result = term(_action,_time);
    }
    else if(action->op == END)
    {
        EC_functor _action = EC_functor((char*)"end_read_topo",1);
        EC_word _time = EC_word(action->time);
        result = term(_action,_time);
    }
    else
    {
        std::cout << "From ICE -> Eclipse: Unknown action" << std::endl;
    }

    return result;
}

EC_word Parser::ice2EclipseFunctMappingAction(FunctionalMappingActionPtr action)
{
    EC_word result;

    if(action->op == START)
    {
        EC_functor _action = EC_functor((char*)"start_func_map",1);
        EC_word _time = EC_word(action->time);
        result = term(_action,_time);
    }
    else if(action->op == END)
    {
        EC_functor _action = EC_functor((char*)"end_func_map",1);
        EC_word _time = EC_word(action->time);
        result = term(_action,_time);
    }
    else
    {
        std::cout << "From ICE -> Eclipse: Unknown action" << std::endl;
    }

    return result;
}

EC_word Parser::ice2EclipseRotatingLaserAction(RotatingLaserActionPtr action)
{
    EC_word result;
    EC_functor _action = EC_functor((char*)"start_rotation",2);
    EC_word speed = EC_word(action->speed);
    EC_word time = EC_word(action->time);
    result = term(_action,speed,time);
    return result;
}

EC_word Parser::ice2EclipseCenterLaserAction(CenterLaserActionPtr action)
{
    EC_word result;
    EC_functor _action = EC_functor((char*)"end_rotation",1);
    EC_word time = EC_word(action->time);
    result = term(_action,time);
    return result;
}

EC_word Parser::ice2EclipseDifferentialAction(DifferentialActionPtr action)
{
    EC_word result;

    if(action->flag == ON)
    {
        EC_functor lock = EC_functor((char*)"lock",1);
        EC_word _time = EC_word(action->time);
        result = term(lock,_time);
    }
    else if(action->flag == OFF)
    {
        EC_functor unlock = EC_functor((char*)"unlock",1);
        EC_word _time = EC_word(action->time);
        result = term(unlock,_time);
    }
    else
    {
        std::cout << "From ICE -> Eclipse: Unknown Action" << std::endl;
    }

    return result;
}

EC_word Parser::ice2EclipseMoveBaseAction(MoveBaseActionPtr action)
{
    EC_word result;
    if(action->command == MOVEFORWARD)
    {
        EC_functor motion = EC_functor((char*)"move_forward",1);
        EC_word _time = EC_word(action->time);
        result = term(motion,_time);
    }
    else if(action->command == MOVELEFT)
    {
        EC_functor motion = EC_functor((char*)"move_left",1);
        EC_word _time = EC_word(action->time);
        result = term(motion,_time);
    }
    else if(action->command == MOVERIGHT)
    {
        EC_functor motion = EC_functor((char*)"move_right",1);
        EC_word _time = EC_word(action->time);
        result = term(motion,_time);
    }
    else if(action->command == MOVEBACK)
    {
        EC_functor motion = EC_functor((char*)"move_back",1);
        EC_word _time = EC_word(action->time);
        result = term(motion,_time);
    }
    else if(action->command == TURNLEFT)
    {
        EC_functor motion = EC_functor((char*)"turn_left",1);
        EC_word _time = EC_word(action->time);
        result = term(motion,_time);
    }
    else if(action->command == TURNRIGHT)
    {
        EC_functor motion = EC_functor((char*)"turn_right",1);
        EC_word _time = EC_word(action->time);
        result = term(motion,_time);
    }
    else
    {
        std::cout << "From ICE -> Eclipse: Unknown Action" << std::endl;
    }

    return result;
}

EC_word Parser::ice2EclipseAutoModeAction(AutoModeActionPtr a)
{
	EC_word result = EC_word(EC_atom((char*)"auto_mode"));
    return result;
}

