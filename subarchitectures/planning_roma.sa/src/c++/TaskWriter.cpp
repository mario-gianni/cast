#include <TaskWriter.hpp>

void TaskWriter::start()
{
}

void TaskWriter::runComponent()
{
/*
	char c_input;
	std::cout<<"Insert a character: "<<std::endl;
	std::cout<<"press a to MOVE FORWARD task"<<std::endl;
	std::cout<<"press b to MOVE BACK task"<<std::endl;
	std::cout<<"press c to MOVE LEFT task"<<std::endl;
	std::cout<<"press d to MOVE RIGHT task"<<std::endl;
	std::cout<<"press e to TURN LEFT task"<<std::endl;
	std::cout<<"press f to TURN RIGHT task"<<std::endl;

	std::cout<<"press q to QUIT"<<std::endl;

	std::cin>>std::noskipws>>c_input;

	int i = static_cast<int>(c_input);

	eu::nifti::Planning::slice::TaskPtr task;

	switch(i)
	{
	case 97: 
		task = new eu::nifti::Planning::slice::MoveBaseTask();
		task->id = newDataID();
		task->status = eu::nifti::Planning::slice::NEW;
		addToWorkingMemory(newDataID(),task);
		break;
	case 98: 
		task = new eu::nifti::Planning::slice::MoveBaseTask();
		task->id = newDataID();
		task->status = eu::nifti::Planning::slice::NEW;
		addToWorkingMemory(newDataID(),task);
		break;
	case 99: 
		task = new eu::nifti::Planning::slice::MoveBaseTask();
		task->id = newDataID();
		task->status = eu::nifti::Planning::slice::NEW;
		addToWorkingMemory(newDataID(),task);
		break;
	case 100: 
		task = new eu::nifti::Planning::slice::MoveBaseTask();
		task->id = newDataID();
		task->status = eu::nifti::Planning::slice::NEW;
		addToWorkingMemory(newDataID(),task);
		break;
	case 101: 
		task = new eu::nifti::Planning::slice::MoveBaseTask();
		task->id = newDataID();
		task->status = eu::nifti::Planning::slice::NEW;
		addToWorkingMemory(newDataID(),task);
		break;
	case 102: 
		task = new eu::nifti::Planning::slice::MoveBaseTask();
		task->id = newDataID();
		task->status = eu::nifti::Planning::slice::NEW;
		addToWorkingMemory(newDataID(),task);
		break;
	default:
		println("ERROR - Task not found");
		break;
	}
*/
    //eu::nifti::Planning::slice::NavTaskPtr task1 = new eu::nifti::Planning::slice::NavTask();
    //eu::nifti::Planning::slice::GUITaskPtr task3 = new eu::nifti::Planning::slice::GUITask();
    eu::nifti::Planning::slice::TopoGraphTaskPtr task4 = new eu::nifti::Planning::slice::TopoGraphTask();
    //eu::nifti::Planning::slice::DetectGapTraversableTaskPtr task5 = new eu::nifti::Planning::slice::DetectGapTraversableTask();
    //eu::nifti::Planning::slice::MoveBaseTaskPtr task6 = new eu::nifti::Planning::slice::MoveBaseTask(); 
    //eu::nifti::Planning::slice::FailurePlanActionPtr a = new eu::nifti::Planning::slice::FailurePlanAction(); 
    //eu::nifti::Planning::slice::TopoGraphTaskPtr task7 = new eu::nifti::Planning::slice::TopoGraphTask();
    
    //task6->status = eu::nifti::Planning::slice::NEW;

    //task6->command = eu::nifti::Planning::slice::MOVEFORWARD;
    //println("*********************** Gap Detection test START *********************************");

    //sleep(10);
    //addToWorkingMemory(newDataID(),task6);*/
    
    //sleep(50);
    //eu::nifti::Planning::slice::TraverseGapTaskPtr task8 = new eu::nifti::Planning::slice::TraverseGapTask();
    //task8->status = eu::nifti::Planning::slice::NEW;
    //println("*********************** Gap Traversability test START *********************************");
    //addToWorkingMemory(newDataID(),task8);


    println("*********************** INITIALIZATION *********************************");
    std::string id = newDataID();
    task4->id = id;
    task4->name = "init";
    task4->status = eu::nifti::Planning::slice::NEW;
    sleep(10);
    addToWorkingMemory(id,task4);

    //task4->name = "read_map";
    //task4->status = eu::nifti::Planning::slice::NEW;
    //sleep(30);
    //addToWorkingMemory(newDataID(),task4);
    //task4->status = eu::nifti::Planning::slice::ADDED;
    //addToWorkingMemory(newDataID(),task4);

    //sleep(10);
    //addToWorkingMemory(newDataID(),task5);
    /*
    std::string id = newDataID();
    //std::string status = "PENDING";
    
    eu::nifti::env::topograph::NodePtr node = new eu::nifti::env::topograph::Node();
    node->label = "n160";
    
    task1->id = id;
    //task1->status = status;
    task1->node = node;
    //addToWorkingMemory(id,task1);

    
    task3->id = id;
    //task3->status = status;
    task3->x = 1;
    task3->y = 2;
    //addToWorkingMemory(id,task3);
    
    task4->id = id;
    task4->name = "test_mixed_init";
    task4->status = eu::nifti::Planning::slice::NEW;
    sleep(10);
    addToWorkingMemory(id,task4);
    task5->status = eu::nifti::Planning::slice::NEW;
    
    sleep(2);
    //addToWorkingMemory(newDataID(),a);
    //println("*****************************************************************************");
    //println("*********************** action failure sent *********************************");
    //println("*****************************************************************************");
    //sleep(50);
    //addToWorkingMemory(id,task5);
    
    task6->id = newDataID();
    //task6->name = "turn_right";
    task6->status = eu::nifti::Planning::slice::ADDED;
    task6->command = eu::nifti::Planning::slice::TURNRIGHT;
    //sleep(10);
    println("*******************************************************************************");
    println("************************** new task requested *********************************");
    println("*******************************************************************************");
    addToWorkingMemory(newDataID(),task6);
    println("done");
    
*/
//    task4->name = "read_map";
//    task4->status = eu::nifti::Planning::slice::NEW;
//    sleep(10);
//    println("************************** new task requested *********************************");
//    addToWorkingMemory(newDataID(),task4);
    //println("************************** new task sent *********************************");
    //sleep(1);
    //task4->status = eu::nifti::Planning::slice::ADDED;
    //addToWorkingMemory(newDataID(),task4);
    //println("************************** new task added *********************************");

/*
    eu::nifti::Planning::slice::TopoGraphWriterActionPtr action = new eu::nifti::Planning::slice::TopoGraphWriterAction();
    action->op = eu::nifti::Planning::slice::START;
    sleep(10);
    addToWorkingMemory(newDataID(),action);
    println("************************** new action added 1s ********************************");
    sleep(10);
    action->op = eu::nifti::Planning::slice::END;
    addToWorkingMemory(newDataID(),action);
    println("************************** new action added 1e********************************");
    sleep(10);
    action->op = eu::nifti::Planning::slice::START;
    addToWorkingMemory(newDataID(),action);
    println("************************** new action added 2s********************************");
*/
}

extern "C" 
{
	cast::CASTComponentPtr newComponent() 
	{
    	return new TaskWriter();
  	}
}
