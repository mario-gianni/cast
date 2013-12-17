/*
	NIFTI project
	Flexible Planning
	Mario Gianni
	01-03-2012
*/

/* select firs action */

select_first_action(Action,Rest,Plan) :- length(Plan,N),
										 nth1(N,Plan,Action,Rest).
										 
/* stubExecution yield the control to the c++ application */

stubExecution(Action,Status) :- yield(Action,Status).

/* Generate the plan according to the task and start the execution of the generated plan */

mEM(Task,Result) :- task(Task,Plan),execution_cycle(Plan,Result).


/* Execution Cycle */

execution_cycle([],Result) :- Result = 0.
execution_cycle(Plan,Result) :- select_first_action(Action,Rest,Plan),	
                                stubExecution(Action,FromRobot),
                                nth1(1,FromRobot,ActionStatus,InternalStatus),
                                ((ActionStatus = 0, updateStatus(InternalStatus),execution_cycle(Rest,Result));
                                 (ActionStatus = 1, updateStatus(InternalStatus),Result = 1);
                                 (ActionStatus = 2, flatten(InternalStatus,NInternalStatus),
                                                    length(NInternalStatus,N),
                                                    nth1(N,NInternalStatus,NewTask),
                                                    mixedInitiative(NewTask,Rest,Result))).
                                                    
mixedInititiative(AddTask,Rest,Result) :- task(AddTask,AddPlan),
                                          flatten([Rest|AddPlan],NewPlan),
                                          execution_cycle(NewPlan,Result).		
                                          
updateStatus(InternalStatus) :- retract_curr_position,
                                retract_current_node,
                                flatten(InternalStatus,NInternalStatus),
                                (foreach(Property,NInternalStatus) do assert(Property)).
