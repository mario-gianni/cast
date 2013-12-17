/*
	NIFTI project
	author: Mario Gianni
	date: 10 03 2011
*/

holds(navigation,at(0,0,0,0),[]).
holds(navigation,in(node(n,0,0,0)),[]).

retract_graph :- findall(Node,graph(Node),NodeList),
                 findall(Edge,edge(Edge),EdgeList),
                 (foreach(N,NodeList) do retract(graph(N))),
                 (foreach(E,EdgeList) do retract(edge(E))).

retract_curr_position :- retract(holds(navigation,at(X,Y,Z,Theta),S)).
retract_current_node :- retract(holds(navigation,in(Node),S)).

/* select firs action */

select_first_action(Action,Rest,Plan) :- length(Plan,N),
										 nth1(N,Plan,Action,Rest).

/* execution monitoring */

em(Plan,Result) :- execution_cycle(Plan,Result).

execution_cycle([],Result) :- Result = 0.
execution_cycle(Plan,Result) :- select_first_action(Action,Rest,Plan),
							    stubExecution(Action,Status),
							    (Status = 0 -> execution_cycle(Rest,Result)
							                 ;
							     Status = 1, Result = 1).
							     
/* stubExecution yield the control to the c++ application */

stubExecution(Action,Status) :- yield(Action,Status).

em2(Plan,Result) :- execution_cycle2(Plan,Result).

execution_cycle2([],Result) :- Result = 0.

execution_cycle2(Plan,Result) :- select_first_action(Action,Rest,Plan),
							     stubExecution1(Action,List),nth1(1,List,Status),
							     writeln(Status),
							     writeln(NewAction),			     
							     ((Status = 0,execution_cycle2(Rest,Result));
							     (Status = 1, Result = 1);
							     (Status = 2,nth1(2,List,NewAction),mixInit(NewAction,Rest,Result))).
							    
mixInit(NewAction,Rest,Result) :- execution_cycle2([Rest|NewAction],Result).

stubExecution1(Action,[Status|NewAction]) :- yield1(Action,[Status|NewAction]).
yield1(a1,[0]).
yield1(a2,[0]).
yield1(a3,[2,read_map]).
yield1(a4,[0]).
yield1(b1,[0]).
yield1(start_read_topo(T1),[2,move_left]).
yield1(move_left(T3),[0]).
yield1(end_read_topo(T2),[0]).

em3(Plan,Result) :- open(execution,write,S),write(S,Plan),close(S),execution_cycle4(Plan,Result).

execution_cycle3([],Result) :- Result = 0.
execution_cycle3(Plan,Result) :- select_first_action(Action,Rest,Plan),
                                 stubExecution(Action,StatusList),
                                 nth1(1,StatusList,Status),
                                 ((Status = 0, execution_cycle3(Rest,Result));
                                  (Status = 1, Result = 1);
                                  (Status = 2, flatten(StatusList,NewStatusList),
                                   nth1(2,NewStatusList,AddTask),mixInit3(AddTask,Rest,Result))).
                                  
mixInit3(AddTask,Rest,Result) :- task(AddTask,AddPlan),
                                 flatten([Rest|AddPlan],NewPlan),open(backup3,write,S),write(S,NewPlan),close(S),
                                 execution_cycle4(NewPlan,Result).
                                 
em4(Task,Result) :- task(Task,Plan),
                    em3(Plan,Result).
                    
execution_cycle4([],Result) :- Result = 0.
execution_cycle4(Plan,Result) :- select_first_action(Action,Rest,Plan),
                                 stubExecution(Action,StatusList),open(backup1,write,S),write(S,StatusList),close(S),
                                 nth1(1,StatusList,Status,NewStatusList),
                                 ((Status = 0, updateStatus(NewStatusList),execution_cycle4(Rest,Result));
                                  (Status = 1, updateStatus(NewStatusList),Result = 1);
                                  (Status = 2, flatten(NewStatusList,NNewStatusList),length(NNewStatusList,N),nth1(N,NNewStatusList,NewTask),mixInit3(NewTask,Rest,Result))).
                                  
updateStatus(StatusList) :- retract_curr_position,retract_current_node,retract_graph,open(backup2,write,S),flatten(StatusList,NewList),(foreach(Property,NewList),param(S) do assert(Property),write(S,Property)),close(S).
