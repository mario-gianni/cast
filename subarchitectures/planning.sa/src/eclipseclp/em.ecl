% NIFTI Project
% from technical report about flexible planning
% author Gianni Mario
% date: 04 - 01 - 2011

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%
% select the actions to be executed according to the time value
% es select_actions([[a(Time)],[b(Time)],[c(Time1)]],Time,[[a(Time)],[b(Time)]],[[c(Time1)]])

select_actions([],Time,Actions,Rest) :- !.

select_actions([List|Bag],Time,Actions,Rest) :- nth0(0,List,Action),
						(time(Action,Time) -> 
							select_actions(Bag,Time,Actions1,Rest),
							my_append(Actions1,[[Action]],Actions);
		      					select_actions(Bag,Time,Actions,Rest1),
							my_append(Rest1,[[Action]],Rest)).

select_actions2([],Time,Actions,Rest) :- !.
select_actions2([List|Bag],Time,Actions,Rest) :- (List = [], select_actions2(Bag,Time,Actions,Rest)); 
					         (length(List,N),
						  nth1(N,List,Action,OtherActions),
						  (time(Action,Time) -> select_actions2(Bag,Time,Actions1,Rest1),
								        my_append(Actions1,[[Action]],Actions),
								        my_append(Rest1,[OtherActions],Rest);
								        select_actions2(Bag,Time,Actions,Rest1),
								        my_append(Rest1,[List],Rest))).

test_sel_acts([],Time) :- !.
test_sel_acts(Bag,Time) :- flatten(Bag,NewBag),NewBag = [] ;(Time1 is Time + 1, select_actions2(Bag,Time1,Actions,Rest),
			   writeln(Actions),writeln(Rest),test_sel_acts(Rest,Time1)).

test_all_exec(Bag,S0) :- select_actions2(Bag,1,Actions1,Rest1),
			 all_executable(Actions1,S0),
			 select_actions2(Rest1,2,Actions2,Rest2),
			 all_executable(Actions2,Actions1),
			 select_actions2(Rest2,3,Actions3,Rest3),
			 all_executable(Actions3,Actions2),
			 select_actions2(Rest3,4,Actions4,Rest4),
			 all_executable(Actions4,Actions3),
			 select_actions2(Rest4,Actions5,Rest5),
			 all_executable(Actions5,Actions4). 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% all_executable([],S) :- !.
all_executable(Actions,S) :- (foreach(Action,Actions),param(S) 
			      do 
			      (S == [] -> executable(Action)
			       ;
			       member(SA,S),
			       eq_ni(I,Action,SA),
			       my_append(Action,SA,NewS),
			       executable(NewS))).
				

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%
% This predicate select the first set of actions from a Bag of situations for the execution monitoring
%

choose_first_action(ListOfActions,Action,Rest) :- length(ListOfActions,N),
						  nth1(N,ListOfActions,Action,Rest).
choose_first_actions([],Actions,Rests) :- !.
choose_first_actions([ListOfActions|Bag],Actions,Rests) :- choose_first_action(ListOfActions,Action,Rest),
							   choose_first_actions(Bag,Actions1,Rests1),
							   my_append(Actions1,[[Action]],Actions),
							   my_append(Rests1,[Rest],Rests).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

consistenceRecover(RealStatus,S0) :- retract_S0, 
				     assert_newS0(RealStatus), 
				     execution_monitoring(S0).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

cycleExecution(Tc,Bag,(Start,End),S0,T,ExeFlag) :- 
						% read the real internal status of the robot 
						stubRead(RealStatus),!,
						
						% read the fluents true in S0
						read_status('s0.ecl',Status),

						% compare the real internal state with fluents true in S0
						checkConsistency(Status,RealStatus,StatusFlag),
						
						(StatusFlag = 'yes',solve(Tc,Bag,Solution,(Start,End)),
								    execution(Bag,S0,T,ExeFlag),!,

							ExeFlag = 'success' -> writeln("Plan executed");

							ExeFlag = 'fail',writeln("Plan failured")); % miss executionRecover

					 	(StatusFlag = 'no',

						 writeln("discrepancies detected"),consistenceRecover(RealStatus,S0)).
%execution(Tc,Bag,(Start,End),S0,T,ExeFlag)
execution([],S0,T,ExeFlag) :- ExeFlag = 'success',!.
execution(Bag,S0,T,ExeFlag) :-
				% solve temporal constraints among activities 
				% solve(Tc,Bag,Solution,(Start,End)),

				% choose the first set of actions
				choose_first_actions(Bag,Start_Actions,Rest_Of_Plan),

				% check if the above set of actions is executable
				all_executable(Start_Actions,S0),

				% select the actions in the set of actions whose time is equal T
				select_actions(Start_Actions,T,SActions,WActions),

				% schedule the execution of the actions
				execution1(T,SActions,WActions,NewT,PartialExeFlag),

				(PartialExeFlag = 'ok' -> writeln("executed"),execution(Rest_Of_Plan,Start_Actions,NewT,ExeFlag);
				 PartialExeFlag = 'ko',writeln("aborted"),ExeFlag = 'fail').


execution1(T,[],[],NewT,PartialExeFlag) :- PartialExeFlag = 'ok',!. 
execution1(T,SActions,WActions,NewT,PartialExeFlag) :- (SAction = [],
						   not(WActions = []), 
						   T1 is T + 1,
						   select_actions(WActions,T1,NSActions,NWActions),
						   execution1(T1,NSActions,NWActions,T1,PartialExeFlag));
						  (stubExecution(SActions,Result),!,
						   (Result = 'succeeded',
						    T1 is T + 1,
						    select_actions(WActions,T1,NSActions,NWActions),
						    execution1(T1,NSActions,NWActions,T1,PartialExeFlag));
						   (Result = 'failed',
						    PartialExeFlag = 'ko')).
							
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% es. execution_monitoring([])

execution_monitoring(S0) :- task(Task,Bag,Tc,(Start,End),S0),!,
			    writeln("task loaded"),
			    time(S0,T), T1 is T + 1,
			    cycleExecution(Tc,Bag,(Start,End),S0,T1,ExeFlag),
		    	    (ExeFlag = 'success' -> writeln("end task"),sleep(1),
						 execution_monitoring(Bag);
			     ExeFlag = 'fail',
						 writeln("task stopped"),
						 execution_monitoring(Bag)
			     ).
			    				

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


