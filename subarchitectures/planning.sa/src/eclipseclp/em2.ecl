% NIFTI Project
% from technical report about flexible planning
% author Gianni Mario
% date: 05 - 01 - 2011

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% EXECUTION MONITORING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

execution_monitoring(S0) :- task(Task,Bag,Tc,(Start,End),S0),!,
			    writeln("task loaded"),
			    time(S0,T), T1 is T + 1,
			    cycleExecution(Tc,Bag,(Start,End),S0,T1,ExeFlag),
		    	    (ExeFlag = 'success' -> writeln("the entire plan was be executed"),
						    sleep(5),
						    execution_monitoring(Bag);
			     ExeFlag = 'fail',writeln("The plan was be aborted"),
			                      execution_monitoring(Bag)
			     ).
			 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

cycleExecution(Tc,Bag,(Start,End),S0,T,ExeFlag) :- % read the real internal status of the robot 
						   stubRead(RealStatus),!,
						   % read the fluents true in S0
						   read_status('s0.ecl',Status),
						   % compare the real internal state with fluents true in S0
						   checkConsistency(Status,RealStatus,StatusFlag),
						   (StatusFlag = 'yes',
						    solve(Tc,Bag,Solution,(Start,End)),
						    execution(Bag,S0,T,ExeFlag),!,
						    ExeFlag = 'success' -> writeln("Plan executed");         					                    ExeFlag = 'fail',writeln("Plan failured")); % miss executionRecover
					           (StatusFlag = 'no',
						    writeln("discrepancies detected"),
						    consistenceRecover(RealStatus,S0)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

execution([],S,T,ExeFlag) :- ExeFlag = 'success',!.
execution(Bag,S0,T,ExeFlag) :- % choose the set of actions whose time is equal to T
			       select_actions(Bag,T,FirstActions,Rest),!,
			       % check if the above set of actions is executable
			       % if FirstActions is empty 
			       (FirstActions = [] ->
					% increase the tick
					T1 is T + 1,
					% call recursively the predicate
					execution(Rest,S0,T1,ExeFlag);
				 (all_executable(FirstActions,S0),
				  stubExecution(FirstActions,Result),!,
				  (Result = 'inprogress' -> T1 is T + 1,
							    execution(Rest,FirstActions,T1,ExeFlag);
				   Result = 'failed',ExeFlag = 'fail' 
				  )
				 )
				).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

consistenceRecover(RealStatus,S0) :- retract_S0, 
				     assert_newS0(RealStatus), 
				     execution_monitoring(S0).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

select_actions([],Time,Actions,Rest) :- !.
select_actions([List|Bag],Time,Actions,Rest) :- (List = [], select_actions(Bag,Time,Actions,Rest)); 
					        (length(List,N),nth1(N,List,Action,OtherActions),
						(time(Action,Time) -> select_actions(Bag,Time,Actions1,Rest1),
								      my_append(Actions1,[[Action]],Actions),
								      my_append(Rest1,[OtherActions],Rest);
								      select_actions(Bag,Time,Actions,Rest1),
								      my_append(Rest1,[List],Rest))).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

all_executable(Actions,S) :- (foreach(Action,Actions),param(S) 
			      do 
			      (S == [] -> executable(Action)
			       ;
			       member(SA,S),
			       eq_ni(I,Action,SA),
			       my_append(Action,SA,NewS),
			       executable(NewS))).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
