% NIFTI Project
% from technical report about flexible planning
% author Gianni Mario
% date: 10 - 12 - 2010

%
%	UTILITY FUNCTIONS
%

% checks if A is an action
is_action(A) :- compound(A),not(is_list(A)).

% Add an element in the head of the list
add(X,[],[X]).
add(X,L,[X|L]).

% checks if P is a property/process belonging to a component
p(navigation,P) :- P = goto_pose(X,Y,Theta) ; P = goto_node(NodeId); P = wander; P = at(X,Y); P = in(NodeId).
p(slam,P) :- P = update_map; P = localize; P = wait.
p(vision,P) :- P = search(Object).
p(func_mapping,P) :- P = identify_interest_area.

belong(Component,Process,[]) :- var(Component),p(Component,Process),!.
belong(Component,Process,Timeline) :- var(Component),p(Component,Process), timeline(Component,Timeline).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

assert_newS0(RealStatus) :- (foreach(Fluent,RealStatus) do assert(Fluent)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

checkConsistency(Status1,Status2,Flag) :- (consistency(Status1,Status2) -> Flag = 'yes'; Flag = 'no').

consistency(List1,List2) :- (foreach(Elem,List1),param(List2) do member(Elem,List2)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% implementation of the PROGRESSION operator

%
% es. progression([start_update_map(Time),start_goto_pose(X,Y,Theta,Time)],CS)
%
progression(S,CurrentState) :- check_state(S,TrueList),store(S,TrueList,CurrentState).

store(S,[],CurrentState) :- !.
store(S,[Fluent|OtherFluents],CurrentState) :- Term = [check|Fluent],
					       Goal =.. Term,
					       restoreArg(Goal,S,Clause),
					       clause(Clause,Body),
					       store(S,OtherFluents,CS1),
					       lists:append(CS1,[Body],CurrentState). 

check_state(S,TrueList) :- is_list(S),setof([Component,Predicate,Params],
				            call(check(Component,Predicate,Params,S)),
				            TrueList).

restoreArg(check(Componet,Predicate,Params),S,check(Componet,Predicate,Params,S)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Read the robot status stored in s0 file
% es read_status('/home/mario/Scrivania/PLANNER/s0.ecl',S)

read_status(File,State) :- open(File,read,Stream),
			    (at_eof(Stream) -> State = [], close(Stream) 
		 	     ; 
			     takeFluents(Stream,State),close(Stream)).

takeFluents(Stream, []):- at_eof(Stream), !. 
takeFluents(Stream,[Term|L]) :- read(Stream,Term),
				 (Term \== end_of_file -> takeFluents(Stream,L)). 
takeFluents(Stream,List):- takeFluents(Stream,List), !. 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

retract_S0 :- read_status('s0.ecl',State),
	      (foreach(Fluent,State) do retract(Fluent)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

stubRead(Status) :- Status = [idle(navigation,S0),idle(slam,S0),idle(vision,S0),idle(func_mapping,S0)].

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


