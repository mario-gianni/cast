% NIFTI Project
% from technical report about flexible planning
% author Gianni Mario
% date: 08 - 12 - 2010

% schema
% poss(A,S) :- eq_ni(I,A,S),(holds(I,P,T,S);idle(I,S)),time(S,Ts),time(A,Ta), Ts #=< Ta.

% checks if an action can be execute
executable(Situation) :- is_list(Situation),nth0(0,Situation,Action,Rest),poss(Action,Rest).

% checks if all actions can be execute
exec([]).
exec([Action|Situation]) :- poss(Action,Situation),!,exec(Situation).

% action precondition axioms

% Navigation Component
poss(start_goto_pose(X,Y,Theta,Time),S) :- eq_ni(navigation,start_goto_pose(X,Y,Theta,Time),S),
					   (holds(navigation,at(XS,YS),S);idle(navigation,S)),
					   time(S,Ts),Ts #=< Time.

poss(end_goto_pose(X,Y,Theta,Time),S) :- eq_ni(navigation,end_goto_pose(X,Y,Theta,Time),S),
					 process(navigation,goto_pose(X,Y,Theta),S),
					 time(S,Ts),Ts #=< Time.

poss(start_goto_node(NodeId,Time),S) :- eq_ni(navigation,start_goto_node(NodeId,Time),S),
					(holds(navigation,in(NodeIdS),S);idle(navigation,S)),
					time(S,Ts),Ts #=< Time.

poss(end_goto_node(NodeId,Time),S) :- eq_ni(navigation,end_goto_node(NodeId,Time),S),
				      process(navigation,goto_node(NodeId),S),time(S,Ts),Ts #=< Time.

poss(start_wander(Time),S) :- eq_ni(navigation,start_wander(Time),S),
		             (holds(navigation,in(NodeIdS),S); 
			      holds(navigation,at(XS,YS),S); 
			      idle(navigation,S)),
			      time(S,Ts),Ts #=< Time.

poss(end_wander(Time),S) :- eq_ni(navigation,end_wander(Time),S),
			    process(navigation,wander,S),
			    time(S,Ts),Ts #=< Time.

% Slam Component
poss(start_update_map(Time),S) :- eq_ni(slam,start_update_map(Time),S),
				 (holds(slam,wait,S); idle(slam,S)),time(S,Ts),Ts #=< Time.

poss(end_update_map(Time),S) :- eq_ni(slam,end_update_map(Time),S),
			        process(slam,update_map,S),time(S,Ts),Ts #=< Time.

poss(start_update_position(Time),S) :- eq_ni(slam,start_update_position(Time),S),
				(holds(slam,wait,S);idle(slam,S)),time(S,Ts),Ts #=< Time.

poss(end_update_position(Time),S) :- eq_ni(slam,end_update_position(Time),S),
				     process(slam,localize,S),time(S,Ts),Ts #=< Time.

% Vision Component
poss(start_search(Object,Time),S) :- eq_ni(vision,start_search(Object,Time),S),
				     idle(vision,S),
				     time(S,Ts),Ts #=< Time.

poss(end_search(Object,Time),S) :- eq_ni(vision,end_search(Object,Time),S),
				   process(vision,search(Object),S),
				   time(S,Ts),Ts #=< Time.

% Functional Mapping Component
poss(start_identify_interest_area(Time),S) :- eq_ni(func_mapping,start_identify_interest_area(Time),S),
					      idle(func_mapping,S),
					      time(S,Ts),Ts #=< Time.

poss(end_identify_interest_area(Time),S) :- eq_ni(func_mapping,end_identify_interest_area(Time),S),
					    holds(func_mapping,identify_interest_area,S),
					    time(S,Ts),Ts #=< Time.

