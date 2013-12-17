% NIFTI Project
% from technical report about flexible planning
% author Gianni Mario
% date: 08 - 12 - 2010

% Started Ended Active Elapsed predicates used for construct temporal constraints

% I -> component
% P -> process/property
% T -> time
% A -> action
% S -> situation
% Ts0 -> time of situation s_0
% Ta -> time of action

%STARTED schema
% started(I,P,T,A,S) :- executable([A|S]),process(I,P,[A|S]), not(process(I,P,S)),time(A,Ta), Ta #= T.

% Navigation Component
started(navigation,goto_pose(X,Y,Theta),T,A,S) :- executable([A|S]),process(navigation,goto_pose(X,Y,Theta),[A|S]),
				       not(process(navigation,goto_pose(X1,Y1,Theta1),S)),time(A,Ta), Ta #= T.

started(navigation,goto_node(NodeId),T,A,S) :- executable([A|S]),process(navigation,goto_node(NodeId),[A|S]),
				       not(process(navigation,goto_node(NodeId1),S)),time(A,Ta), Ta #= T.

started(navigation,wander,T,A,S) :- executable([A|S]),process(navigation,wander,[A|S]),
				    not(process(navigation,wander,S)),time(A,Ta), Ta #= T.

started(navigation,at(X,Y),T,A,S) :- executable([A|S]),holds(navigation,at(X,Y),[A|S]),
				   not(holds(navigation,at(X1,Y1),S)),time(A,Ta), Ta #= T.

started(navigation,in(NodeId),T,A,S) :- executable([A|S]),holds(navigation,in(NodeId),[A|S]),
				   not(holds(navigation,in(NodeId1),S)),time(A,Ta), Ta #= T.

% Slam Component
started(slam,update_map,T,A,S) :- executable([A|S]),process(slam,update_map,[A|S]),
				  not(process(slam,update_map,S)),time(A,Ta), Ta #= T.

started(slam,localize,T,A,S) :- executable([A|S]),process(slam,localize,[A|S]),
				not(process(slam,localize,S)),time(A,Ta), Ta #= T.

started(slam,wait,T,A,S) :- executable([A|S]),holds(slam,wait,[A|S]),not(holds(slam,wait,S)),time(A,Ta), Ta #= T.

% Vision Component
started(vision,search(Object),T,A,S) :- executable([A|S]),process(vision,search(Object),[A|S]),
			 		not(process(vision,search(Object1),S)),time(A,Ta), Ta #= T.

% Functional Mapping Component
started(func_mapping,identify_interest_area,T,A,S) :- executable([A|S]),holds(func_mapping,identify_interest_area,[A|S]),
     				not(holds(func_mapping,identify_interest_area,S)),time(A,Ta), Ta #= T.

%ENDED schema
% ended(I,P,T,A,S) :- executable([A|S]),process(I,P,S), not(process(I,P,[A|S])),time(A,Ta), Ta #= T.

% Navigation Component
ended(navigation,goto_pose(X,Y,Theta),T,A,S) :- executable([A|S]),process(navigation,goto_pose(X,Y,Theta),S),
				       not(process(navigation,goto_pose(X,Y,Theta),[A|S])),time(A,Ta), Ta #= T.

ended(navigation,goto_node(NodeId),T,A,S) :- executable([A|S]),process(navigation,goto_node(NodeId),S),
				       not(process(navigation,goto_node(NodeId),[A|S])),time(A,Ta), Ta #= T.

ended(navigation,wander,T,A,S) :- executable([A|S]),process(navigation,wander,S),
				       not(process(navigation,wander,[A|S])),time(A,Ta), Ta #= T.

ended(navigation,at(X,Y),T,A,S) :- executable([A|S]),holds(navigation,at(X,Y),S),
				 not(holds(navigation,at(X,Y),[A|S])),time(A,Ta), Ta #= T.

ended(navigation,in(NodeId),T,A,S) :- executable([A|S]),holds(navigation,in(NodeId),S),
				 not(holds(navigation,in(NodeId),[A|S])),time(A,Ta), Ta #= T.

% Slam Component
ended(slam,update_map,T,A,S) :- executable([A|S]),process(slam,update_map,S),
				not(process(slam,update_map,[A|S])),time(A,Ta), Ta #= T.

ended(slam,localize,T,A,S) :- executable([A|S]),process(slam,localize,S),
			      not(process(slam,localize,[A|S])),time(A,Ta), Ta #= T.
ended(slam,wait,T,A,S) :- executable([A|S]),holds(slam,wait,S),not(holds(slam,wait,[A|S])),time(A,Ta), Ta #= T.

% Vision Component
ended(vision,search(Object),T,A,S) :- executable([A|S]),process(vision,search(Object),S),
			 		not(process(vision,search(Object),[A|S])),time(A,Ta), Ta #= T.

% Functional Mapping Component
ended(func_mapping,identify_interest_area,T,A,S) :- executable([A|S]),holds(func_mapping,identify_interest_area,S),
     				not(holds(func_mapping,identify_interest_area,[A|S])),time(A,Ta), Ta #= T.

% ACTIVE
% schema active(I,P,T,[]) :- process(I,P,[]),time([],Ts0),Ts0 #= T.

% Navigation Component
active(navigation,goto_pose(X,Y,Theta),T,[]) :- process(navigation,goto_pose(X,Y,Theta),[]),time([],Ts0),Ts0 #= T.

active(navigation,goto_node(NodeId),T,[]) :- process(navigation,goto_node(NodeId),[]),time([],Ts0),Ts0 #= T.

active(navigation,wander,T,[]) :- process(navigation,wander,[]),time([],Ts0),Ts0 #= T.

active(navigation,at(X,Y),T,[]) :- holds(navigation,at(X,Y),[]),time([],Ts0),Ts0 #= T.

active(navigation,in(NodeId),T,[]) :- holds(navigation,in(NodeId),[]),time([],Ts0),Ts0 #= T.

% Slam Component
active(slam,update_map,T,[]) :- process(slam,update_map,[]),time([],Ts0),Ts0 #= T.

active(slam,localize,T,[]) :- process(slam,localize,[]),time([],Ts0),Ts0 #= T.

active(slam,wait,T,[]) :- holds(slam,wait,[]),time([],Ts0),Ts0 #= T.

% Vision Component
active(vision,search(Object),T,[]) :- process(vision,search(Object),[]),time([],Ts0),Ts0 #= T.

% Functional Mapping Component
active(func_mapping,identify_interest_area,T,[]) :- holds(func_mapping,identify_interest_area,[]),
						    time([],Ts0),Ts0 #= T.

% schema active(I,P,T,[A|S]) :- (timeline(I,[A|S]),started(I,P,T,A,S));(active(I,P,T,S),not(ended(I,P,T1,A,S))).

% Navigation Component
active(navigation,goto_pose(X,Y,Theta),T,[A|S]) :- (timeline(navigation,[A|S]),  
						   started(navigation,goto_pose(X,Y,Theta),T,A,S));
	       					   (active(navigation,goto_pose(X,Y,Theta),T,S),
						    not(ended(navigation,goto_pose(X,Y,Theta),T1,A,S))).

active(navigation,goto_node(NodeId),T,[A|S]) :- (timeline(navigation,[A|S]),
						started(navigation,goto_node(NodeId),T,A,S));
						(active(navigation,goto_node(NodeId),T,S),
						 not(ended(navigation,goto_node(NodeId),T1,A,S))).

active(navigation,wander,T,[A|S]) :- (timeline(navigation,[A|S]),started(navigation,wander,T,A,S));
					(active(navigation,wander,T,S),not(ended(navigation,wander,T1,A,S))).

active(navigation,at(X,Y),T,[A|S]) :- (timeline(navigation,[A|S]),started(navigation,at(X,Y),T,A,S));
					(active(navigation,at(X,Y),T,S),not(ended(navigation,at(X,Y),T1,A,S))).

active(navigation,in(NodeId),T,[A|S]) :- (timeline(navigation,[A|S]),started(navigation,in(NodeId),T,A,S));
					 (active(navigation,in(NodeId),T,S),not(ended(navigation,in(NodeId),T1,A,S))).

% Slam Component
active(slam,update_map,T,[A|S]) :- (timeline(slam,[A|S]),started(slam,update_map,T,A,S));
				   (active(slam,update_map,T,S),not(ended(slam,update_map,T1,A,S))).

active(slam,localize,T,[A|S]) :- (timeline(slam,[A|S]),started(slam,localize,T,A,S));
				 (active(slam,localize,T,S),not(ended(slam,localize,T1,A,S))).

active(slam,wait,T,[A|S]) :- (timeline(slam,[A|S]),started(slam,wait,T,A,S));
			     (active(slam,wait,T,S),not(ended(slam,wait,T1,A,S))).

% Vision Component
active(vision,search(Object),T,[A|S]) :- (timeline(vision,[A|S]),started(vision,search(Object),T,A,S));
					 (active(vision,search(Object),T,S),not(ended(vision,search(Object),T1,A,S))).

% Functional Mapping Component
active(func_mapping,identify_interest_area,T,[A|S]) :- (timeline(func_mapping,[A|S]),
							started(func_mapping,identify_interest_area,T,A,S));
				 		       (active(func_mapping,identify_interest_area,T,S),
							not(ended(func_mapping,identify_interest_area,T1,A,S))).

% ELAPSED

% Navigation Component
elapsed(navigation,goto_pose(X,Y,Theta),Ti,Te,[]) :- false.

elapsed(navigation,goto_node(NodeId),Ti,Te,[]) :- false.

elapsed(navigation,wander,Ti,Te,[]) :- false.

elapsed(navigation,at(X,Y),Ti,Te,[]) :- false.

elapsed(navigation,in(NodeId),Ti,Te,[]) :- false.

% Slam Component
elapsed(slam,update_map,Ti,Te,[]) :- false.

elapsed(slam,localize,Ti,Te,[]) :- false.

elapsed(slam,wait,Ti,Te,[]) :- false.

% Vision Component
elapsed(vision,search(Object),Ti,Te,[]) :- false.

% Functional Mapping Component
elapsed(func_mapping,identify_interest_area,Ti,Te,[]) :- false.

% Navigation Component
elapsed(navigation,goto_pose(X,Y,Theta),Ti,Te,[A|S]) :- (ended(navigation,goto_pose(X,Y,Theta),Te,A,S),
							 active(navigation,goto_pose(X,Y,Theta),Ti,S));
					     		(timeline(navigation,[A|S]),
							 elapsed(navigation,goto_pose(X,Y,Theta),Ti,Te,S)).

elapsed(navigation,goto_node(NodeId),Ti,Te,[A|S]) :- (ended(navigation,goto_node(NodeId),Te,A,S),
							 active(navigation,goto_node(NodeId),Ti,S));
					     		(timeline(navigation,[A|S]),
							 elapsed(navigation,goto_node(NodeId),Ti,Te,S)).

elapsed(navigation,wander,Ti,Te,[A|S]) :- (ended(navigation,wander,Te,A,S),
					   active(navigation,wander,Ti,S));
					  (timeline(navigation,[A|S]),
					   elapsed(navigation,wander,Ti,Te,S)).

elapsed(navigation,at(X,Y),Ti,Te,[A|S]) :- (ended(navigation,at(X,Y),Te,A,S),
					    active(navigation,at(X,Y),Ti,S));
					   (timeline(navigation,[A|S]),
					    elapsed(navigation,at(X,Y),Ti,Te,S)).

elapsed(navigation,in(NodeId),Ti,Te,[A|S]) :- (ended(navigation,in(NodeId),Te,A,S),
					    active(navigation,in(NodeId),Ti,S));
					   (timeline(navigation,[A|S]),
					    elapsed(navigation,in(NodeId),Ti,Te,S)).

% Slam Component
elapsed(slam,update_map,Ti,Te,[A|S]) :- (ended(slam,update_map,Te,A,S),active(slam,update_map,Ti,S));
				 (timeline(slam,[A|S]),elapsed(slam,update_map,Ti,Te,S)).

elapsed(slam,localize,Ti,Te,[A|S]) :- (ended(slam,localize,Te,A,S),active(slam,localize,Ti,S));
				      (timeline(slam,[A|S]),elapsed(slam,localize,Ti,Te,S)).

elapsed(slam,wait,Ti,Te,[A|S]) :- (ended(slam,wait,Te,A,S),active(slam,wait,Ti,S));
				      (timeline(slam,[A|S]),elapsed(slam,wait,Ti,Te,S)).


% Vision Component
elapsed(vision,search(Object),Ti,Te,[A|S]) :- (ended(vision,search(Object),Te,A,S),
					       active(vision,search(Object),Ti,S));
					      (timeline(vision,[A|S]),elapsed(vision,search(Object),Ti,Te,S)).

% Functional Mapping Component
elapsed(func_mapping,identify_interest_area,Ti,Te,[A|S]) :- (ended(func_mapping,identify_interest_area,Te,A,S),
							     active(func_mapping,identify_interest_area,Ti,S));
							    (timeline(func_mapping,[A|S]),
							     elapsed(func_mapping,identify_interest_area,Ti,Te,S)).
