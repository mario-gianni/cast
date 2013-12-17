% NIFTI Project
% from technical report about flexible planning
% author Gianni Mario
% date: 08 - 12 - 2010

% For each component it defines a set of processes/properties
% each process is defined as a SSA

:- dynamic process/3,  holds/3, idle/2.

% Navigation Component
process(navigation,goto_pose(X,Y,Theta),[A|S]) :- A = start_goto_pose(X,Y,Theta,Ti);
					    	  (process(navigation,goto_pose(X,Y,Theta),S),
   	  	 	 	 	     	   not(A = end_goto_pose(X,Y,Theta,Te))).

process(navigation,goto_node(NodeId),[A|S]) :- A = start_goto_node(NodeId,Ti);
					       (process(navigation,goto_node(NodeId),S),
					        not(A = end_goto_node(NodeId,Te))).

process(navigation,wander,[A|S]) :- A = start_wander(Ti);
				    (process(navigation,wander,S),
				     not(A = end_wander(Te))).

holds(navigation,at(X,Y),[A|S]) :- A = end_goto_pose(X,Y,Theta,Te);
			           (holds(navigation,at(X,Y),S),
    	 	 	 	    not(A = start_goto_pose(X,Y,Theta,Ti))).

holds(navigation,in(NodeId),[A|S]) :- A = end_goto_node(NodeId,Te);
				      (holds(navigation,in(NodeId),S),
				       not(A = start_goto_node(NodeId,Ti))).

idle(navigation,[A|S]) :- A = end_goto_pose(X,Y,Theta,Te); A = end_goto_node(NodeId,Te);
			  A = end_wander(Te); (idle(navigation,S),not(A = start_goto_pose(X,Y,Theta,Ti)),
			  not(A = start_goto_node(NodeId,Ti)),not(A = start_wander(Ti))).

% Slam Component
process(slam,update_map,[A|S]) :- A = start_update_map(Ti) ; 
			      (process(slam,update_map,S),
			       not(A = end_update_map(Te))).

process(slam,localize,[A|S]) :- A = start_update_position(Ti) ; 
				   (process(slam,localize,S),
				    not(A = end_update_position(Te))).

holds(slam,wait,[A|S]) :- A = end_update_map(Ti) ; (holds(slam,wait,S),not(A = start_update_map(Te))).



idle(slam,[A|S]) :- A = end_update_map(Te) ; A = end_update_position(Te) ; 
		    (idle(slam,S),not(A = start_update_map(Ti)),not(A = start_update_position(Ti))).

% Vision Component
process(vision,search(Object),[A|S]) :- A = start_search(Object,Ti);
	   	 	 	 	(process(vision,holds(Object),S),
					 not(A = end_search(Object,Te))).

idle(vision,[A|S]) :- A = end_search(Object,Te);(idle(vision,S),not(A = start_search(Object,Ti))).

% Functional Mapping Component
holds(func_mapping,identify_interest_area,[A|S]) :- 
					A = start_identify_interest_area(Ti);
					(holds(func_mapping,identify_interest_area,S),
					 not(A = end_identify_interest_area(Te))).

idle(func_mapping,[A|S]) :- A = end_identify_interest_area(Te);
			    (idle(func_mapping,S),
			     not(A = start_identify_interest_area(Ti)))

