% NIFTI Project
% from technical report about flexible planning
% author Gianni Mario
% date: 13 - 12 - 2010

% Time Constarints between starting and ending time of an action.

constraint(end_goto_pose(X,Y,Theta,T2),start_goto_pose(X,Y,Theta,T1)) :- T1 #< T2.
constraint(end_goto_node(NodeId,T2),start_goto_node(NodeId,T1)) :- T1 #< T2.
constraint(end_wander(T2),start_wander(T1)) :- T1 #< T2.
constraint(end_update_map(T2),start_update_map(T1)) :- T1 #< T2.
constraint(end_update_position(T2),start_update_position(T1)) :- T1 #< T2.
constraint(end_search(Object,T2),start_search(Object,T1)) :- T1 #< T2.
constraint(end_identify_interest_area(T2),start_identify_interest_area(T1)) :- T1 #< T2.


not_need(A,end_goto_pose(X,Y,Theta,T2)).
not_need(A,end_goto_node(NodeId,T2)).
not_need(A,end_wander(T2)).
not_need(A,end_update_map(T2)).
not_need(A,end_update_position(T2)).
not_need(A,end_search(Object,T2)).
not_need(A,end_identify_interest_area(T2)).


%
% se_constrainht(Bag)
% es. se_constraint([[end_search(Object,2),start_search(Object,1)],
%		    [end_identify_interest_area(2),start_identify_interest_area(1)]]).
%
se_constraint([]).
se_constraint([Head|Tail]) :- se_constraint1(Head),se_constraint(Tail).

se_constraint1([]).
se_constraint1([A]).
se_constraint1([A1,A2|Rest]) :- not_need(A1,A2),se_constraint1([A2|Rest]);
				constraint(A1,A2),se_constraint1(Rest).
