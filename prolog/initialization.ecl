/*
	NIFTI project
	Flexible Planning
	Mario Gianni
	01-03-2012
*/

holds(navigation,at(0,0,0,0),[]).
holds(navigation,in(node(n,0,0,0)),[]).

retract_curr_position :- retract(holds(navigation,at(X,Y,Z,Theta),S)).
retract_current_node :- retract(holds(navigation,in(Node),S)).
