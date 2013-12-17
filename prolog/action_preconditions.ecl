poss(start_goto_node(Node,Theta,T),S) :- holds(navigation,in(CurrentNode),S),path(CurrentNode,Node,L-Path), L > 0.
poss(end_goto_node(Node,Theta,T),S) :- holds(navigation,in(Node),S).

poss(start_read_topo(T),S) :- holds(toposeg,build_topo,S),not(holds(toposeg,read_topo,S)).
poss(end_read_topo(T),S) :- holds(toposeg,read_topo,S).

poss(start_build_topo(T),S) :- not(holds(toposeg,build_topo,S)).
poss(end_build_topo(T),S) :- holds(toposeg,build_topo,S).

poss(start_func_map(T),S) :- not(holds(func_map,func_map,S)).
poss(end_func_map(T),S) :- holds(func_map,func_map,S).

poss(start_rotation(V,T),S) :- not(holds(lidar,rotate,S)).
poss(end_rotation(T),S) :- holds(lidar,rotate,S).

poss(move(flipper1,Angle,T),S) :- nonground(Angle) ; (ground(Angle), Angle >= -2.5, Angle =< 0).
poss(move(flipper2,Angle,T),S) :- nonground(Angle) ; (ground(Angle), Angle >= -2.5, Angle =< 0).
poss(move(flipper3,Angle,T),S) :- nonground(Angle) ; (ground(Angle), Angle >= 0, Angle =< 2.5).
poss(move(flipper4,Angle,T),S) :- nonground(Angle) ; (ground(Angle), Angle >= 0, Angle =< 2.5).

poss(lock(T),S) :- not(holds(locomotion,lock,S)).
poss(unlock(T),S) :- holds(locomotion,lock,S).

poss(start_gap_detection(T),S) :- holds(lidar,rotate,S),not(holds(gap,gap_tetection,S)).
poss(end_gap_detection(T),S) :- holds(gap,gap_tetection,S).

poss(move_forward(T),S).
poss(move_left(T),S).
poss(move_right(T),S).
poss(move_back(T),S).
poss(turn_left(T),S).
poss(turn_right(T),S).

holds(navigation,in(Node),do(A,S)) :- A = start_goto_node(Node,Theta,T);
									  (holds(navigation,in(Node),S),
									  not(A = end_goto_node(Node1,Theta1,T1))).

holds(toposeg,build_topo,do(A,S)) :- A = start_build_topo(T);
									 (holds(toposeg,build_topo,S),
									 not(A = end_build_topo(T1))).
									 
holds(toposeg,read_topo,do(A,S)) :- A = start_read_topo(T);
									(holds(toposeg,read_topo,S),
									not(A = end_read_topo(T1))).

holds(func_map,func_map,do(A,S)) :- A = start_func_map(T);
									(holds(func_map,func_map,S),
									not(A = end_func_map(T1))).
									
holds(lidar,rotate,do(A,S)) :- A = start_rotation(V,T);
							   (holds(lidar,rotate,S),
							   not(A = end_rotation(T1))).
							   
holds(locomotion,lock,do(A,S)) :- A = lock(T);
								  (holds(locomotion,lock,S),
								  not(A = unlock(T1))).
