/* no operation task default */
task(no_op,Plan) :- Plan = [].

/* dummy task */
task(dummy,Plan) :- holds(navigation,in(Source),S), Plan = [start_goto_node(Source,Theta,T)].

/* generate a list of actions in order to reach a Sink Node from a Source Node */
task(from_to(Source,Sink),Plan) :- path(Source,Sink,L-Path),
								   make_plan(Path,Plan1),
								   lists:reverse(Plan1,Plan).

/* generate the list of actions in order to visit all nodes of the graph */
task(visit_graph,Plan) :- nodes(NodeArrayList),array_length(NodeArrayList,N), N1 is N + 1,
					      visit_graph(NodeArrayList,1,N1,Plan).

visit_graph(NodeArrayList,N,N,Plan) :- !.
visit_graph(NodeArrayList,N1,N,Plan) :- arg(N1,NodeArrayList,Node),
										N2 is N1 + 1,
										visit_graph(NodeArrayList,N2,N,Plan1),
										lists:append(Plan1,[start_goto_node(Node,Theta,T)],Plan).
										
/* generate the list of actions in order to visit a random node not still visited in the graph */
task(visit_a_random_node,Plan) :- select(Node), Plan = [start_goto_node(Node,Theta,T)].

/* generate the list of actions in order to visit a sink node in the graph */
task(visit(Sink),Plan) :- holds(navigation,in(Source),S),
						  Source = node(L,X,Y,F),
						  Source1 = node(L,X1,Y1,F1),
						  task(from_to(Source1,Sink),Plan).
						  
/* move the base link in a position (x,y,theta) from dialogue */
task(move_base(X,Y,Theta),Plan) :- Node = node(n,X,Y,0), Plan = [start_goto_node(Node,Theta,T)].


/* move the base link in a position (x,y) from GUI */
task(goto(X,Y),Plan) :- Node = node(n,X,Y,0), Plan = [start_read_topo(T3),start_goto_node(Node,Theta,T2),end_rotation(T3),start_rotation(1,5)].

/* move the robot within the topological map up to reach a point in unknown space */
task(reach(X,Y,Sink),Plan) :- (open(task,write,S),Sink = node(LS,XS,YS,FS),task(visit(node(LS,XSS,YSS,FSS)),Plan1),write(S,Plan1),Node = node(n,X,Y,0),
                              Plan = [start_goto_node(Node,Theta,T2)|Plan1],write(S,Plan),close(S)); (Node = node(n,X,Y,0),Plan = [start_goto_node(Node,Theta,T2)]).
                              
/* generate the list of actions in order to visit a node in the func_graph */
task(visit_prop(Prop,N),Plan) :- (
								  belong(NodeId,Prop:N),
								  graph(node(NodeId,X,Y,F)),
								  Sink = node(NodeId,X,Y,F),
								  holds(navigation,in(Source),S), 
								  Source = node(L,X2,Y2,F2), 
								  Source1 = node(L,X3,Y3,F3),
								  task(from_to(Source1,Sink),Plan)); 
								 (holds(navigation,in(Source),S), 
								  Plan = [start_goto_node(Source,Theta,T)]).
								  
/* generate the list of actions in order to visit a car */
task(approach(From,vpnode(L,X,Y,Z,Theta,Gain,Flag)),Plan) :- holds(navigation,in(Source),S),
                                                             Sink = node(From,XF,YF,FF),
                                                             task(from_to(Source,Sink),Plan1),
                                                             Node = node(L,X,Y,Flag),
                                                             Plan = [start_goto_node(Node,Theta,T)|Plan1].
                                                             
/* come back poor roobot */
task(come_back,Plan) :- base(navigation,node(Home,X1,Y1,F1),S),
                        holds(navigation,in(node(Current,X2,Y2,F2)),S),
                        task(from_to(node(Current,X2,Y2,F2),node(Home,X1,Y1,F1),Plan)).
			       
/* just for testing */
task(build_map,Plan) :- Plan = [end_build_topo(T2),start_build_topo(T1)].

task(read_map,Plan) :- Plan = [start_read_topo(T1)].

task(func_map,Plan) :- Plan = [end_func_map(T2),start_func_map(T1)].

task(detect_gap,Plan) :- Plan = [start_gap_detection(T2),end_rotation(T3),start_rotation(1,10)].

task(init,Plan) :- Plan = [start_func_map(T3),start_read_topo(T2),start_build_topo(T1)].

task(test_flipper,Plan) :- Plan = [move(flipper4,0,T6),move(flipper3,0,T5),move(flipper2,0,T4),move(flipper1,0,T3)]. 

task(test_rotation,Plan) :- Plan = [end_rotation(T3),start_rotation(1,10)].

task(test_mixed_init,Plan) :- Plan = [move_forward(T3),move_forward(T2),move_forward(T1)]. 

task(move_forward,Plan) :- Plan = [move_forward(T)].
task(move_left,Plan) :- Plan = [move_left(T)].
task(move_right,Plan) :- Plan = [move_right(T)].
task(move_back,Plan) :- Plan = [move_back(T)].
task(turn_left,Plan) :- Plan = [turn_left(T)].
task(turn_right,Plan) :- Plan = [turn_right(T)].

task(traverse_gap,Plan) :- detected_gap(Gap,Xg,Yg,Zg,Thg),
                           (traversable(Gap),
                            starting_pose(Gap,X1,Y1,Z1,Th1),
                            ending_pose(Gap,X2,Y2,Z2,Th2),
                            configuration(Gap,A1,A2,A3,A4),
                            Plan = [move(flipper4,2,T12),move(flipper3,2,T11),move(flipper2,-2.4,T10),move(flipper1,-2.4,T9),unlock(T8),move_forward(T15),move_forward(T14),move_forward(T13)	,move(flipper4,A4,T6),move(flipper3,A3,T5),move(flipper2,-0.4,T4),move(flipper1,-0.4,T3),lock(T2)]);
                           (not_traversable(Gap), Plan = []).
