/*
	NIFTI project
	Flexible Planning
	Mario Gianni
	08-03-2011
	modified 15-03-2011
*/

/* dynamic read all nodes */

nodes(NodeArrayList) :- findall(Node,graph(Node),NodeList),
						length(NodeList,N),
						dim(NodeArrayList,[N]),
						N1 is N + 1,
						from_list_to_array(NodeArrayList,NodeList,1,N1).
						
from_list_to_array(NodeArrayList,[],N,N) :- !.
from_list_to_array(NodeArrayList,[X|Y],N1,N) :- arg(N1,NodeArrayList,X),
												N2 is N1 + 1,
												from_list_to_array(NodeArrayList,Y,N2,N).
						
/* dynamic read all edges */

edges(EdgeList) :- setof(Edge,edge(Edge),EdgeList).

/* Undirected Graph maker */

make_topological_map(GraphStructure) :- nodes(NodeArrayList),
										edges(EdgeList),
										make_graph_symbolic(NodeArrayList,EdgeList,GraphStructure).
										
/* short path evaluation */

path(Source, Sink, Path) :- make_topological_map(Graph),
							nodename_to_node(Graph,Source,NodeSource),
							nodename_to_node(Graph,Sink,NodeSink),
							all_short_paths_as_edges(Graph, 0, NodeSource, 0, Lengths, Preds),
    						possible_path(0, NodeSource, NodeSink, 0, Lengths, Preds, Path).
    						
/* marking */

marking(Node,MNode) :- graph(Node),functor(MNode,node,4),arg(1,Node,Label),arg(1,MNode,Label),
					   arg(2,Node,X),arg(2,MNode,X),arg(3,Node,Y),arg(3,MNode,Y),arg(4,MNode,1).
		
/* check if a node is marked */

is_marked(Node) :- arg(4,Node,Flag), Flag = 1.

/* select a node not still marked */

select(Node) :- graph(Node),not(is_marked(Node)).
		
/* ArrayList Length */

array_length(NodeArrayList,N) :- (foreachelem(_,NodeArrayList),count(_,1,N) do true).
			   


/* get_nodes_from_edge() */

get_nodes_from_edge(e(Node1,Node2,P),NodeName1,NodeName2) :- make_topological_map(Graph),
								   							 node_to_nodename(Graph,Node1,NodeName1),
								   							 node_to_nodename(Graph,Node2,NodeName2).

make_plan([],Plan) :- !.
make_plan([E1|EN],Plan) :- get_nodes_from_edge(E1,N1,N2),
						   make_plan(EN,Plan1),
						   lists:append(Plan1,[start_goto_node(N2,Theta,T)],Plan).

