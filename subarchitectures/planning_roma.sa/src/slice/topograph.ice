#ifndef _EU_NIFTI_ENV_TOPOGRAPH_ICE
#define _EU_NIFTI_ENV_TOPOGRAPH_ICE

module eu {
	module nifti {
		module env {
			module topograph{			
				
				class Node{
					string label;
					double x;
					double y;
					int flag;
				};
				
				class Edge{
					Node a;
					Node b;
				};
				
				sequence<Node> NodeSeq;
				sequence<Edge> EdgeSeq;
				
				class Graph{
					NodeSeq nodes;
					EdgeSeq edges;
				};
			};
		};
	};
};

#endif
