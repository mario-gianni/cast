% NIFTI Project
% from technical report about flexible planning
% author Gianni Mario
% date: 08 - 12 - 2010

% Navigation Component
h(navigation,start_goto_pose(X,Y,Theta,Time)).
h(navigation,end_goto_pose(X,Y,Theta,Time)).
h(navigation,start_goto_node(NodeId,Time)).
h(navigation,end_goto_node(NodeId,Time)).
h(navigation,start_wander(Time)).
h(navigation,end_wander(Time)).

% Slam Component
h(slam,start_update_map(Time)).
h(slam,end_update_map(Time)).
h(slam,start_update_position(Time)).
h(slam,end_update_position(Time)).

% Vision Component
h(vision,start_search(Object,Time)).
h(vision,end_search(Object,Time)).

% Functional Mapping Component
h(func_mapping,start_identify_interest_area(Time)).
h(func_mapping,end_identify_interest_area(Time)).


