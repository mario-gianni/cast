% NIFTI Project
% from technical report about flexible planning
% author Gianni Mario
% date: 13 - 12 - 2010

% heuristic functions to guide the search in plan generator algorithm

wrong(start_goto_pose(_,_,_,_),start_goto_pose(_,_,_,_)).
wrong(start_goto_node(_,_),start_goto_node(_,_)).
wrong(start_wander(_),start_wander(_)).
wrong(start_update_map(_),start_update_map(_)).
wrong(start_update_position(_),start_update_position(_)).
wrong(start_search(_,_),start_search(_,_)).
wrong(start_identify_interest_area(_),start_identify_interest_area(_)).
wrong(end_goto_pose(_,_,_,_),end_goto_pose(_,_,_,_)).
wrong(end_goto_node(_,_),end_goto_node(_,_)).
wrong(end_wander(_),end_wander(_)).
wrong(end_update_map(_),end_update_map(_)).
wrong(end_update_position(_),end_update_position(_)).
wrong(end_search(_,_),end_search(_,_)).
wrong(end_identify_interest_area(_),end_identify_interest_area(_)).

bad_action(A,S) :- is_action(A),is_list(S),nth0(0,S,A1),wrong(A,A1).
