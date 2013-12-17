% NIFTI Project
% from technical report about flexible planning
% author Gianni Mario
% date: 08 - 12 - 2010


%	ACTIONS
%
% Bag = [[end_search(Obj,T12),start_search(Obj,T11)],
% 	 [end_goto_pose(X1,Y1,Theta,T22),start_goto_pose(X1,Y1,Theta,T21)],
%	 [end_update_map(T32),start_update_map(T31)],
%	 [end_identify_interest_area(T42),start_identify_interest_area(T41)]]

%	COMPATIBILITIES
%
% Tc = [comp(search(Obj),[[(during,goto_pose(X,Y,Theta)),(overlaps,identify_interest_area)]]),
%       comp(goto_pose(X,Y,Theta),[[(during,update_map)]]),
%	comp(identify_interest_area,[[(finish,update_map)]])]

%
% Tc = [comp(goto_pose(X,Y,Theta),[[(during,update_map)]]),
% 	comp(update_map,[[(start,search(Obj)),(finish,search(Obj)),
%	    (start,identify_interest_area),(finish,identify_interest_area)]])]

task(lovr,Bag,Tc,(Start,End),S0) :- 
		conditions(lovr,S0),
		Bag = [[end_search(Obj,T12),start_search(Obj,T11)],
	 	       [end_goto_pose(X1,Y1,Theta,T22),start_goto_pose(X1,Y1,Theta,T21)],
	 	       [end_update_map(T32),start_update_map(T31)],
	 	       [end_identify_interest_area(T42),start_identify_interest_area(T41)]],
		Tc = [comp(goto_pose(X,Y,Theta),[[(during,update_map)]]),
 		      comp(update_map,[[(start,search(Obj)),(finish,search(Obj)),
	    		                (start,identify_interest_area),(finish,identify_interest_area)]])],
		Start = 1, End = 10.	


conditions(lovr,S0) :- idle(navigation,S0),
		       idle(slam,S0),
		       idle(vision,S0),
		       idle(func_mapping,S0).
