% NIFTI Project
% from technical report about flexible planning
% author Gianni Mario
% date: 07 - 07 - 2010
% modified: 15 - 09 - 2010

% given the integer domain of variables
% the method solves the temporal constraints

% Tc -> compatibilities
% Bag -> bag of situation


solve(Tc,Bag,Solution,(Start,End)) :- search_variables(Bag,Solution,N),
			  writeln(Solution),
			  Solution :: Start..End,
			  se_constraint(Bag),
			  i(Tc,Bag),
			  labeling(Solution),
			  writeln(Solution).

search_variables(Bag,Solution,N) :- flatten(Bag,L),length(L,N),search(L,Solution).

search([],Solution).
search(L,Solution) :- nth0(0,L,A,Rest),time(A,Ti),
		      (var(Ti) -> add(Ti,S1,Solution),
				search(Rest,S1)
			;	
				S1 = Solution,search(Rest,S1)). 
