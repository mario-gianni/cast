% NIFTI Project
% from technical report about flexible planning
% author Gianni Mario
% date: 01 - 07 - 2010

% this file contain the definition of the axioms in the table 1 of technical report

% Axioms

% H2
eq_ni(I,A,B) :- is_action(A),is_action(B),h(I,A), h(I,B),!.

% E3
eq_ni(I,A,S) :- is_list(S),is_action(A),(foreach(A1,S),param(A),param(I) do eq_ni(I,A,A1)).

% E5
eq_ni(I,S,A) :- is_list(S),is_action(A),eq_ni(I,A,S). 

% E4
eq_ni(I,S1,S2) :- is_list(S1),is_list(S2),(foreach(A1,S1),param(S2),param(I) do eq_ni(I,A1,S2)).
 
% check in the sequence of action belong to the same component
timeline(I,[A|B]) :- (B == [] -> h(I,A); eq_ni(I,A,B)).

% T1
time([],0) :- !.

% T2
time(Action,Time) :- is_action(Action),functor(Action,_F,Arity), arg(Arity,Action,Time).

% T3
time(Situation,Time) :- is_list(Situation),nth0(0,Situation,Action),time(Action,Time).

% T4
ordering(S1,S2) :- time(S1,T1), time(S2,T2), T1 =< T2.


