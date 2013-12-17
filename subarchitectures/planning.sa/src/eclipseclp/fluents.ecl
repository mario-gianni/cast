% NIFTI Project
% from technical report about flexible planning
% author Gianni Mario
% date: 13 - 12 - 2010

:- dynamic check/4.

%
% Fluents definition
%
fluent(process(I,P,S)).
fluent(holds(I,P,S)).
fluent(idle(I,S)).

fluent(started(I,P,T,A,S)).
fluent(ended(I,P,T,A,S)).
fluent(active(I,P,T,S)).
fluent(elapsed(I,P,Ti,Te,S)).

%
% auxiliary predicate used to update and to store the fluents true for every situation S
% check(Component,Predicate,Params,S) :- predicate(Params,S)
%

% Navigation component
check(navigation,process,goto_pose(X,Y,Theta),S) :- process(navigation,goto_pose(X,Y,Theta),S).
check(navigation,started,goto_pose(X,Y,Theta),S) :- nth0(0,S,A,S1),started(navigation,goto_pose(X,Y,Theta),T,A,S1).
check(navigation,ended,goto_pose(X,Y,Theta),S) :- nth0(0,S,A,S1),ended(navigation,goto_pose(X,Y,Theta),T,A,S1).
check(navigation,active,goto_pose(X,Y,Theta),S) :- active(navigation,goto_pose(X,Y,Theta),T,S).
check(navigation,elapsed,goto_pose(X,Y,Theta),S) :- elapsed(navigation,goto_pose(X,Y,Theta),Ti,Te,S).

check(navigation,process,goto_node(NodeId),S) :- process(navigation,goto_node(NodeId),S).
check(navigation,started,goto_node(NodeId),S) :- nth0(0,S,A,S1),started(navigation,goto_node(NodeId),T,A,S1).
check(navigation,ended,goto_node(NodeId),S) :- nth0(0,S,A,S1),ended(navigation,goto_node(NodeId),T,A,S1).
check(navigation,active,goto_node(NodeId),S) :- active(navigation,goto_node(NodeId),T,S).
check(navigation,elapsed,goto_node(NodeId),S) :- elapsed(navigation,goto_node(NodeId),Ti,Te,S).

check(navigation,process,wander,S) :- process(navigation,wander,S).
check(navigation,started,wander,S) :- nth0(0,S,A,S1),started(navigation,wander,T,A,S1).
check(navigation,ended,wander,S) :- nth0(0,S,A,S1),ended(navigation,wander,T,A,S1).
check(navigation,active,wander,S) :- active(navigation,wander,T,S).
check(navigation,elapsed,wander,S) :- elapsed(navigation,wander,Ti,Te,S).

check(navigation,holds,at(X,Y),S) :- holds(navigation,at(X,Y),S).
check(navigation,started,at(X,Y),S) :- nth0(0,S,A,S1),started(navigation,at(X,Y),T,A,S1).
check(navigation,ended,at(X,Y),S) :- nth0(0,S,A,S1),ended(navigation,at(X,Y),T,A,S1).
check(navigation,active,at(X,Y),S) :- active(navigation,at(X,Y),T,S).
check(navigation,elapsed,at(X,Y),S) :- elapsed(navigation,at(X,Y),Ti,Te,S).

check(navigation,holds,in(NodeId),S) :- holds(navigation,in(NodeId),S).
check(navigation,started,in(NodeId),S) :- nth0(0,S,A,S1),started(navigation,in(NodeId),T,A,S1).
check(navigation,ended,in(NodeId),S) :- nth0(0,S,A,S1),ended(navigation,in(NodeId),T,A,S1).
check(navigation,active,in(NodeId),S) :- active(navigation,in(NodeId),T,S).
check(navigation,elapsed,in(NodeId),S) :- elapsed(navigation,in(NodeId),Ti,Te,S).

check(navigation,idle,_P,S) :- idle(navigation,S).

% Slam component
check(slam,process,update_map,S) :- process(slam,update_map,S).
check(slam,started,update_map,S) :- nth0(0,S,A,S1),started(slam,update_map,T,A,S1).
check(slam,ended,update_map,S) :- nth0(0,S,A,S1),ended(slam,update_map,T,A,S1).
check(slam,active,update_map,S) :- active(slam,update_map,T,S).
check(slam,elapsed,update_map,S) :- elapsed(slam,update_map,Ti,Te,S).

check(slam,process,localize,S) :- process(slam,localize,S).
check(slam,started,localize,S) :- nth0(0,S,A,S1),started(slam,localize,T,A,S1).
check(slam,ended,localize,S) :- nth0(0,S,A,S1),ended(slam,localize,T,A,S1).
check(slam,active,localize,S) :- active(slam,localize,T,S).
check(slam,elapsed,localize,S) :- elapsed(slam,localize,Ti,Te,S).

check(slam,holds,wait,S) :- holds(slam,wait,S).
check(slam,started,wait,S) :- nth0(0,S,A,S1),started(slam,wait,T,A,S1).
check(slam,ended,wait,S) :- nth0(0,S,A,S1),ended(slam,wait,T,A,S1).
check(slam,active,wait,S) :- active(slam,wait,T,S).
check(slam,elapsed,wait,S) :- elapsed(slam,wait,Ti,Te,S).

check(slam,idle,_P,S) :- idle(slam,S).

% Vision component
check(vision,process,search(Object),S) :- process(vision,search(Object),S).
check(vision,started,search(Object),S) :- nth0(0,S,A,S1),started(vision,search(Object),T,A,S1).
check(vision,ended,search(Object),S) :- nth0(0,S,A,S1),ended(vision,search(Object),T,A,S1).
check(vision,active,search(Object),S) :- active(vision,search(Object),T,S).
check(vision,elapsed,search(Object),S) :- elapsed(vision,search(Object),Ti,Te,S).

check(vision,idle,_P,S) :- idle(vision,S).

% Functional mapping component
check(func_mapping,holds,identify_interest_area,S) :- holds(func_mapping,identify_interest_area,S).
check(func_mapping,started,identify_interest_area,S) :- nth0(0,S,A,S1),started(func_mapping,identify_interest_area,T,A,S1).
check(func_mapping,ended,identify_interest_area,S) :- nth0(0,S,A,S1),ended(func_mapping,identify_interest_area,T,A,S1).
check(func_mapping,active,identify_interest_area,S) :- active(func_mapping,identify_interest_area,T,S).
check(func_mapping,elapsed,identify_interest_area,S) :- elapsed(func_mapping,identify_interest_area,Ti,Te,S).

check(func_mapping,idle,_P,S) :- idle(func_mapping,S).

%
% auxiliary predicate used to update KB
%

% Navigation component
restartSituation(process(navigation,goto_pose(X,Y,Theta),S),process(navigation,goto_pose(X,Y,Theta),[])).
restartSituation(process(navigation,goto_node(NodeId),S),process(navigation,goto_node(NodeId),[])).
restartSituation(process(navigation,wander,S),process(navigation,wander,[])).
restartSituation(holds(navigation,at(X,Y),S),holds(navigation,at(X,Y),[])).
restartSituation(holds(navigation,in(NodeId),S),holds(navigation,in(NodeId),[])).
restartSituation(idle(navigation,S),idle(navigation,[])).

restartSituation(active(navigation,goto_pose(X,Y,Theta),T,[A|S]),active(navigation,goto_pose(X,Y,Theta),T,[])).
restartSituation(active(navigation,goto_node(NodeId),T,[A|S]),active(navigation,goto_node(NodeId),T,[])).
restartSituation(active(navigation,wander,T,[A|S]),active(navigation,wander,T,[])).
restartSituation(active(navigation,at(X,Y),T,[A|S]),active(navigation,at(X,Y),T,[])).
restartSituation(active(navigation,in(NodeId),T,[A|S]),active(navigation,in(NodeId),T,[])).

restartSituation(elapsed(navigation,goto_pose(X,Y,Theta),Ti,Te,[A|S]),elapsed(navigation,goto_pose(X,Y,Theta),Ti,Te,[])).
restartSituation(elapsed(navigation,goto_node(NodeId),Ti,Te,[A|S]),elapsed(navigation,goto_node(NodeId),Ti,Te,[])).
restartSituation(elapsed(navigation,wander,Ti,Te,[A|S]),elapsed(navigation,wander,Ti,Te,[])).
restartSituation(elapsed(navigation,at(X,Y),Ti,Te,[A|S]),elapsed(navigation,at(X,Y),Ti,Te,[])).
restartSituation(elapsed(navigation,in(NodeId),Ti,Te,[A|S]),elapsed(navigation,in(NodeId),Ti,Te,[])).

% Slam component
restartSituation(process(slam,update_map,[A|S]),process(slam,update_map,[])).
restartSituation(process(slam,localize,[A|S]),process(slam,localize,[])).
restartSituation(holds(slam,wait,[A|S]),holds(slam,wait,[])).
restartSituation(idle(slam,[A|S]),idle(slam,[])).

restartSituation(active(slam,update_map,T,[A|S]),active(slam,update_map,T,[])).
restartSituation(active(slam,localize,T,[A|S]),active(slam,localize,T,[])).
restartSituation(active(slam,wait,T,[A|S]),active(slam,wait,T,[])).

restartSituation(elapsed(slam,update_map,Ti,Te,[A|S]),elapsed(slam,update_map,Ti,Te,[])).
restartSituation(elapsed(slam,localize,Ti,Te,[A|S]),elapsed(slam,localize,Ti,Te,[])).
restartSituation(elapsed(slam,wait,Ti,Te,[A|S]),elapsed(slam,wait,Ti,Te,[])).

% Vision Component
restartSituation(process(vision,search(Object),[A|S]),process(vision,search(Object),[])).
restartSituation(idle(vision,[A|S]),idle(vision,[])).

restartSituation(active(vision,search(Object),T,[A|S]),active(vision,search(Object),T,[])).

restartSituation(elapsed(vision,search(Object),Ti,Te,[A|S]),elapsed(vision,search(Object),Ti,Te,[])).

% Functional Mapping

restartSituation(holds(func_mapping,identify_interest_area,[A|S]),holds(func_mapping,identify_interest_area,[])).
restartSituation(idle(func_mapping,[A|S]),idle(func_mapping,[])).

restartSituation(active(func_mapping,identify_interest_area,T,[A|S]),active(func_mapping,identify_interest_area,T,[])).

restartSituation(elapsed(func_mapping,identify_interest_area,Ti,Te,[A|S]),elapsed(func_mapping,identify_interest_area,Ti,Te,[])).

%
% auxiliary predicate used to generate the plan
%

% Navigation component
restoreSituation(S,process(navigation,goto_pose(X,Y,Theta),S)).
restoreSituation(S,process(navigation,goto_node(NodeId),S)).
restoreSituation(S,process(navigation,wander,S)).
restoreSituation(S,holds(navigation,at(X,Y),S)).
restoreSituation(S,holds(navigation,in(NodeId),S)).
restoreSituation(S,idle(navigation,S)).
restoreSituation(S,active(navigation,goto_pose(X,Y,Theta),T,S)).
restoreSituation(S,active(navigation,goto_node(NodeId),T,S)).
restoreSituation(S,active(navigation,wander,T,S)).
restoreSituation(S,active(navigation,at(X,Y),T,S)).
restoreSituation(S,active(navigation,in(NodeId),T,S)).
restoreSituation(S,elapsed(navigation,goto_pose(X,Y,Theta),Ti,Te,S)).
restoreSituation(S,elapsed(navigation,goto_node(NodeId),Ti,Te,S)).
restoreSituation(S,elapsed(navigation,wander,Ti,Te,S)).
restoreSituation(S,elapsed(navigation,at(X,Y),Ti,Te,S)).
restoreSituation(S,elapsed(navigation,in(NodeId),Ti,Te,S)).


% Slam component
restoreSituation(S,process(slam,update_map,S)).
restoreSituation(S,process(slam,localize,S)).
restoreSituation(S,holds(slam,wait,S)).
restoreSituation(S,idle(slam,S)).
restoreSituation(S,active(slam,update_map,T,S)).
restoreSituation(S,active(slam,localize,T,S)).
restoreSituation(S,active(slam,wait,T,S)).
restoreSituation(S,elapsed(slam,update_map,Ti,Te,S)).
restoreSituation(S,elapsed(slam,localize,Ti,Te,S)).
restoreSituation(S,elapsed(slam,wait,Ti,Te,S)).

% Vision Component
restoreSituation(S,process(vision,holds(Object),S)).
restoreSituation(S,idle(vision,S)).
restoreSituation(S,active(vision,search(Object),T,S)).
restoreSituation(S,elapsed(vision,search(Object),Ti,Te,S)).

% Functional Mapping
restoreSituation(S,holds(func_mapping,identify_interest_area,S)).
restoreSituation(S,idle(func_mapping,S)).
restoreSituation(S,active(func_mapping,identify_interest_area,T,S)).
restoreSituation(S,elapsed(func_mapping,identify_interest_area,Ti,Te,S)).
