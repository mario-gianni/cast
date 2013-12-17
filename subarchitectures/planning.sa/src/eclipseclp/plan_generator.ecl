% NIFTI Project
% from technical report about flexible planning
% author Gianni Mario
% date: 07 - 11 - 2010

/*
	concatena due liste
*/
my_append([],X,X):- !.
my_append([X|L1],L2,[X|L3]):- my_append(L1,L2,L3).

/*
	seleziona un'azione appartenente ad un componente
*/
choose_action(Component,Action) :- h(Component,Action). 

/*
	estrae il nome del componente dal predicato
*/
getComponent(Goal,Component) :- arg(1,Goal,Component).

/*
	It generates a plan for a component up to horizon N
	Situation has to be equal to empty list []
	es: planN(head,[],3,Plan)
*/
planN(Component,Situation,0,Plan) :- !.
planN(Component,Situation,Horizon,Plan) :- choose_action(Component,Action),
					   executable([Action|Situation]),
					   Horizon1 is Horizon - 1,
					   planN(Component,[Action|Situation],Horizon1,Plan1),
					   my_append(Plan1,[Action],Plan).
/*
	It generates all plans of length equal to Horizon for the specified component 
	Situation has to be equal to empty list []
	es: generate_all_plans(head,[],3,plans) 
*/
generate_all_plans(Component,Situation,Horizon,Plans) :- 
		findall(Plan,planN(Component,Situation,Horizon,Plan),Plans).

/*
	It generates all plans of length equal to Horizon 
	for a component and verify if exist a plan that satisfy the goal
	Situation has to be equal to empty list []
	es: verify_all_plans(1,elapsed(navigation,goto(x1,x2),Ti,Te,S),GoodPlan)
	GoodPlan is the plan if exists 	
*/
verify_all_plans(Horizon,Goal,GoodPlan) :- 
					getComponent(Goal,Component),
					generate_all_plans(Component,[],Horizon,Plans),
					verify(Goal,Plans,GoodPlan).
/*	
	Given a set of plans it checks if exists at 
	least a plan that satisfies the goal
*/
verify(Goal,Plans,GoodPlan) :- nth0(0,Plans,FirstPlan,Rest),
		               (restoreSituation(FirstPlan,Goal),call(Goal) -> GoodPlan = FirstPlan ; 
				verify(Goal,Rest,GoodPlan)). 
/*
	It searches the plan that satisfies the goal

	Situation has to be equal to empty list []
	Horizon has to be equal to 1
	
	The Goal is achived in s0([]) OR
	The Goal is achived by a plan of lenght equal to Horizon = 1 OR

	incrementing the Horizon and it checkes if exist a new plan that satisfy the goal
	
	If the algorithm reaches Horizon = 5 then it is stopped

	es: achive_goal(1,elapsed(head,point(p),1,4,S),GoodPlan)
	GoodPlan = [end_point(p, 4), start_point(p, 1)]
	es: achive_goal(1,elapsed(lcm,run,Ti,Te,S),GoodPlan)
	GoodPlan = [end_run(_556{0 .. 1.0Inf}), start_run(_606{0 .. 1.0Inf})]
	es: achive_goal(1,holds(navigation,at(x2),S),GoodPlan)
	GoodPlan = [end_goto(x1, x2, _728{0 .. 1.0Inf}), start_goto(x1, x2, _778{0 .. 1.0Inf})]
*/
achive_goal(5,_Goal,_GoodPlan) :- false. % Termination condition
achive_goal(Horizon,Goal,GoodPlan) :- 
		restoreSituation([],Goal),call(Goal),GoodPlan = [];
		verify_all_plans(Horizon,Goal,GoodPlan);
		(Horizon1 is Horizon + 1, 
		achive_goal(Horizon1,Goal,GoodPlan)).

/*

	Given a list of goals generate the Bag  of situation to reach these goals

	es: reach_goals([holds(navigation,at(x2),S),elapsed(head,point(p),1,4,S)],Bag).
	Bag = [
	       [end_point(p, 4), start_point(p, 1)],
	       [end_goto(x1, x2, _728{0 .. 1.0Inf}), start_goto(x1, x2, _778{0 .. 1.0Inf})]
	      ]
	es: reach_goals([holds(navigation,at(x2),S),
			 elapsed(lcm,run,Ti,Te,S),
			 elapsed(head,point(p),Ti,Te,S)],Bag).
	Bag = [[end_point(p, _1593{0 .. 1.0Inf}), start_point(p, _1643{0 .. 1.0Inf})], 
	       [end_run(_974{0 .. 1.0Inf}), start_run(_1024{0 .. 1.0Inf})], 
	       [end_goto(x1, x2, _641{0 .. 1.0Inf}), start_goto(x1, x2, _691{0 .. 1.0Inf})]]

	es: reach_goals([holds(navigation,at(x2),S),
	     elapsed(slam,map,Ti,Te,S),
	     elapsed(attention,detect(victim),Ti,Te,S),
	     holds(lcm,stop,S)],Bag).

	Bag = [[end_run(_2116{0 .. 1.0Inf}), start_run(_2166{0 .. 1.0Inf})], 
	       [end_detect(victim, _1387{0 .. 1.0Inf}), start_detect(victim, _1437{0 .. 1.0Inf})],
 	       [end_map(_688{0 .. 1.0Inf}), start_map(_738{0 .. 1.0Inf})], 
	       [end_goto(x1, x2, _355{0 .. 1.0Inf}), start_goto(x1, x2, _405{0 .. 1.0Inf})]]

	The goals are in AND
*/
reach_goals([],Bag) :- !.
reach_goals([FirstGoal|RestGoals],Bag) :- copy_term(FirstGoal,Goal),
					  achive_goal(1,Goal,GoodPlan),!,
					  reach_goals(RestGoals,Bag1),
					  my_append(Bag1,[GoodPlan],Bag).
