/*
	NIFTI project
	Flexible Planning
	Mario Gianni
	08-03-2011
*/
:- lib(ic).
:- lib(lists).
:- lib(listut).
:- lib(graph_algorithms).

:- set_flag(variable_names,off).

:- dynamic graph/1, edge/1, task/2, holds/3.

:- [make_topological_map].
:- [task_library].
:- [exe_monitor].
:- [planner].

:- [action_preconditions].

