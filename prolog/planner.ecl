% author: Federico Ferri
% date: 2013-04-03

% SitCalc planner
executable(s0, _MaxLen).
executable(do(A, S), MaxLen) :- MaxLen > 0, L is MaxLen-1, executable(S, L), poss(A, S).

% default max length = 10
solution(Goals, X) :- solution(Goals, X, 0, 10).

solution(Goals, S, MaxLen) :- solution(Goals, S, 0, MaxLen).

solution(Goals, S, CurLen, MaxLen) :- executable(S, MaxLen), call(Goals);
    NextLen is CurLen + 1, !, NextLen < MaxLen, solution(Goals, S, NextLen, MaxLen).

action_list(S, L) :- action_list(S, [], L).
action_list(s0, L, L).
action_list(do(A, S), Accum, L) :- action_list(S, [A|Accum], L).

solve(Goals, S, ActionList) :- solution(Goals, S), action_list(S, ActionList).

