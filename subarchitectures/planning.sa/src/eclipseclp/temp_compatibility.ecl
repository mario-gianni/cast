% NIFTI Project
% from technical report about flexible planning
% author Gianni Mario
% date: 06 - 07 - 2010

% an implementation of i(Tc,Bag) predicate
% Tc = [comp(P,[[(OP1,P1),(OP2,P2)],[(OP3,P3),(OP4,P4)]]),comp(Q,[[(OP1,Q1),(OP2,Q2)],[(OP3,Q3)]])].
% Bag = list of action lists

i([],Bag).

i([comp(P,[List|LList])|Tail],Bag) :- i1(comp(P,[List|LList]),Bag), i(Tail,Bag).

i1(comp(P,[]),Bag) :- false.

i1(comp(P,[List|LList]),Bag) :- i2_0(P,List,Bag); i1(comp(P,LList),Bag).

i2_0(P,List,Bag) :- member(SP,Bag),belong(Ip,P,SP),i2_1(P,List,SP,Bag).

i2_1(P,List,SP,Bag) :- (foreach(Term,List),param(P),param(SP),param(Bag) do i2_2(P,Term,SP,Bag)).

i2_2(P,(OP,Q),SP,Bag) :- member(SQ,Bag),belong(Iq,Q,SQ),i2_3(P,SP,OP,Q,SQ).

i2_3(P,[],OP,Q,SQ).
i2_3(P,SP,OP,Q,[]).
i2_3(P,[],OP,Q,[]).
i2_3(P,[AP|SP],OP,Q,[AQ|SQ]) :- copy_term(P,P1),copy_term(Q,Q1),(wrap(OP,P1,Pl,Pr,[AP|SP],Q1,Ql,Qr,[AQ|SQ]) ->
					i2_3(P,SP,OP,Q,SQ)
				 ;	
					i2_3(P,SP,OP,Q,SQ)).
