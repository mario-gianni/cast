% NIFTI Project
% from technical report about flexible planning
% author Gianni Mario
% date: 06 - 07 - 2010
% modified: 08 - 07 - 2010

% Allen temporal relations between processes

% P,Q -> processes
% Pl,Pr,Ql,Qr -> start/finish time
% Sp -> timeline of P
% Sq -> timeline of Q
% Ip,Iq -> Components of processes P/Q

before(P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- p(Ip,P),p(Iq,Q),
				 (elapsed(Ip,P,Pl,Pr,Sp) ->
				  (elapsed(Iq,Q,Ql,Qr,Sq);active(Iq,Q,Ql,Sq)),Pr #< Ql).

beforei(P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- before(Q,Ql,Qr,Sq,P,Pl,Pr,Sp).

meet(P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- p(Ip,P),p(Iq,Q),
			       (elapsed(Ip,P,Pl,Pr,Sp) ->
				(elapsed(Iq,Q,Ql,Qr,Sq);active(Iq,Q,Ql,Sq)),Pr #= Ql).

meeti(P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- meet(Q,Ql,Qr,Sq,P,Pl,Pr,Sp).

start(P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- p(Ip,P),p(Iq,Q),
			        ((active(Ip,P,Pl,Sp) -> 
				 (active(Iq,Q,Ql,Sq);elapsed(Iq,Q,Ql,Qr,Sq)),Pl #= Ql);
				 (elapsed(Ip,P,Pl,Pr,Sp) ->
				 (elapsed(Iq,Q,Ql,Qr,Sq);active(Iq,Q,Ql,Sq)),Pl #= Ql)).

starti(P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- start(Q,Ql,Qr,Sq,P,Pl,Pr,Sp).

finish(P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- p(Ip,P),p(Iq,Q),
				 (elapsed(Ip,P,Pl,Pr,Sp) ->
				  elapsed(Iq,Q,Ql,Qr,Sq),Pr #= Qr).

finishi(P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- finish(Q,Ql,Qr,Sq,P,Pl,Pr,Sp).

during(P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- p(Ip,P),p(Iq,Q),
				 ((elapsed(Ip,P,Pl,Pr,Sp) ->
				  (elapsed(Iq,Q,Ql,Qr,Sq);active(Iq,Q,Ql,Sq)),(Ql #< Pl and Pr #< Qr)); % !
				 (active(Ip,P,Pl,Sp) -> 
				  (active(Iq,Q,Ql,Sq);elapsed(Iq,Q,Ql,Qr,Sq)),Ql #< Pl)). %% controlla

duringi(P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- during(Q,Ql,Qr,Sq,P,Pl,Pr,Sp).

overlap(P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- p(Ip,P),p(Iq,Q),
				  (elapsed(Ip,P,Pl,Pr,Sp) ->
				   (elapsed(Iq,Q,Ql,Qr,Sq);active(Iq,Q,Ql,Sq)),Pl #< Ql and Pr #< Qr and Ql #< Pr).

overlapi(P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- overlap(Q,Ql,Qr,Sq,P,Pl,Pr,Sp).

% these predicates are used to call the temporal-predicates in the i(Tc,Bag)

wrap(before,P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- before(P,Pl,Pr,Sp,Q,Ql,Qr,Sq).

wrap(beforei,P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- beforei(P,Pl,Pr,Sp,Q,Ql,Qr,Sq).

wrap(meet,P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- meet(P,Pl,Pr,Sp,Q,Ql,Qr,Sq).

wrap(meeti,P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- meeti(P,Pl,Pr,Sp,Q,Ql,Qr,Sq).

wrap(start,P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- start(P,Pl,Pr,Sp,Q,Ql,Qr,Sq).

wrap(starti,P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- starti(P,Pl,Pr,Sp,Q,Ql,Qr,Sq).

wrap(finish,P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- finish(P,Pl,Pr,Sp,Q,Ql,Qr,Sq).

wrap(finishi,P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- finishi(P,Pl,Pr,Sp,Q,Ql,Qr,Sq).

wrap(during,P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- during(P,Pl,Pr,Sp,Q,Ql,Qr,Sq).

wrap(duringi,P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- duringi(P,Pl,Pr,Sp,Q,Ql,Qr,Sq).

wrap(overlap,P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- overlap(P,Pl,Pr,Sp,Q,Ql,Qr,Sq).

wrap(overlapi,P,Pl,Pr,Sp,Q,Ql,Qr,Sq) :- overlapi(P,Pl,Pr,Sp,Q,Ql,Qr,Sq).
