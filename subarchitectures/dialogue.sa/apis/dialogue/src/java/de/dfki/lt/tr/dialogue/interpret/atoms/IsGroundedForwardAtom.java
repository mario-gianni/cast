package de.dfki.lt.tr.dialogue.interpret.atoms;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.dialogue.interpret.InterpretableAtom;
import de.dfki.lt.tr.dialogue.interpret.MatcherUtils;
import de.dfki.lt.tr.dialogue.interpret.TermParsingException;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.proof.ModalisedAtomMatcher;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;
import java.util.List;

public class IsGroundedForwardAtom
implements InterpretableAtom {

	public static final String PRED_SYMBOL = "is_grounded_forward";

	private final String nominal;
	private final String decoded;

	public IsGroundedForwardAtom(String nominal, String decoded) {
		this.nominal = nominal;
		this.decoded = decoded;
	}

	public String getNominal() {
		return nominal;
	}

	public String getDecoded() {
		return decoded;
	}

	@Override
	public ModalisedAtom toModalisedAtom() {
		return TermAtomFactory.modalisedAtom(
				new Modality[] {
					Modality.Understanding
				},
				TermAtomFactory.atom(PRED_SYMBOL, new Term[] {
					nominal == null ? TermAtomFactory.var("Nom") : TermAtomFactory.term(nominal),
					decoded == null ? TermAtomFactory.var("Decoded") : TermAtomFactory.term(decoded)
				}));
	}

	public static class Matcher implements ModalisedAtomMatcher<IsGroundedForwardAtom> {

		@Override
		public IsGroundedForwardAtom match(ModalisedAtom matom) {
			if (matom.a.predSym.equals(PRED_SYMBOL)
					&& matom.a.args.size() == 2) {

				List<Term> args = matom.a.args;

				try {
					String nominal = MatcherUtils.parseTermToString(args.get(0));
					String decoded = MatcherUtils.parseTermToString(args.get(1));

					return new IsGroundedForwardAtom(nominal, decoded);
				}
				catch (TermParsingException ex) {
					return null;
				}

			}

			return null;
		}

	}

	public static boolean isConstTerm(FunctionTerm ft) {
		return ft.args.isEmpty();
	}

}
