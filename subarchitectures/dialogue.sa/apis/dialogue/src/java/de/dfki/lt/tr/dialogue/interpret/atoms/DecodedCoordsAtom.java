package de.dfki.lt.tr.dialogue.interpret.atoms;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
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

public class DecodedCoordsAtom
implements InterpretableAtom {

	public static final String PRED_SYMBOL = "decoded_target";

	private final dFormula encoded;
        private final String arg;

	public DecodedCoordsAtom(dFormula encoded, String arg) {
		this.encoded = encoded;
                this.arg = arg;
	}

	public dFormula getEncoded() {
		return encoded;
	}

	public String getArg() {
		return arg;
	}

	@Override
	public ModalisedAtom toModalisedAtom() {
		return TermAtomFactory.modalisedAtom(
				new Modality[] {
					Modality.Understanding,
					Modality.Truth
				},
				TermAtomFactory.atom(PRED_SYMBOL, new Term[] {
					getEncoded() == null ? TermAtomFactory.var("Enc") : ConversionUtils.stateFormulaToTerm(getEncoded()),
					getArg() == null ? TermAtomFactory.var("Target") : TermAtomFactory.term(getArg())
				}));
	}

	public static class Matcher implements ModalisedAtomMatcher<DecodedCoordsAtom> {

		@Override
		public DecodedCoordsAtom match(ModalisedAtom matom) {
			if (matom.a.predSym.equals(PRED_SYMBOL)
					&& matom.a.args.size() == 2) {

				List<Term> args = matom.a.args;

                                try {
                                    dFormula encoded = ConversionUtils.uniTermToFormula(args.get(0));
                                    String arg = MatcherUtils.parseTermToString(args.get(1));
                                    return new DecodedCoordsAtom(encoded, arg);
                                }
                                catch (TermParsingException ex) {
                                    return null;
                                }
			}

			return null;
		}

	}

}
