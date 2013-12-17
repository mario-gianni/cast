package de.dfki.lt.tr.cast.dialogue;

import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPointer;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.cast.dialogue.FakeReferenceResolution.FakeReferenceResolver;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.ref.EpistemicReferenceHypothesis;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionResult;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolver;
import de.dfki.lt.tr.dialogue.ref.util.ReferenceUtils;
import de.dfki.lt.tr.dialogue.util.EpistemicStatusFactory;
import java.util.LinkedList;
import java.util.List;

public class FakeReferenceResolution
extends AbstractReferenceResolutionComponent<FakeReferenceResolver> {

	public FakeReferenceResolution() {
		super();
	}

	@Override
	protected FakeReferenceResolver initResolver() {
		return new FakeReferenceResolver();
	}

	@Override
	protected void onStart() {
		super.onStart();
                addFakeCoordinatesReferent();
//                addFakeObjectAsReferent();
//		for (int i = 0; i < 4; i++) {
//                        addFakeCoordinatesReferent();
//			addFakeReferent();
//		}
	}

        protected void addFakeCoordinatesReferent() {
            dFormula ref = new ModalFormula(0, "coordinates", new ElementaryFormula(0, "0.1,0.2,0.0"));
            getResolver().addReferent(ref);
        }
        
	protected void addFakeObjectAsReferent() {
		WorkingMemoryAddress wma = new WorkingMemoryAddress(newDataID(), this.getSubarchitectureID());
                WorkingMemoryPointer wmptr = new WorkingMemoryPointer(wma, dBelief.class.getCanonicalName());
        	dFormula referent = new ModalFormula(0, "object", new PointerFormula(0, wmptr.address, wmptr.type));
		getResolver().addReferent(referent);
	}

	public static class FakeReferenceResolver implements ReferenceResolver {

		private List<dFormula> formulas = new LinkedList<dFormula>();
		
		public void addReferent(dFormula formula) {
			formulas.add(formula);
		}

		@Override
		public ReferenceResolutionResult resolve(ReferenceResolutionRequest rr, WorkingMemoryAddress origin) {
			ReferenceResolutionResult result = ReferenceUtils.newEmptyResolutionResult(rr, origin, ReferenceResolver.SORT_OBJECT);

			EpistemicStatus epst = EpistemicStatusFactory.newSharedEpistemicStatus(IntentionManagementConstants.thisAgent, IntentionManagementConstants.humanAgent);
			double score = 0.7;
			for (dFormula f : formulas) {
				result.hypos.add(new EpistemicReferenceHypothesis(epst, f, score));
				score *= 0.85;
			}

			return result;
		}
		
	}

}
