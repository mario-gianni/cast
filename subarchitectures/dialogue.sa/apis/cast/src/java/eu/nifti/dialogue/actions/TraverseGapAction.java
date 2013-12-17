package eu.nifti.dialogue.actions;

import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;

public class TraverseGapAction extends AbstractAction {

	@Override
	public void setContent(InterpretedIntention iint) {
		iint.stringContent.put("type", "gap-traversal-mode");
	}
	
}
