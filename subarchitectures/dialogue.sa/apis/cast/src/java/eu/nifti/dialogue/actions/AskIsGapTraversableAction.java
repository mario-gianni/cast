package eu.nifti.dialogue.actions;

import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;

public class AskIsGapTraversableAction extends AbstractAction {

	@Override
	public void setContent(InterpretedIntention iint) {
		iint.stringContent.put("type", "question");
		iint.stringContent.put("subtype", "ability-check");
		
		iint.stringContent.put("ability", "gap-traversal");
	}
	
}
