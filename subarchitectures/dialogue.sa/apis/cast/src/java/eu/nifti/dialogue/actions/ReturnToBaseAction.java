package eu.nifti.dialogue.actions;

import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;

public class ReturnToBaseAction extends AbstractMovementAction {

	@Override
	public void setContent(InterpretedIntention iint) {
		super.setContent(iint);
		
		iint.stringContent.put("subtype", "return-to-base");
	}
	
}
