package eu.nifti.dialogue.actions;

import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;

public class DifferentialAction extends AbstractAction {
	
	public static enum DifferentialLocking {
		LOCKED,
		UNLOCKED
	}

	protected final DifferentialLocking locking;
	
	public DifferentialAction(DifferentialLocking locking) {
		this.locking = locking;
	}

	@Override
	public void setContent(InterpretedIntention iint) {
		iint.stringContent.put("type", "differential-manip");
		iint.stringContent.put("to-setting", locking.name());
	}
	
}
