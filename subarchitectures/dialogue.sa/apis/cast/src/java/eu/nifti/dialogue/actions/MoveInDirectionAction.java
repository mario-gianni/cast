package eu.nifti.dialogue.actions;

import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;

public class MoveInDirectionAction extends AbstractMovementAction {

	public static enum Direction {
		FORWARD,
		LEFT,
		RIGHT,
		BACKWARD
	}

	protected final Direction dir;

	public MoveInDirectionAction(Direction dir) {
		this.dir = dir;
	}

	@Override
	public void setContent(InterpretedIntention iint) {
		super.setContent(iint);
		
		iint.stringContent.put("subtype", "in-direction");
		iint.stringContent.put("move-direction", dir.name());
	}
	
}
