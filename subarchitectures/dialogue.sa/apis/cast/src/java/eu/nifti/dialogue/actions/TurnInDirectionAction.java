package eu.nifti.dialogue.actions;

import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;

public class TurnInDirectionAction extends AbstractAction {

	public static enum Direction {
		LEFT,
		RIGHT
	}

	protected final Direction dir;

	public TurnInDirectionAction(Direction dir) {
		this.dir = dir;
	}

	@Override
	public void setContent(InterpretedIntention iint) {
		iint.stringContent.put("type", "turning");
		iint.stringContent.put("subtype", "in-direction");
		iint.stringContent.put("turn-direction", dir.name());
	}
	
}
