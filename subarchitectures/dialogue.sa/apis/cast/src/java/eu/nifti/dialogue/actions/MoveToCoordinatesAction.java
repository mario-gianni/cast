package eu.nifti.dialogue.actions;

import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;

public class MoveToCoordinatesAction extends AbstractMovementAction {

	protected final double x;
	protected final double y;
	protected final double z;
	
	public MoveToCoordinatesAction(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	@Override
	public void setContent(InterpretedIntention iint) {
		super.setContent(iint);

                iint.stringContent.put("subtype", "to-target");
                iint.stringContent.put("target", "coordinates=" + Double.toString(x) + ","
                            + Double.toString(y) + ","
                            + Double.toString(z));
                
//		iint.stringContent.put("subtype", "to-coordinates");
//		iint.stringContent.put("coordinates-x", String.valueOf(x));
//		iint.stringContent.put("coordinates-y", String.valueOf(y));
//		iint.stringContent.put("coordinates-z", String.valueOf(z));
	}

}
