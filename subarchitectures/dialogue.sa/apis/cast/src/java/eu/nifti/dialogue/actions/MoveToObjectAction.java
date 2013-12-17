package eu.nifti.dialogue.actions;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;

public class MoveToObjectAction extends AbstractMovementAction {

	private final WorkingMemoryAddress ooiAddress;

	public MoveToObjectAction(WorkingMemoryAddress ooiAddress) {
		this.ooiAddress = ooiAddress;
	}

	@Override
	public void setContent(InterpretedIntention iint) {
		super.setContent(iint);

                iint.stringContent.put("subtype", "to-target");
                iint.stringContent.put("target", "object=" + ooiAddress.subarchitecture + "," + ooiAddress.id);

//		iint.stringContent.put("subtype", "to-object");
//		iint.addressContent.put("object-id", ooiAddress);
	}
	
}
