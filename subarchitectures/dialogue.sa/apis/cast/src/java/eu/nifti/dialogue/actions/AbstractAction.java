package eu.nifti.dialogue.actions;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.ProcessingState;
import java.util.HashMap;
import java.util.Map;

public abstract class AbstractAction implements Action {

	@Override
	public InterpretedIntention toIntention() {
		Map<String, String> stringContent = new HashMap<String, String>();
		Map<String, WorkingMemoryAddress> addressContent = new HashMap<String, WorkingMemoryAddress>();

		InterpretedIntention iint = new InterpretedIntention(stringContent, addressContent, ProcessingState.READY, "human", 1.0f);
		
		setContent(iint);

		return iint;
	}

	public abstract void setContent(InterpretedIntention iint);
	
}
