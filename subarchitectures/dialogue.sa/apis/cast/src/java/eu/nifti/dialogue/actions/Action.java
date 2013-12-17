package eu.nifti.dialogue.actions;

import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;

public interface Action {

	public InterpretedIntention toIntention();
	
}
