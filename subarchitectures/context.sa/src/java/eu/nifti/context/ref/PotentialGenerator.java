package eu.nifti.context.ref;

import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;

public interface PotentialGenerator {

	Potential getHypos(ReferenceResolutionRequest rr);

}
