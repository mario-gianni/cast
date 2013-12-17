package eu.nifti.context.ref;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.dialogue.ref.Constraint;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import eu.nifti.mapping.CarObject;
import eu.nifti.mapping.MapObject;
import java.text.ParseException;
import org.apache.log4j.Logger;

public class MapObjectPotentialGenerator
implements PotentialGenerator {

	private Logger logger = null;
	private Cache<WorkingMemoryAddress, MapObject> cache = null;

	public MapObjectPotentialGenerator(Logger logger, Cache<WorkingMemoryAddress, MapObject> cache) {
		this.logger = logger;
		this.cache = cache;
	}

	@Override
	public Potential getHypos(ReferenceResolutionRequest rr) {
		logger.warn("the potential generator is not temporally-restricted, operates on here-and-now");

		Potential pot = new MapPotential();

		String type = extractType(rr);
		if (type != null) {
			logger.debug("will sift through the map objects, looking for objects of type \"" + type + "\" here");
			for (MapObject mo : cache.asMap().values()) {
				if (type.equals("car") && mo instanceof CarObject) {
					CarObject co = (CarObject) mo;
					logger.debug("got a car: " + mo.label);

					try {
						pot.setScore(new StringReferent(mo.label), 1.0);
					}
					catch (ParseException e) {
						logger.error("hmm, failed to parse the label \"" + mo.label + "\" to a referent");
					}
				}
			}
		}

//		pot = PotentialNormalisation.massNormalisedPotential(pot);

		logger.debug("got " + pot.asMap().size() + " items in the potential");
		return pot;
	}

	public static String extractType(ReferenceResolutionRequest rr) {
		String s = null;
		for (Constraint c : rr.constraints) {
			if (c.feature.equals("Type")) {
				return c.value;
			}
		}
		return s;
	}

}
