package de.dfki.lt.tr.cast.dialogue;

import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

import cast.AlreadyExistsOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPointer;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.framing.TemporalInterval;
import de.dfki.lt.tr.beliefs.slice.framing.SpatioTemporalFrame;
import de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.FloatFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

// FIXME: this needs to be factored out of dialogue.sa !
public class FakeCarCreator extends ManagedComponent{ 

	/**
	 * String specifying the belief type for visual objects.
	 */
	public static final String BELIEF_TYPE_VISUALOBJECT = "visualobject";

	/**
	 * Label used in belief content to mark type of objects (car, barrel, ...).
	 */
	public static final String OBJECTTYPE_LABEL = "objecttype";

	/**
	 * Label used in belief content to mark the type (make) of a car.
	 */
	public static final String CARTYPE_LABEL = "cartype";

	/**
	 * Create a new (visual) belief that there is a car.
	 *
	 * The car is referenced by an anchor in the map (<tt>placeId</tt>), and has
	 * an existence probability (<tt>existProb</tt>), i.e. "how sure the system
	 * is that the <i>existence</i> of the belief is mandated.
	 *
	 * The make of the car will be uncertain in the belief: it is either a Citroen
	 * C4 (with probability 0.4), or Ford Focus (with probability 0.3), or unknown
	 * (with probability 0.3).
	 *
	 * @param existProb existence probability of the belief
	 * @param placeId string identifier anchoring the belief in the map
	 * @return the corresponding belief
	 */
	public dBelief newFakeCarBelief(float existProb, String placeId) {
		SpatioTemporalFrame stf = new SpatioTemporalFrame(placeId, new TemporalInterval(), existProb);
		EpistemicStatus status = new PrivateEpistemicStatus("self");

		BasicProbDistribution typeDistrib = new BasicProbDistribution(OBJECTTYPE_LABEL, new FormulaValues(new LinkedList<FormulaProbPair>()));
		((FormulaValues)typeDistrib.values).values.add(new FormulaProbPair(new ElementaryFormula(0, "car"), 1.0f));

		BasicProbDistribution carTypeDistrib = new BasicProbDistribution(CARTYPE_LABEL, new FormulaValues(new LinkedList<FormulaProbPair>()));
		((FormulaValues)typeDistrib.values).values.add(new FormulaProbPair(new ElementaryFormula(0, "citroen-c4"), 0.4f));
		((FormulaValues)typeDistrib.values).values.add(new FormulaProbPair(new ElementaryFormula(0, "ford-focus"), 0.3f));
		((FormulaValues)typeDistrib.values).values.add(new FormulaProbPair(new ElementaryFormula(0, "UNKNOWN"), 0.3f));

		CondIndependentDistribs content = new CondIndependentDistribs(new HashMap<String,ProbDistribution>());
		content.distribs.put(OBJECTTYPE_LABEL, typeDistrib);
		content.distribs.put(CARTYPE_LABEL, carTypeDistrib);

		CASTBeliefHistory history = new CASTBeliefHistory(new LinkedList<WorkingMemoryPointer>(), new LinkedList<WorkingMemoryPointer>());
		
		return new dBelief (stf, status, newDataID(), BELIEF_TYPE_VISUALOBJECT, content, history); 
	}

	@Override
	public void runComponent() {

		dBelief b = newFakeCarBelief(0.8f, "place-identifier");

		try {
			addToWorkingMemory(b.id, "dialogue", b);
		}
		catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		}
		catch (UnknownSubarchitectureException e) {
			e.printStackTrace();
		}
	}
}
