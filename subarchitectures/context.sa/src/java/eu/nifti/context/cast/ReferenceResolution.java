package eu.nifti.context.cast;

import cast.SubarchitectureComponentException;
import java.util.Map;

import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import de.dfki.lt.tr.cast.dialogue.AbstractReferenceResolutionComponent;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionResult;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolver;
import de.dfki.lt.tr.dialogue.ref.util.ReferenceUtils;
import de.dfki.lt.tr.dialogue.time.TimeConversions;
import de.dfki.lt.tr.dialogue.util.BeliefIntentionUtils;
import eu.nifti.context.cast.ReferenceResolution.GUIResolver;
import eu.nifti.context.ref.Cache;
import eu.nifti.context.ref.FunctionPotentialCombinator;
import eu.nifti.context.ref.SelectionPotentialGenerator;
import eu.nifti.context.ref.IntervalSearch;
import eu.nifti.context.ref.MapObjectPotentialGenerator;
import eu.nifti.context.ref.MapPotential;
import eu.nifti.context.ref.Potential;
import eu.nifti.context.ref.PotentialCombinator;
import eu.nifti.context.ref.PotentialNormalisation;
import eu.nifti.context.ref.Referent;
import eu.nifti.context.ref.ResolutionResultFactory;
import eu.nifti.context.ref.ScoreFunction;
import eu.nifti.context.ref.StringReferent;
import eu.nifti.context.ref.WorkingMemoryPointerReferent;
import eu.nifti.gui.GUISelection;
import eu.nifti.mapping.MapObject;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map.Entry;

/**
 * This is a simple monitor that listens for MapObjects on WM.
 * It then decides whether and how to represent them in the ontology,
 * triggers the reasoner, and performs necessary updated on the 
 * MapObject WME. 
 * 
 * @author zender
 * CAST file arguments:
 * --hfcserver-name String with the name of the HFC server component
 */
public class ReferenceResolution extends AbstractReferenceResolutionComponent<GUIResolver> {

	private final Map<WorkingMemoryAddress, Integer> wma_to_int = new HashMap<WorkingMemoryAddress, Integer>();
	private final IntervalSearch<Referent> selections = new IntervalSearch<Referent>();
	private final Cache<WorkingMemoryAddress, MapObject> mapctx_cache = new Cache<WorkingMemoryAddress, MapObject>();

	private SelectionPotentialGenerator ref_selections = null;
	private MapObjectPotentialGenerator ref_mapctx = null;

	@Override
	public void onConfigure(Map<String, String> args) {
		super.onConfigure(args);

		ref_selections = new SelectionPotentialGenerator(this.getLogger(".rr-select"), selections);
		ref_mapctx = new MapObjectPotentialGenerator(this.getLogger(".rr-map"), mapctx_cache);
	}

	@Override
	public void onStart() {
		super.onStart();

		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(GUISelection.class,  WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleGUISelectionAdd(_wmc);
					}
				});

		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(GUISelection.class,  WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleGUISelectionOverwrite(_wmc);
					}
				});
/*
		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(MapObject.class,  WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleMapObjectAdd(_wmc);
					}
				});

		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(MapObject.class,  WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleMapObjectOverwrite(_wmc);
					}
				});

		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(MapObject.class,  WorkingMemoryOperation.DELETE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleMapObjectDelete(_wmc);
					}
				});
*/
	}
	
	@Override
	protected GUIResolver initResolver() {
		return new GUIResolver();
	}

	private void handleGUISelectionAdd(WorkingMemoryChange _wmc) {
		try {
			GUISelection sel = getMemoryEntry(_wmc.address, GUISelection.class);
			WorkingMemoryPointer wmp = sel.selectedEntityWMP;

			// int id = selections.newInterval(TimeConversions.secDoubleToMillisLong(sel.startTime), new WorkingMemoryPointerReferent(this, wmp));
			long timePoint = sel.startTimeMSec;
			Referent ref = new WorkingMemoryPointerReferent(this, wmp);
			getLogger().debug("this is the WMP I got: " + wmaToString(wmp.address) + ", type=" + wmp.type);
			int id = selections.newInterval(timePoint, ref);
			getLogger().debug("opened interval [" + id + "]: start is set to " + portrayTimePoint(timePoint) + "; referent=" + BeliefIntentionUtils.dFormulaToString(ref.toFormula()));
			wma_to_int.put(_wmc.address, new Integer(id));
			log("handleGUISelectionAdd succesfully processed.");
		}
		catch (SubarchitectureComponentException ex) {
			logException(ex);
		}
	}

	private void handleGUISelectionOverwrite(WorkingMemoryChange _wmc) {
		try {
			GUISelection sel = getMemoryEntry(_wmc.address, GUISelection.class);

			int id = wma_to_int.get(_wmc.address);
			long timePoint = sel.endTimeMSec;
			selections.closeInterval(id, timePoint);
			getLogger().debug("closed interval [" + id + "]: end is set to " + portrayTimePoint(timePoint));
			log("handleGUISelectionOverwrite succesfully processed.");
		}
		catch (SubarchitectureComponentException ex) {
			logException(ex);
		}
	}

	public static String portrayTimePoint(long tp) {
		long now = System.currentTimeMillis();
		return tp + ", which is " + (now - tp) + " ms ago";
	}

	private void handleMapObjectAdd(WorkingMemoryChange _wmc) {
		try {
			MapObject mo = getMemoryEntry(_wmc.address, MapObject.class);
			mapctx_cache.add(_wmc.address, mo);
		}
		catch (SubarchitectureComponentException ex) {
			logException(ex);
		}
	}

	private void handleMapObjectOverwrite(WorkingMemoryChange _wmc) {
		try {
			MapObject mo = getMemoryEntry(_wmc.address, MapObject.class);
			mapctx_cache.overwrite(_wmc.address, mo);
		}
		catch (SubarchitectureComponentException ex) {
			logException(ex);
		}
	}

	private void handleMapObjectDelete(WorkingMemoryChange _wmc) {
		mapctx_cache.delete(_wmc.address);
	}

	void boostSingleHypothesis(Potential p) {
		Iterator<Entry<Referent, Double>> iter = p.positiveElementsIterator();
		if (iter.hasNext()) {
			Entry<Referent, Double> e = iter.next();
			if (!iter.hasNext()) {
				p.setScore(e.getKey(), 1.0);
			}
		}
	}

	public class GUIResolver implements ReferenceResolver {

		@Override
		public ReferenceResolutionResult resolve(ReferenceResolutionRequest rr, WorkingMemoryAddress origin) {

				Potential pot_sel = ref_selections.getHypos(rr);  // normalised by mass
				Potential pot_restrict = ref_mapctx.getHypos(rr);  // binary
				Potential pot = new MapPotential();

				if (pot_sel.getMaxScore() > 0.0 && pot_restrict.getMaxScore() == 0.0) {
					// just selection, NO restrictors
					pot = pot_sel;  // already normalised
				}
				else if (pot_sel.getMaxScore() == 0.0 && pot_restrict.getMaxScore() == 0.0) {
					// no selections, no restrictors
				}
				else {
					// restrictors, possibly selections
					PotentialCombinator combine = new FunctionPotentialCombinator(new ScoreFunction() {
							@Override
							public double apply(double content, double select, double max_content, double max_select) {
								// supposing select is normalised
								// i'm assuming that we do have selections and we do have restrictors
								return content * (1.0 + (2.0 * select));
							}
						}, pot_restrict);
					combine.addPotential(pot_sel);
					pot = combine.toPotential();

					pot = PotentialNormalisation.absNormalisedPotential(pot, 3.0);
					boostSingleHypothesis(pot);
				}

				// todo add situated context

				log(pot.asMap().size() + " items in the final potential");
				return ResolutionResultFactory.potentialToResolutionResult(rr.nom, origin, pot);

		}
		
	}
	
}
