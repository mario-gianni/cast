package eu.nifti.planning.cast;

import cast.AlreadyExistsOnWMException;
import cast.architecture.ManagedComponent;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.SubarchitectureComponentException;
import cast.cdl.FilterRestriction;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryAddress;
import java.util.Map;
import java.util.LinkedList;
import de.dfki.lt.tr.dialogue.slice.StandbyMode;
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.SharedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionalContent;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import eu.nifti.planning.slice.Action;
import eu.nifti.planning.slice.Completion;
import eu.nifti.planning.slice.PlanningTask;

/**
 * A simple intention-to-planning-task generator. Listens for attributed
 * intentions, and in case it is able to translate them to a PlanningTask,
 * it does so, and at the same time updates the attributed intention to
 * a shared one (so that dialogue.sa can verbalise the adoption of the
 * task).
 *
 * <b>FIXME</b>: this bypasses the task manager!
 *
 * @author Miroslav Janicek
 */
public class IntentionTaskGenerator extends ManagedComponent {

	public static final String thisAgent = "self";
	public static final String humanAgent = "human";

	// listen on your own SA by default
	private String listenSA = this.getSubarchitectureID();

	/**
	 * Initialise the class.
	 */
	public IntentionTaskGenerator() {
		super();
	}

	/**
	 * Configure the component. The configuration arguements are typically
	 * specified in the CAST file.
	 *
	 * @param _config  key-value pairs
	 */
	@Override
	protected void configure(Map<String, String> _config) {
		super.configure(_config);
		if (_config.containsKey("--listen-sa")) {
			listenSA = _config.get("--listen-sa");
		}
	}

	/**
	 * Start the component. This method is automatically called by the
	 * CAST server after configure().
	 */
	@Override
	protected void start() {
		super.start();
		log("registering change filters");

		addChangeFilter(ChangeFilterFactory.createChangeFilter(Intention.class,
					WorkingMemoryOperation.ADD, "", "", listenSA,
					FilterRestriction.ALLSA),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleNewIntention(_wmc);
					}
				});
	}

	/**
	 * Run the component.
	 */
	@Override
	protected void runComponent() {
		super.runComponent();
	}

	/**
	 * Handle a new intention. Decide whether the intention in question
	 * is an attributed intention to the human, and if it is, pass it
	 * on for further processing.
	 *
	 * @param _wmc  the working memory change event
	 */
	private void handleNewIntention(WorkingMemoryChange _wmc) {
		try {
			Intention it = getMemoryEntry(_wmc.address, Intention.class);

			// check whether it's an attributed intention of the human
			if (it.estatus instanceof AttributedEpistemicStatus) {
				if (isHumansIntention(it)) {
					processAttributedIntention(_wmc.address, it);
				}
			}

		}
		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Check whether the given intention is an intention of the human.
	 *
	 * @param it  the intention
	 * @return <tt>true</tt> iff the intention is human's
	 */
	private boolean isHumansIntention(Intention it) {
		if (it.content.size() == 1) {
			IntentionalContent itc = it.content.get(0);
			if (itc.agents.size() == 1 && itc.agents.get(0).equals("human")) {
				return true;
			}
		}
		return false;
	}

	/**
	 * Examine an intention and try to convert it to a PlanningTask.
	 * If the intention cannot be converted to a PlanningTask, it is ignored.
	 * Otherwise, it is converted to the task, which is written to the WM.
	 * The intention is then overwritten with shared epistemic status so that
	 * dialogue can verbalise this.
	 *
	 * @param wma  working memory address of the intention
	 * @param it  the intention
	 */
	private void processAttributedIntention(WorkingMemoryAddress wma, Intention it) {
		log("got an attributed intention, will look at it");

		String goalString = null;
		IntentionalContent itc = null;
		boolean switchToStandbyMode = false;

		if (it.content.size() == 1) {
			itc = it.content.get(0);
		}
		else {
			return;
		}

		LinkedList<String> agents = new LinkedList<String>();
		if (it.estatus instanceof AttributedEpistemicStatus) {
			AttributedEpistemicStatus attribEpst = (AttributedEpistemicStatus) it.estatus;
			agents.add(attribEpst.agent);
			agents.addAll(attribEpst.attribagents);
		}
		else {
			return;
		}

		// extract the command (FIXME: this is very ugly & not systematic!)
		int taskID = -1;
		if (itc.postconditions instanceof ComplexFormula) {
			ComplexFormula cF = (ComplexFormula) itc.postconditions;
			if (cF.forms.size() != 1) return;
			if (cF.forms.get(0) instanceof ModalFormula) {
				ModalFormula mF = (ModalFormula) cF.forms.get(0);
				if (!mF.op.equals("state")) return;
				if (mF.form instanceof ComplexFormula) {
					ComplexFormula stateF = (ComplexFormula) mF.form;
					if (stateF.forms.size() < 1) return;
					if (!(stateF.forms.get(0) instanceof ElementaryFormula)) return;
					String actionString = ((ElementaryFormula) stateF.forms.get(0)).prop;

					if (actionString.equals("in-motion") && stateF.forms.size() == 3) {
						if (stateF.forms.get(2) instanceof ModalFormula) {
							ModalFormula argF = (ModalFormula) stateF.forms.get(2);
							if (argF.op.equals("direction-spatial")) {
								if (!(argF.form instanceof ElementaryFormula)) return;
								String argString = ((ElementaryFormula) argF.form).prop;

								String xytheta;
								if (argString.equals("left"))    { xytheta = "1.0,1.0,"+ String.valueOf(Math.PI/4.0); }
								else if (argString.equals("right"))   { xytheta = "1.0,-1.0,-"+ String.valueOf(Math.PI/4.0); } //Math.PI * (7.0/4.0)
								else if (argString.equals("forward")) { xytheta = "2.0,0.0,0.0"; }
								else if (argString.equals("back"))    { xytheta = "-1.0,0.0,0.0"; } // turn or not to turn? + String.valueOf(Math.PI); }
								else { xytheta = "0.0,0.0,0.0"; }
								goalString = "move_base(" + xytheta + ")";
								taskID = 4;
							}
						}
					}
					if (actionString.equals("in-location") && stateF.forms.size() == 3) {
						if (stateF.forms.get(2) instanceof ModalFormula) {
							ModalFormula argF = (ModalFormula) stateF.forms.get(2);

							if (argF.op.equals("location")) {
								if (argF.form instanceof ModalFormula) {
									ModalFormula targetF = (ModalFormula) argF.form;

									if (targetF.op.equals("coordinates")) {
										if (!(targetF.form instanceof ElementaryFormula)) return;
										String argString = ((ElementaryFormula) targetF.form).prop;
										// goalString = "go-to-position(" + argString + ")";
										String[] xyzargs = argString.split(",");
							            String x = xyzargs[0];
							            String y = xyzargs[1];
							            //	String z = "0"; // todo: handle 3D z-coordinates!
							            // Pose3D targetPose = new Pose3D(Float.parseFloat(x), Float.parseFloat(y), 0, Float.parseFloat(z)); // todo: ensure theta is handled!
							            goalString="goto("+ x + "," + y + ")";
							            taskID = 5;
									}
									else if (targetF.op.equals("landmark")) {
										if (!(targetF.form instanceof ElementaryFormula)) return;
										String argString = ((ElementaryFormula) targetF.form).prop;
										// goalString = "go-to-landmark(" + argString + ")";
										// visitProp(car:0)
										String[] parts = argString.split(":");
										// assuming here that parts.size == 2
										if (parts.length == 2) {
											goalString = "visit_prop(" + parts[0] + "," + parts[1] + ")";
											taskID = 6;
										}
										else {
											log("WARN: got wrong landmark string: \"" + argString + "\"");
										}
									}
								}
							}
						}
					}
					if (actionString.equals("in-orientation") && stateF.forms.size() == 3) {
						if (stateF.forms.get(2) instanceof ModalFormula) {
							ModalFormula argF = (ModalFormula) stateF.forms.get(2);
							if (argF.op.equals("direction-spatial")) {
								if (!(argF.form instanceof ElementaryFormula)) return;
								String argString = ((ElementaryFormula) argF.form).prop;
								
								String xytheta;
								if (argString.equals("left"))    { xytheta = "0.0,0.0," + String.valueOf(Math.PI/4.0); }
								else if (argString.equals("right"))   { xytheta = "0.0,0.0,-" + String.valueOf(Math.PI/4.0); } //Math.PI * (11.0/6.0)
								else if (argString.equals("forward")) { xytheta = "0.0,0.0,0.0"; }
								else if (argString.equals("back"))    { xytheta = "0.0,0.0," + String.valueOf(Math.PI); }
								else { xytheta = "0.0,0.0,0.0"; }
								goalString = "move_base(" + xytheta + ")";
								taskID = 4;
							}
						}
					}
/*
					if (actionString.equals("looking") && stateF.forms.size() == 3) {
						if (stateF.forms.get(2) instanceof ModalFormula) {
							ModalFormula argF = (ModalFormula) stateF.forms.get(2);
							if (argF.op.equals("direction-visual")) {
								if (!(argF.form instanceof ElementaryFormula)) return;
								String argString = ((ElementaryFormula) argF.form).prop;

								if (argString.equals("left"))    { goalString = "look-left"; }
								if (argString.equals("right"))   { goalString = "look-right"; }
								if (argString.equals("up"))      { goalString = "look-up"; }
								if (argString.equals("down"))    { goalString = "look-down"; }
							}
						}
					}
*/
					if (actionString.equals("execution-stopped") && stateF.forms.size() == 2) {
						goalString = "stop";
					}
/*
					if (actionString.equals("execution-on-hold") && stateF.forms.size() == 2) {
						goalString = "pause";
					}
					if (actionString.equals("execution-resumed") && stateF.forms.size() == 2) {
						goalString = "continue";
					}
*/
					if (actionString.equals("dialogue-on-hold") && stateF.forms.size() == 2) {
						goalString = "stop";
						switchToStandbyMode = true;
					}
					if (actionString.equals("returned-to-base") && stateF.forms.size() == 2) {
						goalString = "come_back";
						taskID = 7;
					}
				}
			}
		}

		boolean notify = false;

		if (switchToStandbyMode) {
			log("switching to standby mode");
			StandbyMode sm = new StandbyMode();
			log("writing a standby mode signal to the local working memory");
			try {
				addToWorkingMemory(newDataID(), sm);
			}
			catch (AlreadyExistsOnWMException ex) {
				ex.printStackTrace();
			}

			// do notify the user
			notify = true;
		}

		if (goalString == null) {
			log("no work for the planner");
		}
		else {
			// write the task to the WM
			PlanningTask task = new PlanningTask(taskID, goalString, new Action[0], "", Completion.PENDING, 0, Completion.PENDING, 0);

			log("writing planning task \"" + goalString + "\" to the WM");
			try {
				addToWorkingMemory(newDataID(), task);
			}
			catch (AlreadyExistsOnWMException ex) {
				ex.printStackTrace();
			}

			// do notify the user
			notify = true;
		}

		if (notify) {
			// upgrade the intention to a shared one and write it back
			// where it came from
			SharedEpistemicStatus sharedEpst = new SharedEpistemicStatus(agents);
			it.estatus = sharedEpst;
			itc.agents = new LinkedList<String>();
			itc.agents.add(thisAgent);
			log("updating the intention to shared, making it the robot's private intention");
			try {
				overwriteWorkingMemory(wma, it);
			}
			catch (SubarchitectureComponentException ex) {
				ex.printStackTrace();
			}
		}
	}

}
