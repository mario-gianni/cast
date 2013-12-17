package eu.nifti.planning.cast;

import java.util.Map;

import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import eu.nifti.Planning.slice.Brake;
import eu.nifti.Planning.slice.DetectGapTraversableTask;
import eu.nifti.Planning.slice.DifferentialAction;
import eu.nifti.Planning.slice.FailurePlanAction;
import eu.nifti.Planning.slice.FlipperAction;
import eu.nifti.Planning.slice.GUITask;
import eu.nifti.Planning.slice.GoHomeTask;
import eu.nifti.Planning.slice.Motion;
import eu.nifti.Planning.slice.MoveBaseTask;
import eu.nifti.Planning.slice.State;
import eu.nifti.Planning.slice.Task;
import eu.nifti.Planning.slice.TraverseGapTask;
import eu.nifti.Planning.slice.VantagePointTask;
import eu.nifti.gui.GUISelection;
import de.dfki.lt.tr.cast.dialogue.util.VerbalisationUtils;

public class IntentionToRomaPlanningTaskGenerator extends ManagedComponent {

	private String m_plannerSA = "PlanningROMA";
	
	protected void configure(Map<String, String> _config) {
		super.configure(_config);
		// init roma planner sa name
		if (_config.containsKey("--planner_sa")) {
			m_plannerSA = _config.get("--planner_sa");
		}
	}

	protected void start() {
		super.start();
		// add changefilter for InterpretedIntention
		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(
						InterpretedIntention.class, 
						WorkingMemoryOperation.ADD),
			new WorkingMemoryChangeReceiver() {
				@Override
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					handleNewIntention(_wmc);
				}
			});
		
		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(
						GUISelection.class,
						WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleNewGUISelection(_wmc);
					}
				});
		
		
		
	}

	
	protected void runComponent() {
		super.runComponent();
		// probably nothing needs to be done!
	}

	protected Task lastTask = null;
	

	private void handleNewGUISelection(WorkingMemoryChange wmc) {
		GUISelection selectedLocation = null; 
		try {
			selectedLocation = getMemoryEntry(wmc.address, GUISelection.class);
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		if (selectedLocation==null) {
			log("Error: got a null GUISelection. Returning.");
			return;
		}
		log("received new GUISelection with selectedEntityWMP = " + selectedLocation.selectedEntityWMP);
	}
			
	private void handleNewIntention(WorkingMemoryChange wmc) {
		InterpretedIntention receivedIntention = null; 
		try {
			receivedIntention = getMemoryEntry(wmc.address, InterpretedIntention.class);			
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		if (receivedIntention==null) {
			log("Error: got a null intention. Returning.");
			return;
		}
		
		
		
		String userIntention = wmc.address.toString();
		String type = receivedIntention.stringContent.get("type");
		String subtype = receivedIntention.stringContent.get("subtype");
		
		if ("movement".equals(type) && "to-target".equals(subtype)) {
                        // TODO: distinguish the two cases
                        publishToTargetTask(receivedIntention, userIntention);
		} else if ("question".equals(type) && "ability-check".equals(subtype)){
			if ("gap-traversal".equals(receivedIntention.stringContent.get("ability"))) {
				log("ask for GAP TRAVERSABILITY");
				publishAskGapTraversable(receivedIntention, userIntention);
			}
		} else if("gap-traversal-mode".equals(type)) {
			log("initiate GAP TRAVERSAL");
			publishInitGapTraversal(receivedIntention, userIntention);
		} else if ("movement".equals(type) && "return-to-base".equals(subtype)) {
			log("initiate RETURN TO BASE");
			publishReturnToBase(receivedIntention, userIntention);
		} else if ("movement".equals(type) && "in-direction".equals(subtype)) {
			if ("FORWARD".equals(receivedIntention.stringContent.get("move-direction"))) {
				log("MOVE FORWARD!");
				publishMoveCommand(receivedIntention, userIntention, Motion.MOVEFORWARD);
			} else if ("BACKWARD".equals(receivedIntention.stringContent.get("move-direction"))) {
				log("MOVE BACKWARD!");
				publishMoveCommand(receivedIntention, userIntention, Motion.MOVEBACK);
			} else if ("LEFT".equals(receivedIntention.stringContent.get("move-direction"))) {
				log("MOVE LEFT!");
				publishMoveCommand(receivedIntention, userIntention, Motion.MOVELEFT);
			} else if ("RIGHT".equals(receivedIntention.stringContent.get("move-direction"))) {
				log("MOVE RIGHT!");
				publishMoveCommand(receivedIntention, userIntention, Motion.MOVERIGHT);
			}
		} else if ("turning".equals(type) && "in-direction".equals(subtype)) {
			if ("LEFT".equals(receivedIntention.stringContent.get("turn-direction"))) {
				log("TURN LEFT!");
				publishTurnCommand(receivedIntention, userIntention, Motion.TURNLEFT);
			} else if ("RIGHT".equals(receivedIntention.stringContent.get("turn-direction"))) {
				log("TURN RIGHT!");
				publishTurnCommand(receivedIntention, userIntention, Motion.TURNRIGHT);
			}   
		} else if ("continue".equals(type)) {
			log("got CONTINUE command");
			republishLastTask();
		} else if ("stop".equals(type)) {
			log("issue STOP COMMAND!");
			publishStopAction(receivedIntention, userIntention);
		} else {
			log("intention does not have a required type. Not doing anything");
			return;
		}
		
	}
	
	private void publishStopAction(InterpretedIntention receivedIntention,
			String userIntention) {
		String wmid = newDataID();
		FailurePlanAction stopAction = new FailurePlanAction("stop", "", 0, State.PENDING);
		log("generated a FailurePlanAction for the planner to STOP -- going to add it to " + m_plannerSA);
		try {
			addToWorkingMemory(new WorkingMemoryAddress(wmid, m_plannerSA), stopAction);
		} catch (AlreadyExistsOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			log("could not write FailurePlanAction to WM!");
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			log("could not write FailurePlanAction to WM!");
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			log("could not write FailurePlanAction to WM!");
		}	
	}

	private void publishTurnCommand(InterpretedIntention receivedIntention,
			String userIntention, Motion direction) {
		String wmid = newDataID();
		MoveBaseTask moveTask = new MoveBaseTask(wmid,  
				userIntention, State.NEW, direction); 
		log("generated a MoveBaseTask with direction to turn " + direction.name() + " for the planner -- going to add it to " + m_plannerSA);
		try {
			addToWorkingMemory(new WorkingMemoryAddress(wmid, m_plannerSA), moveTask);
			lastTask = moveTask;
		} catch (AlreadyExistsOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			log("could not write MoveBaseTask to WM!");
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			log("could not write MoveBaseTask to WM!");
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			log("could not write MoveBaseTask to WM!");
		}		
	}

	private void publishMoveCommand(InterpretedIntention receivedIntention,
			String userIntention, Motion direction) {
		String wmid = newDataID();
		MoveBaseTask moveTask = new MoveBaseTask(wmid,  
				userIntention, State.NEW, direction); 
		log("generated a MoveBaseTask with direction " + direction.name() + " for the planner -- going to add it to " + m_plannerSA);
		try {
			addToWorkingMemory(new WorkingMemoryAddress(wmid, m_plannerSA), moveTask);
			lastTask = moveTask;
		} catch (AlreadyExistsOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			log("could not write MoveBaseTask to WM!");
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			log("could not write MoveBaseTask to WM!");
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			log("could not write MoveBaseTask to WM!");
		}	
	}

	private void publishReturnToBase(InterpretedIntention receivedIntention,
			String userIntention) {
		String wmid = newDataID();
		GoHomeTask returnTask = new GoHomeTask(wmid,  
				userIntention, State.NEW, "name"); 
		log("generated a GoHomeTask for the planner -- going to add it to " + m_plannerSA);
		try {
			addToWorkingMemory(new WorkingMemoryAddress(wmid, m_plannerSA), returnTask);
			lastTask = returnTask;
		} catch (AlreadyExistsOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			log("could not write GoHomeTask to WM!");
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			log("could not write GoHomeTask to WM!");
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			log("could not write GoHomeTask to WM!");
		}			
	}

	private void publishInitGapTraversal(InterpretedIntention receivedIntention, String userIntention) {
		String wmid = newDataID();
		TraverseGapTask travTask = new TraverseGapTask(wmid,  
				userIntention, State.NEW, "name"); 
		log("generated a TraverseGapTask for the planner -- going to add it to " + m_plannerSA);
		try {
			addToWorkingMemory(new WorkingMemoryAddress(wmid, m_plannerSA), travTask);
			lastTask = travTask;
		} catch (AlreadyExistsOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			log("could not write TraverseGapTask to WM!");
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			log("could not write TraverseGapTask to WM!");
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			log("could not write TraverseGapTask to WM!");
		}	
	}

	private void publishAskGapTraversable(
			InterpretedIntention receivedIntention, String userIntention) {
		String wmid = newDataID();
		DetectGapTraversableTask travTask = new DetectGapTraversableTask(wmid,  
				userIntention, State.NEW, "name"); 
		log("generated a DetectGapTraversableTask for the planner -- going to add it to " + m_plannerSA);
		try {
			addToWorkingMemory(new WorkingMemoryAddress(wmid, m_plannerSA), travTask);
			lastTask = travTask;
		} catch (AlreadyExistsOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			log("could not write DetectGapTraversableTask to WM!");
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			log("could not write DetectGapTraversableTask to WM!");
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			log("could not write DetectGapTraversableTask to WM!");
		}	
	}

        private void publishToTargetTask(InterpretedIntention receivedIntention, String userIntention) {

            String full = receivedIntention.stringContent.get("target");
            String[] parts = full.split("=");
            
            assert (parts.length == 2);
            if (parts.length != 2) {
                getLogger().error("couldn't decode target string \"" + full + "\": wrong number of parts");
            }
            else {
                if ("coordinates".equals(parts[0])) {
                    String[] xyargs = parts[1].split(",");
                    if (xyargs.length == 3) {
                        double x = Double.parseDouble(xyargs[0]);
                        double y = Double.parseDouble(xyargs[1]);
                        double z = Double.parseDouble(xyargs[2]);

                        publishGUITask(receivedIntention, userIntention, x, y);
                    }
                    else {
                        getLogger().error("couldn't decode target string \"" + full + "\": wrong number of xyz args");
                    }
                }
                else if  ("object".equals(parts[0])) {
                    String[] wmaargs = parts[1].split(",");
                    if (wmaargs.length == 2) {
                        String subarch = wmaargs[0];
                        String id = wmaargs[1];

                        publishVantagePointTask(receivedIntention, userIntention, new WorkingMemoryAddress(id, subarch));
                    }

                }
                else {
                    getLogger().error("couldn't decode target string \"" + full + "\": unknown type");
                }
            }
        }
        
	private void publishGUITask(InterpretedIntention receivedIntention, String userIntention, double x, double y) {
		String wmid = newDataID();
		GUITask gotoTask = new GUITask(wmid, 
				userIntention, State.NEW, 
				x, y);
		log("generated a GUITask [" + gotoTask.x +"," + gotoTask.y + "] for the planner -- going to add it to " + m_plannerSA);
		try {
			addToWorkingMemory(new WorkingMemoryAddress(wmid, m_plannerSA), gotoTask);
			lastTask = gotoTask;
		} catch (AlreadyExistsOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			log("could not write GUITask to WM!");
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			log("could not write GUITask to WM!");
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			log("could not write GUITask to WM!");
		}
	}
	
	private void publishVantagePointTask(InterpretedIntention receivedIntention, String userIntention, WorkingMemoryAddress objectWma) {
		String wmid = newDataID();
		VantagePointTask vpTask = new VantagePointTask(wmid, 
				userIntention, State.NEW, 
				objectWma);
		log("generated a VantagePointTask [" + CASTUtils.toString(vpTask.carWMA) + "] for the planner -- going to add it to " + m_plannerSA);
		try {
			addToWorkingMemory(new WorkingMemoryAddress(wmid, m_plannerSA), vpTask);
			lastTask = vpTask;
		} catch (AlreadyExistsOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	private void republishLastTask() {
		log("about to republishing the last non-STOP task");
		Task t = lastTask;
		if (t != null) {
			log("task type = " + t.getClass().getCanonicalName());
			try {
				addToWorkingMemory(new WorkingMemoryAddress(newDataID(), m_plannerSA), lastTask);
				lastTask = null;
			}
			catch (SubarchitectureComponentException ex) {
				logException(ex);
			}
		}
		else {
			log("no eligible task");
			VerbalisationUtils.verbaliseString(this, "Sorry, I don't know what I should continue doing.");
		}
	}
	
	private void otherActions(String userIntention) {
		String wmid = newDataID();
		
		// DifferentialAction
		// what do the following parameters denote?
		// - String component
		// - double time
		// - Brake flag (ON vs OFF)
		DifferentialAction daOff = 
			new DifferentialAction(wmid, this.getComponentID(), now(), State.NEW, Brake.OFF);
		DifferentialAction daOn =  
			new DifferentialAction(wmid, this.getComponentID(), now(), State.NEW, Brake.ON);
		// LOCK DIFFERENTIAL
		// UNLOCK DIFFERENTIAL
		
		// FlipperAction
		// what does alfa denote?
		double alfa = 0.0;
		FlipperAction fa1 = 
			new FlipperAction(wmid, this.getComponentID(), now(), State.NEW, alfa);
		// FLIPPER PRESETS 1--5
		// RESET FLIPPERS
		
		// MoveBaseTask
		double x = 0.0, y = 0.0, theta = 0.0;
		//MoveBaseTask mb = new MoveBaseTask(wmid, userIntention, State.NEW, x, y, theta);
		// MOVE FORWARD, BACKWARD, LEFT, RIGHT
		// TURN LEFT, RIGHT, AROUND
		
		// MISSING TASKS?
		// - STOP
		// - EXPLORE
		// - TRAVERSE GAP
		// - RETURN TO BASE -> GoToNodeAction with node0?
		
		// for STOP
		FailurePlanAction fpa = new FailurePlanAction(wmid, this.getComponentID(), now(), State.NEW);
	}
	
	private long now() {
		return (long) System.currentTimeMillis();
	}
	
/*
  		String wmid = newDataID();
		GUITask gotoTask = new GUITask(wmid, 
				wmc.address.toString(), State.NEW, 
				selectedLOI.point.x, 
				selectedLOI.point.y);
		log("generated a GUITask [" + gotoTask.x +"," + gotoTask.y + "] for the planner -- NOT going to add it to " + m_plannerSA);

 */

}
