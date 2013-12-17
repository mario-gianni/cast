package de.dfki.lt.tr.cast.dialogue;

import java.util.Map;

import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import de.dfki.lt.tr.cast.dialogue.util.VerbalisationUtils;
import eu.nifti.Planning.slice.DetectGapTraversableTask;
import eu.nifti.Planning.slice.FailurePlanAction;
import eu.nifti.Planning.slice.GUITask;
import eu.nifti.Planning.slice.MoveBaseTask;
import eu.nifti.Planning.slice.Task;
import eu.nifti.Planning.slice.TraverseGapTask;
import eu.nifti.Planning.slice.VantagePointTask;
import eu.nifti.env.CarObjectOfInterest;
import eu.nifti.env.ElementOfInterest;
import eu.nifti.env.SignObjectOfInterest;
import eu.nifti.env.VictimObjectOfInterest;
import java.util.HashSet;

public class NiftiY2PlanVerbGenerator extends ManagedComponent {

	private String m_startUpString = "Hello human!";
	private boolean m_stopSent = false;
	private int m_timeWaitForSettle = 1500;
	private boolean m_newCarPending = false;
	private int m_pendingCarsCounter = 0;
	private WorkingMemoryChange m_latestCarDetection;
        private HashSet<Integer> detectedCarIDs = new HashSet<Integer>();

	protected void configure(Map<String, String> config) {
		super.configure(config);

		if (config.containsKey("--startup_utterance")) {
			m_startUpString = config.get("--startup_utterance");
		}
		if (config.containsKey("--wait_for_new_car")) {
			m_timeWaitForSettle = Integer.parseInt(config.get("--wait_for_new_car"));
		}
	}

	protected void start() {
		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(ElementOfInterest.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						log("GOT NEW EOI!");
						handleNewEOI(_wmc);
					}
				});

		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(Task.class, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						log("GOT OVERWRITTEN TASK!");
						handleOverwrittenTask(_wmc);
					}
				});

		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(FailurePlanAction.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						log("GOT ADDED FAILUREPLANACTION!");
						handleAddedFPA(_wmc);
					}
				});

	}

	private void handleAddedFPA(WorkingMemoryChange wmc) {
		try {
			FailurePlanAction fpa = getMemoryEntry(wmc.address, FailurePlanAction.class);
			if ("stop".equals(fpa.name)) {
				m_stopSent = true;
				VerbalisationUtils.verbaliseString(this, "Okay. I will stop moving.");				
			}
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	private void handleNewEOI(WorkingMemoryChange wmc) {
		try {
			ElementOfInterest receivedEoi = getMemoryEntry(wmc.address, ElementOfInterest.class);
			if (receivedEoi instanceof CarObjectOfInterest) {
				synchronized (this) {
					this.m_latestCarDetection = wmc;
                                }
                                if (detectedCarIDs.contains(receivedEoi.uuid)) {
                                    log("re-detected car EOI with UUID " + receivedEoi.uuid + " -- not reporting verbally!");
                                    return;
                                } else {
                                    log("detected car EOI with UUID " + receivedEoi.uuid);
                                    detectedCarIDs.add(receivedEoi.uuid);
                                    gotNewCar();            
                                }
			} else if (receivedEoi instanceof VictimObjectOfInterest) {
				VerbalisationUtils.verbaliseString(this, "I have detected someone.");
			} else if (receivedEoi instanceof SignObjectOfInterest) {
				VerbalisationUtils.verbaliseString(this, "I have detected a danger sign.");
			}
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	private void handleOverwrittenTask(WorkingMemoryChange wmc) {
		log("handleOverwrittenTask entered");
		try {
			Task receivedTask = getMemoryEntry(wmc.address, Task.class);

			switch (receivedTask.status) {
			case REJECTED:
				VerbalisationUtils.verbaliseString(this, "I am sorry. I cannot do that.");
				break;
			case COMPLETED:
				if ((!(receivedTask instanceof DetectGapTraversableTask)) && (!(receivedTask instanceof MoveBaseTask))) { 
					VerbalisationUtils.verbaliseString(this, "I have reached the goal successfully.");
				} else if (receivedTask instanceof DetectGapTraversableTask) {
					VerbalisationUtils.verbaliseString(this, "I can move over the gap in front of me. Do you want me to do it?");
				}
				break;
			case FAILED:
				if (m_stopSent) {
					log("got a FAILED task because of the STOP signal sent by the user. Not verbalizing it!");
					m_stopSent = false;
				} else if (receivedTask instanceof DetectGapTraversableTask) {
					VerbalisationUtils.verbaliseString(this, "I am sorry. I cannot move over the gap.");
				} else {
					VerbalisationUtils.verbaliseString(this, "I am sorry. I cannot reach the goal.");
				}
				break;
			case CONFIRMED:
				WorkingMemoryPointer newTopic = null;
				String confirmation;
				if (receivedTask instanceof VantagePointTask) {
					newTopic = new WorkingMemoryPointer(((VantagePointTask) receivedTask).carWMA, CASTUtils.typeName(CarObjectOfInterest.class));
					confirmation = "Okay. I will inspect the car."; //  Let me scan my environment first.";
				} else if (receivedTask instanceof GUITask) {
					confirmation = "Okay. I will go there."; // Let me scan my environment first.";
				} else if (receivedTask instanceof TraverseGapTask) {
					confirmation = "Okay. I will move over the gap."; 
				} else if (receivedTask instanceof DetectGapTraversableTask) {
					confirmation = "Okay. I will scan my environment to see if it is possible to move over the gap.";
				} else {
					confirmation = "Okay.";
				}

				if (newTopic!=null) {
					VerbalisationUtils.verbaliseStringWithTopic(this, confirmation, newTopic);
				} else {
					VerbalisationUtils.verbaliseString(this, confirmation);
				}
				break;
			default:
				log("got an unknown/unimplemented task status: " + receivedTask.status.name());
				break;
			}
			return;
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	
	private void gotNewCar() {
		synchronized(this) {
			log("gotNewCar() called: new car detection pending.");
			this.m_newCarPending = true;
			this.m_pendingCarsCounter+=1;
			this.notifyAll();
		}
	}

	private void sayGotNewCar() {
		synchronized(this) {
			log("sayGotNewCar() with car detections counter = " + this.m_pendingCarsCounter);
			if (this.m_pendingCarsCounter == 1) {
				// VerbalisationUtils.verbaliseStringWithTopic(this, "I have detected a car. Do you want me to go there?", new WorkingMemoryPointer(m_latestCarDetection.address, CASTUtils.typeName(CarObjectOfInterest.class)));
				VerbalisationUtils.verbaliseStringWithTopic(this, "I have made a car detection.", new WorkingMemoryPointer(m_latestCarDetection.address, CASTUtils.typeName(CarObjectOfInterest.class)));
			} else {
				VerbalisationUtils.verbaliseString(this, "I have made several car detections. They are shown in the map.");
			}
			this.m_pendingCarsCounter=0;
		}
	}

	
	
	protected void runComponent() {
		VerbalisationUtils.verbaliseString(this, m_startUpString);
		while (isRunning()) {
			try {
				synchronized (this) {
					// if there is no pending task, continue the loop 
					if (!this.m_newCarPending) {
						log("no new car received pending. waiting...");
						this.wait();
						continue;
					}
				}
				synchronized (this) {
					log("pending new car. acknowledged.");
					this.m_newCarPending = false;
				}
				// wait for another change
				log("wait for some time to let it settle.");
				Thread.sleep(m_timeWaitForSettle);
				synchronized (this) {			
					// if there were no more changes
					if (this.m_newCarPending) {
						log("ok. got a fresh car detection. not yest reporting it.");
						continue;
					}
				}
				log("waited long enough. no more fresh car detections for a while. gonna process them!");
				// execute the room maintenance algorithm
				this.lockComponent();
				sayGotNewCar();
				this.unlockComponent();
			}  catch (InterruptedException e) {
				logException(e);
			}
		}

	}




}
