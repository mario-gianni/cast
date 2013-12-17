package eu.nifti.wizard;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.architecture.WorkingMemoryWriterComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.cast.dialogue.AbstractDialogueComponent;
import de.dfki.lt.tr.cast.dialogue.NewIntentionRecognizer.MixingCombinator;
import de.dfki.lt.tr.cast.dialogue.NewIntentionRecognizer.ReferenceResolutionResultWrapper;
import de.dfki.lt.tr.cast.dialogue.util.VerbalisationUtils;
import de.dfki.lt.tr.dialogue.interpret.InterpretedUserIntention;
import de.dfki.lt.tr.dialogue.interpret.ResultGatherer;
import de.dfki.lt.tr.dialogue.ref.Constraint;
import de.dfki.lt.tr.dialogue.ref.EpistemicReferenceHypothesis;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionResult;
import de.dfki.lt.tr.dialogue.ref.util.ReferenceUtils;
import de.dfki.lt.tr.dialogue.slice.time.Interval;
import de.dfki.lt.tr.dialogue.time.Point;
import de.dfki.lt.tr.dialogue.time.TimeInterval;
import de.dfki.lt.tr.dialogue.ref.MentionedEntityPointer;
import de.dfki.lt.tr.dialogue.slice.synthesize.SpokenOutputItem;
import eu.nifti.dialogue.actions.Action;
import eu.nifti.dialogue.actions.AskIsGapTraversableAction;
import eu.nifti.dialogue.actions.ExploreAction;
import eu.nifti.dialogue.actions.MoveActionFactory;
import eu.nifti.dialogue.actions.MoveInDirectionAction;
import eu.nifti.dialogue.actions.ReturnToBaseAction;
import eu.nifti.dialogue.actions.StopAction;
import eu.nifti.dialogue.actions.TraverseGapAction;
import eu.nifti.dialogue.actions.TurnInDirectionAction;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

public class DialogueWizard extends AbstractDialogueComponent {

	private final DialogueWizardClient gui;
	private final WizardVerbalisation verbalisation;
	private final Map<WorkingMemoryAddress, ResultGatherer<ReferenceResolutionResultWrapper>> gatherers;
	private String lastUtterance = null;

	private long lastMentionMSec = -1;
	private WorkingMemoryPointer lastMentionWMP = null;

	public DialogueWizard() {
		gatherers = new HashMap<WorkingMemoryAddress, ResultGatherer<ReferenceResolutionResultWrapper>>();
		verbalisation = new BasicWizardVerbalisation();
		
		gui = new NiceDialogueWizardClient();
		gui.addWindowListener(new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent e) {
				scheduleOwnDeath();
			}
		});
	}

	@Override
	protected void onConfigure(Map<String, String> args) {
		super.onConfigure(args);
		final WorkingMemoryWriterComponent component = this;
		
		gui.setListener(new DialogueWizardListener() {

			@Override
			public void onStop() {
				addTask(new ProcessingTask() {
					@Override
					public void run() {
						getLogger().info("asked to STOP");
						addIntentionForAction(new StopAction());
					}
				});
			}

			@Override
			public void onExplore() {
				addTask(new ProcessingTask() {
					@Override
					public void run() {
						getLogger().info("asked to EXPLORE");
						addIntentionForAction(new ExploreAction());
					}
				});
			}

			@Override
			public void onAskGapTraversable() {
				addTask(new ProcessingTask() {
					@Override
					public void run() {
						getLogger().info("asked to CHECK GAP TRAVERSIBILITY");
						addIntentionForAction(new AskIsGapTraversableAction());
					}
				});
			}

			@Override
			public void onTraverseGap() {
				addTask(new ProcessingTask() {
					@Override
					public void run() {
						getLogger().info("asked to TRAVERSE GAP");
						addIntentionForAction(new TraverseGapAction());
					}
				});
			}

			@Override
			public void onReturnToBase() {
				addTask(new ProcessingTask() {
					@Override
					public void run() {
						getLogger().info("asked to RETURN TO BASE");
						addIntentionForAction(new ReturnToBaseAction());
					}
				});
			}

			@Override
			public void onGoToSelection() {
				addTask(new ProcessingTask() {

					@Override
					public void run() {
						
						long now = System.currentTimeMillis();
						Interval ival = new TimeInterval(new Point(now - 100), new Point(now)).toIce();
						List<Constraint> constraints = new ArrayList<Constraint>();
						// constraints.add(new Constraint("Type", "car"));
						ReferenceResolutionRequest rr = new ReferenceResolutionRequest("some-nom", "location", constraints, ival);
						
						if (rr == null) {
							getLogger().error("constructed ReferenceResolutionRequest is null");
							return;
						}

						WorkingMemoryAddress wma = new WorkingMemoryAddress(newDataID(), getSubarchitectureID());

						ResultGatherer<ReferenceResolutionResultWrapper> gatherer = new ResultGatherer(wma, new MixingCombinator(getLogger(), 1));
						
						assert gatherer != null;
						gatherers.put(wma, gatherer);

						ReferenceResolutionResultWrapper result = null;

						try {
							getLogger().info("adding a ReferenceResolutionRequest to the WM: " + ReferenceUtils.resolutionRequestToString(rr));
							addToWorkingMemory(wma, rr);
						}
						catch (SubarchitectureComponentException ex) {
							logException(ex);
						}

						result = gatherer.ensureStabilization(1, TimeUnit.SECONDS);
						stopGathererObservation(wma);

						getLogger().debug("got " + gatherer.getNumOfResults() + " results in total");
						if (gatherer.wasStabilizedByResult()) {
							getLogger().info("reason for resume: the results were good enough");
						}
						else {
							getLogger().info("reason for resume: timeouted while listening in hope for better results");
						}
						
						if (result != null) {

							EpistemicReferenceHypothesis bestHypo = null;

							for (EpistemicReferenceHypothesis hypo : result.getResult().hypos) {
								if (bestHypo == null || hypo.score > bestHypo.score) {
									bestHypo = hypo;
								}
							}
							
							if (bestHypo != null) {
								dFormula referent = bestHypo.referent;
								Action action = MoveActionFactory.dFormulaToAction(referent);

								addIntentionForAction(action);
							}
							else {
								getLogger().warn("got no hypotheses in the result");
								VerbalisationUtils.verbaliseString(component, verbalisation.getShowMeOnTheMap());
							}
						}
						else {
							getLogger().warn("got a null result");
						}
					}
					
				});
			}

			@Override
			public void onGoToLastMention() {
				addTask(new ProcessingTask() {
					@Override
					public void run() {
						getLogger().info("asked to GO TO LAST MENTION");
						
						WorkingMemoryPointer wmp = lastMentionWMP;
						
						if (wmp == null) {
							getLogger().error("there's no such thing as a last mentioned entity --> not doing anything");
							VerbalisationUtils.verbaliseString(component, verbalisation.getShowMeOnTheMap());
						}
						else {
							Action action = MoveActionFactory.targetWMPToAction(wmp);
							addIntentionForAction(action);
						}

					}
				});
			}

			@Override
			public void onSayOk() {
				addTask(new ProcessingTask() {
					@Override
					public void run() {
						VerbalisationUtils.verbaliseString(component, verbalisation.getOk());
					}
				});
			}

			@Override
			public void onSorryCannotDo() {
				addTask(new ProcessingTask() {
					@Override
					public void run() {
						VerbalisationUtils.verbaliseString(component, verbalisation.getSorryCannotDo());
					}
				});
			}

			@Override
			public void onSorryDidNotUnderstand() {
				addTask(new ProcessingTask() {
					@Override
					public void run() {
						VerbalisationUtils.verbaliseString(component, verbalisation.getSorryDidNotUnderstand());
					}
				});
			}

			@Override
			public void onRepeatLastUtterance() {
				addTask(new ProcessingTask() {
					@Override
					public void run() {
						if (lastUtterance != null && !lastUtterance.equals("")) {
							VerbalisationUtils.verbaliseString(component, lastUtterance);
						}
						else {
							getLogger().error("nothing to repeat!");
						}
					}
				});
			}

			@Override
			public void onShowMeOnTheMap() {
				addTask(new ProcessingTask() {
					@Override
					public void run() {
						VerbalisationUtils.verbaliseString(component, verbalisation.getShowMeOnTheMap());
					}
				});
			}

			@Override
			public void onPleaseUseGUIToDoThat() {
				addTask(new ProcessingTask() {
					@Override
					public void run() {
						VerbalisationUtils.verbaliseString(component, verbalisation.getUseTheGUIToDoThat());
					}
				});
			}

			@Override
			public void onGoInDirection(final MoveInDirectionAction.Direction dir) {
				addTask(new ProcessingTask() {
					@Override
					public void run() {
						getLogger().info("asked to MOVE " + dir.name());
						addIntentionForAction(new MoveInDirectionAction(dir));
					}
				});
			}

			@Override
			public void onTurnInDirection(final TurnInDirectionAction.Direction dir) {
				addTask(new ProcessingTask() {
					@Override
					public void run() {
						getLogger().info("asked to TURN " + dir.name());
						addIntentionForAction(new TurnInDirectionAction(dir));
					}
				});
			}

		});
	}

	protected void addIntentionForAction(Action action) {
		if (action != null) {
			InterpretedIntention iint = action.toIntention();
			getLogger().info("the wizard wishes to do the following: " + InterpretedUserIntention.interpretedIntentionToString(iint));
			try {
//				log("exit point of WOZ interface reached -- going to write out intention");
				addToWorkingMemory(newDataID(), iint);
			}
			catch (SubarchitectureComponentException ex) {
				logException(ex);
			}
		}
		else {
			getLogger().error("could not generate an intention from action (the action was null)");
		}
	}

	@Override
	protected void onStart() {
		super.onStart();
		
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
			ReferenceResolutionResult.class, WorkingMemoryOperation.ADD),
			new WorkingMemoryChangeReceiver() {
				@Override
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
//					addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

//						@Override
//						public void execute(WorkingMemoryAddress addr) {
							try {
								getLogger().info("got a ReferenceResolutionResult");
								ReferenceResolutionResult result = getMemoryEntry(_wmc.address, ReferenceResolutionResult.class);
								ResultGatherer<ReferenceResolutionResultWrapper> gatherer = gatherers.get(result.requestAddress);
								if (gatherer != null) {
									gatherer.addResult(new ReferenceResolutionResultWrapper(result));
								}
								else {
									getLogger().error("gatherer is null!");
								}
							}
							catch (SubarchitectureComponentException ex) {
								logException(ex);
							}
//						}
//					});
				}
			});


		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(MentionedEntityPointer.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {

					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

							@Override
							public void execute(WorkingMemoryAddress addr) {
								try {
									MentionedEntityPointer mep = getMemoryEntry(addr, MentionedEntityPointer.class);
									
									getLogger().debug("got a new MentionedEntityPointer, its time=" + mep.timeOfMentionMsec + ", ptr=" + wmaToString(mep.mentionedEntity.address) + ", (type=" + mep.mentionedEntity.type + ")");
									
									if (mep.timeOfMentionMsec > lastMentionMSec) {
										getLogger().debug("seems this is a newer mentioned entity than the previous one (which had the time " + lastMentionMSec + ")");
										lastMentionMSec = mep.timeOfMentionMsec;
										lastMentionWMP = mep.mentionedEntity;
										getLogger().debug("the last mentioned entity shall henceforth be " + wmaToString(mep.mentionedEntity.address) + " of type=" + mep.mentionedEntity.type);
									}
									else {
										getLogger().warn("something is fishy -- the mentioned entity is NOT newer than the one I've already seen ... (not doing anything)");
									}
									
								}
								catch (SubarchitectureComponentException ex) {
									logException(ex);
								}
							}
							
						});
					}
				});

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
			SpokenOutputItem.class, WorkingMemoryOperation.ADD),
			new WorkingMemoryChangeReceiver() {
				@Override
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					try {
						getLogger().info("got a SpokenOutputItem");
						SpokenOutputItem result = getMemoryEntry(_wmc.address, SpokenOutputItem.class);
						setLastSpokenOutputItem(result);
					}
					catch (SubarchitectureComponentException ex) {
						logException(ex);
					}
				}
			});

		gui.setVisible(true);

	}

	public void setLastSpokenOutputItem(SpokenOutputItem soi) {
		getLogger().debug("this is now the latest SOI: " + soi.phonString);
		gui.setLastRobotSpokenItem(soi.phonString);
		lastUtterance = soi.phonString;
	}

	public void stopGathererObservation(WorkingMemoryAddress wma) {
		gatherers.remove(wma);
		try {
			deleteFromWorkingMemory(wma);
		}
		catch (SubarchitectureComponentException ex) {
			logException(ex);
		}
	}

}
