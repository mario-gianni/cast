package eu.nifti.wizard;

import eu.nifti.dialogue.actions.MoveInDirectionAction;
import eu.nifti.dialogue.actions.TurnInDirectionAction;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import javax.swing.JButton;

class UglyDialogueWizardClient extends DialogueWizardClient {

	public UglyDialogueWizardClient() {
//		super("Dialogue Whizzard");
		setTitle("Dialogue Whizzard");
		
		setLayout(new FlowLayout());
		
		setPreferredSize(new Dimension(1000, 200));
		
		add(newButton("!! STOP !!", new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent ae) {
				getListener().onStop();
			}
		}));

		add(newButton("Explore", new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent ae) {
				getListener().onExplore();
			}
		}));

		add(newButton("Traverse Gap", new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent ae) {
				getListener().onTraverseGap();
			}
		}));

		add(newButton("Baby Come Home", new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent ae) {
				getListener().onReturnToBase();
			}
		}));

		add(newButton("Go to SELECTION", new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent ae) {
				getListener().onGoToSelection();
			}
		}));

		add(newButton("Go to LAST MENTION", new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent ae) {
				getListener().onGoToLastMention();
			}
		}));
		
		add(newButton("SORRY Can't Do", new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent ae) {
				getListener().onSorryCannotDo();
			}
		}));

		add(newButton("SORRY Don't Understand", new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent ae) {
				getListener().onSorryDidNotUnderstand();
			}
		}));

		add(newButton("[Repeat Last Utterance]", new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent ae) {
				getListener().onRepeatLastUtterance();
			}
		}));

		add(newButton("Show Me On Map", new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent ae) {
				getListener().onShowMeOnTheMap();
			}
		}));

		add(newButton("Move FORWARD", new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent ae) {
				getListener().onGoInDirection(MoveInDirectionAction.Direction.FORWARD);
			}
		}));

		add(newButton("Move LEFT", new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent ae) {
				getListener().onGoInDirection(MoveInDirectionAction.Direction.LEFT);
			}
		}));
		
		add(newButton("Move RIGHT", new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent ae) {
				getListener().onGoInDirection(MoveInDirectionAction.Direction.RIGHT);
			}
		}));
		
		add(newButton("Move BACKWARD", new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent ae) {
				getListener().onGoInDirection(MoveInDirectionAction.Direction.BACKWARD);
			}
		}));

		add(newButton("Turn LEFT", new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent ae) {
				getListener().onTurnInDirection(TurnInDirectionAction.Direction.LEFT);
			}
		}));

		add(newButton("Turn RIGHT", new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent ae) {
				getListener().onTurnInDirection(TurnInDirectionAction.Direction.RIGHT);
			}
		}));

		pack();
	}

	private JButton newButton(String label, ActionListener listener) {
		JButton button = new JButton(label);
		button.addActionListener(listener);
		return button;
	}

	@Override
	public void setLastRobotSpokenItem(String s) {
	}
	
}
