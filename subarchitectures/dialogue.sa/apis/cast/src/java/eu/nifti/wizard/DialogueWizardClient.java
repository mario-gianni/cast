package eu.nifti.wizard;

import javax.swing.JFrame;

public abstract class DialogueWizardClient extends JFrame {

	private DialogueWizardListener listener = null;

	public final DialogueWizardListener getListener() {
		return listener;
	}
	
	public void setListener(DialogueWizardListener listener) {
		this.listener = listener;
	}

	public abstract void setLastRobotSpokenItem(String s);
	
}
