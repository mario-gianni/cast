package eu.nifti.wizard;

import eu.nifti.dialogue.actions.MoveInDirectionAction;
import eu.nifti.dialogue.actions.TurnInDirectionAction;
import java.util.EventListener;

public interface DialogueWizardListener extends EventListener {

	public void onStop();

	public void onExplore();
	public void onReturnToBase();
	public void onAskGapTraversable();
	public void onTraverseGap();
	
	public void onGoToLastMention();
	public void onGoToSelection();
	
	public void onSayOk();
	public void onSorryCannotDo();	
	public void onSorryDidNotUnderstand();
	public void onRepeatLastUtterance();
	public void onShowMeOnTheMap();
	public void onPleaseUseGUIToDoThat();

	public void onGoInDirection(MoveInDirectionAction.Direction dir);	
	
	public void onTurnInDirection(TurnInDirectionAction.Direction dir);

}
