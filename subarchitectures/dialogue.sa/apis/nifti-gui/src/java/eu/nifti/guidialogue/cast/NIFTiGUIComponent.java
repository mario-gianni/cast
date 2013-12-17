package eu.nifti.guidialogue.cast;

import cast.PermissionException;
import cast.architecture.ManagedComponent;

import Ice.ObjectAdapter;
import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.FilterRestriction;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;
import de.dfki.lt.tr.dialogue.slice.synthesize.SpokenOutputItem;
import de.dfki.lt.tr.dialogue.time.Point;
import de.dfki.lt.tr.dialogue.time.TimeInterval;
import java.util.Map;
import eu.nifti.ocu.msg.cast.DialogueUtteranceMessage;
import java.util.logging.Level;
import java.util.logging.Logger;
import eu.nifti.ros.Header;

public class NIFTiGUIComponent extends ManagedComponent
{

    private static final String DIALOGUE_UTTERANCE_MESSAGE_PROVIDER = "ROS_CAST_Converter"; // Todo put that in a parameter

    @Override
    protected void start()
    {

        // register change filters
        addChangeFilter(ChangeFilterFactory.createChangeFilter(DialogueUtteranceMessage.class, WorkingMemoryOperation.ADD, DIALOGUE_UTTERANCE_MESSAGE_PROVIDER, "", "", FilterRestriction.LOCALSA),
                new WorkingMemoryChangeReceiver()
                {

                    @Override
                    public void workingMemoryChanged(WorkingMemoryChange _wmc)
                    {
                        handleDialogueUtteranceMessage(_wmc);
                    }
                });

        addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(SpokenOutputItem.class, WorkingMemoryOperation.ADD),
                new WorkingMemoryChangeReceiver()
                {

                    @Override
                    public void workingMemoryChanged(WorkingMemoryChange _wmc)
                    {
                        handleSpokenOutputItem(_wmc);
                    }
                });

    }

    /**
     * Handle a PhonString.
     * I.e. convert it to a DisplayDialogueString and send to the GUI.
     *
     * @param _wmc  the working memory change event
     */
    private void handleDialogueUtteranceMessage(WorkingMemoryChange _wmc)
    {
        try
        {
            DialogueUtteranceMessage msg = getMemoryEntry(_wmc.address, DialogueUtteranceMessage.class);
            log("received user string \"" + msg.utterance + "\" from the GUI");

            if (msg.userID.length() == 0)
            {
                log("BUT I'LL IGNORE IT");

            }
            else
            {
				long now = System.currentTimeMillis();
				long past = now - 50;
				TimeInterval ival = new TimeInterval(new Point(past), new Point(now));
                PhonString ps = new PhonString(newDataID(), msg.utterance, 0, 1.0f, 1.0f, 1, false, ival.toIce());
                log("adding PhonString to the WM");
                try
                {
                    addToWorkingMemory(ps.id, ps);
                }
                catch (AlreadyExistsOnWMException ex)
                {
                    ex.printStackTrace();
                }

                testSendBackToROS(msg.utterance);
            }
        }
        catch (SubarchitectureComponentException e)
        {
            e.printStackTrace();
        }
        finally
        {
            try
            {
                // Removes the message about the string
                deleteFromWorkingMemory(_wmc.address);
            }
            catch (DoesNotExistOnWMException ex)
            {
                Logger.getLogger(NIFTiGUIComponent.class.getName()).log(Level.SEVERE, null, ex);
            }
            catch (PermissionException ex)
            {
                Logger.getLogger(NIFTiGUIComponent.class.getName()).log(Level.SEVERE, null, ex);
            }
            catch (UnknownSubarchitectureException ex)
            {
                Logger.getLogger(NIFTiGUIComponent.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
    }

    /**
     * Handle a SpokenOutputItem.
     * I.e. convert it to a DisplayDialogueString and send to the GUI.
     *
     * @param _wmc  the working memory change event
     */
    private void handleSpokenOutputItem(WorkingMemoryChange _wmc)
    {
        try
        {
            SpokenOutputItem soi = getMemoryEntry(_wmc.address, SpokenOutputItem.class);
            log("sending robot string \"" + soi.phonString + "\" to the GUI");
            //clientsHandler.clientsMgr.onRobotMessageReceived(soi.phonString);

            DialogueUtteranceMessage msg = new DialogueUtteranceMessage();
            msg.header = new Header();
            msg.header.stamp = null;
            msg.utterance = soi.phonString;
            log("adding DialogueUtteranceMessage to the WM");
            try
            {
                addToWorkingMemory(newDataID(), msg);
            }
            catch (AlreadyExistsOnWMException ex)
            {
                ex.printStackTrace();
            }

        }
        catch (SubarchitectureComponentException e)
        {
            e.printStackTrace();
        }
    }

    private void testSendBackToROS(String text)
    {
        DialogueUtteranceMessage msg = new DialogueUtteranceMessage();
        msg.header = new Header();
        msg.header.stamp = null;
        msg.utterance = "RETURN " + text;
        log("adding DialogueUtteranceMessage to the WM");
        try
        {
            addToWorkingMemory(newDataID(), msg);
        }
        catch (AlreadyExistsOnWMException ex)
        {
            ex.printStackTrace();
        }
    }
}
