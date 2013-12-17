package eu.nifti.gui.cast.components;

import java.util.HashMap;
import java.util.Map;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import eu.nifti.env.ElementOfInterest;
import eu.nifti.env.LocationOfInterest;
import eu.nifti.gui.GUISelection;
//import eu.nifti.ocu.msg.cast.SelectionMessageType;

import eu.nifti.ocu.msg.cast.SelectionMessage;
import eu.nifti.ocu.msg.cast.SelectionMessageAction;
import eu.nifti.ros.Time;

/**
 * This is a reference implementation of a CAST component for the NIFTi GUI.
 * Its purpose is to maintain the GUI state on its own CAST WorkingMemory (WM).
 * 
 * The first instance of the GUI state that is relevant for the rest of the system
 * is a representation of the current and past active selection(s).
 * 
 * Further aspects of the GUI state can later be added if/when necessary.
 * 
 * @author zender
 * @date 2011-03-10
 */
public class GUIStateMonitor extends ManagedComponent
{

    //HashMap<String, WorkingMemoryAddress> selectionMapOLD;
    
    // Look-up for quickly retrieving objects in the working memory when receiving deselection messages
    // HashMap<Integer, WorkingMemoryAddress> selectionMap;
    
	// Look-up for UUIDs to WMAs
	HashMap<Integer, WorkingMemoryPointer> eoiUuidToEoiWmp;
	HashMap<Integer, GUISelection> eoiUuidToBufferedGUISelection;
    HashMap<Integer, WorkingMemoryAddress> eoiUuidToGUISelectionWma;

    // todo: refactor to make use of WMP references instead of simple strings
    // HashMap<WorkingMemoryPointer,WorkingMemoryAddress> selectionMap;
    //private String adapterName = "ObjectAdapterForGUIState";
    //private String adapterEndpoints = "tcp -p 10124";
    //private String identityString = "IGUIStateHandlerIdentity";

    /**
     * Start-up sequence: #1
     * This is the place to read the 'command line' arguments, 
     * i.e., the arguments specified in the CAST file,
     * and to initialize class members.
     */
    @Override
    public void configure(Map<String, String> args)
    {
        super.configure(args);
        log("configure() called");
        //this.selectionMapOLD = new HashMap<String, WorkingMemoryAddress>();
        this.eoiUuidToEoiWmp = new HashMap<Integer, WorkingMemoryPointer>();
        this.eoiUuidToBufferedGUISelection = new HashMap<Integer, GUISelection>();
        this.eoiUuidToGUISelectionWma = new HashMap<Integer, WorkingMemoryAddress>();
        
        // nothing else to be done right now...
    }

    /**
     * Start-up sequence: #2
     * This is the place to register change filters that should be
     * in effect from the start of the system on.
     */
    @Override
    public void start()
    {
        //System.out.println("IN start " + this.getSubarchitectureID());

        // register monitoring change filters here
        // nothing to be done right now...

        // Todo add Filter for selection message
        addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(SelectionMessage.class, WorkingMemoryOperation.ADD),
                new WorkingMemoryChangeReceiver() {
					@Override
                    public void workingMemoryChanged(WorkingMemoryChange wmc) {
                        onSelectionMessageReceived(wmc);
                    };
        		});
        
        addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(ElementOfInterest.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleNewEOI(_wmc);
					}
				});
    }

    /**
     * Start-up sequence: #3
     * If the component is supposed to do something 'on its own', this is the
     * place to add that code. E.g., if the component is supposed to write something
     * to WM, it should not happen earlier than in this method. 
     *  
     * If the component is only reacting to events it
     * gets 'from the outside' then this method can remain empty.
     */
    @Override
    protected void runComponent()
    {
        // Register the ICE server here such that it only starts listening to incoming events
        // once the rest of the CAST system is up and running (and listening!)...
       // log("registering ICE server IGUIStateHandler with identity = " + identityString);
       // IGUIStateHandlerI myGUIStateHandler = new IGUIStateHandlerI();
        //this.registerIceServer(IGUIStateHandler.class, myGUIStateHandler);

       // ObjectAdapter adapter = super.getCommunicator().createObjectAdapterWithEndpoints(adapterName, adapterEndpoints);

        // Creates an instance of NIFTiGUIMessagesHandler, and allows any client to access it via its identity string
       // adapter.add(myGUIStateHandler, super.getCommunicator().stringToIdentity(identityString));

        // This line does not work because I don't know the identity of the adapter, or the end-points
        //super.getObjectAdapter().add(clientsHandler, super.getCommunicator().stringToIdentity("NIFTiGUIRequestsHandlerIdentity"));

        // Activates the adapter so that we can start receiving connections
       // adapter.activate();

        //log("this.getIceIdentity().category = " + this.getIceIdentity().category);
        //log("this.getIceIdentity().name = " + this.getIceIdentity().name);

        //log("myGUIStateHandler.ice_id() = " + myGUIStateHandler.ice_id());
        //log("myGUIStateHandler.ice_staticId() = " + myGUIStateHandler.ice_staticId());
        //myGUIStateHandler.ice_ping();
    }

//    public GUISelection newSelection(String entityId, double timeStamp) 
//    {
//    	log("entering newSelection (returns GUISelection) with entityId = " + entityId);
//        String s = entityId;
//        if (entityId.startsWith("Click "))
//        {
//            // it's a click, leave it
//        }
//        else
//        {
//            // it's a marker, find the first number and add a colon in front of it
//            // e.g. "car1" -> "car:1"
//            String[] type = entityId.split("[0-9]+");
//            String[] nums = entityId.split("[a-z]+");
//			for (int i = 0; i < type.length; i++) {
//				log("type[" + i + "] = \"" + type[i] + "\"");
//			}
//			for (int i = 0; i < nums.length; i++) {
//				log("nums[" + i + "] = \"" + nums[i] + "\"");
//			}
//            if (type.length == 1 && nums.length == 2)
//            {
//                s = type[0] + ":" + nums[1];
//            }
//        }
//        return new GUISelection(s, timeStamp, 0);
//    }

    private void onSelectionMessageReceived(WorkingMemoryChange wmc)
    {
    	log("received a SelectionMessage!");

        SelectionMessage msg = null;

        try {
            msg = (SelectionMessage) getMemoryEntry(wmc.address, SelectionMessage.class);
        }
        catch (SubarchitectureComponentException ex) {
			logException(ex);
        }

        if (msg == null)
        {
            log("Could not read the SelectionMessage at " + wmc);
            return;
        }

        // Creates/modifies the selection from the message
        if (msg.action == SelectionMessageAction.SELECT)
        {
            onSelectMessage(msg);
        }
        else
        {
            onDeselectMessage(msg);
        }


        // Deletes the message
        try {
            deleteFromWorkingMemory(wmc.address);
        }
        catch (SubarchitectureComponentException ex) {
			logException(ex);
        }
    }

    
    private void onSelectMessage(SelectionMessage msg)
    {
    	
        //Selection newSelection = new Selection(msg.userID, msg.header.stamp, null, msg.wmp);
        //GUISelection newSelection = newSelection(msg.userID, msg.header.stamp.sec + (msg.header.stamp.nsec / 1000000000));
    	long startTime = computeTimeMSec(msg.header.stamp);
    	WorkingMemoryPointer selectedEntityWMP = eoiUuidToEoiWmp.get(msg.elementUUID);
    	log("onSelectMessage: UUID = " + msg.elementUUID + 
    			", time " + msg.header.stamp.sec + ":" + msg.header.stamp.nsec + 
    			" -- which corresponds to " + startTime + 
    			" -- and has WMP-type " + (selectedEntityWMP!=null ? selectedEntityWMP.type : "null"));
    	
    	GUISelection newSelection = new GUISelection(selectedEntityWMP, startTime, Long.MAX_VALUE);
    	
    	// if EOI was received earlier, we are good to proceed
    	// otherwise we need to buffer the pending GUISelection and process it later
    	if (newSelection.selectedEntityWMP==null) {
    		// cannot write to CAST yet!
    		eoiUuidToBufferedGUISelection.put(msg.elementUUID, newSelection);
    	} else {
    		// good to go!
    		addGUISelectionToWM(newSelection, msg.elementUUID);
    	}
    }
    
    private void addGUISelectionToWM(GUISelection newSelection, Integer elementUUID) {
    	WorkingMemoryAddress newSelectionWMA = new WorkingMemoryAddress(newDataID(), getSubarchitectureID());
		try {
			addToWorkingMemory(newSelectionWMA, newSelection);
		}
        catch (SubarchitectureComponentException ex) {
			logException(ex);
        }
        eoiUuidToGUISelectionWma.put(elementUUID, newSelectionWMA);
    }
    
    private static long computeTimeMSec(Time tstamp) {
    	return ((long) tstamp.sec * 1000) + ((long) tstamp.nsec / 1000000);	
    }

    private void onDeselectMessage(SelectionMessage msg)
    {
    	log("onDeselectMessage entered");
        // Retrieves the actual selection from the working memory
        WorkingMemoryAddress selectionWMA = eoiUuidToGUISelectionWma.get(msg.elementUUID);
        long endTime = computeTimeMSec(msg.header.stamp);

        if (selectionWMA == null) {
            log("Received a deselection message for element " + msg.elementUUID + " but we have no record of it being selected");
            return;
        }
        log("Received a deselection message for element " + msg.elementUUID + ". Selection end time = " + endTime);
        
        GUISelection oldSelection = null;

        try {
            oldSelection = (GUISelection) getMemoryEntry(selectionWMA, GUISelection.class);
            oldSelection.endTimeMSec = endTime;
            overwriteWorkingMemory(selectionWMA, oldSelection);
        }
        catch (SubarchitectureComponentException ex) {
			logException(ex);
        }

        eoiUuidToGUISelectionWma.remove(msg.elementUUID);
    }
    
    
    private void handleNewEOI(WorkingMemoryChange wmc) {
		ElementOfInterest receivedEOI = null; 
		try {
			receivedEOI = getMemoryEntry(wmc.address, ElementOfInterest.class);
		}
		catch (SubarchitectureComponentException ex) {
			logException(ex);
        }
		
		log("got a new EOI of type: " + CASTUtils.typeName(receivedEOI));
		
		if (receivedEOI==null) {
			log("Error: got a null ElementOfInterest. Returning.");
			return;
		}
		WorkingMemoryPointer receivedEoiWMP = new WorkingMemoryPointer(wmc.address, CASTUtils.typeName(receivedEOI));
		
		Integer loiUuid = receivedEOI.uuid;
		GUISelection pendingSelection = eoiUuidToBufferedGUISelection.remove(loiUuid);
		if (pendingSelection != null) {
			pendingSelection.selectedEntityWMP=receivedEoiWMP;
			addGUISelectionToWM(pendingSelection, loiUuid);
		} else{
			eoiUuidToEoiWmp.put(loiUuid, receivedEoiWMP);
		}
		
	}
}

//    /**
//     * Implement the callback methods of the ICE interface
//     */
//    protected class IGUIStateHandlerI extends eu.nifti.gui._IGUIStateHandlerDisp
//    {
//
//        /**
//         * autogen
//         */
//        /**
//         * 
//         */
//        private static final long serialVersionUID = 3439459159713973323L;
//
//        /* (non-Javadoc)
//         * @see eu.nifti.gui._IGUIStateHandlerOperations#onSelected(java.lang.String, double, Ice.Current)
//         */
//        @Override
//        public String onSelected(String selectedEntityID, double timeStamp, Current current)
//        {
//            log("got an Ice call for onSelected(" + selectedEntityID + "," + timeStamp + ")");
//            GUISelection newSelection = newSelection(selectedEntityID, timeStamp);
//            WorkingMemoryAddress newSelectionWMA = new WorkingMemoryAddress(newDataID(), getSubarchitectureID());
//            try
//            {
//                log("selecting \"" + newSelection.selectedEntityID + "\"");
//                addToWorkingMemory(newSelectionWMA, newSelection);
//            }
//            catch (AlreadyExistsOnWMException ex)
//            {
//                ex.printStackTrace();
//            }
//            catch (DoesNotExistOnWMException e)
//            {
//                e.printStackTrace();
//            }
//            catch (UnknownSubarchitectureException e)
//            {
//                e.printStackTrace();
//            }
//            selectionMapOLD.put(selectedEntityID, newSelectionWMA);
//
//            return newSelectionWMA.toString();
//        }
//
//        /* (non-Javadoc)
//         * @see eu.nifti.gui._IGUIStateHandlerOperations#onDeselected(java.lang.String, double, Ice.Current)
//         */
//        @Override
//        public String onDeselected(String deselectedEntityID, double timeStamp, Current current)
//        {
//            log("got an Ice call for onDeselected(" + deselectedEntityID + "," + timeStamp + ")");
//            WorkingMemoryAddress selectionEntryWMA = selectionMapOLD.get(deselectedEntityID);
//            // todo: this intentionally will throw a null pointer exception if there is no entry for that entity yet!
//            try
//            {
//                GUISelection selectionEntry = getMemoryEntry(selectionEntryWMA, GUISelection.class);
//                log("unselecting \"" + selectionEntry.selectedEntityID + "\"");
//                selectionEntry.endTime = timeStamp;
//                overwriteWorkingMemory(selectionEntryWMA, selectionEntry);
//            }
//            catch (DoesNotExistOnWMException e)
//            {
//                e.printStackTrace();
//            }
//            catch (UnknownSubarchitectureException e)
//            {
//                e.printStackTrace();
//            }
//            catch (ConsistencyException e)
//            {
//                e.printStackTrace();
//            }
//            catch (PermissionException e)
//            {
//                e.printStackTrace();
//            }
//            return null;
//        }
//    }
