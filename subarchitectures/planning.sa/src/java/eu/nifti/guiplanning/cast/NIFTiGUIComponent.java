package eu.nifti.guiplanning.cast;

import cast.architecture.ManagedComponent;

import Ice.ObjectAdapter;
import eu.nifti.guiplanning.NIFTiGUIPlanningMessagesHandler;
import eu.nifti.planning.slice.PlanningTask;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.architecture.ChangeFilterFactory;

public class NIFTiGUIComponent extends ManagedComponent
{

    @Override
    protected void start()
    {
        
        // Creates an instance of NIFTiGUIMessagesHandler (ICE servant)
        Ice.Object clientsHandler = new NIFTiGUIPlanningMessagesHandler(super.getCommunicator()); // This is the servant
        
        // Creates an object adapter (although there is already one)
        ObjectAdapter adapter = super.getCommunicator().createObjectAdapterWithEndpoints("ObjectAdapterForGUIPlanningConnections", "tcp -p 10123");

        // Creates an instance of NIFTiGUIMessagesHandler, and allows any client to access it via its identity string
        adapter.add(clientsHandler, super.getCommunicator().stringToIdentity("NIFTiGUIPlanningMessagesHandlerIdentity"));
        
        // This line does not work because I don't know the identity of the adapter, or the end-points
        //super.getObjectAdapter().add(clientsHandler, super.getCommunicator().stringToIdentity("NIFTiGUIRequestsHandlerIdentity"));

        // Activates the adapter so that we can start receiving connections
        adapter.activate();

        addChangeFilter(ChangeFilterFactory.createTypeFilter(PlanningTask.class, WorkingMemoryOperation.OVERWRITE),
            new WorkingMemoryChangeReceiver()
            {
                @Override
                public void workingMemoryChanged(WorkingMemoryChange _wmc)
                {
                    planChanged(_wmc);
                }
            });
    }

    private void planChanged(WorkingMemoryChange _wmc)
    {
//        NIFTiGUIClientsManager clientsMgr = new NIFTiGUIClientsManager(communicator);
//
//        try
//        {
//            PlanningTask task = getMemoryEntry(_wmc.address, PlanningTask.class);
//
//            String[] actionStringList = new String[task.plan.length];
//            for(int i=0; i < task.plan.length; i++)
//            {
//                actionStringList[i] = task.plan[i].name;
//            }
//            //clientsMgr.onPlanReceived(task.id, actionStringList, this);
//        }
//        catch (DoesNotExistOnWMException e)
//        {
//            logger.debug("error while processing planning task\n" + e);
//        }
//        catch (UnknownSubarchitectureException e)
//        {
//            logger.debug("error while processing planning task\n" + e);
//        }
    }
}
