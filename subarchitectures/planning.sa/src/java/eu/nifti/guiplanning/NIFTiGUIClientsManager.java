// Benoit 2010-11-30

package eu.nifti.guiplanning;

import Ice.Current;
import java.util.Map;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map.Entry;

public class NIFTiGUIClientsManager
{

    private Ice.Communicator communicator;
    private Map<Ice.Identity, ICASTPlanningMessageReceiverPrx> clients;

    public NIFTiGUIClientsManager(Ice.Communicator communicator)
    {
        this.communicator = communicator;
        clients = new HashMap<Ice.Identity, ICASTPlanningMessageReceiverPrx>();
    }

    public void onRegisterClient(Ice.Identity ident, Ice.Current __current)
    {
        //IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this); // Todo lock the collection of clients

        // I create a one way proxy because I want only async calls
        // Todo: check why it complains when I do a checked cast (it does not in C++)
        ICASTPlanningMessageReceiverPrx callbackHandlerPrx = ICASTPlanningMessageReceiverPrxHelper.uncheckedCast(__current.con.createProxy(ident).ice_oneway());        

        clients.put(ident, callbackHandlerPrx);
        
        System.out.println("ClientsManager@" + this + ": added client `" + communicator.identityToString(ident) + "'");

    }

    public void onUnregisterClient(Ice.Identity ident, Ice.Current __current)
    {
        //IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this); // Todo lock the collection of clients

        clients.remove(ident);
        
        System.out.println("removed client `" + communicator.identityToString(ident) + "'");
    }
    
    
    
    
    
    public void onPlanReceived(int planID, String[] items, Current __current)
    {
        Iterator<Entry<Ice.Identity, ICASTPlanningMessageReceiverPrx>> it = clients.entrySet().iterator();
        
        while(it.hasNext())
        {
            Entry<Ice.Identity, ICASTPlanningMessageReceiverPrx> entry = it.next();
            try
            {
                entry.getValue().onPlanReceived(planID, items);
            } 
            catch (Exception ex)
            {
                //IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this);

                it.remove();
                
                System.err.println("removed client `" + communicator.identityToString(entry.getKey()) + "':\n" + ex.toString());
            }
        }
    }
    
    public void onPlanItemCompleted(int planID, int indexOfLastCompletedItem, Current __current)
    {
        Iterator<Entry<Ice.Identity, ICASTPlanningMessageReceiverPrx>> it = clients.entrySet().iterator();
        
        while(it.hasNext())
        {
            Entry<Ice.Identity, ICASTPlanningMessageReceiverPrx> entry = it.next();
            try
            {
                entry.getValue().onPlanItemCompleted(planID, indexOfLastCompletedItem);
            } 
            catch (Exception ex)
            {
                //IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this);

                it.remove();
                
                System.err.println("removed client `" + communicator.identityToString(entry.getKey()) + "':\n" + ex.toString());
            }
        }
    }

}