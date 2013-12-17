// Benoit 2010-11-30

package eu.nifti.guiplanning;

import Ice.Current;

public class NIFTiGUIPlanningMessagesHandler extends _INIFTiGUIPlanningMessagesHandlerDisp
{

    private NIFTiGUIClientsManager clientsMgr;

    public NIFTiGUIPlanningMessagesHandler(Ice.Communicator communicator)
    {
        clientsMgr = new NIFTiGUIClientsManager(communicator);
    }

    public void onRegisterClient(Ice.Identity ident, Ice.Current __current)
    {
        clientsMgr.onRegisterClient(ident, __current);
        //System.out.println("onRegisterClient");
    }

    public void onUnregisterClient(Ice.Identity ident, Ice.Current __current)
    {
        clientsMgr.onUnregisterClient(ident, __current);
        //System.out.println("onUnregisterClient");
    }

    // Todo Harmish
    public void onRequestCurrentPlan(Current __current)
    {
        System.out.println("onRequestCurrentPlan");
        
	// Note: In the GUI, I use the first string as the goal, and the rest as actions
	clientsMgr.onPlanReceived(42, new String[]{"GO TO FIRST CAR","Go around the small crate","Go forward 10 m","Turn right at the barrel","Go forward 5 m","Make a very long step that would certainly need to be wrapped in the GUI","Extra step to display long plans","Extra step to display long plans","Extra step to display long plans","Extra step to display long plans","Extra step to display long plans","Extra step to display long plans","Extra step to display long plans","Extra step to display long plans","Extra step to display long plans","Extra step to display long plans","Extra step to display long plans","Extra step to display long plans","Extra step to display long plans","Extra step to display long plans","Extra step to display long plans","Extra step to display long plans"}, __current);
    }

    // Todo Harmish
    public void onRemovePlanItem(int planID, int itemID, Current __current)
    {
        System.out.println("onRemovePlanItem");
        throw new UnsupportedOperationException("Not supported yet.");
    }

    // Todo Harmish
    public void onMovePlanItem(int planID, int itemID, int newPosition, Current __current)
    {
        System.out.println("onMovePlanItem");
        throw new UnsupportedOperationException("Not supported yet.");
    }

    // Todo Harmish
    public void onAddNavGoal(int planID, int positionInList, float x, float y, float z, Current __current)
    {
        System.out.println("onAddNavGoal");
        throw new UnsupportedOperationException("Not supported yet.");
    }
}
