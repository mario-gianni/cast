// Benoit 2010-11-24

#ifndef I_NIFTI_GUI_PLANNING_MESSAGES_HANDLER_ICE
#define I_NIFTI_GUI_PLANNING_MESSAGES_HANDLER_ICE

#include <Ice/Identity.ice>
 
module eu
{

module nifti
{

module guiplanning
{

  // SERVER-SIDE
  interface INIFTiGUIPlanningMessagesHandler
  {
    void onRegisterClient(Ice::Identity ident);
    void onUnregisterClient(Ice::Identity ident);

    void onRequestCurrentPlan();
    void onRemovePlanItem(int planID, int itemID);
    void onMovePlanItem(int planID, int itemID, int newPosition);
    void onAddNavGoal(int planID, int positionInList, float x, float y, float z);
  };

};

};

};

#endif // I_NIFTI_GUI_PLANNING_MESSAGES_HANDLER_ICE
