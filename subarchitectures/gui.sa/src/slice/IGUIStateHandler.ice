#ifndef GUI_ICE
#define GUI_ICE

module eu 
{
  module nifti 
  {
    module gui 
    { 

      // ICE interface for handling callbacks from the GUI-proper
      interface IGUIStateHandler 
      {

        // string onSelected(cast::cdl::WorkingMemoryPointer selectedEntityWMP);
        string onSelected(string selectedEntityID, double timeStamp);

        // string onDeselected(cast::cdl::WorkingMemoryPointer deselectedEntityWMP);
        string onDeselected(string deselectedEntityID, double timeStamp);
      };
    

    };
  };
};

#endif
