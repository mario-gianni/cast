#ifndef GUI_ICE
#define GUI_ICE

#include <cast/slice/CDL.ice>

module eu {
  module nifti {
    module gui { 
    
      // struct for maintaining selections in the GUI on CAST WM
      class GUISelection {
        cast::cdl::WorkingMemoryPointer selectedEntityWMP;
        long startTimeMSec;
        long endTimeMSec;
      };

    };
  };
};

#endif
