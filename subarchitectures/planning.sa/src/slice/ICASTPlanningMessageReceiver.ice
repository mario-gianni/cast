// Benoit 2010-11-24

#ifndef I_CAST_PLANNING_MESSAGE_RECEIVER_ICE
#define I_CAST_PLANNING_MESSAGE_RECEIVER_ICE
 
module eu
{

module nifti
{

module guiplanning
{

  sequence<string> SequenceOfString;

  // CLIENT-SIDE
  interface ICASTPlanningMessageReceiver
  {
    void onPlanReceived(int planID, SequenceOfString items); // When I receive a new plan from the robot
    void onPlanItemCompleted(int planID, int indexOfLastCompletedItem); // When I get notified that the robot has achieved a goal
  };

};

};

};

#endif // I_CAST_PLANNING_MESSAGE_RECEIVER_ICE
