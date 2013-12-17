package eu.nifti.planning.test;

import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.logging.ComponentLogger;
import eu.nifti.planning.slice.Action;
import eu.nifti.planning.slice.Completion;
import java.util.Map;

/**
 *
 * @author harmish
 */
public class PlanningTester extends ManagedComponent
{
    private ComponentLogger logger;

    public PlanningTester()
    {
        super();
        logger = ComponentLogger.getLogger("PlanningTester");
    }

    @Override
    protected void configure(Map<String, String> _config)
    {
        super.configure(_config);
        //TODO: configuration of the eclipse clp engine
    }

    @Override
    protected void runComponent() {
        super.runComponent();
    }

    @Override
    protected void start()
    {
        super.start();

        //setting change filter for receiving action status
        addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(Action.class, WorkingMemoryOperation.ADD),
                new WorkingMemoryChangeReceiver()
                {
                    @Override
                    public void workingMemoryChanged(WorkingMemoryChange _wmc)
                    {
                        actionReceived(_wmc);
                    }
                });
    }

    private void actionReceived(WorkingMemoryChange _wmc)
    {
        Action action;
        try
        {
            action = getMemoryEntry(_wmc.address, Action.class);
            logger.info("action received: " + action);

            action.status = Completion.INPROGRESS;
            logger.info("now performing action: " + action);

            action.status = Completion.SUCCEEDED;
            logger.info("finished performing action: " + action);
        }
        catch (DoesNotExistOnWMException e)
        {
            logger.debug("error while retrieving action\n" + e);
        }
        catch (UnknownSubarchitectureException e)
        {
            logger.debug("error while retrieving action\n" + e);
        }
    }
//    private void moveRobot(ComplexFormula formula)
//    {
//        int linmovement = 1;
//        int angmovement = 1;
//
//        ModalFormula mf = (ModalFormula)formula.forms.get(2);
//
//        Argument[] arguments = null;
//        if(mf.op.equals("Direction"))
//        {
//            ElementaryFormula ef = (ElementaryFormula)mf.form;
//            if (ef.prop.equals("forward"))
//            {
//                arguments = new Argument[] { new Argument("linx", Integer.toString(linmovement)),
//                                             new Argument("liny", Integer.toString(linmovement)),
//                                             new Argument("linz","0.0"),
//                                             new Argument("angx","0"),
//                                             new Argument("angy","0"),
//                                             new Argument("angz","0") };
//            }
//            else if (ef.prop.equals("left"))
//            {
//                arguments = new Argument[] { new Argument("linx", "0"),
//                                             new Argument("liny","0"),
//                                             new Argument("linz","0"),
//                                             new Argument("angx","0"),
//                                             new Argument("angy","0"),
//                                             new Argument("angz","0") };
//            }
//            else if (ef.prop.equals("right"))
//            {
//                arguments = new Argument[] { new Argument("linx", "0"),
//                                             new Argument("liny","0"),
//                                             new Argument("linz","0"),
//                                             new Argument("angx","0"),
//                                             new Argument("angy","0"),
//                                             new Argument("angz","0") };
//            }
//            else if (ef.prop.equals("back"))
//            {
//                arguments = new Argument[] { new Argument("linx", "0"),
//                                             new Argument("liny","0"),
//                                             new Argument("linz","0"),
//                                             new Argument("angx","0"),
//                                             new Argument("angy","0"),
//                                             new Argument("angz","0") };
//            }
//            else
//            {
//                arguments = new Argument[] { new Argument("linx", "0"),
//                                             new Argument("liny","0"),
//                                             new Argument("linz","0"),
//                                             new Argument("angx","0"),
//                                             new Argument("angy","0"),
//                                             new Argument("angz","0") };
//            }
//        }
//        else
//        {
//
//        }
//
//            Action action = new Action(0, "TwistMsg", arguments, Completion.PENDING);
//            String id = newDataID();
//            try
//            {
//                addToWorkingMemory(id, "navigation", action);
//                addChangeFilter(ChangeFilterFactory.createIDFilter(id),
//                    new WorkingMemoryChangeReceiver()
//                    {
//                        @Override
//                        public void workingMemoryChanged(WorkingMemoryChange wmc)
//                        {
//                            actionStatusChanged(wmc);
//                        }
//                    });
//            }
//            catch (AlreadyExistsOnWMException ex)
//            {
//                log(ex.message);
//            }
//            catch (UnknownSubarchitectureException e)
//            {
//                log(e.message);
//            }
//    }
}
