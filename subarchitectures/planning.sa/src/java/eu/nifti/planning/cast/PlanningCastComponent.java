package eu.nifti.planning.cast;

import cast.AlreadyExistsOnWMException;
import cast.architecture.ManagedComponent;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPointer;
import cast.core.logging.ComponentLogger;
import eu.nifti.mapping.MapObject;
import eu.nifti.mapping.Pose3D;
import eu.nifti.navigation.slice.GoalState;
import eu.nifti.navigation.slice.NavigationGoal;
import java.util.Map;
import java.util.LinkedList;
import eu.nifti.planning.eclp.EclipsePlanner;
import eu.nifti.planning.eclp.GoalException;
import eu.nifti.planning.eclp.PlanningException;
import eu.nifti.planning.slice.Argument;
import eu.nifti.planning.slice.PlanningTask;
import eu.nifti.planning.slice.Action;
import eu.nifti.planning.slice.Completion;
import java.awt.event.ActionEvent;


/**
 *
 * @author harmish
 */
public class PlanningCastComponent extends ManagedComponent
{
    private ComponentLogger logger;
    
    //connector to the eclipse clp compiler

    //this is the path to stored inital KB files on local machine
    //this path shoudl be given as --initalkb parameter while starting this cast component
    private String initialKBPath;
    //the class to call and manage connection with eclipseclp embedded engine
    //private EclipsePlanner eclipsePlanner;

    //wm address list of the beliefs that represent the state of the world
    private LinkedList<WorkingMemoryAddress> beliefAddressList;

    public PlanningCastComponent()
    {
        super();
        logger = ComponentLogger.getLogger("PlanningCastComponent");
        //beliefAddressList = new LinkedList<WorkingMemoryAddress>();
    }

    @Override
    protected void configure(Map<String, String> _config) {
        super.configure(_config);

        //check for availability of eclipseclp
        String eclipsePath = System.getenv("ECLP_ROOT");

        if(eclipsePath == null || eclipsePath.isEmpty())
        {
            logger.info("eclipse path: " + eclipsePath);
            logger.error("could not find eclipse engine (have you set the environmental variable ECLP_ROOT correctly?)");
        }
        else
        {
            //eclipsePlanner = new EclipsePlanner(eclipsePath, this);
        }

        //configuring the initial kb
        if (!_config.containsKey("--initialkb"))
        {
            logger.error("could not find initial kb (did you forget to provide the path in argument --initialkb ?)");
        }
        else
        {
            initialKBPath = _config.get("--initialkb");
        }
    }

    @Override
    protected void runComponent() {
        super.runComponent();
    }

    @Override
    protected void start()
    {
        super.start();
        logger.debug("PlanningCastComponent: initializing");

        //eclipsePlanner.compileActionModel(initialKBPath);

        //setting change filter for new planning task
        //planning task should be generated and put on the wm of planning.sa
        addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(PlanningTask.class,
                WorkingMemoryOperation.ADD),
                new WorkingMemoryChangeReceiver()
                {
                    @Override
                    public void workingMemoryChanged(WorkingMemoryChange _wmc)
                    {
                        planningTaskReceived(_wmc);
                    }
                });

        //getting the information if newcar is received
        addChangeFilter(ChangeFilterFactory.createTypeFilter(MapObject.class, WorkingMemoryOperation.ADD),
                new WorkingMemoryChangeReceiver()
                {
                    @Override
                    public void workingMemoryChanged(WorkingMemoryChange _wmc)
                    {
                        mappingObjectAdded(_wmc);
                    }
                });

        //setting change filter for change in state of the world (beliefs)
//        addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(dBelief.class,
//                WorkingMemoryOperation.WILDCARD),
//                new WorkingMemoryChangeReceiver()
//                {
//                    @Override
//                    public void workingMemoryChanged(WorkingMemoryChange _wmc)
//                    {
//                        stateChanged(_wmc);
//                    }
//                });

        //setting change filter for intentions
//        addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(Intention.class,
//                WorkingMemoryOperation.WILDCARD),
//                new WorkingMemoryChangeReceiver()
//                {
//                    @Override
//                    public void workingMemoryChanged(WorkingMemoryChange _wmc)
//                    {
//                        intentionReceived(_wmc);
//                    }
//                });
    }

    //retrives planning task from wm and forwards it to eclipse clp
    private void planningTaskReceived(WorkingMemoryChange _wmc)
    {
        logger.debug("PlanningCastComponent: new planning task received");

        try
        {
            //get the planning task from wm
            PlanningTask task = getMemoryEntry(_wmc.address, PlanningTask.class);

            //TODO: update the KB
            
            //generate local list of beliefs that represent the world
//            LinkedList<dBelief> beliefList = new LinkedList<dBelief>();
//            for (WorkingMemoryAddress beliefAddress : beliefAddressList)
//            {
//                beliefList.add(getMemoryEntry(beliefAddress, dBelief.class));
//            }

            //generate a string from belief that could be understood by eclipse clp
//            for (dBelief belief : beliefList)
//            {
//                String beliefString = beliefToString(belief);
//                //prologConnector.callRemotePrologEngine(beliefString);
//            }
            Argument[] argList;
            String actionName;
            
            if (task.goal.startsWith("go-to-position")) {
            	String[] xyzargs = task.goal.split("\\(")[1].split("\\)")[0].split(",");
            	String x = xyzargs[0];
            	String y = xyzargs[1];
            	String z = "0"; // todo: handle 3D z-coordinates!
            	Pose3D targetPose = new Pose3D(Float.parseFloat(x), Float.parseFloat(y), 0, Float.parseFloat(z)); // todo: ensure theta is handled!
            	
            	WorkingMemoryAddress targetPoseWMA = new WorkingMemoryAddress(newDataID(), getSubarchitectureID());
    			try {
    				addToWorkingMemory(targetPoseWMA, targetPose);
    			}
    			catch (AlreadyExistsOnWMException ex) {
    				ex.printStackTrace();
    			} catch (DoesNotExistOnWMException e) {
    				// TODO Auto-generated catch block
    				e.printStackTrace();
    			} catch (UnknownSubarchitectureException e) {
    				// TODO Auto-generated catch block
    				e.printStackTrace();
    			}
            	
            	Argument arg = new Argument("targetPose3D", new WorkingMemoryPointer(targetPoseWMA, targetPose.getClass().getSimpleName()));
            	argList = new Argument[1];
            	argList[0] = arg;
            	actionName = "go-to-position";
            }
            else {
            	argList = new Argument[0];
            	actionName = task.goal;
            }
            
            Action action = new Action(0, actionName, argList, Completion.PENDING);
            onActionReceived(action, "navigation");
            //*************** real planner switched off *****************//
            //parse the task.goal record in eclispe-readable format and then create the goal
            //eclipsePlanner.parseGoals(task.goal);

            //now generate acutal plann !!
            //task.plan = eclipsePlanner.generatePlan();

            //TODO: send plan to GUI

            //starting the execution of the plan
            //eclipsePlanner.executePlan();
            //for(int i = 0; i < task.plan.length; i++)
            //{
            //    onActionReceived(task.plan[i], "navigation");
            //}
            
        }
//        catch(GoalException ex)
//        {
//            logger.error("PlanningCastComponent: error while parsign goal\n" + ex);
//        }
//        catch(PlanningException ex)
//        {
//            logger.error("PlanningCastComponent: error while generating plan\n" + ex.exc);
//        }
        catch (DoesNotExistOnWMException ex)
        {
            logger.debug("planningcastcomponent: error while processing planning task\n" + ex);
        }
        catch (UnknownSubarchitectureException ex)
        {
            logger.debug("planningcastcomponent: error while processing planning task\n" + ex);
        }
    }

    private void mappingObjectAdded(WorkingMemoryChange _wmc)
    {
        try
        {
            MapObject mapObject = getMemoryEntry(_wmc.address, MapObject.class);

            //TODO: forward knowledge of eclipse
        }
        catch (DoesNotExistOnWMException e)
        {
            logger.debug("error while reading map object\n" + e);
        }
        catch (UnknownSubarchitectureException e)
        {
            logger.debug("error while processing map object\n" + e);
        }
    }

    //retrives changed action from wm and notifies the eclipse clp
    private void actionStatusChanged(WorkingMemoryChange _wmc)
    {
        logger.debug("PlanningCastComponent: action status change received");

//        try
//        {
//            //Action action = getMemoryEntry(_wmc.address, Action.class);
//            NavigationGoal navgoal = getMemoryEntry(_wmc.address, NavigationGoal.class);
//
//            //TODO: imform planner about changed aciton status
//            switch(navgoal.status)
//            {
//                case PENDING:
//                    break;
//                case INPROGRESS:
//                    logger.info(navgoal.goal + " is now in progress");
//                    break;
//                case ABORTED:
//                    logger.info(navgoal.goal + " is aborted");
//                    break;
//                case FAILED:
//                    logger.info(navgoal.goal + " has failed");
//                    break;
//                case SUCCEEDED:
//                    logger.info(navgoal.goal + " has succeeded");
//                    break;
//            }
//        }
//        catch (DoesNotExistOnWMException e)
//        {
//            logger.debug("error while processing action status change\n" + e);
//        }
//        catch (UnknownSubarchitectureException e)
//        {
//            logger.debug("error while processing action status change\n" + e);
//        }
    }

    //adds the belief addres which represents change in state to the local beliefAddressList
    private void stateChanged(WorkingMemoryChange _wmc)
    {
        logger.debug("PlanningCastComponent: changed state received");

        switch(_wmc.operation)
        {
            case ADD:
                beliefAddressList.add(_wmc.address);
                break;

            case OVERWRITE:
                break;

            case DELETE:
                beliefAddressList.remove(_wmc.address);
                break;

            case GET:
                break;
        }
    }

    public void actionReceived(ActionEvent e)
    {
        throw new UnsupportedOperationException("Not supported yet.");
    }
    
    public void onActionReceived(Action action, String sa)
    {
        logger.info("action received: " + action.name + " for sa: " + sa);
        try
        {
            String id = newDataID();

            //setting change filter for change in action status
            addChangeFilter(ChangeFilterFactory.createIDFilter(id, WorkingMemoryOperation.OVERWRITE),
                    new WorkingMemoryChangeReceiver()
                    {
                        @Override
                        public void workingMemoryChanged(WorkingMemoryChange _wmc)
                        {
                            actionStatusChanged(_wmc);
                        }
                    });

            Action navaction = new Action(0, action.name, action.arguments, Completion.PENDING);
            addToWorkingMemory(id, sa, navaction);
	    logger.info("action sent");
        }
        catch (AlreadyExistsOnWMException ex)
        {
            logger.error("wm alreadt exists\n" + ex);
        }
        catch (UnknownSubarchitectureException ex)
        {
            logger.error("trying to add wm to unknown sa\n" + ex);
        }
    }

//    private String beliefToString(dBelief belief)
//    {
//        String beliefString = "";
//        return beliefString;
//
//        //TODO: actually retrive predicates from the belief
//    }

//    private void intentionReceived(WorkingMemoryChange wmc)
//    {
//        logger.debug("PlanningCastComponent: new intention task received");
//
//        try
//        {
//            Intention intention = getMemoryEntry(wmc.address, Intention.class);
//
//            ComplexFormula formula = (ComplexFormula)intention.content.get(0).postconditions;
//
//            for (int i = 0; i < formula.forms.size(); i++)
//            {
//                if(formula.forms.get(i).getClass().equals(ElementaryFormula.class))
//                {
//                    ElementaryFormula emf = (ElementaryFormula)formula.forms.get(0);
//                    if (emf.prop.equals("move"))
//                    {
//                        //moveRobot(formula);
//                        break;
//                    }
//                }
//            }
//        }
//        catch (DoesNotExistOnWMException e)
//        {
//            logger.debug(e);
//        }
//        catch (UnknownSubarchitectureException e)
//        {
//            logger.debug(e);
//        }
//    }
}
