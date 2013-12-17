package eu.nifti.planning.eclp;

import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPointer;
import cast.core.logging.ComponentLogger;
/*import com.parctechnologies.eclipse.Atom;
import com.parctechnologies.eclipse.CompoundTerm;
import com.parctechnologies.eclipse.CompoundTermImpl;
import com.parctechnologies.eclipse.EclipseEngine;
import com.parctechnologies.eclipse.EclipseEngineOptions;
import com.parctechnologies.eclipse.EclipseException;
import com.parctechnologies.eclipse.EclipseTerminatedException;
import com.parctechnologies.eclipse.EmbeddedEclipse;*/
import eu.nifti.planning.cast.PlanningCastComponent;
import eu.nifti.planning.slice.Action;
import eu.nifti.planning.slice.Argument;
import eu.nifti.planning.slice.Completion;
import java.io.File;
import java.io.IOException;
import java.util.LinkedList;

/**
 *
 * @author mario, matia, panos
 */
public class EclipsePlanner
{
    /*private EclipseEngine engine;

    private ComponentLogger logger;

    private PlanningCastComponent planningComponent;

    private CompoundTerm goal;
    private CompoundTerm plan;*/

    public EclipsePlanner(String path, PlanningCastComponent planningComponent)
    {
        /*logger = ComponentLogger.getLogger("EclipsePlanner");
        this.planningComponent = planningComponent;

        EclipseEngineOptions eclipseEngineOptions = new EclipseEngineOptions(new File(path));

        //setting options for eclipse engine
        eclipseEngineOptions.setUseQueues(false);
        
        try
        {
            this.engine = EmbeddedEclipse.getInstance(eclipseEngineOptions);
        }
        catch(EclipseException e)
        {
            logger.debug(e);
        }
        catch(EclipseTerminatedException e)
        {
            logger.debug(e);
        }
        catch(IOException e)
        {
            logger.debug(e);
        }*/
    }

    /*
     * this method loads Basic theory of Actions
     */
    public void compileActionModel(String file)
    {
        /*try
        {
            //this.engine.rpc("compile('" + file + "')");)
            this.engine.compile(new File(file));
            logger.info("eclipseplanner: action model loaded from file " + file);

        }
        catch(EclipseException e)
        {
            logger.debug(e);
            logger.info("eclipseplanner: action model not loaded...");

        }
        catch(IOException e)
        {
            logger.debug(e);
            logger.info("eclipseplanner: action model not loaded...");
        }*/
    }

    /*
    //parses the goal string so that eclipseclp can undertand and process it
    public void parseGoals(String goals) throws GoalException
    {
        logger.info("eclipseplanner: goal received for planning");
        
        LinkedList<CompoundTerm> parsedGoal = new LinkedList<CompoundTerm>();
        //TODO: parsing of goal
        
        if(goals.equals("move-left"))
        {
            Atom a1 = new Atom("navigation");
            CompoundTerm t2 = new CompoundTermImpl("goto_pose",new Integer(1),new Integer(3),new Integer(45));
            CompoundTerm t1 = new CompoundTermImpl("process",a1,t2,null);
            parsedGoal.add(t1);
            //parsedGoal.add(new CompoundTermImpl("process",new Atom("navigation"),new CompoundTermImpl("goto_pose",new Integer(1),new Integer(3),new Integer(45)),null));
        }
 
        //now generate eclipse_goal from parsed goal
        //Object[] os = new Object[]{parsedGoal, null};
        goal = new CompoundTermImpl("reach_goals", parsedGoal, null);
    }

    //generate plan from the eclipse_goal
    public Action[] generatePlan() throws PlanningException
    {
        logger.info("eclipseplanner: planning started for goal:\n" + goal);
        try
        {
            //call eclipse engine to generate plan
            plan = this.engine.rpc(goal);

            logger.info("eclipseplanner: the plan was generated sucessfully\n" + plan.arg(2));

            //decode actionList from plan
            LinkedList<LinkedList<CompoundTerm>> actions = (LinkedList<LinkedList<CompoundTerm>>)plan.arg(2);
	    
            return createListOfActions(actions);
        }
        catch(EclipseException e)
        {
            System.out.println(e.getMessage());
            throw new PlanningException(e.getMessage());
	}
        catch(IOException e)
        {
            System.out.println(e.getMessage());
	    throw new PlanningException(e.getMessage());
        }
    }
    
    public void executePlan()
    {
        logger.info("signal received for starting execution:\n");
        //this.engine.rpc(plan);
    }

    //cretaes list of action, ready to forward to other sas
//    public Action[] createListOfActions(LinkedList<LinkedList<CompoundTerm>> actionsList)
//    {
//        LinkedList<String> actions = actionsToString(actionsList);
//	Action[] listOfActions = new Action[actions.size()];
//	for(int i = 0; i < actions.size(); i++)
//	{
//	   Action a = new Action(i, actions.get(i), null, Completion.PENDING);
//	   listOfActions[i] = a;
//	}
//	return listOfActions;
//    }

    //generate string representation of actions generated by eclispe engine
    //public LinkedList<String> actionsToString(LinkedList<LinkedList<CompoundTerm>> result)
    public Action[] createListOfActions(LinkedList<LinkedList<CompoundTerm>> result)
    {
        Action[] listOfActions = new Action[result.size()];

        //LinkedList<String> actions = new LinkedList<String>();

            for(int j = 0; j < result.size(); j++)
                {
                    LinkedList<CompoundTerm> temp = (LinkedList<CompoundTerm>)result.get(j);
                    for(int k = 0; k < temp.size(); k++)
                    {
                        CompoundTerm c = (CompoundTerm)temp.get(k);

                        int actionID = j;
                        String actionName = c.functor();

                        //String s = c.functor() + "(";

                        Argument[] listOfArgs = new Argument[c.arity()];

                        for(int h = 1; h < c.arity() + 1; h++)
                        {
                            if (c.arg(h) == null)
                            {
                            	// the 2 lines below were fixed by Hendrik & Harmish, March 15, 2011
                                // listOfArgs[h-1] = new Argument("T","T");
                                listOfArgs[h-1] = new Argument("T",new WorkingMemoryPointer(new WorkingMemoryAddress("", ""), ""));
                                //s = s + "T" + new Integer(h).toString();
                                //if(h != c.arity())
                                    //s = s + ",";
                            }
                            else
                            {
                                Integer a = (Integer)c.arg(h);
                                // the 2 lines below were fixed by Hendrik & Harmish, March 15, 2011
                                // listOfArgs[h-1] = new Argument(a.toString(),a.toString());
                                listOfArgs[h-1] = new Argument(a.toString(),new WorkingMemoryPointer(new WorkingMemoryAddress("", ""), ""));
                                //s = s + a.functor();
                                //if(h != c.arity())
                                    //s = s + ",";
                            }
                        }
                        //s = s + ")";
                        //actions.addFirst(s);
                        listOfActions[j] = new Action(actionID,actionName,listOfArgs,Completion.PENDING);
                    }
                }
        return listOfActions;
    }*/
}
