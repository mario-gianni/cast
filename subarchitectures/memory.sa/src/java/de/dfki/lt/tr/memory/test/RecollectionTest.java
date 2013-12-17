/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package de.dfki.lt.tr.memory.test;

import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.memory.conversion.ConversionException;
import de.dfki.lt.tr.memory.conversion.EventConverter;
import de.dfki.lt.tr.memory.slice.MemoryEventQuery;
import de.dfki.lt.tr.memory.slice.MemoryStoredEvent;
import elm.event.EventTemplate;
import elm.event.EventType;
import eu.nifti.planning.slice.PlanningTask;
import java.io.ByteArrayInputStream;
import java.io.EOFException;
import java.io.IOException;
import java.io.ObjectInputStream;
import org.apache.log4j.Logger;

/**
 *
 * @author harmishhk
 */
public class RecollectionTest extends ManagedComponent
{
    private Logger logger;
    private EventConverter converter = new EventConverter();

    public RecollectionTest()
    {
        super();
        logger = Logger.getLogger(RecollectionTest.class);
    }

    @Override
    protected void start()
    {
        addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(MemoryEventQuery.class, WorkingMemoryOperation.OVERWRITE),
                new WorkingMemoryChangeReceiver()
                {
                    @Override
                    public void workingMemoryChanged(WorkingMemoryChange _wmc)
                    {
                        queryAnswered(_wmc);
                    }
                });
        super.start();
    }

    @Override
    protected void runComponent()
    {
        super.runComponent();

        //sleep for 1 sec for other stuff to finish
        sleepComponent(1000);

        try
        {
            EventTemplate template = new EventTemplate();
            template.eventType = new EventType("::eu::nifti::planning::slice::PlanningTask");
            MemoryEventQuery query = new MemoryEventQuery(0, converter.getEventCue(template),new MemoryStoredEvent[0]);
            addToWorkingMemory(newDataID(), query);
        }
        catch (AlreadyExistsOnWMException ex)
        {
            logger.debug("RecollectionTest: wm entry already exists");
        }
        catch (ConversionException ex)
        {
            logger.debug("RecollectionTest: conversion exception");
        }
    }

    private void queryAnswered(WorkingMemoryChange wmc)
    {
        try
        {
            MemoryEventQuery meq = getMemoryEntry(wmc.address, MemoryEventQuery.class);
            logger.debug("got resutls of query with " + meq.events.length + " entries");
            
            for(int i = 0; i < meq.events.length; i++)
            {
                MemoryStoredEvent event = meq.events[i];
                logger.debug("recollected event of type " + event.event.eventType + " with " + event.event.eventSpecificFeatures.length + " features");

                PlanningTask task = getPlanningTaskFromBytes(event.event.eventSpecificBinaryData);
                if (task != null)
                {
                    logger.debug("recoverd goal: " + task.goal);
                }
            }
        }
        catch (DoesNotExistOnWMException ex)
        {
            logger.debug("RecollectionTest: error while getting result of query\n" + ex);
        }
        catch (UnknownSubarchitectureException ex)
        {
            logger.debug("RecollectionTest: error while getting result of query\n" + ex);
        }

    }

    private PlanningTask getPlanningTaskFromBytes(byte[] planningTaskBytes)
    {
        ObjectInputStream ois = null;
        PlanningTask task = null;
        try
        {
            logger.debug("going to convert bytes");
            ByteArrayInputStream bis = new ByteArrayInputStream(planningTaskBytes);
            ois = new ObjectInputStream(bis);
            Object obj = ois.readObject();
            logger.debug("bytes converted");
            if (obj != null)
            {
                logger.debug("object is not null");
                //logger.debug("bytes converted, object is of type" + obj.getClass().getName());
                if(obj.getClass().equals(PlanningTask.class))
                {
                    task = (PlanningTask)obj;
                }
            }
        }
        catch (ClassNotFoundException ex)
        {
            logger.debug(ex);
        }
        catch (IOException ex)
        {
            logger.debug(ex);
        }
        catch (Exception ex)
        {
            logger.debug(ex);
        }
        finally
        {
            try
            {
                ois.close();
            }
            catch (IOException ex)
            {
                logger.debug(ex);
            }
            return task;
        }
    }
}
