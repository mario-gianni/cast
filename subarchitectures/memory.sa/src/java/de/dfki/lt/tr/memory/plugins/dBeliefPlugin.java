package de.dfki.lt.tr.memory.plugins;

import Ice.ObjectImpl;
import de.dfki.lt.tr.beliefs.slice.framing.SpatioTemporalFrame;
import de.dfki.lt.tr.beliefs.slice.framing.TemporalInterval;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import elm.event.AtomicEvent;
import elm.event.EventLocation;
import elm.event.EventLocationFactory;
import elm.event.EventTime;
import elm.event.EventType;
import elm.event.WKTParseException;
import org.apache.log4j.Logger;

/*
 * plugin for dBelief class, converts dBelief to elm AtomicEvent
 */
public class dBeliefPlugin implements Plugin
{
    private Logger logger = Logger.getLogger(dBeliefPlugin.class);
    
    public AtomicEvent toEvent(ObjectImpl iceObject) 
    {
        if (iceObject != null && iceObject instanceof dBelief)
        {
            dBelief belief = (dBelief)iceObject;
            AtomicEvent event = new AtomicEvent(new EventType("dBelief"), null,
                        null, new byte[0], null, null);
            if (belief.frame != null && belief.frame instanceof SpatioTemporalFrame)
            {
                SpatioTemporalFrame stFrame = (SpatioTemporalFrame)belief.frame;
                event.setTime(convertTime(stFrame.interval));
                event.setLocation(convertLocation(stFrame.place));
            }
            return event;
        }
        else
        {
            logger.error("dBeliefPlugin: cannot convert belief to event, "
                    + "given iceObject is not instance of dBelief!");
            return null;
        }
    }

    private EventTime convertTime(TemporalInterval interval)
    {
        //TODO: look for TemporalInterval type which is not there in slice definition!
        long time = System.currentTimeMillis();
        return new EventTime(time, time);
    }

    private EventLocation convertLocation(String location)
    {
        try
        {
            return new EventLocationFactory().fromString(location);
        }
        catch (WKTParseException ex)
        {
            logger.error("cannot convert belief location.\n" + ex.getMessage()
                    + "\nwill use locatin (0,0)");
            double[] pos = { 0.0, 0.0 };
            return new EventLocationFactory().fromPoint(pos, 0.0001);
        }
    }
}
