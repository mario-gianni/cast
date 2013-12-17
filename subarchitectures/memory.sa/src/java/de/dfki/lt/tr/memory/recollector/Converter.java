package de.dfki.lt.tr.memory.recollector;

import de.dfki.lt.tr.memory.slice.MemoryCoreEvent;
import de.dfki.lt.tr.memory.slice.MemoryCueMatchMode;
import de.dfki.lt.tr.memory.slice.MemoryEventCue;
import de.dfki.lt.tr.memory.slice.MemoryEventLocation;
import de.dfki.lt.tr.memory.slice.MemoryEventSpecificFeaturesEntry;
import de.dfki.lt.tr.memory.slice.MemoryStoredEvent;
import de.dfki.lt.tr.memory.conversion.EventTimeConverter;
import elm.event.Event;
import elm.event.EventID;
import elm.event.EventLocationFactory;
import elm.event.EventSpecificFeatures;
import elm.event.EventTemplate;
import elm.event.EventType;
import elm.event.WKTParseException;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Vector;

public class Converter
{
    private EventLocationFactory elFactory = new EventLocationFactory();

    public Converter()
    {
    }

    public EventTemplate getEventTemplate(MemoryEventCue cue) throws WKTParseException, Exception
    {
        EventTemplate template = new EventTemplate();

        if (cue.matchEventID)
        {
            template.minEventID = new EventID(cue.minEventID);
            template.maxEventID = new EventID(cue.maxEventID);
        }

        if (cue.matchEventType)
        {
            template.eventType = new EventType(cue.eventType);
            template.exactTypeMatch = !cue.eventTypeIncludeSubtypes;
        }

        template.matchApexEvent = cue.matchApexEvent;
        template.apexEvent = cue.apexEvent; // getTemplateTriBool(cue.apexEvent);

        if (cue.matchEventDegree)
        {
            template.minDegree = cue.minEventDegree;
            template.maxDegree = cue.maxEventDegree;
        }

        template.subEventIDs = longArrayToEventIDVector(cue.subevents);
        template.subEventMatchMode = getMatchMode(cue.subeventMatchMode);

        template.superEventIDs = longArrayToEventIDVector(cue.superevents);
        template.superEventMatchMode = getMatchMode(cue.supereventMatchMode);

        template.time = EventTimeConverter.toEventTime(cue.eventTime);
        template.timeMatchMode = getMatchMode(cue.timeMatchMode);

        if (cue.location != null)
        {
            template.location = elFactory.fromString(cue.location.wktString);
        }
        else
        {
            assert(cue.locationMatchMode == MemoryCueMatchMode.noMatch);
        }

        template.locationMatchMode = getMatchMode(cue.locationMatchMode);
	template.esfMatchMode = getMatchMode(cue.esfMatchMode);
	template.esfRestrictMatchToKeys = cue.esfRestrictMatchToKeys;
	template.esf = toEventSpecificFeatures(cue.eventSpecificFeatures);

        if (cue.matchEventSpecificBinaryData)
        {
            template.binaryEventData = cue.eventSpecificBinaryData;
        }

        return template;
    }

    public static Vector<EventID> longArrayToEventIDVector(long[] l)
    {
        if (l == null)
        {
            return null;
        }

        Vector<EventID> ev = new Vector<EventID>(l.length);
        for (long li : l)
        {
            ev.add(new EventID(li));
        }

        return ev;
    }

    public static long[] eventIDVectorToLongArray(Vector<EventID> ev)
    {
        if (ev == null)
        {
            return new long[0];
        }

        long[] l = new long[ev.size()];
        for (int i = 0; i < l.length; i++)
        {
            l[i] = ev.get(i).getLong();
        }

        return l;
    }

    public static int getMatchMode(MemoryCueMatchMode cmm) throws Exception
    {
		switch (cmm) {
			case noMatch:
				return EventTemplate.noMatch;

			case matchExact:
				return EventTemplate.matchExact;

			case matchSubset:
				return EventTemplate.matchSubset;

			case matchSuperset:
				return EventTemplate.matchSuperset;

			case matchIntersectionNotEmpty:
				return EventTemplate.matchIntersectionNotEmpty;

			default:
				throw new Exception("unknown match mode: " + cmm.name());
		}
    }

    public static EventSpecificFeatures toEventSpecificFeatures(MemoryEventSpecificFeaturesEntry[] entryArray)
    {
        if (entryArray == null)
        {
            return null;
        }

        EventSpecificFeatures data = new EventSpecificFeatures(entryArray.length);

        for (int i = 0; i < entryArray.length; i++)
        {
            HashSet<String> hs = new HashSet<String>(entryArray[i].featureValues.length);
            for (int j = 0; j < entryArray[i].featureValues.length; j++)
            {
                hs.add(entryArray[i].featureValues[j]);
            }
            data.put(entryArray[i].featureName, hs);
        }

        return data;
    }

    public static MemoryEventSpecificFeaturesEntry[] toCEventSpecificFeatures(EventSpecificFeatures data)
    {
        if (data == null)
        {
            return new MemoryEventSpecificFeaturesEntry[0];
        }

        MemoryEventSpecificFeaturesEntry[] entryArray = new MemoryEventSpecificFeaturesEntry[data.size()];

        Iterator<String> keyIterator = data.keySet().iterator();

        for (int i = 0; i < entryArray.length && keyIterator.hasNext(); i++)
        {
            String featureName = keyIterator.next();
            Iterator<String> valuesIterator = data.get(featureName).iterator();

            Vector<String> valuesVector = new Vector<String>();
            while (valuesIterator.hasNext())
            {
                valuesVector.add(valuesIterator.next());
            }

            entryArray[i] = new MemoryEventSpecificFeaturesEntry(featureName, valuesVector.toArray(new String[0]));
        }

        return entryArray;
    }

    public MemoryStoredEvent getStoredEvent(Event e)
    {
        return new MemoryStoredEvent(e.getEventID().getLong(), getCoreEvent(e));
    }

    public MemoryCoreEvent getCoreEvent(Event e)
    {
        byte[] data = e.getEventSpecificBinaryDataAsByteArray();

        return new MemoryCoreEvent(e.getEventType().getName(), EventTimeConverter.toCEventTime(e.getTime()),
                new MemoryEventLocation(e.getLocation().getWKTString()), e.isApexEvent(), e.getDegree(),
                eventIDVectorToLongArray(e.getSubEventIDs()),
                toCEventSpecificFeatures(e.getEventSpecificFeatures()),
                data == null ? new byte[0] : data);
    }
}
