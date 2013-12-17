package de.dfki.lt.tr.memory.conversion;

import java.util.HashSet;
import java.util.Iterator;
import java.util.Vector;

import de.dfki.lt.tr.memory.slice.MemoryCoreEvent;
import de.dfki.lt.tr.memory.slice.MemoryCueMatchMode;
import de.dfki.lt.tr.memory.slice.MemoryEventCue;
import de.dfki.lt.tr.memory.slice.MemoryEventLocation;
import de.dfki.lt.tr.memory.slice.MemoryEventSpecificFeaturesEntry;
import de.dfki.lt.tr.memory.slice.MemoryEventToStore;
import de.dfki.lt.tr.memory.slice.MemoryPartialEventToStore;
import de.dfki.lt.tr.memory.slice.MemoryStoredEvent;
import elm.event.Event;
import elm.event.EventID;
import elm.event.EventLocation;
import elm.event.EventLocationFactory;
import elm.event.EventSpecificFeatures;
import elm.event.EventTemplate;
import elm.event.EventType;
import elm.event.WKTParseException;
import elm.event.EventTime;
import java.util.Date;

/**
 * EventConverter is a class which can perform all sorts of conversions between
 * the (core) ELM system's data structures and those of the C-ELM layer.
 *
 * @author Dennis Stachowicz
 */
public class EventConverter {

	private EventLocationFactory elFactory = new EventLocationFactory();

	public MemoryCoreEvent getCoreEvent(Event e) {

		byte[] data = e.getEventSpecificBinaryDataAsByteArray();

		return new MemoryCoreEvent(e.getEventType().getName(), EventTimeConverter
				.toCEventTime(e.getTime()), new MemoryEventLocation(e
				.getLocation().getWKTString()), e.isApexEvent(), e.getDegree(),
				eventIDVectorToLongArray(e.getSubEventIDs()),
				toCEventSpecificFeatures(e.getEventSpecificFeatures()),
				data == null ? new byte[0] : data);

	}

	public MemoryStoredEvent getStoredEvent(Event e) {
		return new MemoryStoredEvent(e.getEventID().getLong(), getCoreEvent(e));
	}

	public MemoryEventToStore getEventToStore(Event e) {
		return new MemoryEventToStore(getCoreEvent(e));
	}

	public MemoryEventToStore getEventToStore(MemoryPartialEventToStore e,
			MemoryEventLocation location) {

		return new MemoryEventToStore(getCoreEvent(e, location));
	}

	public MemoryCoreEvent getCoreEvent(MemoryPartialEventToStore e,
			MemoryEventLocation location) {

		return new MemoryCoreEvent(e.eventType, e.eventTime, location, true,
					 Event.degreeAtomic, new long[0],
					 e.eventSpecificFeatures, e.eventSpecificBinaryData);
	}

	public Event getEvent(MemoryCoreEvent cevent) throws WKTParseException {

		EventType eventType = new EventType(cevent.eventType);
		EventLocation location = elFactory
				.fromString(cevent.location.wktString);

		return new ConvEvent(null, cevent.apexEvent, cevent.eventDegree,
				eventType, EventTimeConverter.toEventTime(cevent.eventTime),
				location, cevent.eventSpecificBinaryData,
				longArrayToEventIDVector(cevent.subevents),
				toEventSpecificFeatures(cevent.eventSpecificFeatures));
	}

	public Event getEvent(MemoryEventToStore cevent) throws WKTParseException {

		return getEvent(cevent.event);
	}

	public Event getEvent(MemoryStoredEvent cevent) throws WKTParseException {

		EventType eventType = new EventType(cevent.event.eventType);
		EventLocation location = elFactory
				.fromString(cevent.event.location.wktString);

		return new ConvEvent(
				new EventID(cevent.eventID),
				cevent.event.apexEvent,
				cevent.event.eventDegree,
				eventType,
				EventTimeConverter.toEventTime(cevent.event.eventTime),
				location,
				cevent.event.eventSpecificBinaryData,
				longArrayToEventIDVector(cevent.event.subevents),
				toEventSpecificFeatures(cevent.event.eventSpecificFeatures));
	}

        public Event getEvent(byte[] belief) throws WKTParseException
        {
            EventLocation location = elFactory.fromString("POLYGON((1 1,5 1,5 5,1 5,1 1),(2 2,2 3,3 3,3 2,2 2))");
            return new ConvEvent(
                    null,
                    true,
                    0,
                    new EventType("type"),
                    new EventTime(new Date(), new Date()),
                    location,
                    belief,
                    null,
                    null
                    );
        }

	public MemoryEventCue getEventCue(EventTemplate template)
			throws ConversionException {

		MemoryEventCue cue = new MemoryEventCue();

		cue.matchEventID = template.minEventID != null
				|| template.maxEventID != null;

		if (template.minEventID != null)
			cue.minEventID = template.minEventID.getLong();
		if (template.maxEventID != null)
			cue.maxEventID = template.maxEventID.getLong();

		if (template.eventType != null) {
			cue.matchEventType = true;
			cue.eventType = template.eventType.getName();
			cue.eventTypeIncludeSubtypes = !template.exactTypeMatch;
		} else {
			cue.matchEventType = false;
			cue.eventType = "";
		}
		cue.matchApexEvent = template.matchApexEvent;
		cue.apexEvent = template.apexEvent;

		cue.matchEventDegree = template.minDegree != elm.event.Event.degreeUndefined
				|| template.maxDegree != elm.event.Event.degreeUndefined;
		cue.minEventDegree = (template.minDegree != elm.event.Event.degreeUndefined ? template.minDegree
				: Integer.MIN_VALUE);
		cue.maxEventDegree = (template.maxDegree != elm.event.Event.degreeUndefined ? template.maxDegree
				: Integer.MAX_VALUE);

		cue.subeventMatchMode = getMatchMode(template.subEventMatchMode);
		cue.subevents = eventIDVectorToLongArray(template.subEventIDs);

		cue.supereventMatchMode = getMatchMode(template.superEventMatchMode);
		cue.superevents = eventIDVectorToLongArray(template.superEventIDs);

		cue.timeMatchMode = getMatchMode(template.timeMatchMode);
                if (template.time != null)
                {
                    cue.eventTime = EventTimeConverter.toCEventTime(template.time);
                }
                else
                {
                    cue.eventTime = null;
                }

		cue.locationMatchMode = getMatchMode(template.locationMatchMode);
		if (template.locationMatchMode != EventTemplate.noMatch)
			cue.location = new MemoryEventLocation(template.location
					.getWKTString());
		else
			cue.location = new MemoryEventLocation("");

		cue.esfMatchMode = getMatchMode(template.esfMatchMode);
		cue.esfRestrictMatchToKeys = template.esfRestrictMatchToKeys;
		cue.eventSpecificFeatures = toCEventSpecificFeatures(template.esf);

		if (template.binaryEventData == null) {
			cue.matchEventSpecificBinaryData = false;
			cue.eventSpecificBinaryData = new byte[0];
		} else {
			cue.matchEventSpecificBinaryData = true;
			cue.eventSpecificBinaryData = template.binaryEventData;
		}

		return cue;
	}

	public EventTemplate getEventTemplate(MemoryEventCue cue)
			throws WKTParseException, ConversionException {

		EventTemplate template = new EventTemplate();

		if (cue.matchEventID) {
			template.minEventID = new EventID(cue.minEventID);
			template.maxEventID = new EventID(cue.maxEventID);
		}

		if (cue.matchEventType) {
			template.eventType = new EventType(cue.eventType);
			template.exactTypeMatch = !cue.eventTypeIncludeSubtypes;
		}

		template.matchApexEvent = cue.matchApexEvent;
		template.apexEvent = cue.apexEvent; // getTemplateTriBool(cue.apexEvent);

		if (cue.matchEventDegree) {
			template.minDegree = cue.minEventDegree;
			template.maxDegree = cue.maxEventDegree;
		}

		template.subEventIDs = longArrayToEventIDVector(cue.subevents);
		template.subEventMatchMode = getMatchMode(cue.subeventMatchMode);

		template.superEventIDs = longArrayToEventIDVector(cue.superevents);
		template.superEventMatchMode = getMatchMode(cue.supereventMatchMode);

		template.time = EventTimeConverter.toEventTime(cue.eventTime);
		template.timeMatchMode = getMatchMode(cue.timeMatchMode);

		if (cue.location != null) {
			template.location = elFactory.fromString(cue.location.wktString);
		}
		else {
			assert(cue.locationMatchMode == MemoryCueMatchMode.noMatch);
		}
		template.locationMatchMode = getMatchMode(cue.locationMatchMode);

		template.esfMatchMode = getMatchMode(cue.esfMatchMode);
		template.esfRestrictMatchToKeys = cue.esfRestrictMatchToKeys;
		template.esf = toEventSpecificFeatures(cue.eventSpecificFeatures);

		if (cue.matchEventSpecificBinaryData)
			template.binaryEventData = cue.eventSpecificBinaryData;

		return template;
	}

	public static MemoryCueMatchMode getMatchMode(int imm)
			throws ConversionException {

		if (imm == EventTemplate.noMatch)
			return MemoryCueMatchMode.noMatch;
		else if (imm == EventTemplate.matchExact)
			return MemoryCueMatchMode.matchExact;
		else if (imm == EventTemplate.matchSubset)
			return MemoryCueMatchMode.matchSubset;
		else if (imm == EventTemplate.matchSuperset)
			return MemoryCueMatchMode.matchSuperset;
		else if (imm == EventTemplate.matchIntersectionNotEmpty)
			return MemoryCueMatchMode.matchIntersectionNotEmpty;
		else
			throw new ConversionException("unknown match mode: " + imm);

	}

	public static int getMatchMode(MemoryCueMatchMode cmm)
			throws ConversionException {

		if (cmm == MemoryCueMatchMode.noMatch)
			return EventTemplate.noMatch;
		else if (cmm == MemoryCueMatchMode.matchExact)
			return EventTemplate.matchExact;
		else if (cmm == MemoryCueMatchMode.matchSubset)
			return EventTemplate.matchSubset;
		else if (cmm == MemoryCueMatchMode.matchSuperset)
			return EventTemplate.matchSuperset;
		else if (cmm == MemoryCueMatchMode.matchIntersectionNotEmpty)
			return EventTemplate.matchIntersectionNotEmpty;
		else
			throw new ConversionException("unknown match mode: " + cmm.name());
	}

	public static Vector<EventID> longArrayToEventIDVector(long[] l) {

		if (l == null)
			return null;

		Vector<EventID> ev = new Vector<EventID>(l.length);

		for (long li : l)
			ev.add(new EventID(li));

		return ev;
	}


	public static long[] eventIDVectorToLongArray(Vector<EventID> ev) {

		if (ev == null)
			return new long[0];

		long[] l = new long[ev.size()];
		for (int i = 0; i < l.length; i++)
			l[i] = ev.get(i).getLong();

		return l;
	}


	public static MemoryEventSpecificFeaturesEntry[] toCEventSpecificFeatures(
			EventSpecificFeatures data) {

		if (data == null)
			return new MemoryEventSpecificFeaturesEntry[0];

		MemoryEventSpecificFeaturesEntry[] entryArray = new MemoryEventSpecificFeaturesEntry[data
				.size()];

		Iterator<String> keyIterator = data.keySet().iterator();

		for (int i = 0; i < entryArray.length && keyIterator.hasNext(); i++) {
			String featureName = keyIterator.next();
			Iterator<String> valuesIterator = data.get(featureName).iterator();

			Vector<String> valuesVector = new Vector<String>();
			while (valuesIterator.hasNext())
				valuesVector.add(valuesIterator.next());

			entryArray[i] = new MemoryEventSpecificFeaturesEntry(featureName,
					valuesVector.toArray(new String[0]));
		}

		return entryArray;
	}

	public static EventSpecificFeatures toEventSpecificFeatures(
			MemoryEventSpecificFeaturesEntry[] entryArray) {

		if (entryArray == null)
			return null;

		EventSpecificFeatures data = new EventSpecificFeatures(
				entryArray.length);

		for (int i = 0; i < entryArray.length; i++) {
			HashSet<String> hs = new HashSet<String>(
					entryArray[i].featureValues.length);
			for (int j = 0; j < entryArray[i].featureValues.length; j++)
				hs.add(entryArray[i].featureValues[j]);
			data.put(entryArray[i].featureName, hs);
		}

		return data;
	}

}