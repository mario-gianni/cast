package de.dfki.lt.tr.memory.conversion;

import java.util.Date;
import elm.event.*;
import de.dfki.lt.tr.memory.slice.*;

/**
 * converts between different data structures used for event times.
 */
public class EventTimeConverter {

	public static MemoryEventTime toCEventTime(Date begin, Date end) {
		return toCEventTime(begin.getTime(), end.getTime());
	}

	public static MemoryEventTime toCEventTime(long begin, long end) {
		return new MemoryEventTime(new MemoryEventTimestamp(begin),
				new MemoryEventTimestamp(end));
	}

	public static MemoryEventTime toCEventTime(EventTime e) {
		return toCEventTime(e.getMicroTimeBegin(), e.getMicroTimeEnd());
	}

	public static EventTime toEventTime(MemoryEventTime c) {
		if (c == null) {
			return null;
		} else {
			return new EventTime(c.begin.milliseconds, c.end.milliseconds);
		}
	}

}