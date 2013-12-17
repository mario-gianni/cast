package de.dfki.lt.tr.dialogue.time;

import de.dfki.lt.tr.dialogue.util.ConvertibleToIce;
import de.dfki.lt.tr.dialogue.slice.time.TimePoint;

public class Point implements ConvertibleToIce<TimePoint> {

	private final long msec;

	public Point(long msec) {
		this.msec = msec;
	}

	public Point(TimePoint tp) {
		this(tp.msec);
	}

	@Override
	public TimePoint toIce() {
		return new TimePoint(msec);
	}

	public long getMSec() {
		return msec;
	}

}
