package eu.nifti.context.ref;

import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;

public interface Potential {

	public Iterator<Entry<Referent, Double>> positiveElementsIterator();

	public Map<Referent, Double> asMap();

	public double getScore(Referent r);

	public void setScore(Referent r, double score);

	public double getMaxScore();

}
