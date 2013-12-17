package eu.nifti.context.ref;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;

public interface Referent {

	/**
	 * Convert the referent to a dFormula so that it can be included
	 * in the intention.
	 *
	 * @return corresponding dFormula
	 */
	public dFormula toFormula();

	@Override
	public boolean equals(Object r);

	@Override
	public int hashCode();
}
