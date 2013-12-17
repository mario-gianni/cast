package eu.nifti.context.ref;

public interface PotentialCombinator {

	/**
	 * Add a potential to the combination.
	 *
	 * @param p the potential
	 */
	public void addPotential(Potential p);

	/**
	 * Return the corresponding potential.
	 *
	 * @return the potential
	 */
	public Potential toPotential();

}
