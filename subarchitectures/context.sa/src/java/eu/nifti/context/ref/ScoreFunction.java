package eu.nifti.context.ref;

public interface ScoreFunction {

	/**
	 * Apply the right operand on the left one.
	 *
	 * @param left
	 * @param right
	 * @return
	 */
	double apply(double left, double right, double max_left, double max_right);

}
