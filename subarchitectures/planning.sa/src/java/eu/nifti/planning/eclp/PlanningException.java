/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package eu.nifti.planning.eclp;

/**
 *
 * @author harmishhk
 */
public class PlanningException extends Exception{

    public String exc;

    public PlanningException(String ex)
    {
        exc = ex;
    }


}
