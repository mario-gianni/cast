package eu.nifti.dialogue.actions;

import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPointer;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;
import java.util.LinkedList;
import java.util.List;

public abstract class MoveActionFactory {

        public static String dFormulaToEncodedString(dFormula f) {

                if (f instanceof ModalFormula) {
			ModalFormula mf = (ModalFormula) f;
			
			if (mf.op.equals("coordinates")) {
				if (!(mf.form instanceof ElementaryFormula)) {
					return null;
				}
				
				String argString = ((ElementaryFormula) mf.form).prop;

				String[] xyargs = argString.split(",");
				if (xyargs.length == 3) {
					double x = Double.parseDouble(xyargs[0]);
					double y = Double.parseDouble(xyargs[1]);
					double z = Double.parseDouble(xyargs[2]);
                            
                                        return "coordinates=" + Double.toString(x) + ","
                                                    + Double.toString(y) + ","
                                                    + Double.toString(z);
				}
				
			}
			else if (mf.op.equals("object")) {
				if (!(mf.form instanceof PointerFormula)) {
					return null;
				}
                                PointerFormula pf = (PointerFormula) mf.form;

                                return "object=" + pf.pointer.subarchitecture + "," + pf.pointer.id;
			}
		}
                return null;
        }
    
	public static Action dFormulaToAction(dFormula f) {
		
		if (f instanceof ModalFormula) {
			ModalFormula mf = (ModalFormula) f;
			
			if (mf.op.equals("coordinates")) {
				if (!(mf.form instanceof ElementaryFormula)) {
					return null;
				}
				
				String argString = ((ElementaryFormula) mf.form).prop;

				String[] xyargs = argString.split(",");
				if (xyargs.length == 3) {
					double x = Double.parseDouble(xyargs[0]);
					double y = Double.parseDouble(xyargs[1]);
					double z = Double.parseDouble(xyargs[2]);
					
					return new MoveToCoordinatesAction(x, y, z);
				}
				
			}
			else if (mf.op.equals("object")) {
				if (!(mf.form instanceof PointerFormula)) {
					return null;
				}
				
				WorkingMemoryAddress argAddress = ((PointerFormula) mf.form).pointer;
				return new MoveToObjectAction(argAddress);
			}
		}
		
		return null;
	}

	// FIXME FIXME XXX
	// this is duplicated code!!
	public static Action targetWMPToAction(WorkingMemoryPointer wmp) {
		dFormula f = new ModalFormula(0, "object", new PointerFormula(0, wmp.address, wmp.type));
		return dFormulaToAction(f);
	}
}
