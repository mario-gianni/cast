package eu.nifti.context.ref;

import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryPointer;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import eu.nifti.env.CarObjectOfInterest;
import eu.nifti.env.ElementOfInterest;
import eu.nifti.env.LocationOfInterest;

public class WorkingMemoryPointerReferent implements Referent {

	private final ManagedComponent component;
	protected final WorkingMemoryPointer wmp;
	protected final dFormula formula;
	
	public WorkingMemoryPointerReferent(ManagedComponent component, WorkingMemoryPointer wmp) throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		this.component = component;
		this.wmp = wmp;
		this.formula = wmPointerToFormula(wmp);
		
		assert (this.formula != null);
	}

	@Override
	public boolean equals(Object o) {
		if (o instanceof WorkingMemoryPointerReferent) {
			return toString().equals(((WorkingMemoryPointerReferent) o).toString());
		}
		else {
			return false;
		}
	}

	@Override
	public dFormula toFormula() {
		return formula;
	}

	private dFormula wmPointerToFormula(WorkingMemoryPointer wmp) throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		ElementOfInterest eoi = component.getMemoryEntry(wmp.address, ElementOfInterest.class);
		
		if (eoi instanceof LocationOfInterest) {
			LocationOfInterest loi = (LocationOfInterest) eoi;
			String s = Double.toString(loi.point.x) + "," + Double.toString(loi.point.y) + "," + Double.toString(loi.point.z);
			return new ModalFormula(0, "coordinates", new ElementaryFormula(-1, s));
		}
		if (eoi instanceof CarObjectOfInterest) {
			CarObjectOfInterest coi = (CarObjectOfInterest) eoi;
			return new ModalFormula(0, "object", new PointerFormula(0, wmp.address, wmp.type));
		}
		
		return null;
	}
	
}
