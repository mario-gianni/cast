package eu.nifti.wizard;

public class BasicWizardVerbalisation implements WizardVerbalisation {

	@Override
	public String getOk() {
		return "Okay.";
	}

	@Override
	public String getSorryCannotDo() {
		return "Sorry, I cannot do that.";
	}

	@Override
	public String getSorryDidNotUnderstand() {
		return "Sorry, I did not understand.";
	}

	@Override
	public String getShowMeOnTheMap() {
		return "Please show me on the map what you mean.";
	}

	@Override
	public String getUseTheGUIToDoThat() {
		return "Please use manual control to do that.";
	}

}
