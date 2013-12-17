package de.dfki.lt.tr.cast.dialogue;

import de.dfki.lt.tr.dialogue.ref.ReferenceResolver;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import eu.nifti.context.slice.WhatIsForward;
import eu.nifti.context.slice.ThisIsForward;

public class DummyForwardResolver
extends AbstractDialogueComponent {

	public DummyForwardResolver() {
		super();
	}

	@Override
	protected void onStart() {
		super.onStart();

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(WhatIsForward.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						processForwardResolutionRequest(_wmc);
					};
		});
	}

	private void processForwardResolutionRequest(final WorkingMemoryChange _wmc) {
		getLogger().debug("got a callback for WhatIsForward [" + _wmc.address.id + "," + _wmc.address.subarchitecture + "]: " + _wmc.operation.toString());

		try {
			final WhatIsForward rr = this.getMemoryEntry(_wmc.address, WhatIsForward.class);
			addTask(new ProcessingTaskWithData<WorkingMemoryChange>(_wmc) {

				@Override
				public void execute(WorkingMemoryChange _wmc) {
					try {
						getLogger().debug("will act on the WhatIsForward [" + _wmc.address.id + "," + _wmc.address.subarchitecture + "]");

						// simulate the delay
						Thread.sleep(300);

						ThisIsForward tif = new ThisIsForward(-1.0, -1.0);
						getLogger().debug("adding a new ThisIsForward in response to " + wmaToString(_wmc.address));
						overwriteWorkingMemory(_wmc.address, tif);
					}
					catch (InterruptedException ex) {
						getLogger().error("interrupted", ex);
					}
					catch (SubarchitectureComponentException ex) {
						getLogger().error("subarch component exception", ex);
					}
				}
			});
		}
		catch (SubarchitectureComponentException ex) {
			getLogger().error("subarch component exception", ex);
		}
	}

}
