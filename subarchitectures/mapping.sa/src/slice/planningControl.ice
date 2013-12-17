#ifndef MAPPING_SA_PLANNINGCONTROL_ICE
#define MAPPING_SA_PLANNINGCONTROL_ICE

module eu
{
	module nifti
	{
		module mapping
		{
                    module planningControl
                    {
			enum State {COMPLETED, FAILED, PENDING, CONFIRMED, ABORTED, ADDED, DELETED, EXECUTED, REQUESTED};
			enum ActionMode {START, END, WAIT};
			class Control
			{
				string name;
				string component;
				double time;
				State status;
				ActionMode op;
			};
                    };
		};
	};
};

#endif
