package eu.nifti.context.cast;

import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import eu.nifti.context.ref.AbstractWMInterface;
import eu.nifti.context.ref.Cache;
import eu.nifti.mapping.CarObject;
import eu.nifti.mapping.MapObject;
import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.Map;
import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.DefaultListModel;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.ListSelectionModel;

public class MapContextDebugger
extends ManagedComponent {

	private Viewer viewer = null;
	private String subarch = null;

	@Override
	public void configure(Map<String, String> args) {
		super.configure(args);
		this.subarch = this.getSubarchitectureID();
		log("will be using WM \"" + subarch + "\"");
	}

	@Override
	public void start() {
		viewer = new Viewer();

		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(MapObject.class,  WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleMapObjectAdd(_wmc);
					}
				});
		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(MapObject.class,  WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleMapObjectOverwrite(_wmc);
					}
				});
		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(MapObject.class,  WorkingMemoryOperation.DELETE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleMapObjectDelete(_wmc);
					}
				});
	}

	private void handleMapObjectAdd(WorkingMemoryChange _wmc) {
		try {
			MapObject mo = getMemoryEntry(_wmc.address, MapObject.class);
			viewer.add(_wmc.address, mo);
		}
		catch (DoesNotExistOnWMException ex) {
			log("caught DoesNotExistOnWMException: " + ex.message);
		}
		catch (UnknownSubarchitectureException ex) {
			log("caught UnknownSubarchitectureException(" + ex.subarchitecture + "): " + ex.message);
		}
	}

	private void handleMapObjectOverwrite(WorkingMemoryChange _wmc) {
		try {
			MapObject mo = getMemoryEntry(_wmc.address, MapObject.class);
			viewer.overwrite(_wmc.address, mo);
		}
		catch (DoesNotExistOnWMException ex) {
			log("caught DoesNotExistOnWMException: " + ex.message);
		}
		catch (UnknownSubarchitectureException ex) {
			log("caught UnknownSubarchitectureException(" + ex.subarchitecture + "): " + ex.message);
		}
	}

	private void handleMapObjectDelete(WorkingMemoryChange _wmc) {
		viewer.delete(_wmc.address);
	}

	@Override
	protected void runComponent() {
	}

	public void deleteItem(WorkingMemoryAddress addr) {
		log("removing address [" + addr.id + "," + addr.subarchitecture + "]");
		try {
			deleteFromWorkingMemory(addr);
		}
		catch (DoesNotExistOnWMException ex) {
			log("caught DoesNotExistOnWMException: " + ex.message);
		}
		catch (PermissionException ex) {
			log("caught PermissionException: " + ex.message);
		}
		catch (UnknownSubarchitectureException ex) {
			log("caught UnknownSubarchitectureException(" + ex.subarchitecture + "): " + ex.message);
		}
	}

	public void addMapObject() {
		MapObject mo = new MapObject();

		WorkingMemoryAddress wma = new WorkingMemoryAddress(newDataID(), subarch);
		log("adding a new map object to [" + wma.id + "," + wma.subarchitecture + "]");
		try {
			addToWorkingMemory(wma, mo);
		}
		catch (AlreadyExistsOnWMException ex) {
			log("caught AlreadyExistsOnWMException: " + ex.message);
		}
		catch (DoesNotExistOnWMException ex) {
			log("caught DoesNotExistOnWMException: " + ex.message);
		}
		catch (UnknownSubarchitectureException ex) {
			log("caught UnknownSubarchitectureException(" + ex.subarchitecture + "): " + ex.message);
		}
	}

	public void addCarObject(String label) {
		CarObject co = new CarObject();
		co.label = label;
		co.carClass = "citroen-c6";

		WorkingMemoryAddress wma = new WorkingMemoryAddress(newDataID(), subarch);
		log("will add a new car (label=\"" + co.label + "\", " + co.carClass + ") to " + wma.id + "," + wma.subarchitecture + "]");
		try {
			addToWorkingMemory(wma, co);
		}
		catch (AlreadyExistsOnWMException ex) {
			log("caught AlreadyExistsOnWMException: " + ex.message);
		}
		catch (DoesNotExistOnWMException ex) {
			log("caught DoesNotExistOnWMException: " + ex.message);
		}
		catch (UnknownSubarchitectureException ex) {
			log("caught UnknownSubarchitectureException(" + ex.subarchitecture + "): " + ex.message);
		}
	}

	private interface ModelItem {
		@Override
		public String toString();

		public WorkingMemoryAddress getAddress();
	}

	private class MapObjectModelItem implements ModelItem {
		protected WorkingMemoryAddress addr;
		private MapObject mo;

		public MapObjectModelItem(WorkingMemoryAddress addr, MapObject mo) {
			this.addr = addr;
			this.mo = mo;
		}

		@Override
		public String toString() {
			return "[" + addr.id + "," + addr.subarchitecture + "] MapObject";
		}

		public WorkingMemoryAddress getAddress() {
			return addr;
		}
	}

	private class CarObjectModelItem implements ModelItem {
		protected WorkingMemoryAddress addr;
		private CarObject co;

		public CarObjectModelItem(WorkingMemoryAddress addr, CarObject co) {
			this.addr = addr;
			this.co = co;
		}

		@Override
		public String toString() {
			return "[" + addr.id + "," + addr.subarchitecture + "] CarObject (\"" + co.label + "\"" + co.carClass + ")";
		}

		public WorkingMemoryAddress getAddress() {
			return addr;
		}
	}

	private class Viewer
	extends JFrame
	implements AbstractWMInterface<WorkingMemoryAddress, MapObject> {

		private DefaultListModel model;
		private JList list;

		String[] selections = new String[0];
		Cache<WorkingMemoryAddress, MapObject> cache = new Cache<WorkingMemoryAddress, MapObject>();

		private String lastSelection = null;
		private int idx = 1;

		public Viewer() {
			setTitle("Map Context Debugger");

			model = new DefaultListModel();

			JPanel panel = new JPanel();
			panel.setLayout(new BoxLayout(panel, BoxLayout.X_AXIS));

			JPanel leftPanel = new JPanel();
			JPanel rightPanel = new JPanel();

			leftPanel.setLayout(new BorderLayout());
			rightPanel.setLayout(new BoxLayout(rightPanel, BoxLayout.Y_AXIS));

			list = new JList(model);
			list.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
			list.setBorder(BorderFactory.createEmptyBorder(2, 2, 2, 2));

			list.addMouseListener(new MouseAdapter() {

				@Override
				public void mouseClicked(MouseEvent e) {
					if(e.getClickCount() == 1) {
						if (!model.isEmpty()) {
							deleteMapObject(list.locationToIndex(e.getPoint()));
						}
					}
				}
			});

			JScrollPane pane = new JScrollPane();
			pane.getViewport().add(list);
			leftPanel.setBorder(BorderFactory.createEmptyBorder(20, 20, 20, 20));

			leftPanel.add(pane);

			JButton newMapObjectButton = new JButton("MapObject");
			JButton newCarObjectButton = new JButton("CarObject");
			newCarObjectButton.setMaximumSize(newMapObjectButton.getMaximumSize());

			newMapObjectButton.addActionListener(new ActionListener() {
				public void actionPerformed(ActionEvent e) {
					addMapObject();
				}
			});

			newCarObjectButton.addActionListener(new ActionListener() {
				public void actionPerformed(ActionEvent e) {
					addCarObject(newCarId());
				}
			});

			rightPanel.add(newMapObjectButton);
			rightPanel.add(Box.createRigidArea(new Dimension(0,4)));
			rightPanel.add(newCarObjectButton);
			rightPanel.add(Box.createRigidArea(new Dimension(0,4)));

			rightPanel.setBorder(BorderFactory.createEmptyBorder(0, 0, 0, 20));

			panel.add(leftPanel);
			panel.add(rightPanel);

			add(panel);

			setSize(350, 250);
			setLocationRelativeTo(null);
			setDefaultCloseOperation(EXIT_ON_CLOSE);
			setVisible(true);
		}

		private String newCarId() {
			return "car" + ":" + idx++;
		}

		public ModelItem mapObjectToModelItem(WorkingMemoryAddress addr, MapObject obj) {
			if (obj instanceof CarObject) {
				CarObject co = (CarObject) obj;
				return new CarObjectModelItem(addr, co);
			}
			else {
				return new MapObjectModelItem(addr, obj);
			}
		}

		public void add(WorkingMemoryAddress addr, MapObject obj) {
			cache.add(addr, obj);
			model.addElement(mapObjectToModelItem(addr, obj));
		}

		public void overwrite(WorkingMemoryAddress addr, MapObject obj) {
			for (int i = 0; i < model.size(); i++) {
				Object o = model.get(i);
				if (o instanceof ModelItem) {
					ModelItem mi = (ModelItem) o;
					if (mi.getAddress().equals(addr)) {
						// okay, rewrite
						model.set(i, mapObjectToModelItem(addr, obj));
					}
				}
			}
			cache.overwrite(addr, obj);
		}

		public void delete(WorkingMemoryAddress addr) {
			removeFromList(addr);
			cache.delete(addr);
		}

		private void deleteMapObject(int index) {
			Object o = model.get(index);
			if (o instanceof ModelItem) {
				ModelItem mi = (ModelItem) o;
				deleteItem(mi.getAddress());
			}
		}

		private void removeFromList(WorkingMemoryAddress addr) {
			int index = -1;
			for (int i = 0; i < model.size(); i++) {
				Object o = model.get(i);
				if (o instanceof ModelItem) {
					ModelItem mi = (ModelItem) o;
					if (mi.getAddress().equals(addr)) {
						index = i;
					}
				}
			}
			if (index != -1) {
				model.remove(index);
			}
		}
	}

}
