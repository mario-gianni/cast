package eu.nifti.gui.testing;

// Java
import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;

import javax.swing.JFrame;
import javax.swing.JScrollPane;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.DefaultListModel;
import javax.swing.JButton;
import javax.swing.JList;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.ListSelectionModel;

import eu.nifti.gui.IGUIStateHandlerPrx;
import eu.nifti.gui.IGUIStateHandlerPrxHelper;

class DebugClickWindow
extends JFrame
{
	private DefaultListModel model;
	private JList list;

	String[] selections = { "Click ABC", "car1" };

	private static int freespaceIdx = 0;
	private static int objectIdx = 1;

	private String lastSelection = null;

	private static String adapterName = "ObjectAdapterForGUIState";
	private static String adapterEndpoints = "tcp -p 10124";
	private static String identityString = "IGUIStateHandlerIdentity";

	IGUIStateHandlerPrx prx = null;

	public DebugClickWindow(IGUIStateHandlerPrx _prx)
	{
		setTitle("Selection Testing Window");

		prx = _prx;

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
						int index = list.locationToIndex(e.getPoint());
						String arg = (String) model.get(index);
						if (prx != null) {
							prx.onDeselected(arg, currentTimeStamp());
						}
						model.remove(index);
					}
				}
			}
		});

		JScrollPane pane = new JScrollPane();
		pane.getViewport().add(list);
		leftPanel.setBorder(BorderFactory.createEmptyBorder(20, 20, 20, 20));

		leftPanel.add(pane);

		JButton newFreespace = new JButton("Freespace");
		JButton newObject = new JButton("Object");
		newObject.setMaximumSize(newFreespace.getMaximumSize());

		newFreespace.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				if (lastSelection != null) {
					if (model.removeElement(lastSelection)) {
						if (prx != null) {
							prx.onDeselected(lastSelection, currentTimeStamp());
						}
					}
				}
				lastSelection = newFreespaceString();
				model.addElement(lastSelection);
				if (prx != null) {
					prx.onSelected(lastSelection, currentTimeStamp());
				}
			}
		});

		newObject.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				if (lastSelection != null) {
					if (model.removeElement(lastSelection)) {
						if (prx != null) {
							prx.onDeselected(lastSelection, currentTimeStamp());
						}
					}
				}
				lastSelection = newObjectString();
				model.addElement(lastSelection);
				if (prx != null) {
					prx.onSelected(lastSelection, currentTimeStamp());
				}
			}
		});

		rightPanel.add(newFreespace);
		rightPanel.add(Box.createRigidArea(new Dimension(0,4)));
		rightPanel.add(newObject);
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

	public String newFreespaceString() {
		return "Click " + freespaceIdx++ + ".0 0.0 0.0";
	}

	public String newObjectString() {
		return "car" + objectIdx++;
	}

	public double currentTimeStamp() {
		return (double) System.currentTimeMillis() / 1000.0;
	}

	public static void main(String[] args) {
		Ice.Communicator ic = Ice.Util.initialize();
		IGUIStateHandlerPrx startPrx = null;

		try {
			Ice.ObjectPrx base = ic.stringToProxy(identityString + ":" + adapterEndpoints);
			startPrx = IGUIStateHandlerPrxHelper.checkedCast(base);
			if (startPrx == null) {
				System.err.println("Failed to create proxy");
			}
		}
		catch (Ice.LocalException e) {
			System.err.println("Ice local exception: " + e.toString());
		}

		if (startPrx != null) {
			System.err.println("All is well, we're connected.");
		}
		new DebugClickWindow(startPrx);
	}

};