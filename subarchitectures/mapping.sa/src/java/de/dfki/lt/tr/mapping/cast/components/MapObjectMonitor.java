package de.dfki.lt.tr.mapping.cast.components;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import eu.nifti.env.Point3D;
import eu.nifti.env.Polygon;
import eu.nifti.env.CarObjectOfInterest;
import eu.nifti.env.ObjectOfInterest;
import eu.nifti.env.Pose;
import eu.nifti.env.Window;
import eu.nifti.mapping.HFCInterface;
import eu.nifti.mapping.HFCInterfacePrx;
import eu.nifti.mapping.QueryResults;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.hfc.types.XsdFloat;
import de.dfki.lt.hfc.types.XsdInt;
import Ice.Identity;

import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.framing.AbstractFrame;
import de.dfki.lt.tr.beliefs.slice.intentions.CommunicativeIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionalContent;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BinaryOp;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula; 


/**
 * This is a simple monitor that listens for MapObjects on WM.
 * It then decides whether and how to represent them in the ontology,
 * triggers the reasoner, and performs necessary updated on the 
 * MapObject WME. 
 * 
 * @author zender
 * CAST file arguments:
 * --hfcserver-name String with the name of the HFC server component
 */
public class MapObjectMonitor extends ManagedComponent {

	Identity m_hfcserver_id;
	HFCInterfacePrx m_hfcserver;
	String m_dialogueSA;
	
	private static final int TIME_TO_WAIT_TO_SETTLE = 2000;
	private boolean verbTaskPending = false;
	private HashMap<String, Integer> verbTasks = new HashMap<String, Integer>();
	private LinkedList<String> verbTaskQ = new LinkedList<String>();
	
	public void configure(Map<String, String> args) {
		super.configure(args);
		log("configure() called");
		m_hfcserver_id = new Identity();
		m_hfcserver_id.name="";
		m_hfcserver_id.category="coma.components.HFCserver";

		if (args.containsKey("--hfcserver-name")) {
			m_hfcserver_id.name=args.get("--hfcserver-name");
		}
		
		if (args.containsKey("--dsa")) {
			this.m_dialogueSA=args.get("--dsa");
		} else {
			this.m_dialogueSA="";
		}
	}

	public void start() {
		// register the monitoring change filters
		// this is the "proper" Place monitor
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(ObjectOfInterest.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                            processAddedMapObject(_wmc);
			};
		});
		
		// register connection to HFC server via ICE
		log("Initiating connection to server " + m_hfcserver_id.category + "::" + m_hfcserver_id.name + "...");
		
		try {
			m_hfcserver = getIceServer(m_hfcserver_id.name, HFCInterface.class, HFCInterfacePrx.class);
		} catch (CASTException e) {
			e.printStackTrace();
			log("Connection to the HFCServer Ice server at "+ m_hfcserver_id.toString() + " failed! Exiting. (Specify the correct component name using --hfcserver-name)");
			System.exit(-1);
		}		
		initializeRobot(); 	
	}

	public void initializeRobot()
		{
			String individualRobotName = "<funcmap:nifti-ugv>";
			String[] newRobotTup = new String[]{individualRobotName, "<rdf:type>", "<funcmap:Robot>"};
			m_hfcserver.addTuple(newRobotTup);
			log("added tuple: " + newRobotTup[0] + " " + newRobotTup[1] + " " + newRobotTup[2]);
			String individualCameraName = "<funcmap:ugv-omnicamera>";
			String[] newCamTup = new String[]{individualCameraName, "<rdf:type>", "<funcmap:Ladybug3>"};
			m_hfcserver.addTuple(newCamTup);
			log("added tuple: " + newCamTup[0] + " " + newCamTup[1] + " " + newCamTup[2]);
			String[] newRobotCamTup = new String[]{individualRobotName, "<funcmap:hasPart>", individualCameraName};
			m_hfcserver.addTuple(newRobotCamTup);
			log("added tuple: " + newRobotCamTup[0] + " " + newRobotCamTup[1] + " " + newRobotCamTup[2]);
			String[] newXPosTup = new String[]{individualCameraName, "<funcmap:hasPosX>", "\"0.07\"^^<xsd:float>"};
			m_hfcserver.addTuple(newXPosTup);
			log("added tuple: " + newXPosTup[0] + " " + newXPosTup[1] + " " + newXPosTup[2]);
			String[] newYPosTup = new String[]{individualCameraName, "<funcmap:hasPosY>", "\"0.068\"^^<xsd:float>"};
			m_hfcserver.addTuple(newYPosTup);
			log("added tuple: " + newYPosTup[0] + " " + newYPosTup[1] + " " + newYPosTup[2]);
			String[] newZPosTup = new String[]{individualCameraName, "<funcmap:hasPosZ>", "\"0.342\"^^<xsd:float>"};
			m_hfcserver.addTuple(newZPosTup);
			log("added tuple: " + newZPosTup[0] + " " + newZPosTup[1] + " " + newZPosTup[2]);
			return;
		}
		
	private void processAddedMapObject(WorkingMemoryChange _wmc) {
		log("Got a callback for an ADDed MapObject WME.");
		// read MapObject WME
		ObjectOfInterest newMapObj;
		String objectType = "object";
		try {
			newMapObj = getMemoryEntry(_wmc.address, ObjectOfInterest.class);
			if (newMapObj instanceof CarObjectOfInterest) {
				log("The MapObject is a CarObject!");
				CarObjectOfInterest newCar = (CarObjectOfInterest) newMapObj;
				objectType = "car";
				boolean needToUpdateCarWME = false;
				
				/*// TODO this is test code, remove later!
				addChangeFilter(ChangeFilterFactory.createAddressFilter(_wmc.address), new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _mywmc)
							throws CASTException {
						log("TEST CODE: My WME caused a callback! Reason: " + _mywmc.operation.name());
					}
				});
				// TODO end test code!*/
				
				// generate name for car individual
				String individualName = "<funcmap:" + newCar.name.replace(":", "_") + "-" + _wmc.address.id.replace(":", "_") + ">";
				if (newCar.carClass.equals(""))  {
					newCar.carClass = "DefaultCar";
				} else {
					// todo: all cars are interpreted as default cars...
					newCar.carClass = "DefaultCar";
				}
				String carClassName = "<funcmap:" + newCar.carClass + ">";
				
				// add car instance to ontology
				String[] newTup = new String[]{individualName, "<rdf:type>", carClassName};
				m_hfcserver.addTuple(newTup);
				log("added tuple: " + newTup[0] + " " + newTup[1] + " " + newTup[2]);
				
				// TODO temporary: use an existing test instance as long as tuple adding does not work in hfc
				// individualName = "<funcmap:alfaromeo159sw>";

				// check special properties
				// dimensions:  
				// width <funcmap:hasWidth> "1828"^^<xsd:integer>
				float carwidth;
				String queryW = "SELECT ?width where " + individualName + " <funcmap:hasWidth> ?width";
				QueryResults resultsW = m_hfcserver.querySelect(queryW);
				if (resultsW.bt != null && resultsW.bt.length>0) {
					String _carwidth  =  resultsW.bt[0][0];
					carwidth = new XsdFloat(_carwidth).value;
					log(individualName + " has width " + carwidth);
				} else {
					log("could not determine width of the car! setting it to -1.");
					carwidth = -1;
				}
				
				// length
				float carlength;
				String queryL = "SELECT ?length where " + individualName + " <funcmap:hasLength> ?length";
				QueryResults resultsL = m_hfcserver.querySelect(queryL);
				if (resultsL.bt !=null && resultsL.bt.length>0) {
					String _carlength  =  resultsL.bt[0][0];
					carlength = new XsdFloat(_carlength).value;
					log(individualName + " has length " + carlength);
				} else {
					log("could not determine length of the car! setting it to -1.");
					carlength = -1;
				}
				
				// update car's dimensions
				//newCar.carDimensions = new Dimensions(carwidth, carlength); // Benoit: I removed this because it seems like it's not used
				
				if (carwidth >= 0.0 && carlength >=0.0) {
					needToUpdateCarWME = true;
				} else {
					log("could not determine dimensions of the car");
				}

				//Bounding Box
				Point3D _boundingBoxPnt1 = new Point3D(carlength/2,carwidth/2,0.00);
				Point3D _boundingBoxPnt2 = new Point3D(carlength/2,-carwidth/2,0.00);
				Point3D _boundingBoxPnt3 = new Point3D(-carlength/2,-carwidth/2,0.00);
				Point3D _boundingBoxPnt4 = new Point3D(-carlength/2,carwidth/2,0.00);				
				Point3D[] _polygonPoints = {_boundingBoxPnt1,_boundingBoxPnt2,_boundingBoxPnt3,_boundingBoxPnt4};
 				Polygon _boundingBoxPolygon = new Polygon(_polygonPoints);
				newCar.boundingBox = _boundingBoxPolygon;

				// window positions
				// ArrayList<Window> allWindows = new ArrayList<Window>();
				String queryAllWindows = "SELECT DISTINCT ?window WHERE " + individualName + " <funcmap:hasPart> ?window & ?window <rdf:type> <funcmap:Window> & ?window ?p ?num FILTER ?p != <rdf:type>";
				QueryResults resultsAllWindows = m_hfcserver.querySelect(queryAllWindows);
				Window[] allWindows = new Window[resultsAllWindows.bt.length];
				for (int i=0; i<resultsAllWindows.bt.length; i++) {
					for (int j=0; j<resultsAllWindows.bt[i].length; j++) {
						String _currWindow = resultsAllWindows.bt[i][j];
						// get its properties:
						float _width = new XsdFloat(m_hfcserver.querySelect("SELECT ?width WHERE " + _currWindow + " <funcmap:hasWidth> ?width ").bt[0][0]).value;
						float _height = new XsdFloat(m_hfcserver.querySelect("SELECT ?height WHERE " + _currWindow + " <funcmap:hasHeight> ?height ").bt[0][0]).value;
						float _theta = new XsdFloat(m_hfcserver.querySelect("SELECT ?theta WHERE " + _currWindow + " <funcmap:hasOrientation> ?theta ").bt[0][0]).value;
						float _xPos = new XsdFloat(m_hfcserver.querySelect("SELECT ?xpos WHERE " + _currWindow + " <funcmap:hasPosX> ?xpos ").bt[0][0]).value;
						float _yPos = new XsdFloat(m_hfcserver.querySelect("SELECT ?ypos WHERE " + _currWindow + " <funcmap:hasPosY> ?ypos ").bt[0][0]).value;
						float _zPos = new XsdFloat(m_hfcserver.querySelect("SELECT ?zpos WHERE " + _currWindow + " <funcmap:hasHeightAboveGround> ?zpos ").bt[0][0]).value;
						int _wIndex = new XsdInt(m_hfcserver.querySelect("SELECT ?windex WHERE " + _currWindow + " <funcmap:hasIndex> ?windex ").bt[0][0]).value;
						
                                                Pose pose = new Pose(_xPos, _yPos, _zPos, _theta*(3.1415/180));
						allWindows[_wIndex] = new Window(_width, _height, pose);
						log("created new window(x,y,z,theta,width,height): " + _xPos + ", " + _yPos + ", " + _zPos + ", " + _theta + ", " + _width + ", " + _height + ".Its index is " + _wIndex + ".");
					}
				}

				// Get position of the robot's camera
				String queryCameraRobotPosition = "SELECT ?x WHERE <funcmap:nifti-ugv> <funcmap:hasPart> ?camera & ?camera <rdf:type> <funcmap:Camera> & ?camera <funcmap:hasPosX> ?x";
				QueryResults resultCamPosition = m_hfcserver.querySelect(queryCameraRobotPosition);
				float xPosCameraRobot = new XsdFloat(resultCamPosition.bt[0][0]).value;
				log("The X Position is" + xPosCameraRobot);
				queryCameraRobotPosition = "SELECT ?y WHERE <funcmap:nifti-ugv> <funcmap:hasPart> ?camera & ?camera <rdf:type> <funcmap:Camera> & ?camera <funcmap:hasPosY> ?y";
				resultCamPosition = m_hfcserver.querySelect(queryCameraRobotPosition);
				float yPosCameraRobot = new XsdFloat(resultCamPosition.bt[0][0]).value;
				log("The Y Position is" + yPosCameraRobot);				
				queryCameraRobotPosition = "SELECT ?z WHERE <funcmap:nifti-ugv> <funcmap:hasPart> ?camera & ?camera <rdf:type> <funcmap:Camera> & ?camera <funcmap:hasPosZ> ?z";
				resultCamPosition = m_hfcserver.querySelect(queryCameraRobotPosition);
				float zPosCameraRobot = new XsdFloat(resultCamPosition.bt[0][0]).value;
				log("The Z Position is" + zPosCameraRobot);

				// update car's window positions
				// Window[] carWindows = new Window[0]; 
				if (allWindows.length>0) {
					newCar.windows=allWindows;
					needToUpdateCarWME = true;
				} else {
					log("could not determine window information for the car!");
				}
				
				if (needToUpdateCarWME) {
					log("overwriting the car WME at "  
							+ _wmc.address.subarchitecture + "::" + _wmc.address.id  
							+ " with the new information...");
					// update Car WME!
					try {
						overwriteWorkingMemory(_wmc.address, newCar); 
					} catch (ConsistencyException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					} catch (PermissionException e) {
						// TODO Auto-generated catch block
						e.printStackTrace(); 
					}									// BLOCK COMMENTED FOR TESTING SHANKER
				} else{
					log("I don't have any new information about the car, so I am not overwriting the WME!");
				}
			}
			
			// String objectType = newMapObj.label;
			if (!this.m_dialogueSA.equals("")) {
				// generate verbalization report
				//if (objectType==null || 
				//		(!objectType.equals("car") && ! objectType.equals("victim") 
				//				&& ! objectType.equals("debris") && !objectType.equals("barrel") 
				//				&& !objectType.equals("sign"))) {
				//	objectType = "object";
				//}
				//if (newMapObj instanceof CarObject) {
				//	objectType = "car";
				//}
				log("going to register a verbalization for an object detection of type: " + objectType);
				registerVerbalizationIntention(objectType);
			} 
			else {
				log("no dialogue SA specified, not going to verbalize an object detection of type: " + objectType);
			}
		} 
		catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} 
		catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}


	}
	
	private void registerVerbalizationIntention(String type) {
		synchronized(this) {
			log("registerVerbalizationIntention() called: new verbalization task pending.");
			this.verbTaskPending = true;
			Integer i = this.verbTasks.get(type);
			if (i==null) {
				i = 0;
				this.verbTaskQ.add(type);
			}
			i+=1;
			this.verbTasks.put(type, i);
			log("registered verbalization task: "+type +","+i);
			this.notifyAll();
		}
	}
	
	private void generateVerbalizationIntention(String type, Integer i) {
		log("generateVerbalizationIntention("+type+","+i+") called.");
		// code taken from Mira
		// subtype will be ignored
		String subtype = "subtype";
		// type can be one of car, victim, debris, barrel, sign, or object
		Intention it = new Intention();
		it.id = newDataID();
		it.estatus = new PrivateEpistemicStatus("self");
		it.content = new LinkedList<IntentionalContent>();

		// it's the robot's intention
		List<String> ags = new LinkedList<String>();
		ags.add("self");

		// construct the postcondition (the state)
		ComplexFormula inState = new ComplexFormula(0, new LinkedList<dFormula>(), BinaryOp.conj);
		inState.forms.add(new ElementaryFormula(0, "visual-object-presented"));
		inState.forms.add(new ModalFormula(0, "type", new ElementaryFormula(0, type)));
		inState.forms.add(new ModalFormula(0, "subtype", new ElementaryFormula(0, subtype)));

		ModalFormula state = new ModalFormula(0, "state", inState);

		ComplexFormula post = new ComplexFormula(0, new LinkedList<dFormula>(), BinaryOp.conj);
		post.forms.add(state);

		IntentionalContent itc = new IntentionalContent(ags, new ComplexFormula(0, new LinkedList<dFormula>(), BinaryOp.conj), post, 1.0f);
		it.content.add(itc);
		it.frame = new AbstractFrame();

		CommunicativeIntention cit = new CommunicativeIntention(it);

		// XXX: make sure it's written to Dialogue SA!
		try {
			addToWorkingMemory(it.id, this.m_dialogueSA, cit);
		}
		catch (AlreadyExistsOnWMException ex) {
			ex.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			log("unknown subarchitecture \"" + m_dialogueSA + "\", verbalization request failed.");
			e.printStackTrace();
		} 
	}
	
	
	/**
	 * This is code that executes some fake test!
	 * 
	 */
	@Override
	protected void runComponent() {
		log("simple ICE connection test: ping");
		log(m_hfcserver.ping());
		
		while (isRunning()) {
			try {
				synchronized (this) {
					// if there is no pending task, continue the loop 
					if (!this.verbTaskPending) {
						log("no verbalization task pending. waiting...");
						this.wait();
						continue;
					}
				}
				synchronized (this) {
					log("pending verbalization task. acknowledged.");
					this.verbTaskPending = false;
				}
				// wait for another change
				log("wait for some time to let it settle.");
				Thread.sleep(TIME_TO_WAIT_TO_SETTLE);
				synchronized (this) {			
					// if there were no more changes
					if (this.verbTaskPending) {
						log("ok. got a fresh pending verbalization task. not yet processing it.");
						continue;
					}
				}
				log("waited long enough. no more fresh verbalization tasks for a while. gonna process them!");
				// execute the room maintenance algorithm
				this.lockComponent();
				while (!verbTasks.isEmpty()) {
					String type = verbTaskQ.pollFirst();
					Integer i = verbTasks.remove(type);
					generateVerbalizationIntention(type, i);
				}
				this.unlockComponent();
			}  catch (InterruptedException e) {
				logException(e);
			}
		}
		
		
		
//	    if(true) return;   // SHANKER --
//	    // the rest that follows below are just some stupid tests
//	    // they should not be executed under real system integration circumstances
//		CarObjectOfInterest car = new CarObjectOfInterest(new Pose(1.0f,1.0f,1.0f), "car", 
//				0, new WorkingMemoryPointer[0], new Window[0], new Dimensions(0.0f,0.0f), "Alfa159Sportwagon");
//                
//		try {
//			addToWorkingMemory(new WorkingMemoryAddress(newDataID(), this.getSubarchitectureID()), car);
//		} catch (AlreadyExistsOnWMException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		} catch (DoesNotExistOnWMException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		} catch (UnknownSubarchitectureException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
//		
//		if (true) return;
//		// don't execute that code...
//		
//		String query = "SELECT  ?y ?z where <http://www.nifti.eu/fem-0.1/#Alfa159Sportwagon> ?y ?z";
////		String query = "SELECT ?x ?y ?z where ?x ?y ?z";
////		String query = "SELECT ?x ?y ?z ?num where ?x ?y ?z ?num";
//		
//		QueryResults _results = m_hfcserver.querySelect(query);
//		log("query results:");
//
//		log("Query: " + _results.query);
//		log("Variable to array position mapping: " + _results.varPosMap.toString());
//		log("Variable result bindings: ");
//
//		for (int i = 0; i < _results.bt.length; i++) {
//			String[] _currLine = _results.bt[i];
//			StringBuffer _currLineResult = new StringBuffer();
//			for (int j = 0; j < _currLine.length; j++) {
//				String _res  = _currLine[j];
//				_currLineResult.append(_res + " ");
//			}
//			log(_currLineResult);
//		}
//		
//		
////		query = "SELECT ?x ?y where <dora:corridor> ?x ?y ";
//		query = "SELECT ?x ?y where <http://www.nifti.eu/fem-0.1/#alfaromeo159sw> ?x ?y ";
//		
//		_results = m_hfcserver.querySelect(query);
//		log("query results:");
//
//		log("Query: " + _results.query);
//		log("Variable to array position mapping: " + _results.varPosMap.toString());
//		log("Variable result bindings: ");
//
//		for (int i = 0; i < _results.bt.length; i++) {
//			String[] _currLine = _results.bt[i];
//			StringBuffer _currLineResult = new StringBuffer();
//			for (int j = 0; j < _currLine.length; j++) {
//				String _res  = _currLine[j];
//				_currLineResult.append(_res + " ");
//			}
//			log(_currLineResult);
//		}
//		
//		
//		
//		query = "SELECT ?x ?y ?num where <http://www.nifti.eu/fem-0.1/#alfaromeo159sn> ?x ?y ?num";
//		
//		_results = m_hfcserver.querySelect(query);
//		log("query results:");
//
//		log("Query: " + _results.query);
//		log("Variable to array position mapping: " + _results.varPosMap.toString());
//		log("Variable result bindings: ");
//
//		for (int i = 0; i < _results.bt.length; i++) {
//			String[] _currLine = _results.bt[i];
//			StringBuffer _currLineResult = new StringBuffer();
//			for (int j = 0; j < _currLine.length; j++) {
//				String _res  = _currLine[j];
//				_currLineResult.append(_res + " ");
//			}
//			log(_currLineResult);
//		}
//	
//		query = "SELECT ?x ?y ?num where <http://www.nifti.eu/fem-0.1/#busX> ?x ?y ?num";
//		
//		_results = m_hfcserver.querySelect(query);
//		log("query results:");
//
//		log("Query: " + _results.query);
//		log("Variable to array position mapping: " + _results.varPosMap.toString());
//		log("Variable result bindings: ");
//
//		for (int i = 0; i < _results.bt.length; i++) {
//			String[] _currLine = _results.bt[i];
//			StringBuffer _currLineResult = new StringBuffer();
//			for (int j = 0; j < _currLine.length; j++) {
//				String _res  = _currLine[j];
//				_currLineResult.append(_res + " ");
//			}
//			log(_currLineResult);
//		}
	}
}
