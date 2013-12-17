package de.dfki.lt.tr.memory.writer;

import Ice.ObjectImpl;
import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMEntrySet;
import castutils.castextensions.WMEntrySet.ChangeHandler;
import de.dfki.lt.tr.memory.global.DBConfig;
import de.dfki.lt.tr.memory.global.GlobalSettings;
import de.dfki.lt.tr.memory.plugins.Plugin;
import elm.dbio.ELMDatabaseWriter;
import elm.event.AtomicEvent;
import elm.event.EventIDException;
import elm.event.EventLocationFactory;
import elm.event.EventTime;
import elm.event.EventType;
import java.io.ByteArrayOutputStream;
import java.io.ObjectOutputStream;
import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.SQLException;
import java.util.HashMap;
import java.util.Map;
import java.util.StringTokenizer;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;

public class MemoryWriter extends ManagedComponent implements ChangeHandler
{
    private Logger logger;

    private boolean flushDB = false;
    private DBConfig dbConfig = new DBConfig();

    private ELMDatabaseWriter dbWriter = null;
    private WMEntrySet entrySet;
    private Map<Class<?>, Plugin> objectDispatcherMap;

    public MemoryWriter()
    {
        super();
        logger = Logger.getLogger(MemoryWriter.class);
        logger.setLevel(Level.DEBUG);
        objectDispatcherMap = new HashMap<Class<?>, Plugin>();
    }

    @Override
    protected void configure(Map<String, String> _config)
    {
        super.configure(_config);

        //configuring the database
        dbConfig.configure(_config);
        if (_config.containsKey(GlobalSettings.configKeyFlushDB))
        {
            flushDB = true;
        }

        //configuring class registration
        entrySet = WMEntrySet.create(this);
        entrySet.setHandler(this);
        String subscrStr = _config.get(GlobalSettings.subscribeKey);
        if (subscrStr != null)
        {
            StringTokenizer st = new StringTokenizer(subscrStr, ",");
            while (st.hasMoreTokens())
            {
                String className = st.nextToken();
                className = className.trim();
                try
                {
                    logger.info("adding type '" + className+"'");
                    //ClassLoader.getSystemClassLoader().loadClass(className);
                    entrySet.addType((Class<? extends ObjectImpl>) Class.forName(className));
                }
                catch (ClassNotFoundException e)
                {
                    logger.warn("trying to register for a class that doesn't exist!\n" + e.getMessage());
                }
            }
        }
        else
        {
//            try
//            {
//                logger.info("memory: registering for all objects on working memory");
//                entrySet.addType((Class<? extends ObjectImpl>) Class.);
//            }
//            catch (ClassNotFoundException e)
//            {
//                logger.warn("failed while trying to register for all classes.\n" + e.getMessage());
//            }
        }
    }

    @Override
    protected void start()
    {
        entrySet.start();
        try
        {
            dbWriter = new ELMDatabaseWriter(connect("jdbc:postgresql://"
                    + dbConfig.server + "/" + dbConfig.name, dbConfig.user,
                    dbConfig.passwd));

            if (flushDB)
            {
                logger.debug("flushing old ELM db entries...");
                dbWriter.flushMemoryStore();
            }

            logger.debug("ElmWriter waking up...");
            addWakeupEvent();
        }
        catch (SQLException se)
        {
            logger.error("Could not initialize ELMDatabaseWriter.\n" + se);
            System.exit(GlobalSettings.exitValueOnException);
        }
        super.start();
    }

    @Override
    protected void runComponent()
    {
        super.runComponent();
    }

    @Override
    protected void stop()
    {
        super.stop();
    }
    
    private Connection connect(String url, String user, String password)
    {
        Connection connection = null;

        try
        {
            Class.forName("org.postgresql.Driver");
        }
        catch (ClassNotFoundException cnfe)
        {
            logger.fatal("Could not find the jdbc driver for postgresql!\n" + cnfe);
            System.exit(1);
        }
        logger.debug("Registered the driver.\nConnecting to the database... ");

        try
        {
            connection = DriverManager.getConnection(url, user, password);
        }
        catch (SQLException se)
        {
                logger.error("Could not connect to postgresql database.\n" + se);
                System.exit(1);
        }
        logger.debug("Connected to postgresql database.");

        return connection;
    }

    private void addWakeupEvent()
    {
        long time = System.currentTimeMillis();
        double[] pos = { 0.0, 0.0 };
        AtomicEvent event = new AtomicEvent(new EventType("--MemoryWriter wakes up--"),
                new EventTime(time, time), new EventLocationFactory().fromPoint(pos, 0.0001),
                (byte[]) null, null, null);
	logger.debug("adding wakeup event");
        addEvent(event);
    }

    private void addEvent(AtomicEvent event)
    {
        try
        {
            dbWriter.storeEvent(event);
            logger.debug("stored event " + event.getEventID().toString());
        }
        catch (SQLException ex)
        {
            logger.error("could not store event\n" + ex.getMessage());
            if (GlobalSettings.exitOnException)
            {
                System.exit(GlobalSettings.exitValueOnException);
            }
        }
        catch (EventIDException ex)
        {
            logger.error("could not store event\n" + ex.getMessage());
            if (GlobalSettings.exitOnException)
            {
                System.exit(GlobalSettings.exitValueOnException);
            }
        }
    }

    public void entryChanged(Map<WorkingMemoryAddress, ObjectImpl> map, WorkingMemoryChange wmc,
            ObjectImpl newEntry, ObjectImpl oldEntry) throws CASTException
    {
        if(newEntry != null)
        {
            switch (wmc.operation)
            {
                case ADD:
                case OVERWRITE:
                    Plugin pluginToCall = findPlugin(newEntry.getClass());
                    if (pluginToCall != null)
                    {
                        addEvent(pluginToCall.toEvent(newEntry));
                    }
                    else //serialize
                    {
                        logger.debug("no plugin found for "
                                + newEntry.getClass().getCanonicalName()
                                + "\nso i will serialize the object");

                        long time = System.currentTimeMillis();
                        double[] pos = { 0.0, 0.0 };
                        AtomicEvent event = new AtomicEvent(new EventType(newEntry.getClass().getName()),
                            new EventTime(time, time), new EventLocationFactory().fromPoint(pos, 0.0001),
                            getBytes(newEntry), null, null);
                        addEvent(event);
                    }
                    break;
                case DELETE:
                    //nothing shoud be ever deleted from elm!!
                    break;
            }
        }
        else
        {
            logger.warn("entryChanged: expected an entry, found null!");
        }
    }
    
    /**
     * dynamically looks for suitable plug-ins and stores the association in a
     * map. plug-ins are expected to be in package "plug-ins" relative to this
     * package, implement the plug-in interface and their should be the
     * respective SimpleName of the class it work with suffixed by "Info" an
     * example is: MotiveInfo (for Motives...)
     */
    private Plugin findPlugin(Class<?> origType)
    {
        Class<?> objType = origType;
        Plugin pluginToCall = null;

        if(objType != null)
        {
            if (objectDispatcherMap.containsKey(objType))
            {
                return objectDispatcherMap.get(objType);
            }

            // if not yet found, look for plugins
            while (pluginToCall == null)
            {
                String SimpleName = objType.getSimpleName();
                String fullName = "de.dfki.lt.tr.memory.plugins." + SimpleName + "Plugin";

                try
                {
                    logger.debug("trying to load class " + fullName);
                    ClassLoader.getSystemClassLoader().loadClass(fullName);
                    pluginToCall = (Plugin) Class.forName(fullName).newInstance();
                    logger.debug("succeeded... memorizing plugin " + fullName
                            + "for type " + objType.getSimpleName());
                    objectDispatcherMap.put(origType, pluginToCall);
                    break;
                }
                catch (ClassNotFoundException e)
                {
                    logger.debug("no class " + fullName + "exists.", e);
                }
                catch (InstantiationException e)
                {
                    logger.debug("while loading " + fullName + ": ", e);
                }
                catch (IllegalAccessException e)
                {
                    logger.debug("while loading " + fullName + ": ", e);
                }
            
                objType = objType.getSuperclass();
                if (objType == null) // if no superclass exists, we have to give up
                {
                    break;
                }
                if (objType == Ice.Object.class) // we don't need to look up further
                {
                    break;
                }
            }
        }

        return pluginToCall;
    }

    private byte[] getBytes(Object obj)
    {
        try
        {
            ByteArrayOutputStream bos = new ByteArrayOutputStream();
            ObjectOutputStream oos = new ObjectOutputStream(bos);
            oos.writeObject(obj);
            oos.flush();
            oos.close();
            bos.close();
            byte [] data = bos.toByteArray();
            return data;
        }
        catch (java.io.IOException e)
        {
            System.out.println("error converting object to byte array");
            return null;
        }
    }
}
