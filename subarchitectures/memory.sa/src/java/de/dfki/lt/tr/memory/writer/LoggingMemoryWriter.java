package de.dfki.lt.tr.memory.writer;

import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.FilterRestriction;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.memory.global.DBConfig;
import de.dfki.lt.tr.memory.global.GlobalSettings;
import elm.dbio.ELMDatabaseWriter;
import elm.event.AtomicEvent;
import elm.event.EventIDException;
import elm.event.EventLocationFactory;
import elm.event.EventSpecificFeatures;
import elm.event.EventTime;
import elm.event.EventType;
import java.io.ByteArrayOutputStream;
import java.io.ObjectOutputStream;
import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.SQLException;
import java.util.Map;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;

public class LoggingMemoryWriter extends ManagedComponent
{
    private Logger logger;

    private boolean flushDB = false;
    private DBConfig dbConfig = new DBConfig();

    private ELMDatabaseWriter dbWriter = null;

    private boolean gotHBAddress = false;
    private WorkingMemoryAddress HBWMA = null;

    public LoggingMemoryWriter()
    {
        super();
        logger = Logger.getLogger(LoggingMemoryWriter.class);
        logger.setLevel(Level.DEBUG);
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

        logger.info("memory: registering for all objects on working memory");
    }

    @Override
    protected void start()
    {
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

        addChangeFilter(ChangeFilterFactory.createOperationFilter(WorkingMemoryOperation.WILDCARD, FilterRestriction.ALLSA),
                new WorkingMemoryChangeReceiver()
                {
                    @Override
                    public void workingMemoryChanged(WorkingMemoryChange _wmc)
                    {
                        entryChanged(_wmc);
                    }
                });

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
        AtomicEvent event = new AtomicEvent(new EventType("--LoggingMemoryWriter wakes up--"),
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

    private void entryChanged(WorkingMemoryChange _wmc)
    {
        logger.debug("LoggingMemoryWriter: new entry received");

        long time = System.currentTimeMillis();
        double[] pos = { 0.0, 0.0 };

        EventSpecificFeatures efs = new EventSpecificFeatures();
        efs.addKeyValuePair("WorkingMemoryOperation", _wmc.operation.name());
        efs.addKeyValuePair("WMA_Id", _wmc.address.id);
        efs.addKeyValuePair("WMA_Subarch", _wmc.address.subarchitecture);
        efs.addKeyValuePair("Origin", _wmc.src);
        efs.addKeyValuePair("CASTTime", castutils.CASTTimeUtil.msString(getCASTTime()));

        Object entry = null;
        try
        {
            //get the entry from wm
            if(_wmc.operation == WorkingMemoryOperation.DELETE || _wmc.operation == WorkingMemoryOperation.GET)
            {
                //in this case do not need to get entry from  memory
            }
            else if(_wmc.operation == WorkingMemoryOperation.ADD || _wmc.operation == WorkingMemoryOperation.OVERWRITE)
            {
//                String processedType = _wmc.type.replace("::", ".").substring(1);
//                ClassLoader.getSystemClassLoader().loadClass(processedType);
//                Class specClass = Class.forName(processedType);
                  entry = this.getMemoryEntry(_wmc.address, Ice.ObjectImpl.class);
            }
            else
            {
                logger.error("LoggingMemoryWriter: undefined working memory operation!");
            }

        }
//        catch (ClassNotFoundException ex)
//        {
//            logger.debug(ex);
//        }
        catch (DoesNotExistOnWMException ex)
        {
            logger.debug("RosHBReader: error while memorizing wm\n" + ex);
            entry = ex;
        }
        catch (UnknownSubarchitectureException ex)
        {
            logger.debug("RosHBReader: error while memorizing wm\n" + ex);
            entry = ex;
        }

        AtomicEvent event = new AtomicEvent(new EventType(_wmc.type),
                                new EventTime(time, time), new EventLocationFactory().fromPoint(pos, 0.0001),
                                getBytes(entry), null, efs);

        //adding the event
        addEvent(event);
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
