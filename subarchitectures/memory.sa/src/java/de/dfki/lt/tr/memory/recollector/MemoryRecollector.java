/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package de.dfki.lt.tr.memory.recollector;

/**
 *
 * @author harmish khambhaita
 */

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import de.dfki.lt.tr.memory.slice.MemoryEventQuery;
import de.dfki.lt.tr.memory.slice.MemoryStoredEvent;
import de.dfki.lt.tr.memory.global.DBConfig;
import de.dfki.lt.tr.memory.global.GlobalSettings;
import elm.dbio.ELMDatabaseReader;
import elm.event.Event;
import elm.event.EventLocationFactory;
import elm.event.EventTemplate;
import elm.event.WKTParseException;
import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.SQLException;
import java.util.Map;
import java.util.Vector;
import org.apache.log4j.Logger;

public class MemoryRecollector extends ManagedComponent
{
    private Logger logger;
    private final int defaultLimit = GlobalSettings.defaultRecollectionLimit;
    private DBConfig dbConfig = new DBConfig();

    private ELMDatabaseReader dbReader = null;
    private EventLocationFactory elFactory = new EventLocationFactory();
    private Converter converter = new Converter();

    public MemoryRecollector()
    {
        super();
        logger = Logger.getLogger(MemoryRecollector.class);
    }

    @Override
    public void configure(Map<String, String> _config)
    {
        super.configure(_config);
        dbConfig.configure(_config);
    }

    @Override
    public void start()
    {
        super.start();
        try
        {
            dbReader = new ELMDatabaseReader(connect("jdbc:postgresql://" + dbConfig.server
                    + "/" + dbConfig.name, dbConfig.user, dbConfig.passwd), elFactory);
            dbReader.setDebugMode(logger.isDebugEnabled());

            addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(MemoryEventQuery.class, WorkingMemoryOperation.ADD),
                    new WorkingMemoryChangeReceiver()
                    {
                        public void workingMemoryChanged(WorkingMemoryChange wmc)
                        {
                            answerQuery(wmc);
                        }
                    });
        }
        catch (SQLException se)
        {
            logger.fatal("could not initialize MemoryDatabaseReader." + se.getMessage());
            System.exit(GlobalSettings.exitValueOnException);
        }
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
            logger.fatal("Could not find the driver!\n" + cnfe.getMessage());
            System.exit(1);
        }
        logger.debug("Registered the driver.\nConnecting to the database... ");

        try
        {
            connection = DriverManager.getConnection(url, user, password);
        }
        catch (SQLException se)
        {
                logger.error("Could not connect to the postgres database.\n" + se.getMessage());
                System.exit(1);
        }
        logger.debug("Connected.");

        return connection;
    }

    private void answerQuery(WorkingMemoryChange wmc)
    {
        try
        {
            logger.debug("got a new query...");

            CASTData<?> wme = getWorkingMemoryEntry(wmc.address);
            MemoryEventQuery query = (MemoryEventQuery)wme.getData();

            logger.debug("query read");

            EventTemplate template = converter.getEventTemplate(query.cue);

            logger.debug("template generated");

            int limit = query.limit < 0 ? defaultLimit : query.limit;

            logger.debug("query limit set");

            Vector<Event> results = dbReader.getMatchingEvents(template, limit, 0);

            logger.debug("got results from db");
            
            MemoryStoredEvent[] storedEvents = new MemoryStoredEvent[results.size()];
            for (int i= 0; i < results.size(); i++)
            {
                storedEvents[i] = converter.getStoredEvent(results.get(i));
            }

            logger.debug("got stored events");
            
            query.events = storedEvents;
            overwriteWorkingMemory(wmc.address, query);

            logger.debug("results written to working memory");
        }
        catch (WKTParseException ex)
        {
            logger.error(ex);
            if (GlobalSettings.exitOnException)
            {
                System.exit(GlobalSettings.exitValueOnException);
            }
        }
        catch (SubarchitectureComponentException ex)
        {
            logger.error(ex);
            if (GlobalSettings.exitOnException)
            {
                System.exit(GlobalSettings.exitValueOnException);
            }
        }
        catch (Exception ex)
        {
            logger.error(ex);
            if (GlobalSettings.exitOnException)
            {
                System.exit(GlobalSettings.exitValueOnException);
            }
	}
    }
}
