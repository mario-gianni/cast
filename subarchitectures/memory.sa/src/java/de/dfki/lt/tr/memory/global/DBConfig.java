package de.dfki.lt.tr.memory.global;

import java.util.Map;

/**
 *  Unifies the configuration of MemoryWriter and Recollector's
 *  database parameters.
 */
public class DBConfig
{
    public static final String configKeyServer  = "--server";
    public static final String configKeyDB      = "--db";
    public static final String configKeyUser    = "--user";
    public static final String configKeyPasswd  = "--passwd";
    
    public String server                        = "localhost";
    public String name                          = "elm";
    public String user                          = "elm";
    public String passwd                        = "elm";
        
    public void configure(Map<String, String> config)
    {
	if (config.containsKey(configKeyServer)) 
	    server = config.get(configKeyServer);
	if (config.containsKey(configKeyDB)) 
	    name   = config.get(configKeyDB);
	if (config.containsKey(configKeyUser)) 
	    user   = config.get(configKeyUser);
	if (config.containsKey(configKeyPasswd)) 
	    passwd = config.get(configKeyPasswd);
    }
}
