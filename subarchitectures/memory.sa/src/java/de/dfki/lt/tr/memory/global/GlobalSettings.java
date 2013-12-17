package de.dfki.lt.tr.memory.global;

/**
 *  This class holds some global compile time switches / settings
 *  for the memory system.
 */
public class GlobalSettings {
    
    // for debugging:
    public static final boolean exitOnException       = false;
    public static final int     exitValueOnException  = 2;

    public static final double  defaultAtomicBuffer   = 0.01; // 20;
    public static final int defaultRecollectionLimit = 50;

    public static final String configKeyFlushDB = "--flush-db";
    public static final String subscribeKey = "--subscribe";

    //public static final boolean verbose               = false;
    //public static final boolean singleSA              = false;
}
