package de.dfki.lt.tr.memory.plugins;

import elm.event.AtomicEvent;

/**
 * interface for all plugins which are used to convert given datatype to elm.
 * to add plugins make a new class named as 'classNamePlugin' where className
 * is the name of the original class. plugins must go under the namespace
 * de.dfki.lt.tr.memory.plugins
 */
public interface Plugin
{
    public AtomicEvent toEvent(Ice.ObjectImpl iceObject);
}

