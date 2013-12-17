/** 
 *  \file memory.idl
 *  This file defines structs in two modules: memory::slice and locationConversion::slice. 
 *  memory::slice is the module for (almost) all interactions with the memory/elm system 
 *  whereas locationConversion::slice contains interfaces to some other utilities 
 *  not considered core parts of memory/elm.
 *  <br>
 *  In case you wonder why there is this memory prefix everywhere: 
 *  ICE (3?) seems to have EventType as a keyword. 
 *  So this prefix is a workaround to avoid confusion and cryptic errors
 *  with Event* typenames. It might be dropped later on.
 *
 *  @author Dennis Stachowicz
 *
 * Converted to Slice by Nick Hawes, 10th June 2009
 */


#ifndef MEMORY_SLICE
#define MEMORY_SLICE

#include <cast/slice/CDL.ice>

module de {
module dfki {
module lt {
module tr {
module memory 
{
	/**
	*  memory::slice defines the CAST interfaces for interactions with the memory/elm system.
	*  Other SAs may report events by writing MemoryEventToStore or MemoryPartialEventToStore 
	*  to ElmWriter's WM and query the memory store by writing a MemoryEventQuery to Recollector's 
	*  WM.
	*  <br> 
	*  In case you wonder why there is this Memory prefix everywhere: 
	*  CORBA (3?) seems to have EventType as a keyword. 
	*  So this prefix is a workaround to avoid confusion and cryptic errors
	*  with Event* typenames. It might be dropped later on.
	*  @see locationConversion::slice
	*/  
	module slice
	{
		/**
		*  Unspecific type for binary event data. Can be a serialised object, for example.
		*  Binary event data are event-specific and optional.
		*/
		["java:array"] sequence<byte> MemoryEventSpecificBinaryData; 

    
		/**
		*  EventTimestamp - represents the onset or offset of an event.
		*  (Conversion utility from/to BALTTime (in Java): memory.conversion.BALTTimeConverter)
		*/
		class MemoryEventTimestamp
		{
			/**
			*  Milliseconds since January 1, 1970 00:00:00 GMT ("Epoch").
			*  In java you can get it from System.currentTimeMillis().
			*/
			long milliseconds;
		};  

		/**
		*  EventTime represents *both* start and end of an event.
		*  (Conversion utility in Java: memory.conversion.EventTimeConverter)
		*/
		class MemoryEventTime
		{
			MemoryEventTimestamp begin;
			MemoryEventTimestamp end;
		};

		/**
		*  Location of an event. 
		*/
		class MemoryEventLocation
		{
			/**
			*  A string representing a polygone in the WKT format.
			*/
			string wktString;
		};

		sequence<long> LongList;    
		sequence<string> StringList;

		class MemoryEventSpecificFeaturesEntry
		{
		string featureName;
		StringList featureValues;
		};

		/**
		*  Optional event-specific features. Representated in a string multi-map. 
		*  Can be used a generic means to store searchable and easily readable 
		*  data with an event.
		*/
		sequence<MemoryEventSpecificFeaturesEntry>  MemoryEventSpecificFeatures;

		/**
		*  Definition of the core contents of an event. Used by 
		*  the MemoryStoredEvent and MemoryEventToStore classures.
		*  NOT to be written to any working memory directly.
		*/
		class MemoryCoreEvent
		{
			string eventType;      
			MemoryEventTime eventTime;      
			MemoryEventLocation location;
			bool apexEvent;
			int eventDegree;
			LongList subevents;
			MemoryEventSpecificFeatures eventSpecificFeatures;
			MemoryEventSpecificBinaryData eventSpecificBinaryData;
		};

		/**
		*  Representation of an event already stored in the database. 
		*  After storage ELMWriter writes it to the EventRecognizers WM.
		*  Can also be part of the answer to a MemoryEventQuery. 
		*/

		class MemoryStoredEvent
		{
			long eventID;
			MemoryCoreEvent  event;
		};

	    	sequence<MemoryStoredEvent> MemoryStoredEventList;

		/**
		*  A fully specified (atomic or complex) event which
		*  shall be written to the database by ELMWriter.
		*  <br>
		*  Other subarchitectures / monitors may write this to 
		*  ELMWriter's WM. <br>
		*  Cave: If you do not know about the location you must use 
		*  a MemoryPartialEventToStore class instead!!!
		*  @see MemoryPartialEventToStore
		*/
		class MemoryEventToStore
		{
			MemoryCoreEvent event;
		};

		/**
		*  An incompletely specified (atomic!) event which
		*  shall be written to the database by ELMWriter
		*  after fusion with location information by LocationMonitor.
		*  <br>
		*  Other subarchitectures / monitors may write this to 
		*  ELMWriter's WM. <br>
		*  If you do already know about the location you should use 
		*  a MemoryEventToStore class instead.
		*  @see MemoryEventToStore
		*/
		class MemoryPartialEventToStore
		{
			string eventType;      
			MemoryEventTime eventTime; 

			// below: optional information
			// if not needed, create 0-sized arrays 
			// or ICE will make trouble (when passing null's)
			MemoryEventSpecificFeatures eventSpecificFeatures;
			MemoryEventSpecificBinaryData eventSpecificBinaryData;
		};

		/**
		*  MemoryCueMatchMode defines modes for matching against 
		*  fields in a MemoryEventCue used in a MemoryEventQuery.
		*  @see MemoryEventQuery
		*  @see MemoryEventCue
		*/
		enum MemoryCueMatchMode
		{ 
			/**
			*  No match, do NOT use given property as cue.
			*/
			noMatch, 

			/**
			*  Match the given property EXACTLY as specified.
			*/ 
			matchExact, 

			/**
			*  MATCHING EVENTS have happened WITHIN the given bounds,
			*  e.g. matching events' time interval is a subset of the 
			*  specified interval.
			*/
			matchSubset, 
      
			/**
			*  MATCHING EVENTS' property ENCOMPASSES the given bounds,
			*  e.g. matching events' location is at least the 
			*  specified one (or a larger one).
			*/
			matchSuperset, 

			/**
			*  There is at least one COMMON ELEMENT.
			*/
			matchIntersectionNotEmpty 
		};


		/**
		*  A MemoryEventCue class is similar to a MemoryCoreEvent
		*  but can be underspecified. To leave a field unspecified 
		*  you can set the appropriate match flag to false or noMatch 
		*  and assign members some arbitrary values (except null references).
		*  <br>
		*  As part of a MemoryEventQuery such a cue is matched with 
		*  previously stored events.
		*  @see MemoryEventQuery
		*  @see MemoryCueMatchMode
		*/ 
		class MemoryEventCue 
		{
			bool matchEventID;
			long minEventID;
			long maxEventID;

			bool matchEventType;
			string eventType;
			bool eventTypeIncludeSubtypes;
			      
			MemoryCueMatchMode timeMatchMode;
			MemoryEventTime eventTime;      
			      
			MemoryCueMatchMode locationMatchMode;
			MemoryEventLocation location;
			      
			bool matchApexEvent;
			bool apexEvent;

			bool matchEventDegree;
			int minEventDegree;
			int maxEventDegree;

			MemoryCueMatchMode subeventMatchMode;
			LongList subevents;

			MemoryCueMatchMode supereventMatchMode;
			LongList superevents;
			       
			MemoryCueMatchMode esfMatchMode;
			bool esfRestrictMatchToKeys;
			MemoryEventSpecificFeatures eventSpecificFeatures;
			      
			bool matchEventSpecificBinaryData;
			MemoryEventSpecificBinaryData eventSpecificBinaryData;
		};

		/**
		*  MemoryEventQuery objects can be written to the
		*  working memory of the Recollector process which
		*  tries to find events matching the cue and return 
		*  them through the "events" member. For this to work
		*  the process issuing the query should register a 
		*  WM change receiver on this particular item before
		*  actually issuing it!
		*  @see MemoryEventCue
		*/
		class MemoryEventQuery
		{
			/**
			*  How many events should be retrieved at most?
			*  Set this to -1 for the default behaviour.
			*  Set this to 0  for NO LIMIT.
			*/
			int limit;

			// order???

			/**
			*  The cue against which events are matched. 
			*/
			MemoryEventCue cue;

			      
			/**
			*  This list of stored events should be empty 
			*  initially (a 0-sized array). The Recollector process 
			*  will fill it with matching events (if there are any).
			*/
			MemoryStoredEventList  events;
		};
	
		/**
		* The locationConversion module and the associated processes are work in progress.
		* NEVER TESTED! Use at your own risk or better: do not use it!
		* @see locationConversion::slice
		*/

		class ROSPointPosition
		{
					  double x;
					  double y;
					  double z;
		};

		class ROSQuaternionOrientation
		{
					  double x;
					  double y;
					  double z;
					  double w;
		};

		class ROSPose
		{
					 ROSPointPosition position;
					 ROSQuaternionOrientation orientation;
		};

		class ROSHeartBeat
		{
					  //int rosTimeSec;
					  long rosTimeNSec;
					  ROSPose pose;
		};

		module locationConversion
		{
			/** 
			*  The module locationConversion::slice contains data classures describing the
			*  interface to the LocationConverter process which converts 'here', 
			*  'at (x, y)' or areaIDs from nav.sa to the MemoryEventLocation format 
			*  (well-known text (WKT)) 
			*
			*  @author Dennis Stachowicz
			*/

			/**
			*  Returns a polygon approximating a circle with radius 
			*  bufferDistance around the current position.
			*/
			class ConvertHere
			{
				double bufferDistance;

				/**
				*  Leave this empty (filled with an empty string)!
				*  It will be filled and the whole classure overwritten.
				*/
				memory::slice::MemoryEventLocation location; 
			};

			/**
			*  Returns a polygon approximating a circle with radius 
			*  bufferDistance around the specified position.
			*/
			class ConvertPosition
			{
				double x;
				double y;
				double bufferDistance;

				/**
				*  Leave this empty (filled with an empty string)!
				*  It will be filled and the whole classure overwritten.
				*/
				memory::slice::MemoryEventLocation location; 
			};

			/**
			*  Returns a polygon approximating (!) the specified area
			*  based on FNodes from Nav.sa
			*/
			class ConvertArea
			{
				long areaID;

				/**
				*  Leave this empty (filled with an empty string)!
				*  It will be filled and the whole classure overwritten.
				*/
				memory::slice::MemoryEventLocation location; 
			};
		}; 
	};
};
};
};
};
};

#endif
