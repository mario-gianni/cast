#!/bin/bash

function notFound()
{
	echo "$1 could not be found."
	#exit 1
}

if [ -e /usr/share/java/postgis.jar ]; then
	export POSTGIS_JAR_PATH="/usr/share/java/postgis.jar"
else
	notFound postgis.jar
fi;

if [ -e /usr/share/java/postgresql.jar ]; then
        export POSTGRESQL_JAR_PATH="/usr/share/java/postgresql.jar"
else
        notFound postgresql.jar
fi;
                
if [ -e /usr/share/java/postgresql-jdbc3.jar ]; then
        export PGSQL_JDBC_JAR_PATH="/usr/share/java/postgresql-jdbc3-8.4.jar"
else
        notFound postgresql-jdbc3.jar               
fi;

export CLASSPATH=$POSTGIS_JAR_PATH:$POSTGRESQL_JAR_PATH:$PGSQL_JDBC_JAR_PATH:$CLASSPATH