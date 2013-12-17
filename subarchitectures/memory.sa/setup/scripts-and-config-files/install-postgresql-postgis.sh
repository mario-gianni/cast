#!/bin/bash

export PACKAGES="postgresql-client-common postgresql-client-8.4 postgresql-8.4-postgis postgresql-common postgresql-8.4 postgresql-contrib-8.4 libpostgis-java libpg-java"

apt-get install $PACKAGES

