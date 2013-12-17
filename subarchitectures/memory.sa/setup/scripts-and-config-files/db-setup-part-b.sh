#!/bin/bash

export PGHBACONF="/etc/postgresql/8.4/main/pg_hba.conf"
export PGHBASUFFIX="backup.memory"
export PGHBANEW="pg_hba.conf.new"

echo "copying $PGHBACONF to $PGHBACONF.$PGHBASUFFIX."
cp $PGHBACONF $PGHBACONF.$PGHBASUFFIX || exit 1

echo "installing new $PGHBACONF"
cp $PGHBANEW $PGHBACONF || exit 1

/etc/init.d/postgresql restart || exit 1



