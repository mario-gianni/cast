#!/bin/bash

echo "WARNING: this script is likely to work only on specific ubuntu"
echo " system configurations only "
echo "If you have a different configuration you will have to instead"
echo "follow the stepwise setup procedure."
echo 
echo "Hit enter to proceed or Ctrl-C to cancel..."
read

./install-postgresql-postgis.sh
./db-setup-all.sh

