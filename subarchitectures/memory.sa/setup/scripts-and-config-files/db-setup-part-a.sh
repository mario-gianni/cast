#!/bin/bash

export RUNASPG="su - postgres -c "

echo "creating user elm..."
$RUNASPG "createuser -SdRlP elm" || exit 1

echo "creating DB elm..."
$RUNASPG "createdb -O elm elm" || exit 1

echo "running createlang..."
$RUNASPG "createlang plpgsql elm" || exit 1

sleep 1
echo "running postgis init scripts..."
sleep 1
$RUNASPG "psql -d elm -f /usr/share/postgresql/8.4/contrib/postgis.sql" || exit 1
$RUNASPG "psql -d elm -f /usr/share/postgresql/8.4/contrib/spatial_ref_sys.sql" || exit 1

