#!/bin/bash

export RUNASPG="su - postgres -c "

export ELM_CREATE_SCRIPT="`pwd`/createELMTables.sql"
export ELM_MODIFY_SCRIPT="`pwd`/modifyELMTables.sql"

echo "setting up DB tables for ELM..."
$RUNASPG "psql -U elm -f $ELM_CREATE_SCRIPT" || exit 1
$RUNASPG "psql -d elm -f $ELM_MODIFY_SCRIPT" || exit 1





