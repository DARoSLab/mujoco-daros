#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

echo -e "Dir is ${DIR}"
export PYTHONPATH=${DIR}/../../LCM-Types/python/:${DIR}:${DIR}/lcm-log2smat/python:/usr/local/lib/python3/dist-packages:/usr/bin/python${PYTHONPATH}
echo -e "Python path is ${PYTHONPATH}"

# exec /usr/local/bin/python -m lcmlog2smat.log_to_smat $1 -o $2
exec /home/simon/miniconda3/bin/python3 -m lcmlog2smat.log_to_smat $1 -o $2 #linux default
