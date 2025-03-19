#!/bin/bash
if [ ! -n "$1" ] ;then
    echo "Usage: ./findport_kill.sh port_number"
else
    echo $(netstat -tunlp | grep $1) > 1
        sed 's/ //g' 1 > 2
        echo | grep -o LISTEN.* 2 > 3
        v1=$(cat 3)
        v2=$(echo ${v1#LISTEN})
        v3=$(echo ${v2%/.*})
        if [ ! $v3 ] ; then
                echo "No process is occupying the port !!" 
        else 
        				 echo "Port $v3 will be killed"
                kill -9 $v3
                echo "kill success"
        fi
        rm -f 1
        rm -f 2
        rm -f 3
fi

