#!/bin/bash

trigger_file=/tmp/resttemp_in_user
return_file=/tmp/resttemp_out_user
digest=$1

# fall back values in case no environment variables are set
key_id=EM_SDK_user_ram
server=https://10.49.1.47:2443

# check existence of environment variables, otherwise keep default fallback value
if [ -v REST_USER_KEY ]; then
    key_id=$REST_USER_KEY
fi

if [ -v REST_SERVER ]; then
    server=$REST_SERVER
fi

if [ -v REST_PASSW ]; then
    passw=$REST_PASSW
else
    passw=''
fi

if [ -v REST_USERNAME ]; then
    user=$REST_USERNAME
else
    # no username provided via env. variable; retrieve domain name from hostname in style 'de.bosch.com'
    IFS='.';read -r -a domain_array <<< "$(hostname -d)" ; unset IFS;
    domain_name="${domain_array[-3]}"."${domain_array[-2]}"."${domain_array[-1]}"
    user="$domain_name"\\"$(whoami)"
fi

if [ -v REST_CURL_PARAM ]; then
    curl_param=$REST_CURL_PARAM
else
    curl_param=''
fi


if [ -z "$digest" ]; then
   echo "usage: $0 digest"
   exit 1
fi

# Starting background process
if [ "$digest" = "server" ]; then
    read -s password
    mknod $trigger_file p
    mknod $return_file p
    digest=
    while [ "$digest" != "stop" ]; do
        read -t 120 digest <> $trigger_file
        if [ "$digest" == "" ]; then
            # Timeout occured
            break
        fi
        # digest received from client process, call REST API to retrieve signature
        if [ "$digest" != "stop" ]; then
            curl $curl_param -s -u $user:$password -X GET "$server/API/Sign/SignDigest?keyIdentifier=$key_id&&digest=$digest" | tr -d '"' > $return_file
        fi
    done
    # clean up pipe files before exiting
    rm $trigger_file >/dev/null 2>&1
    rm $return_file >/dev/null 2>&1
    exit

# First call, pipes not existing, prompt password
elif [ ! -e $trigger_file ]; then
    if [ "$digest" == "stop" ]; then
        exit 0
    fi
    if [ ! -w "/tmp" ]; then
        echo "Error: no write access to /tmp directory"
        exit 1
    fi
    # prompt pw only if not given in environment variable
    if [ -z "$passw" ]; then
        read -s -p $'\nUser: Enter REST password for user '$user':'$'\n' password
    else
        password=$passw
    fi

    echo $password | $0 server &
    while [ ! -e $trigger_file ] &&  [ ! -e $return_file ]; do
      sleep .1
    done
fi

# Sending the command / digest to the running background process
if [ "$digest" != "start" ]; then
    echo $digest > $trigger_file
        # "if" clause is required to not read return_file after deletion
    if [ "$digest" != "stop" ]; then
        read signature < $return_file
        echo $signature
    fi
fi
