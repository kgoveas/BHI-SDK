#!/bin/bash

# fall back values in case no environment variables are set
key_id=EM_SDK_user_ram
server=https://10.49.1.47:2443

# check existence of environment variables, otherwise keep default fallback value
if [ -v REST_PUBKEY ]; then
    key_id=$REST_PUBKEY
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

# remove old key
rm -f temp_key.pub

# prompt pw only if not given in environment variable
if [ -z "$passw" ]; then
    RESP=$(curl $curl_param -s -u $user -X GET "$server/API/Keys/GetPublicKey?keyIdentifier=$key_id")
else
    RESP=$(curl $curl_param -s -u $user:$passw -X GET "$server/API/Keys/GetPublicKey?keyIdentifier=$key_id")
fi

ERR=$?
if [ $ERR -eq 0 ]
then
    # translate ascii hex chars into hex binary output
    xxd -r -p <<< "$RESP" > temp_key.pub
else
    exit $ERR
fi
