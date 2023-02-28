#!/bin/bash

if [ $# -ge 1 ]; then 
    TARGET=$1
else
    echo "This script needs the hostname of the relationship to configure"
    echo "./relationship.bash <TARGET_HOSTNAME>"
    exit
fi

if [ $# -ge 2 ]; then 
    USERNAME=$2
else
    USERNAME="ros"
fi

# test the connection
ping $TARGET -c 2 > /dev/null
HAS_CONNECTION=$?
# DO NOT SEPARATE THE TWO ABOVE LINES! the ping command sets the value for $?

if [ $HAS_CONNECTION -gt 0 ]; then
    echo "Failed to connect to $TARGET"
    echo "Make sure network is setup properly then re-run this script"
    exit
fi

mkdir ~/.ssh/

if [ "$(grep ssh-agent ~/.bashrc)" = "" -a "$USER" = $USERNAME ]
then
	cat > ~/.bashrc <<EOF 
eval "$(ssh-agent -s)" > /dev/null
ssh-add -D > /dev/null
shopt -s extglob
ssh-add ~/.ssh/sshkey_!(*.pub) > /dev/null
shopt -u extglob
EOF
    
	eval $(ssh-agent -s)
fi

cd ~/.ssh

ssh-keygen -b 2048 -t rsa -f /tmp/sshkey_${TARGET} -q -N ""
ssh-copy-id -i /tmp/sshkey_${TARGET} $USERNAME@${TARGET}

mv /tmp/sshkey_${TARGET} ~/.ssh/sshkey_${TARGET}
mv /tmp/sshkey_${TARGET}.pub ~/.ssh/sshkey_${TARGET}.pub

ssh-add -D
ssh-add sshkey_${TARGET}

ssh-keyscan ${TARGET} >> ~/.ssh/known_hosts 