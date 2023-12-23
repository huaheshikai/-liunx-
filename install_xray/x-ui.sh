#!/bin/bash
rm  /usr/local/x-ui/ /usr/bin/x-ui -rf
wget https://github.riluowuhai.eu.org/https://github.com/FranzKafkaYu/x-ui/releases/download/0.3.4.4/x-ui-linux-amd64.tar.gz
tar zxvf x-ui-linux-amd64.tar.gz
chmod +x x-ui/x-ui x-ui/bin/xray-linux-* x-ui/x-ui.sh
cp x-ui/x-ui.sh /usr/bin/x-ui
cp -f x-ui/x-ui.service /etc/systemd/system/
mv x-ui/ /usr/local/
systemctl daemon-reload
systemctl enable x-ui
systemctl restart x-ui