#!/bin/zsh
export WEBOTS_HOME=/Volumes/Node/Applications/Webots.app/Contents
export DYLD_LIBRARY_PATH=/Volumes/Node/Applications/Webots.app/Contents/lib/controller
# /Volumes/Node/Applications/Webots.app/Contents/MacOS/webots-controller --protocol=tcp --ip-address=192.168.2.105 --robot-name=heron_1  ./target/release/heronc &
# /Volumes/Node/Applications/Webots.app/Contents/MacOS/webots-controller --protocol=tcp --ip-address=192.168.2.105 --robot-name=heron_2  ./target/release/heronc &
# /Volumes/Node/Applications/Webots.app/Contents/MacOS/webots-controller --protocol=tcp --ip-address=192.168.2.105 --robot-name=heron_3  ./target/release/heronc &
# /Volumes/Node/Applications/Webots.app/Contents/MacOS/webots-controller --protocol=tcp --ip-address=192.168.2.105 --robot-name=heron_4  ./target/release/heronc

/Volumes/Node/Applications/Webots.app/Contents/MacOS/webots-controller  --robot-name=heron_1  ./target/release/heronc &
/Volumes/Node/Applications/Webots.app/Contents/MacOS/webots-controller  --robot-name=heron_2  ./target/release/heronc &
/Volumes/Node/Applications/Webots.app/Contents/MacOS/webots-controller  --robot-name=heron_3  ./target/release/heronc &
/Volumes/Node/Applications/Webots.app/Contents/MacOS/webots-controller  --robot-name=heron_4  ./target/release/heronc
