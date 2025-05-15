#!/bin/zsh
export WEBOTS_HOME=/Volumes/Node/Applications/Webots.app/Contents
export DYLD_LIBRARY_PATH=/Volumes/Node/Applications/Webots.app/Contents/lib/controller
/Volumes/Node/Applications/Webots.app/Contents/MacOS/webots-controller --robot-name=heron_1  ./target/debug/econtroller &
/Volumes/Node/Applications/Webots.app/Contents/MacOS/webots-controller --robot-name=heron_2  ./target/debug/econtroller &
/Volumes/Node/Applications/Webots.app/Contents/MacOS/webots-controller --robot-name=heron_3  ./target/debug/econtroller &
/Volumes/Node/Applications/Webots.app/Contents/MacOS/webots-controller --robot-name=heron_4  ./target/debug/econtroller
