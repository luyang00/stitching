#!/bin/bash
sleep_time=9

case $1 in

1)
	while :
	do
		./camera-test -d /dev/video0 &
		sleep $sleep_time
		killall camera-test
	
	done
;;

2)
	while :
	do
		./camera-test -d /dev/video0 &
		sleep $sleep_time
		./camera-test -d /dev/video1 &
		sleep $sleep_time
		killall camera-test
	
	done
;;

3)
	while :
	do
		./camera-test -d /dev/video0 &
		sleep $sleep_time
		./camera-test -d /dev/video1 &
		sleep $sleep_time
		./camera-test -d /dev/video2 &
		sleep $sleep_time
		killall camera-test
	
	done
;;

4)
	while :
	do
		./camera-test -d /dev/video0 &
		sleep $sleep_time
		./camera-test -d /dev/video1 &
		sleep $sleep_time
		./camera-test -d /dev/video2 &
		sleep $sleep_time
		./camera-test -d /dev/video3 &
		sleep $sleep_time
		killall camera-test
	
	done
;;

5)
	while :
	do
		./camera-test -d /dev/video0 &
		sleep $sleep_time
		./camera-test -d /dev/video1 &
		sleep $sleep_time
		./camera-test -d /dev/video2 &
		sleep $sleep_time
		./camera-test -d /dev/video3 &
		sleep $sleep_time
		./camera-test -d /dev/video4 &
		sleep $sleep_time
		killall camera-test
	
	done
;;

6)
	while :
	do
		./camera-test -d /dev/video0 &
		sleep $sleep_time
		./camera-test -d /dev/video1 &
		sleep $sleep_time
		./camera-test -d /dev/video2 &
		sleep $sleep_time
		./camera-test -d /dev/video3 &
		sleep $sleep_time
		./camera-test -d /dev/video4 &
		sleep $sleep_time
		./camera-test -d /dev/video5 &
		sleep $sleep_time
		killall camera-test
	
	done
;;

esac


