#!/bin/bash

./terp2_set_goal.sh -4 0
sleep 15
# arm action (book pickup)

./terp2_set_goal.sh 0 -4
sleep 15
# arm action (book dropoff)

