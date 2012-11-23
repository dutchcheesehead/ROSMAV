#!/bin/bash
rosservice call /turn false 5
sleep 160
rosservice call /turn false 0
