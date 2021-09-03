#!/bin/bash -e

#-------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------
SHORE_IP="192.168.0.40" #Change here
SHORE_LISTEN="9300"
TIME_WARP=1
VNAME="heron"
M200_IP=192.168.0.180 #Change Here
VPORT="9005"
VEHICLE_LISTEN="9305"
JUST_BUILD="no"

for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
	printf "\n"
        echo "  --evan,       -e  : Evan vehicle."
        echo "  --sim,        -s  : Simulation mode."
        echo "  --just_build, -j  : Build file but don't launch"
        echo "  --timewarp=,  -t= , set up TimeWarp. (Default is $TIME_WARP)"
	printf "\n"
	echo "For example: $0 -e $TIME_WARP -p=$POSITION -h=$HEADING"
        echo "             $0 -kirk -t=$TIME_WARP -p=$POSITION -h=$HEADING"
	exit 0;

    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI:0:11}" = "--timewarp=" ] ; then
        TIME_WARP="${ARGI#--timewarp=*}"
    elif [ "${ARGI:0:3}" = "-t=" ] ; then
        TIME_WARP="${ARGI#-t=*}"

    elif [ "${ARGI}" = "--just_build" -o "${ARGI}" = "-J" ] ; then
        JUST_BUILD="yes"
        echo "Just building files; no vehicle launch."
    elif [ "${ARGI}" = "--sim" -o "${ARGI}" = "-s" ] ; then
        SIM="SIM"
        echo "Simulation mode ON."

    else 
	printf "Bad Argument: %s \n" $ARGI
	printf "Please use -h for help."
	exit 0
    fi
done
#-------------------------------------------------------
#  Part 2: Handle Ill-formed command-line arguments
#-------------------------------------------------------

if [ -z $VNAME ]; then
    echo "No vehicle has been selected..."
    echo "Exiting."
    exit 2
fi

#-------------------------------------------------------
#  Part 3: Create the .moos files.
#-------------------------------------------------------

nsplug meta_vehicle.moos targ_${VNAME}.moos -f \
    VNAME=$VNAME                 \
    VPORT=$VPORT                 \
    WARP=$TIME_WARP              \
    V_LISTEN=$VEHICLE_LISTEN     \
    SHORE_LISTEN=$SHORE_LISTEN   \
    SHORE_IP=$SHORE_IP           \
    M200_IP=$M200_IP             \
    $SIM                         


if [ ${JUST_BUILD} = "yes" ] ; then
    echo "Files assembled; vehicle not launched; exiting per request."
    exit 0
fi

if [ ! -e targ_$VNAME.moos ]; then echo "no targ_$VNAME.moos"; exit 1; fi


#-------------------------------------------------------
#  Part 4: Launch the processes
#-------------------------------------------------------

echo "Launching $VNAME MOOS Community "
pAntler targ_${VNAME}.moos >& /dev/null &
uMAC targ_${VNAME}.moos
UMAC_PID=$!

echo "Killing all processes ..."
kill -- -$$
#kill $UMAC_PID
echo "Done killing processes."

