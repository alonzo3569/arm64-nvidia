#!/bin/bash -e

#-------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------
TIME_WARP=1
#POSITION=40,-35
#HEADING=0
#COMMUNITY="logan"

SNAME="shoreside"
VNAME1="heron"
VNAME2="henry"
START_POS1="0,0"
START_POS2="80,0"

SHORE_LISTEN="9300"
SPORT="9000"

for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
        echo " --timewarp=,-t=  , set up TimeWarp. (Default is $TIME_WARP)"
        echo " --position=,-p=  , set up destionation. (Default is $POSITION)"
        echo " --heading= ,-h=  , set up heading. (Default is $HEADING)"
	printf "\n"
	echo "For example: $0 $TIME_WARP -p=$POSITION -h=$HEADING"
        echo "             $0 -t=$TIME_WARP -p=$POSITION -h=$HEADING"
	exit 0;

    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI:0:11}" = "--timewarp=" ] ; then
        TIME_WARP="${ARGI#--timewarp=*}"
    elif [ "${ARGI:0:3}" = "-t=" ] ; then
        TIME_WARP="${ARGI#-t=*}"

    elif [ "${ARGI:0:11}" = "--position=" ] ; then
        POSITION="${ARGI#--timewarp=*}"
    elif [ "${ARGI:0:3}" = "-p=" ] ; then
        POSITION="${ARGI#-p=*}"


    elif [ "${ARGI:0:10}" = "--heading=" ] ; then
        HEADING="${ARGI#--heading=*}"
    elif [ "${ARGI:0:3}" = "-h=" ] ; then
        HEADING="${ARGI#-h=*}"

    else 
	printf "Bad Argument: %s \n" $ARGI
	printf "Usage: %s %s -p=%s -h=%s\n" $0 $TIME_WARP $POSITION $HEADING
	exit 0
    fi
done

#-------------------------------------------------------
#  Part 2: Create the Shoreside MOOS file
#-------------------------------------------------------
nsplug heron.moos targ_$VNAME1.moos -f WARP=$TIME_WARP \
     VNAME=$VNAME1      START_POS=$START_POS1          \
     VPORT="9005"       SHARE_LISTEN="9305"            \
     VTYPE="kayak"      SHORE_LISTEN=$SHORE_LISTEN     \
     POS=$POSITION  HEADING=$HEADING

#nsplug duckie.moos targ_$VNAME2.moos -f WARP=$TIME_WARP \
#    VNAME=$VNAME2      START_POS=$START_POS2                  \
#    VPORT="9006"       SHARE_LISTEN="9306"                    \
#    VTYPE="kayak"      SHORE_LISTEN=$SHORE_LISTEN

nsplug shoreside.moos targ_$SNAME.moos -f WARP=$TIME_WARP \
   SHARE_LISTEN=$SHORE_LISTEN  VPORT=$SPORT COMMUNITY=$SNAME


if [ ! -e targ_$SNAME.moos ]; then echo "no targ_$SNAME.moos"; exit 1; fi
if [ ! -e targ_$VNAME1.moos ]; then echo "no targ_$VNAME1.moos"; exit 1; fi
#if [ ! -e targ_$VNAME2.moos ]; then echo "no targ_$VNAME2.moos"; exit 1; fi


#-------------------------------------------------------
#  Part 3: Launch the processes
#-------------------------------------------------------

printf "Launching $SNAME MOOS Community (WARP=%s) \n"  $TIME_WARP
pAntler targ_$SNAME.moos >& /dev/null &
printf "Launching $VNAME1 MOOS Community (WARP=%s) \n" $TIME_WARP
pAntler targ_$VNAME1.moos >& /dev/null &
#printf "Launching $VNAME2 MOOS Community (WARP=%s) \n" $TIME_WARP
#pAntler targ_$VNAME2.moos >& /dev/null &
printf "Done \n"

uMAC targ_$SNAME.moos

printf "Killing all processes ... \n"
kill -- -$$
printf "Done killing processes.   \n"





