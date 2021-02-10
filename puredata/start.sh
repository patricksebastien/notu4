sudo chmod 777 /dev/ttyS0
killall /usr/bin/mate-screensaver &
killall caja &
sleep 6
/home/khadas/sooperlooper/bin/sooperlooper -l 4 -c 2 -p 9951 -L /home/khadas/jas/sl.slsess &
sleep 2
/home/khadas/pd0502/bin/pd -nogui -rt -jack -channels 6 -open /home/khadas/jas/main.pd &
sleep 2
/home/khadas/Code/qmidiarp-0.6.5/src/qmidiarp -a /home/khadas/jas/qmidiarp.qmax &
#phasex -A jack &
#phasex &
sleep 2
#/home/khadas/sooperlooper/bin/slgui &
#fluidsynth -i -a jack /usr/share/sounds/sf2/FluidR3_GM.sf2 &
zynaddsubfx -P10000 &
#sleep 30
#jack_disconnect phasex:out_L system:playback_1
#jack_disconnect phasex:out_R system:playback_2
#jack_disconnect system:capture_2 pure_data:input1
#sleep 5
#fluidsynth -i -a jack /usr/share/sounds/sf2/FluidR3_GM.sf2 &
timemachine -f wav -p /home/khadas/Public/tm- &
sleep 2
wmctrl -r "TimeMachine" -b add,above
