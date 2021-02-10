sudo killall wish &
sudo killall jackd &
sudo killall pd &
sudo killall /home/khadas/Code/qmidiarp-0.6.5/src/qmidiarp &
sudo killall /home/khadas/sooperlooper/bin/sooperlooper &
sudo killall /home/khadas/sooperlooper/bin/slgui &
sudo killall zynaddsubfx &
sudo killall jackd &
sudo killall timemachine &
sleep 5
sudo qjackctl -s &
