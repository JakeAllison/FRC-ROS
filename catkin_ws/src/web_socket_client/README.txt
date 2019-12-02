build with:
g++ Main.cpp --std=c++11 -pthread -I include/ -o test

execute with:
./test

this will publish cmd_vel messages to the sim as well as subscribe to cmd_vel messages. Look at main.cpp for more info
