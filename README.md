# Collective motion control algorithm on swarm of e-Pucks using elastic model

We provide the program of elastic model of collective motion control on e-Puck robots, based on the official e-Puck program.
To implemnet the algorithm, the e-puck has to be installed with an extra wifi module to receive the information of the other robots using a serial port. Once the Wi-Fi module connects to a router or a wireless exchange board, it can achieve one-to-all broadcast communication. The module has to configure the name, password, IP address and subnet mask. 

## Communication mechanism
We use FSM(Finite State Machine) to deal with the messages received fromm the serial port, details can be seen through the file of UART.h and UART.c
