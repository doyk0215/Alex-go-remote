#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdint.h>
#include "packet.h"
#include "serial.h"
#include "serialize.h"
#include "constants.h"
#define PORT_NAME			"/dev/ttyACM0"
#define BAUD_RATE			B9600

int exitFlag=0;
sem_t _xmitSema;

int FIXED_F_DISTANCE = 10;
int FIXED_M_DISTANCE = 30; 
int FIXED_F_POWER = 80; 
int FIXED_M_POWER = 70; 
int FIXED_R_DISTANCE = 10; 
int FIXED_R_POWER = 80; 
int FIXED_T_ANGLE = 3; 
int FIXED_T_POWER = 60;

int FIXED_P_DISTANCE = 3;
int FIXED_P_POWER = 60; 
//int FIXED_P_DISTANCE = 10; 
//int FIXED_P_POWER = 50; 
int FIXED_P_ANGLE = 1; 
int FIXED_P_T_POWER = 50;
//int FIXED_p_POWER = 50;

void handleError(TResult error)
{
	switch(error)
	{
		case PACKET_BAD:
			printf("ERROR: Bad Magic Number\n");
			break;
		case PACKET_CHECKSUM_BAD:
			printf("ERROR: Bad checksum\n");
			break;
		default:
			printf("ERROR: UNKNOWN ERROR\n");
	}
}

void handleStatus(TPacket *packet)
{
	printf("\n ------- ALEX STATUS REPORT ------- \n\n");
	printf("Left Forward Ticks:\t\t%d\n", packet->params[0]);
	printf("Right Forward Ticks:\t\t%d\n", packet->params[1]);
	printf("Left Reverse Ticks:\t\t%d\n", packet->params[2]);
	printf("Right Reverse Ticks:\t\t%d\n", packet->params[3]);
	printf("Left Forward Ticks Turns:\t%d\n", packet->params[4]);
	printf("Right Forward Ticks Turns:\t%d\n", packet->params[5]);
	printf("Left Reverse Ticks Turns:\t%d\n", packet->params[6]);
	printf("Right Reverse Ticks Turns:\t%d\n", packet->params[7]);
	printf("Forward Distance:\t\t%d\n", packet->params[8]);
	printf("Reverse Distance:\t\t%d\n", packet->params[9]);
	printf("\n---------------------------------------\n\n");
}

void handleColour(TPacket *packet)
{
	printf("\n ------- ALEX COLOUR ------- \n\n");
	char colour = '3';
	if (packet->params[0] == 0) {
		colour = 'w'; 
	} else if (packet->params[0] == 1) {
		colour = 'r';
	} else if (packet->params[0] == 2) {
		colour = 'g';
	} else if (packet->params[0] == 3) {
		printf("Not aligned.");
	}
	printf("The colour is :\t\t%c\n", colour);
	printf("R: %d\t", packet->params[1]);
	printf("G: %d\t", packet->params[2]);
	printf("B: %d\t", packet->params[3]);
	printf("\n---------------------------------------\n\n");
} 

void handleUltrasonic(TPacket *packet) 
{
	printf("\n ------- ALEX Ultrasonic ------- \n\n");
	printf("The distance is :\t\t%d\n", packet->params[0]);
	printf("\n---------------------------------------\n\n");
	
}

void handleResponse(TPacket *packet)
{
	// The response code is stored in command
	switch(packet->command)
	{
		case RESP_OK:
			printf("Command OK\n");
		break;
		case RESP_STATUS:
			handleStatus(packet);
		break;
		case RESP_COLOUR: 
			handleColour(packet); 
		break; 
		case RESP_ULTRASONIC: 
			handleUltrasonic(packet); 
		break; 
		default:
			printf("Arduino is confused\n");
	}
}

void handleErrorResponse(TPacket *packet)
{
	// The error code is returned in command
	switch(packet->command)
	{
		case RESP_BAD_PACKET:
			printf("Arduino received bad magic number\n");
		break;
		case RESP_BAD_CHECKSUM:
			printf("Arduino received bad checksum\n");
		break;
		case RESP_BAD_COMMAND:
			printf("Arduino received bad command\n");
		break;
		case RESP_BAD_RESPONSE:
			printf("Arduino received unexpected response\n");
		break;
		default:
			printf("Arduino reports a weird error\n");
	}
}

void handleMessage(TPacket *packet)
{
	printf("Message from Alex: %s\n", packet->data);
}

void handlePacket(TPacket *packet)
{
	switch(packet->packetType)
	{
		case PACKET_TYPE_COMMAND:
				// Only we send command packets, so ignore
			break;
		case PACKET_TYPE_RESPONSE:
				handleResponse(packet);
			break;
		case PACKET_TYPE_ERROR:
				handleErrorResponse(packet);
			break;
		case PACKET_TYPE_MESSAGE:
				handleMessage(packet);
			break;
	}
}

void sendPacket(TPacket *packet)
{
	char buffer[PACKET_SIZE];
	int len = serialize(buffer, packet, sizeof(TPacket));
	serialWrite(buffer, len);
}

void *receiveThread(void *p)
{
	char buffer[PACKET_SIZE];
	int len;
	TPacket packet;
	TResult result;
	int counter=0;
	while(1)
	{
		len = serialRead(buffer);
		counter+=len;
		if(len > 0)
		{
			result = deserialize(buffer, len, &packet);
			if(result == PACKET_OK)
			{
				counter=0;
				handlePacket(&packet);
			}
			else 
				if(result != PACKET_INCOMPLETE)
				{
					printf("PACKET ERROR\n");
					handleError(result);
				}
		}
	}
}

void flushInput()
{
	char c;
	while((c = getchar()) != '\n' && c != EOF);
}

void getParams(TPacket *commandPacket)
{
	printf("Enter distance/angle in cm/degrees (e.g. 50) and power in %% (e.g. 75) separated by space.\n");
	printf("E.g. 50 75 means go at 50 cm at 75%% power for forward/backward, or 50 degrees left or right turn at 75%%  power\n");
	scanf("%d %d", &commandPacket->params[0], &commandPacket->params[1]);
	flushInput();
}

void sendCommand(char command)
{
	TPacket commandPacket;
	commandPacket.packetType = PACKET_TYPE_COMMAND;
	switch (command) {
		//###################################### WASD Control (E to Stop) ###########################################################
        case 'w':
        case 'W':
		    commandPacket.params[0] = FIXED_F_DISTANCE;
			commandPacket.params[1] = FIXED_F_POWER; 
			commandPacket.command = COMMAND_FORWARD;
			sendPacket(&commandPacket);
            break;
        case 'e':
        case 'E':
		    commandPacket.params[0] = FIXED_M_DISTANCE;
			commandPacket.params[1] = FIXED_M_POWER; 
			commandPacket.command = COMMAND_FORWARD;
			sendPacket(&commandPacket);
            break;
        case 's':
        case 'S':
		    commandPacket.params[0] = FIXED_R_DISTANCE;
			commandPacket.params[1] = FIXED_R_POWER; 
			commandPacket.command = COMMAND_REVERSE;
			sendPacket(&commandPacket);
			break;
        case 'a':
        case 'A':
			commandPacket.params[0] = FIXED_T_ANGLE;
			commandPacket.params[1] = FIXED_T_POWER; 
			commandPacket.command = COMMAND_TURN_LEFT;
			sendPacket(&commandPacket);
			break;
        case 'd':
        case 'D':
			commandPacket.params[0] = FIXED_T_ANGLE;
			commandPacket.params[1] = FIXED_T_POWER; 
			commandPacket.command = COMMAND_TURN_RIGHT;
			sendPacket(&commandPacket);
			break;
		//###################################### Short Distance ######################################################
		case 'u':
		case 'U':
		    commandPacket.params[0] = FIXED_P_DISTANCE;
			commandPacket.params[1] = FIXED_P_POWER; 
			commandPacket.command = COMMAND_FORWARD;
			sendPacket(&commandPacket);
            break;
        case 'j':
		case 'J':
		    commandPacket.params[0] = FIXED_P_DISTANCE;
			commandPacket.params[1] = FIXED_P_POWER; 
			commandPacket.command = COMMAND_REVERSE;
			sendPacket(&commandPacket);
			break;
        case 'h':
        case 'H':
			commandPacket.params[0] = FIXED_P_ANGLE;
			commandPacket.params[1] = FIXED_P_T_POWER; 
			commandPacket.command = COMMAND_TURN_LEFT;
			sendPacket(&commandPacket);
			break;
        case 'k':
        case 'K':
			commandPacket.params[0] = FIXED_P_ANGLE;
			commandPacket.params[1] = FIXED_P_T_POWER; 
			commandPacket.command = COMMAND_TURN_RIGHT;
			sendPacket(&commandPacket);
			break;
		//###################################### Colour Identification #################################################################
		case 'i':
		case 'I':
			commandPacket.command = COMMAND_GET_COLOUR; 
			sendPacket(&commandPacket);
			break; 
		case 'o':
		case 'O':
			commandPacket.command = COMMAND_GET_ULTRASONIC; 
			sendPacket(&commandPacket);
			break; 
		// #################################### Command Control ######################################################################
		case 'f':
		case 'F':
			getParams(&commandPacket);
			commandPacket.command = COMMAND_FORWARD;
			sendPacket(&commandPacket);
			break;
		case 'b':
		case 'B':
			getParams(&commandPacket);
			commandPacket.command = COMMAND_REVERSE;
			sendPacket(&commandPacket);
			break;
		case 'l':
		case 'L':
			getParams(&commandPacket);
			commandPacket.command = COMMAND_TURN_LEFT;
			sendPacket(&commandPacket);
			break;
		case 'r':
		case 'R':
			getParams(&commandPacket);
			commandPacket.command = COMMAND_TURN_RIGHT;
			sendPacket(&commandPacket);
			break;
		case 'y':
		case 'Y':
			commandPacket.command = COMMAND_STOP;
			sendPacket(&commandPacket);
			break;
		case 'c':
		case 'C':
			commandPacket.command = COMMAND_CLEAR_STATS;
			commandPacket.params[0] = 0;
			sendPacket(&commandPacket);
			break;
		case 'g':
		case 'G':
			commandPacket.command = COMMAND_GET_STATS;
			sendPacket(&commandPacket);
			break;
		case 'q':
		case 'Q':
			exitFlag=1;
			break;
		default:
			printf("Bad command\n");
	}
}

int main()
{
	// Connect to the Arduino
	startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 5);
	// Sleep for two seconds
	printf("WAITING TWO SECONDS FOR ARDUINO TO REBOOT\n");
	sleep(2);
	printf("DONE\n");
	// Spawn receiver thread
	pthread_t recv;
	pthread_create(&recv, NULL, receiveThread, NULL);
	// Send a hello packet
	TPacket helloPacket;
	helloPacket.packetType = PACKET_TYPE_HELLO;
	sendPacket(&helloPacket);
	while(!exitFlag)
	{
		char ch;
		printf("WASD OR Command (i=identify_colour, f=forward, b=reverse, l=turn left, r=turn right, e=emergency_brake, c=clear stats, g=get stats q=exit)\n");
		scanf("%c", &ch);
		// Purge extraneous characters from input stream
		flushInput();
		sendCommand(ch);
	}
	printf("Closing connection to Arduino.\n");
	endSerial();
}
