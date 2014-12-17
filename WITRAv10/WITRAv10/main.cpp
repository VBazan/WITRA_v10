//#################################################################################
// WITRA - Wearable Interface for Teleoperation of a Robotic Arm
// Author: Vinícius Bazan Pinto Fernandes
// Company: Universidade de São Paulo - EESC
// Creation date: 11-Dec-2014
// Version: 10 - 3 IMUs, recording datalog, gripper button added
//#################################################################################

#include <windows.h>
#include <iostream>
#include <fstream>
#include <string>
#include <Winbase.h>
#include <sstream>
#include <vector>
#include <math.h>
#include <stdio.h>
#include <cmath>
#include <winsock2.h>
// 
#include <stdlib.h>

//INTERRUPT
#include "intlib.h"
#include "CoProcLib.h"
#include "MapRegLib.h"
#include "GpioLib.h"
//--------

#pragma comment(lib, "Ws2.lib")



//******************************************************************************
/// Configures COM port for specified baudrate
/// @param[in]    port         COM port
/// @param[in]    baudRate     Baudrate for communication
/// @retval       TRUE         Success
///               FALSE        Failure

/// Set IP address and port number according to server IP address and port number

//#define SERVER_IP_ADDRESS    "192.168.27.1"       // SERVER LAPTOP
//#define SERVER_IP_ADDRESS    "169.254.189.208"  // SERVER SCARA
#define SERVER_IP_ADDRESS    "192.168.27.50"  // SERVER SCARA

#define PORT_NUMBER          8000
//#define BUFFER_SIZE          21
#define BUFFER_SIZE          128
#define BUFFER_SIZE_JSON     128 

//******************************************************************************
// Global variables

bool stop1 = false;						// stop flag for MainThread
bool offsetFlag = true;					// flag used to offset yaw the first time MainThread is executed
char OPTION;

//******  UART  ******
HANDLE portHandle;
HANDLE portHandle2;
HANDLE portHandle3;
DWORD noOfBytesRead = 0;
DWORD noOfBytesRead2 = 0;
DWORD noOfBytesRead3 = 0;
char receiveBuffer[23] = {0};			// buffer for incoming serial data
char receiveBuffer2[23] = {0};
char receiveBuffer3[28] = {0};
std::vector<double> angles;				// vector of floating point numbers corresponding to the data from the IMU

//******  SOCKETS  *******
WSADATA ws;								///< Structure to contain information about the Windows Socket implementation
SOCKET commSocket;
int retVal_S = 0;
char sendBuffer[BUFFER_SIZE] = {0};
char recvBuffer[BUFFER_SIZE] = {0};
struct sockaddr_in serverinfo;

//***** Human Arm Parameters *****
double theta1 = 0.0;
double theta2 = 0.0;
double theta3 = 0.0;
double phi1 = 0.0;
double phi2 = 0.0;
double phi3 = 0.0;

const double L1 = 0.25;				   // meters  
const double L2 = 0.25;				   // meters
const double L3 = 0.08;				   // meters

double px = 0.0;
double py = 0.0;
double pz = L1 + L2 + L3;
//*******************************

//******** Hysteresys Loop *******
double difTheta1 = 0.0;
double difTheta2 = 0.0;
double difTheta3 = 0.0;
double difPhi1 = 0.0;
double difPhi2 = 0.0;
double difPhi3 = 0.0;
double lastTheta1 = 0.0;
double lastTheta2 = 0.0;
double lastTheta3 = 0.0;
double lastPhi1 = 0.0;
double lastPhi2 = 0.0;
double lastPhi3 = 0.0;

double tolerance = 0.05;
//*******************************

//********** Datalog *************
std::vector<double> log_px;
std::vector<double> log_py;
std::vector<double> log_pz;
std::vector<int> log_g;
//********************************

//******** INTERRUPT - GRIPPER BUTTON *******
    PIN_INSTANCE gpio;
    HANDLE hEvent = NULL;  
    BOOL pinLevel = FALSE;
    BOOL returnFlag = FALSE;
    DWORD irq = 0;   
    DWORD gpioNumber = 0;
    DWORD interruptEdge = 0;
    DWORD systemInterrupt = 0;
	BOOL gripperFlag = FALSE;
//*******************************************

// Contador para o número de iterações do loop da MainThread()
int i = 1;

//*******************************************************************************************************

BOOL PortOpen(HANDLE *port, DWORD baudRate)
{
    DCB portDCB;                                              ///< COM port configuration structure
    BOOL returnValue = FALSE;
    COMMTIMEOUTS comTimeOut;
	baudRate = strtol("57600", 0,0);
   
	/// Open interface to reader
    *port = CreateFile(TEXT("COM3:"), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_FLAG_WRITE_THROUGH, NULL);
    if (*port == INVALID_HANDLE_VALUE)
    {
        printf("Error Opening COM Port 1\n");
        return FALSE;
    }
 
    /// COM Port Configuration
    portDCB.DCBlength = sizeof (DCB);                         ///< Initialize the DCBlength member
    GetCommState (*port, &portDCB);                           ///< Get the default port setting information.
    /// Change the DCB structure settings
    portDCB.BaudRate = baudRate;                              ///< Current baud 
    portDCB.fBinary = TRUE;                                   ///< Binary mode; no EOF check 
    portDCB.fParity = FALSE;                                  ///< Disable parity checking 
    portDCB.fOutxCtsFlow = FALSE;                             ///< No CTS output flow control 
    portDCB.fOutxDsrFlow = FALSE;                             ///< No DSR output flow control 
    portDCB.fDtrControl = DTR_CONTROL_DISABLE;                ///< Disable DTR flow control type 
    portDCB.fDsrSensitivity = FALSE;                          ///< DSR sensitivity 
    portDCB.fTXContinueOnXoff = TRUE;                         ///< XOFF continues Tx 
    portDCB.fOutX = FALSE;                                    ///< No XON/XOFF out flow control 
    portDCB.fInX = FALSE;                                     ///< No XON/XOFF in flow control 
    portDCB.fErrorChar = FALSE;                               ///< Disable error replacement 
    portDCB.fNull = FALSE;                                    ///< Disable null stripping 
    portDCB.fRtsControl = RTS_CONTROL_DISABLE;                ///< Disable RTS flow control 
    portDCB.fAbortOnError = FALSE;                            ///< Do not abort reads/writes on error
    portDCB.ByteSize = 8;                                     ///< Number of bits/byte, 4-8 
    portDCB.Parity = NOPARITY;                                ///< 0-4 = no, odd, even, mark, space 
    portDCB.StopBits = ONESTOPBIT;                            ///< 0, 1, 2 = 1, 1.5, 2 
 
    /// Configure the port according to the specifications of the DCB structure
    if (!SetCommState (*port, &portDCB))
    {
      printf("Error Configuring COM Port 1\n");                 ///< Could not configure the serial port
      return FALSE;
    }
 
    /// Get communication time out values
    returnValue = GetCommTimeouts(*port, &comTimeOut);
    comTimeOut.ReadIntervalTimeout = 10;
    comTimeOut.ReadTotalTimeoutMultiplier = 1;
    comTimeOut.ReadTotalTimeoutConstant = 1;
    /// Set communication time out values
    returnValue = SetCommTimeouts(*port, &comTimeOut);
 
    return TRUE;
}

BOOL PortOpen2(HANDLE *port, DWORD baudRate)
{
    DCB portDCB;                                              ///< COM port configuration structure
    BOOL returnValue = FALSE;
    COMMTIMEOUTS comTimeOut;
	baudRate = strtol("57600", 0,0);
    
	/// Open interface to reader
    *port = CreateFile(TEXT("COM2:"), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_FLAG_WRITE_THROUGH, NULL);
    if (*port == INVALID_HANDLE_VALUE)
    {
        printf("Error Opening COM Port 2\n");
        return FALSE;
    }
 
    /// COM Port Configuration
    portDCB.DCBlength = sizeof (DCB);                         ///< Initialize the DCBlength member
    GetCommState (*port, &portDCB);                           ///< Get the default port setting information.
    /// Change the DCB structure settings
    portDCB.BaudRate = baudRate;                              ///< Current baud 
    portDCB.fBinary = TRUE;                                   ///< Binary mode; no EOF check 
    portDCB.fParity = FALSE;                                  ///< Disable parity checking 
    portDCB.fOutxCtsFlow = FALSE;                             ///< No CTS output flow control 
    portDCB.fOutxDsrFlow = FALSE;                             ///< No DSR output flow control 
    portDCB.fDtrControl = DTR_CONTROL_DISABLE;                ///< Disable DTR flow control type 
    portDCB.fDsrSensitivity = FALSE;                          ///< DSR sensitivity 
    portDCB.fTXContinueOnXoff = TRUE;                         ///< XOFF continues Tx 
    portDCB.fOutX = FALSE;                                    ///< No XON/XOFF out flow control 
    portDCB.fInX = FALSE;                                     ///< No XON/XOFF in flow control 
    portDCB.fErrorChar = FALSE;                               ///< Disable error replacement 
    portDCB.fNull = FALSE;                                    ///< Disable null stripping 
    portDCB.fRtsControl = RTS_CONTROL_DISABLE;                ///< Disable RTS flow control 
    portDCB.fAbortOnError = FALSE;                            ///< Do not abort reads/writes on error
    portDCB.ByteSize = 8;                                     ///< Number of bits/byte, 4-8 
    portDCB.Parity = NOPARITY;                                ///< 0-4 = no, odd, even, mark, space 
    portDCB.StopBits = ONESTOPBIT;                            ///< 0, 1, 2 = 1, 1.5, 2 
 
    /// Configure the port according to the specifications of the DCB structure
    if (!SetCommState (*port, &portDCB))
    {
      printf("Error Configuring COM Port 2\n");                 ///< Could not configure the serial port
      return FALSE;
    }
 
    /// Get communication time out values
    returnValue = GetCommTimeouts(*port, &comTimeOut);
    comTimeOut.ReadIntervalTimeout = 10;
    comTimeOut.ReadTotalTimeoutMultiplier = 1;
    comTimeOut.ReadTotalTimeoutConstant = 1;
    /// Set communication time out values
    returnValue = SetCommTimeouts(*port, &comTimeOut);
 
    return TRUE;
}

BOOL PortOpen3(HANDLE *port, DWORD baudRate)
{
    DCB portDCB;                                              ///< COM port configuration structure
    BOOL returnValue = FALSE;
    COMMTIMEOUTS comTimeOut;
	baudRate = strtol("57600", 0,0);
    
	/// Open interface to reader
    *port = CreateFile(TEXT("COM1:"), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_FLAG_WRITE_THROUGH, NULL);
    if (*port == INVALID_HANDLE_VALUE)
    {
        printf("Error Opening COM Port 3\n");
        return FALSE;
    }
 
    /// COM Port Configuration
    portDCB.DCBlength = sizeof (DCB);                         ///< Initialize the DCBlength member
    GetCommState (*port, &portDCB);                           ///< Get the default port setting information.
    /// Change the DCB structure settings
    portDCB.BaudRate = baudRate;                              ///< Current baud 
    portDCB.fBinary = TRUE;                                   ///< Binary mode; no EOF check 
    portDCB.fParity = FALSE;                                  ///< Disable parity checking 
    portDCB.fOutxCtsFlow = FALSE;                             ///< No CTS output flow control 
    portDCB.fOutxDsrFlow = FALSE;                             ///< No DSR output flow control 
    portDCB.fDtrControl = DTR_CONTROL_DISABLE;                ///< Disable DTR flow control type 
    portDCB.fDsrSensitivity = FALSE;                          ///< DSR sensitivity 
    portDCB.fTXContinueOnXoff = TRUE;                         ///< XOFF continues Tx 
    portDCB.fOutX = FALSE;                                    ///< No XON/XOFF out flow control 
    portDCB.fInX = FALSE;                                     ///< No XON/XOFF in flow control 
    portDCB.fErrorChar = FALSE;                               ///< Disable error replacement 
    portDCB.fNull = FALSE;                                    ///< Disable null stripping 
    portDCB.fRtsControl = RTS_CONTROL_DISABLE;                ///< Disable RTS flow control 
    portDCB.fAbortOnError = FALSE;                            ///< Do not abort reads/writes on error
    portDCB.ByteSize = 8;                                     ///< Number of bits/byte, 4-8 
    portDCB.Parity = NOPARITY;                                ///< 0-4 = no, odd, even, mark, space 
    portDCB.StopBits = ONESTOPBIT;                            ///< 0, 1, 2 = 1, 1.5, 2 
 
    /// Configure the port according to the specifications of the DCB structure
    if (!SetCommState (*port, &portDCB))
    {
      printf("Error Configuring COM Port 3\n");                 ///< Could not configure the serial port
      return FALSE;
    }
 
    /// Get communication time out values
    returnValue = GetCommTimeouts(*port, &comTimeOut);
    comTimeOut.ReadIntervalTimeout = 10;
    comTimeOut.ReadTotalTimeoutMultiplier = 1;
    comTimeOut.ReadTotalTimeoutConstant = 1;
    /// Set communication time out values
    returnValue = SetCommTimeouts(*port, &comTimeOut);
 
    return TRUE;
}

//******************************************************************************
/// Close UART port
/// @param[in]    port     COM port
/// @retval       TRUE     Success
///               FALSE    Failure
BOOL PortClose(HANDLE *port)
{
    if (*port == NULL)
    {
        return FALSE;
    }
    CloseHandle(*port);
    *port = NULL;
    return TRUE;
}
 
//**************************************************************************************************
// Main Thread - deals with serial data and kinematics

DWORD WINAPI MainThread(LPVOID pParam)
	{
		 		
		//***** Variables for Timing *****
		DWORD dwOldTime;
		DWORD dwTimeElapsed;
		//********************************

		//***** Variables for string manipulation *****
		std::string delimiter1 = " ";          // delimiter to split the string of Y,P,R
		std::string delimiter2 = "\r";         // delimiter to split the end of the string of Y,P,R

		std::string str;                       // receiveBuffer is converted into this string
		size_t pos = 0;                        // position of the string for using substrings
		std::string token;                     // token (part) of a string
		double temp;                           // temporary variable to store the convertion from string to float (of Y,P,R)
		
		std::string str2;                      // receiveBuffer is converted into this string
		size_t pos2 = 0;                       // position of the string for using substrings
		std::string token2;                    // token (part) of a string
		double temp2;                          // temporary variable to store the convertion from string to float (of Y,P,R)

		std::string str3;                      // receiveBuffer is converted into this string
		size_t pos3 = 0;                       // position of the string for using substrings
		std::string token3;                    // token (part) of a string
		double temp3;                          // temporary variable to store the convertion from string to float (of Y,P,R)
		//*********************************************

		// initialization of the angles vector
		angles.push_back(0.0);  // IMU 1 YAW
		angles.push_back(0.0);  // IMU 1 PITCH
		angles.push_back(0.0);  // IMU 1 ROLL
		angles.push_back(0.0);  // IMU 2 YAW
		angles.push_back(0.0);  // IMU 2 PITCH
		angles.push_back(0.0);  // IMU 2 ROLL
		angles.push_back(0.0);  // IMU 3 YAW
		angles.push_back(0.0);  // IMU 3 PITCH
		angles.push_back(0.0);  // IMU 3 ROLL

		// offsets
		double yawOffset;
		double yawOffset2;
		double yawOffset3;
		double pitchOffset;
		double pitchOffset2;
		double pitchOffset3;
		double rollOffset;
		double rollOffset2;
		double rollOffset3; 

		// Intermediate kinematic variables
		double x1 = 0.0;
		double y1 = 0.0;
		double z1 = 0.0;
		
		double x2 = 0.0;
		double y2 = 0.0;
		double z2 = 0.0;
		
		//if (OPTION == '1'){
                while( stop1 == false )
                {						
					dwOldTime = GetTickCount();    // starts time counter
                   
					// ************* Reads data on serial port ****************************
					ReadFile(portHandle, receiveBuffer, 23, &noOfBytesRead, NULL);
					ReadFile(portHandle2, receiveBuffer2, 23, &noOfBytesRead2, NULL);
					ReadFile(portHandle3, receiveBuffer3, 23, &noOfBytesRead3, NULL);
					
					str = std::string(receiveBuffer);           // converts receiveBuffer (a char[]) to string
					str2 = std::string(receiveBuffer2);         // converts receiveBuffer (a char[]) to string
					str3 = std::string(receiveBuffer3);         // converts receiveBuffer (a char[]) to string
							//std::cout << str << std::endl;
							//std::cout << str2 << std::endl;
							//std::cout << str3 << std::endl << std::endl;
					// ********************************************************************

					// ************ String handling - separates the variables *************
					
					pos = str.find(delimiter1);                 // finds the position of the first white space on the string
					token = str.substr(0, pos);                 // parses the string at this position
					std::istringstream(token) >> temp;          // passes a string (token) into a stream and stores in temp (float) - casting
					angles[0] = temp;                           // angles[0] = IMU 1 Yaw;
					
					pos2 = str2.find(delimiter1);               // finds the position of the first white space on the string
					token2 = str2.substr(0, pos2);              // parses the string at this position
					std::istringstream(token2) >> temp2;        // passes a string (token) into a stream and stores in temp (float) - casting
					angles[3] = temp2;                          // angles[3] = IMU 2 Yaw

					pos3 = str3.find(delimiter1);               // finds the position of the first white space on the string
					token3 = str3.substr(0, pos3);              // parses the string at this position
					std::istringstream(token3) >> temp3;        // passes a string (token) into a stream and stores in temp (float) - casting
					angles[6] = temp3;                          // angles[6] = IMU 3 Yaw
					
					str.erase(0, pos + delimiter1.length());    // erases the part of the string that was already read (i.e. yaw value plus the white space)
					str2.erase(0, pos2 + delimiter1.length());  // erases the part of the string that was already read (i.e. yaw value plus the white space)
					str3.erase(0, pos3 + delimiter1.length());  // erases the part of the string that was already read (i.e. yaw value plus the white space)
					

					pos = str.find(delimiter1);                 // finds the second white space
					token = str.substr(0, pos);
					std::istringstream(token) >> temp;
					angles[1] = temp;                           // angles[1] = IMU 1 Pitch;

					pos2 = str2.find(delimiter1);               // finds the second white space
					token2 = str2.substr(0, pos2);
					std::istringstream(token2) >> temp2;
					angles[4] = temp2;                           // angles[4] = IMU 2 Pitch;

					pos3 = str3.find(delimiter1);               // finds the second white space
					token3 = str3.substr(0, pos3);
					std::istringstream(token3) >> temp3;
					angles[7] = temp3;                           // angles[7] = IMU 3 Pitch;

					
					str.erase(0, pos + delimiter1.length());
					str2.erase(0, pos2 + delimiter1.length());
					str3.erase(0, pos3 + delimiter1.length());

					pos = str.find(delimiter2);                // finds the end of the string (\r)
					token = str.substr(0, pos);
					std::istringstream(token) >> temp;
					angles[2] = temp;                          // angles[2] = IMU 1 Roll

					pos2 = str2.find(delimiter2);                // finds the end of the string (\r)
					token2 = str2.substr(0, pos2);
					std::istringstream(token2) >> temp2;
					angles[5] = temp2;                          // angles[5] = IMU 2 Roll

					pos3 = str3.find(delimiter2);                // finds the end of the string (\r)
					token3 = str3.substr(0, pos3);
					std::istringstream(token3) >> temp3;
					angles[8] = temp3;                          // angles[8] = IMU 3 Roll
					// ******************************************************************************


					// ************************ Offsets ************************************
			
					// Sets a offset for Yaw. That is, when the program starts, it will set Yaw to zero
					if (offsetFlag == true){    // does on the first time. Down we change the flag to false
						yawOffset = angles[0];
						pitchOffset = angles[1];
						rollOffset = angles[2];
						
						yawOffset2 = angles[3];
						pitchOffset2 = angles[4];
						rollOffset2 = angles[5];

						yawOffset3 = angles[6];
						pitchOffset3 = angles[7];
						rollOffset3 = angles[8];

						
					}
					
					// If, in initialization, yaw is in the second quadrant, it may cross the 180/-180 border
					  // this removes this border and sets it in the fourth quadrant to avoid crossing it
					if (yawOffset >= 90.0 && yawOffset <= 180.0){
						if (angles[0] < 0.0){
							angles[0] = 360.0 + angles[0];
						}
					}

					// If, in initialization, yaw is in the third quadrant, it may cross the 180/-180 border
					  // this removes this border and sets it in the first quadrant to avoid crossing it
					if (yawOffset >= -180.0 && yawOffset <= -90.0){
						if (angles[0] > 0) {
							angles[0] = angles[0] - 360.0;
						}
					}

					// If, in initialization, yaw is in the second quadrant, it may cross the 180/-180 border
					  // this removes this border and sets it in the fourth quadrant to avoid crossing it
					if (yawOffset2 >= 90.0 && yawOffset2 <= 180.0){
						if (angles[3] < 0.0){
							angles[3] = 360.0 + angles[3];
						}
					}

					// If, in initialization, yaw is in the third quadrant, it may cross the 180/-180 border
					  // this removes this border and sets it in the first quadrant to avoid crossing it
					if (yawOffset2 >= -180 && yawOffset2 <= -90){
						if (angles[3] > 0) {
							angles[3] = angles[3] - 360;
						}
					}

					// If, in initialization, yaw is in the second quadrant, it may cross the 180/-180 border
					  // this removes this border and sets it in the fourth quadrant to avoid crossing it
					if (yawOffset3 >= 90.0 && yawOffset3 <= 180.0){
						if (angles[6] < 0.0){
							angles[6] = 360.0 + angles[6];
						}
					}

					// If, in initialization, yaw is in the third quadrant, it may cross the 180/-180 border
					  // this removes this border and sets it in the first quadrant to avoid crossing it
					if (yawOffset3 >= -180 && yawOffset3 <= -90){
						if (angles[6] > 0) {
							angles[6] = angles[6] - 360;
						}
					}
//std::cout << angles[0] << std::endl;
					// sets the offsets
					angles[0] = angles[0] - yawOffset;           // yaw starts at 0 degrees
					angles[1] = angles[1] - pitchOffset;         // pitch starts at 0 degrees
					angles[2] = angles[2] - rollOffset;			 // roll starts at 0 degrees
					angles[3] = angles[3] - yawOffset2;     
					angles[4] = angles[4] - pitchOffset2;
					angles[5] = angles[5] - rollOffset2;
					angles[6] = angles[6] - yawOffset3;     
					angles[7] = angles[7] - pitchOffset3;
					angles[8] = angles[8] - rollOffset3;
					// ***************************************************************************
//std::cout << angles[0] << std::endl;
								// Prints data
								//std::cout << yawOffset << ", " << yawOffset2 << ", " << pitchOffset << ", " << pitchOffset2 << ", " << rollOffset << ", " << rollOffset2 << std::endl;
								/*std::cout << "Yaw 1:" << angles[0] << ", ";
								std::cout << "Pitch 1:" << angles[1]<< ", ";
								std::cout << "Roll 1:" << angles[2] << std::endl;
								std::cout << "Yaw 2:" << angles[3] << ", ";
								std::cout << "Pitch 2:" << angles[4]<< ", ";
								std::cout << "Roll 2:" << angles[5] << std::endl << std::endl;*/
					

					

//std::cout << angles[0] << std::endl;

					//**************** Calculates the Direct Kinematics of the Human Arm ********************
							// YAW 1 = theta1
							// PITCH 1 = phi1
							// YAW 2 = theta2


							// PITCH 2 = phi2

					theta1 = angles[0];
					phi1 = -angles[1];
					theta2 = angles[3];
					phi2 = -angles[4];
					theta3 = angles[6];
					phi3 = -angles[7];

					if (offsetFlag == false){
						difTheta1 = theta1 - lastTheta1;
						difTheta2 = theta2 - lastTheta2;
						difTheta3 = theta3 - lastTheta3;
						difPhi1 = phi1 - lastPhi1;
						difPhi2 = phi2 - lastPhi2;
						difPhi3 = phi3 - lastPhi3;
												
						if (abs(difTheta1) <= tolerance){ //  Lower Tolerance 
							theta1 = lastTheta1;
						}
						else {
							lastTheta1 = theta1;
						}

						if (abs(difTheta2) <= tolerance){ //  Lower Tolerance 
							theta2 = lastTheta2;
						}
						else {
							lastTheta2 = theta2;
						}

						if (abs(difTheta3) <= tolerance){ //  Lower Tolerance 
							theta3 = lastTheta3;
						}
						else {
							lastTheta3 = theta3;
						}

						if (abs(difPhi1) <= tolerance){ //  Lower Tolerance 
							phi1 = lastPhi1;
						}
						else {
							lastPhi1 = phi1;
						}

						if (abs(difPhi2) <= tolerance){ //  Lower Tolerance 
							phi2 = lastPhi2;
						}
						else {
							lastPhi2 = phi2;
						}

						if (abs(difPhi3) <= tolerance){ //  Lower Tolerance 
							phi3 = lastPhi3;
						}
						else {
							lastPhi3 = phi3;
						}
					}
					else {
						lastTheta1 = angles[0];
						lastTheta2 = angles[3];
						lastTheta3 = angles[6];
						lastPhi1 = -angles[1];
						lastPhi2 = -angles[4];
						lastPhi3 = -angles[7];

						offsetFlag = false;
					}

					//std::cout << theta1 << ", " << theta2 << ", " << theta3 << ", " << phi1 << ", " << phi2 << ", " << phi3 << std::endl;
					//std::cout << theta3 << ", " << phi3 << std::endl;


					theta1 = theta1*3.14159/180.0;
					phi1 = phi1*3.14159/180.0; // because how the IMU is positioned, a negative reading indicates a positive angle at the human arm
					theta2 = theta2*3.14159/180.0;
					phi2 = phi2*3.14159/180.0; // because how the IMU is positioned, a negative reading indicates a positive angle at the human arm
					theta3 = theta3*3.14159/180.0;
					phi3 = phi3*3.14159/180.0; // because how the IMU is positioned, a negative reading indicates a positive angle at the human arm
					
					theta1 = std::floor(100 * theta1) / 100;
					phi1 = std::floor(100 * phi1) / 100;
					theta2 = std::floor(100 * theta2) / 100;
					phi2 = std::floor(100 * phi2) / 100;
					theta3 = std::floor(100 * theta3) / 100;
					phi3 = std::floor(100 * phi3) / 100;

					//std::cout << theta1 << ", " << theta2 << ", " << theta3 << ", " << phi1 << ", " << phi2 << ", " << phi3 << std::endl;
					//std::cout << angles[0] << std::endl;
 
					x1 = L1*sin(phi1);
					y1 = L1*cos(phi1)*sin(theta1);
					z1 = L1*cos(phi1)*cos(theta1);

					x2 = L2*sin(phi2) + x1;
					y2 = L2*cos(phi2)*sin(theta2) + y1;
					z2 = L2*cos(phi2)*cos(theta2) + z1;

					px = L3*sin(phi3) + x2;
					py = L3*cos(phi3)*sin(theta3) + y2;
					pz = L3*cos(phi3)*cos(theta3) + z2;

							// Former direct kinematics - must be calculated using relative angles (relative to moving frames)
							/*px = sin(t1)*cos(t2)*(L1+cos(t4)*L2)+sin(t4)*L2*(sin(t1)*sin(t2)*sin(t3)+cos(t1)*cos(t3));
							py = cos(t2)*sin(t3)*sin(t4)*L2-sin(t2)*(L1+cos(t4)*L2);
							pz = cos(t1)*cos(t2)*(L1+cos(t4)*L2)+cos(t1)*sin(t2)*sin(t3)*sin(t4)*L2-sin(t1)*cos(t3)*sin(t4)*L2;*/
					


					//std::cout << "px: " << px << ", py: " << py << ", pz: " << pz << std::endl;
					
					//********************* SENDS DATA THROUGH SOCKET *********************
					 //// JSON conversion
					 //rapidjson::Document document;
					 //document.SetObject();

					 //rapidjson::Document::AllocatorType& allocator = document.GetAllocator();
					 //document.AddMember("Command", "Impedance", allocator);
					 //document.AddMember("px", px, allocator);
					 //document.AddMember("py", py, allocator);
					 //document.AddMember("pz", pz, allocator);

					 //// Converte de JSON document para String
					 //rapidjson::StringBuffer buffer;
					 //rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
					 //document.Accept(writer);

					 //printf("JSON (String): %s\n", buffer.GetString());
					 //// End of JSON conversion
					
					
					double tempx = px;
					double tempy = py;
					double tempz = pz;
					

					py = 0.8*tempz + 0.16;
					px = 0.8*tempy + 0.16;
					pz = tempx/3.0;

					if (pz > 0.0){
						pz = 0.0;
					}
					
					log_px.push_back(px);
					log_py.push_back(py);
					log_pz.push_back(pz);
					log_g.push_back(gripperFlag);

					// {"Command":"Impedance","X":0.3571,"Y":0.3886,"Z":-0.0351}
					sprintf(sendBuffer,"{\"Command\":\"Impedance\",\"X\":%.4lf,\"Y\":%.4lf,\"Z\":%.4lf,\"Gripper\":%d}",px,py,pz,gripperFlag);
					//sprintf(sendBuffer,"{\"Command\":\"Impedance\",\"X\":%.4lf,\"Y\":%.4lf,\"Z\":%.4lf}, %d",px,py,pz,i);
					
					//sprintf(sendBuffer, "%.3lf;%.3lf;%.3lf;%.3lf;%.3lf;%.3lf;%.3lf;%.3lf;%.3lf\n",x1,y1,z1,x2,y2,z2,px,py,pz);
					
					//std::cout << sendBuffer << std::endl;
					    //sprintf(sendBuffer, "%f,%f,%f,%f\n",theta1,phi1,theta2,phi2);
						//sprintf(sendBuffer, "%lf, %lf, %lf %d\n",px,py,pz,i);					
					
					
					retVal_S = send(commSocket, sendBuffer, BUFFER_SIZE, 0);
					 //retVal_S = send(commSocket, buffer.GetString(), BUFFER_SIZE_JSON, 0);
					if (retVal_S == SOCKET_ERROR)
					{
						printf("\nCould not send message to Server with error code : %d", WSAGetLastError());
					}
					//*********************************************************************
					
					dwTimeElapsed = GetTickCount();             // gets the final thread time
					dwTimeElapsed = dwTimeElapsed - dwOldTime;  // thread execution time
					
					// Prints Thead execution time
					std::cout << "Thread Time: " << dwTimeElapsed << std::endl << std::endl;
					//i++;
					PurgeComm(portHandle, PURGE_RXCLEAR);        ///< Clear receive buffer
					PurgeComm(portHandle2, PURGE_RXCLEAR);
					PurgeComm(portHandle3, PURGE_RXCLEAR);
					Sleep(20);
                }
			//}
			/*if (OPTION == '2'){
                while( stop1 == false )
                {
					std::cout << "repeating..." << std::endl;
					Sleep(20);
				}
			}
			if (OPTION != '1' && OPTION != '2'){
				std::cout << "waiting..." << std::endl;
				Sleep(100);
			}*/
                return 0;  
	}

// **************************************************************************************************
DWORD WINAPI GripperThread(LPVOID pParam)
{
	while( stop1 == false )
    {
		if (WaitForSingleObject(hEvent, INFINITE) == WAIT_OBJECT_0)
        {
            //pinLevel = GetPinLevel(135);
			gripperFlag = !gripperFlag;
            if (gripperFlag == TRUE)
            {
                printf("CLOSE\n");
            }
            else 
            {
                printf("OPEN\n");
            }
            //if (MessageBox(NULL, L"Interrupt Event1 Detected, continue Waiting?", L"Interrupt", MB_YESNO) == IDNO) break;
            InterruptDoneCompat(systemInterrupt);
			Sleep(100);
        }
	}
	return 0;
}

//******************************************************************************
int wmain(void)
{
    /*HANDLE portHandle;
    DWORD noOfBytesRead = 0;*/
	/*char receiveBuffer[24] = {0};*/
	//DWORD GPIOPin = 100; // parâmetro passado para a thread
	//DWORD exit = 0;

    DWORD firstChoice = 0;
    BOOL retVal = FALSE;
	BOOL retVal2 = FALSE;
	BOOL retVal3 = FALSE;
	DWORD ThreadID;
	DWORD GripperThreadID;
	HANDLE hMainThread;
	HANDLE hGripperThread;
	DWORD StopID;
	
	
	
	//************************  SOCKET CONFIGURATION  ******************************
	retVal_S = WSAStartup(0x0101, &ws);                              ///< Initialize ws2.dll (library used for socket programming)
    if (retVal_S != 0)                                               ///< If WSAStartup failed to initialize
    {
        printf("WSAStartup failed with error: %d\n", WSAGetLastError());
        exit(1);
    }
 
    commSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);        ///< Socket creation
    if (commSocket == INVALID_SOCKET)
    {
        printf("\nSocket creation failed with error code : %d", WSAGetLastError());
        WSACleanup();
        exit(1);
    }
 
    /// Socket binding
    serverinfo.sin_family = AF_INET;                                ///< TCP/UDP socket
    serverinfo.sin_addr.s_addr = inet_addr(SERVER_IP_ADDRESS);      ///< IP Address of Server
    serverinfo.sin_port = htons(PORT_NUMBER);                       ///< Port number used for communication
 
    retVal_S = connect(commSocket, (LPSOCKADDR)&serverinfo, sizeof(struct sockaddr));   ///< Connect to Server
    if (retVal_S == SOCKET_ERROR)
    {
        printf("\nCould not connect to Server with error code : %d", WSAGetLastError()); 
        WSACleanup();
		getchar();
        return FALSE;
    }
 
	//**************************  UART CONFIGURATION  ***************************	
    retVal = PortOpen(&portHandle, 57600);
	
	retVal2 = PortOpen2(&portHandle2, 57600);

	retVal3 = PortOpen3(&portHandle3, 57600);

    if (!retVal)
    {
        printf("Could not open COM port 1");
        getchar();
        return FALSE;
    }   
	else if (!retVal2)
    {
        printf("Could not open COM port 2");
        getchar();
        return FALSE;
    } 
	else if (!retVal3){
		printf("Could not open COM port 3");
        getchar();
        return FALSE;
	}
    else
    {
        retVal = FALSE;
		retVal2 = FALSE;
		retVal3 = FALSE;
       
            memset(receiveBuffer, 0, 24); 

			memset(receiveBuffer2, 0, 24);

			memset(receiveBuffer3, 0, 24);

	// ********************* GRIPPER ************************

	SetPinAltFn(133, -1, DIR_IN);                      ///< Set SODIMM pin 133 as GPIO input to capture interrupt
	GetGPIOFromPin(133, FALSE, &gpio);                 ///< Get GPIO number for SODIMM pin 133
    gpioNumber = gpio.inst1;
    //printf("\nGPIO number for SODIMM pin 133 is: %d", gpioNumber);
    irq = GetGPIOIrq(gpioNumber);                      ///< Request interrupt (IRQ) on GPIO number
    if (!irq)
    {
        printf("cannot obtain IRQ for GPIO Number %d", gpioNumber);
        getchar();
        return -1;
    }
    //printf("\nGPIO Interrupt request: %d", irq);
    interruptEdge = GPIO_EDGE_RISING | GPIO_EDGE_FALLING;                   ///< Configure external interrupt signal as rising edge
    if (!SetGPIOIrqEdge(gpioNumber, interruptEdge)) 
    {
        printf("cannot set GPIO interrupt edge detect");
        getchar();
        return -1;
    }
    hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);     ///< Create event or check for existing event to wait for
    if (!hEvent) 
    {
        printf("Event cannot be created");
        getchar();
        return -1;
    }
    systemInterrupt = RequestSysInterrupt(irq);         ///< Get the system interrupt number for corresponding irq
    if (!systemInterrupt)
    {
        printf("cannot obtain system interrupt number");
        getchar();
        return -1;
    }
    //printf("\nSystem Interrupt request: %d", systemInterrupt);
 
    if (!InterruptInitializeCompat(systemInterrupt, hEvent, NULL, 0))     ///<  Initialize system interrupt
    {
        printf("cannot Initialize interrupt");
        ReleaseSysIntr(systemInterrupt);
        getchar();
        return -1;
    }

            printf("\n\n\n*****************************************\n");
			printf("Welcome to WITRA: Wearable Interface for Teleoperation of Robot Arms\n");
            printf("*****************************************\n\n");
			//printf("[1] START TELEOPERATION\n");
			//printf("[2] REPEAT RECORDED TELEOPERATION\n\n");
			//printf("OPTION: ");
			//scanf("%d", &OPTION);
			//std::cin.get(OPTION);
			
           
			//OPTION = 2;
			// Creates the Main Thread 
			hMainThread = CreateThread( NULL,
										   0,
										   MainThread,
										   NULL, // parâmetro passado para a thread
										   0,
										   &ThreadID );
			
			hGripperThread = CreateThread( NULL,
										   0,
										   GripperThread,
										   NULL, // parâmetro passado para a thread
										   0,
										   &GripperThreadID );
			

        
					getchar();
					stop1 = true;

					//if (OPTION == 1){
						// Save data to datalog
						std::ofstream logx;
						std::ofstream logy;
						std::ofstream logz;
						std::ofstream logg;

						logx.open("px.txt");
						for (size_t i = 0; i < log_px.size(); i++){
							logx << log_px[i] << std::endl;
						}
						logx.close();

						logy.open("py.txt");
						for (size_t i = 0; i < log_py.size(); i++){
							logy << log_py[i] << std::endl;
						}
						logy.close();
						
						logz.open("pz.txt");
						for (size_t i = 0; i < log_pz.size(); i++){
							logz << log_pz[i] << std::endl;
						}
						logz.close();

						logg.open("gripper.txt");
						for (size_t i = 0; i < log_g.size(); i++){
							logg << log_g[i] << std::endl;
						}
						logg.close();

						std::cout << "Log saved" << std::endl;
					//}


					CloseHandle(hMainThread);
					CloseHandle(hGripperThread);
					getchar();  //usado se retirar o StopThread
					closesocket(commSocket);
					WSACleanup();  

					InterruptDisableCompat(systemInterrupt);                  ///< Disable interrupt
					if (!ReleaseSysIntr(systemInterrupt)) 
					{
						MessageBox(NULL, L"An Error occurred!\ninterrupt was not released correctly", L"ERROR", MB_OK);
						return -1;
					}
					
					return 0;
    }
}
 