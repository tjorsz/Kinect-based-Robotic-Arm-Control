#include "main.h"
#include "glut.h"

#include <cmath>
#include <cstdio>

#include <Windows.h>
#include <Ole2.h>

#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>


// OpenGL Variables
long depthToRgbMap[width*height*2];
// We'll be using buffer objects to store the kinect point cloud
GLuint vboId;
GLuint cboId;

// Kinect variables
HANDLE depthStream;
HANDLE rgbStream;
INuiSensor* sensor;

//Arm variables: (in meters)
const float l_1 = .1016;
const float l_2 = .1016;
const float l_3 = .14605;

// Stores the coordinates of each joint
Vector4 skeletonPosition[NUI_SKELETON_POSITION_COUNT];

bool initKinect() {
    // Get a working kinect sensor
    int numSensors;
    if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1) return false;
    if (NuiCreateSensorByIndex(0, &sensor) < 0) return false;

    // Initialize sensor
    sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_SKELETON);
    sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, // Depth camera 
        NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
        0,        // Image stream flags, e.g. near mode
        2,        // Number of frames to buffer
        NULL,     // Event handle
        &depthStream);
	sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, //  rgb camera
        NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
        0,      // Image stream flags, e.g. near mode
        2,      // Number of frames to buffer
        NULL,   // Event handle
		&rgbStream);
	sensor->NuiSkeletonTrackingEnable(NULL, 0); // NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT for only upper body
    return sensor;
}

void getDepthData(GLubyte* dest) {
	float* fdest = (float*) dest;
	long* depth2rgb = (long*) depthToRgbMap;
    NUI_IMAGE_FRAME imageFrame;
    NUI_LOCKED_RECT LockedRect;
    if (sensor->NuiImageStreamGetNextFrame(depthStream, 0, &imageFrame) < 0) return;
    INuiFrameTexture* texture = imageFrame.pFrameTexture;
    texture->LockRect(0, &LockedRect, NULL, 0);
    if (LockedRect.Pitch != 0) {
        const USHORT* curr = (const USHORT*) LockedRect.pBits;
        for (int j = 0; j < height; ++j) {
			for (int i = 0; i < width; ++i) {
				// Get depth of pixel in millimeters
				USHORT depth = NuiDepthPixelToDepth(*curr++);
				// Store coordinates of the point corresponding to this pixel
				Vector4 pos = NuiTransformDepthImageToSkeleton(i, j, depth<<3, NUI_IMAGE_RESOLUTION_640x480);
				*fdest++ = pos.x/pos.w;
				*fdest++ = pos.y/pos.w;
				*fdest++ = pos.z/pos.w;
				// Store the index into the color array corresponding to this pixel
				NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
					NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, NULL,
					i, j, depth<<3, depth2rgb, depth2rgb+1);
				depth2rgb += 2;
			}
		}
    }
    texture->UnlockRect(0);
    sensor->NuiImageStreamReleaseFrame(depthStream, &imageFrame);
}

void getRgbData(GLubyte* dest) {
	float* fdest = (float*) dest;
	long* depth2rgb = (long*) depthToRgbMap;
	NUI_IMAGE_FRAME imageFrame;
    NUI_LOCKED_RECT LockedRect;
    if (sensor->NuiImageStreamGetNextFrame(rgbStream, 0, &imageFrame) < 0) return;
    INuiFrameTexture* texture = imageFrame.pFrameTexture;
    texture->LockRect(0, &LockedRect, NULL, 0);
    if (LockedRect.Pitch != 0) {
        const BYTE* start = (const BYTE*) LockedRect.pBits;
        for (int j = 0; j < height; ++j) {
			for (int i = 0; i < width; ++i) {
				// Determine rgb color for each depth pixel
				long x = *depth2rgb++;
				long y = *depth2rgb++;
				// If out of bounds, then don't color it at all
				if (x < 0 || y < 0 || x > width || y > height) {
					for (int n = 0; n < 3; ++n) *(fdest++) = 0.0f;
				}
				else {
					const BYTE* curr = start + (x + width*y)*4;
					for (int n = 0; n < 3; ++n) *(fdest++) = curr[2-n]/255.0f;
				}

			}
		}
    }
    texture->UnlockRect(0);
    sensor->NuiImageStreamReleaseFrame(rgbStream, &imageFrame);
}

void getSkeletalData() {
	NUI_SKELETON_FRAME skeletonFrame = {0};
    if (sensor->NuiSkeletonGetNextFrame(0, &skeletonFrame) >= 0) {
		sensor->NuiTransformSmooth(&skeletonFrame, NULL);
		// Loop over all sensed skeletons
		for (int z = 0; z < NUI_SKELETON_COUNT; ++z) {
			const NUI_SKELETON_DATA& skeleton = skeletonFrame.SkeletonData[z];
			// Check the state of the skeleton
			if (skeleton.eTrackingState == NUI_SKELETON_TRACKED) {
				// Copy the joint positions into our array
				for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i) {
					skeletonPosition[i] = skeleton.SkeletonPositions[i];
					if (skeleton.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_NOT_TRACKED) {
						skeletonPosition[i].w = 0;
					}
				}
				return; // Only take the data for one skeleton
			}
		}
	}
}
void getKinectData() {
	const int dataSize = width*height*3*4;
	GLubyte* ptr;
	glBindBuffer(GL_ARRAY_BUFFER, vboId);
	ptr = (GLubyte*) glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	if (ptr) {
		getDepthData(ptr);
	}
	glUnmapBuffer(GL_ARRAY_BUFFER);
	glBindBuffer(GL_ARRAY_BUFFER, cboId);
	ptr = (GLubyte*) glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	if (ptr) {
		getRgbData(ptr);
	}
	glUnmapBuffer(GL_ARRAY_BUFFER);
	getSkeletalData();
}

void rotateCamera() {
	static double angle = 0.;
	static double radius = 3.;
	double x = radius*sin(angle);
	double z = radius*(1-cos(angle)) - radius/2;
	glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
	gluLookAt(x,0,z,0,0,radius/2,0,1,0);
	angle += 0.05;
}

void drawKinectData() {
	getKinectData();
	//rotateCamera();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);

	glBindBuffer(GL_ARRAY_BUFFER, vboId);
	glVertexPointer(3, GL_FLOAT, 0, NULL);

	glBindBuffer(GL_ARRAY_BUFFER, cboId);
	glColorPointer(3, GL_FLOAT, 0, NULL);

	glPointSize(1.f);
	glDrawArrays(GL_POINTS, 0, width*height);

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	// Draw some arms
	const Vector4& lh = skeletonPosition[NUI_SKELETON_POSITION_HAND_LEFT];
	const Vector4& le = skeletonPosition[NUI_SKELETON_POSITION_ELBOW_LEFT];
	const Vector4& ls = skeletonPosition[NUI_SKELETON_POSITION_SHOULDER_LEFT];
	const Vector4& rh = skeletonPosition[NUI_SKELETON_POSITION_HAND_RIGHT];
	const Vector4& re = skeletonPosition[NUI_SKELETON_POSITION_ELBOW_RIGHT];
	const Vector4& rs = skeletonPosition[NUI_SKELETON_POSITION_SHOULDER_RIGHT];

	//Not Drawn, but introduced later
	const Vector4& cs = skeletonPosition[NUI_SKELETON_POSITION_SHOULDER_CENTER];
	glBegin(GL_LINES);
		glColor3f(1.f, 0.f, 0.f);
		if (lh.w > 0 && le.w > 0 && ls.w > 0) {
			glVertex3f(lh.x, lh.y, lh.z);
			glVertex3f(le.x, le.y, le.z);
			glVertex3f(le.x, le.y, le.z);
			glVertex3f(ls.x, ls.y, ls.z);
		}
		if (rh.w > 0 && re.w > 0 && rs.w > 0) {
			glVertex3f(rh.x, rh.y, rh.z);
			glVertex3f(re.x, re.y, re.z);
			glVertex3f(re.x, re.y, re.z);
			glVertex3f(rs.x, rs.y, rs.z);
		}
	glEnd();
}



//_________________________________________Maestro_Movement_functions___________________________________________________________


/** Opens a handle to a serial port in Windows using CreateFile.
* portName: The name of the port.
* baudRate: The baud rate in bits per second.
* Returns INVALID_HANDLE_VALUE if it fails.  Otherwise returns a handle to the port.
*   Examples: "COM4", "\\\\.\\USBSER000", "USB#VID_1FFB&PID_0089&MI_04#6&3ad40bf600004# */
HANDLE openPort(const char * portName, unsigned int baudRate)
{
	HANDLE port;
	DCB commState;
	BOOL success;
	COMMTIMEOUTS timeouts;

	/* Open the serial port. */
	port = CreateFileA(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (port == INVALID_HANDLE_VALUE)
	{
		switch (GetLastError())
		{
		case ERROR_ACCESS_DENIED:
			fprintf(stderr, "Error: Access denied.  Try closing all other programs that are using the device.\n");
			break;
		case ERROR_FILE_NOT_FOUND:
			fprintf(stderr, "Error: Serial port not found.  "
				"Make sure that \"%s\" is the right port name.  "
				"Try closing all programs using the device and unplugging the "
				"device, or try rebooting.\n", portName);
			break;
		default:
			fprintf(stderr, "Error: Unable to open serial port.  Error code 0x%x.\n", GetLastError());
			break;
		}
		return INVALID_HANDLE_VALUE;
	}

	/* Set the timeouts. */
	success = GetCommTimeouts(port, &timeouts);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to get comm timeouts.  Error code 0x%x.\n", GetLastError());
		CloseHandle(port);
		return INVALID_HANDLE_VALUE;
	}
	timeouts.ReadIntervalTimeout = 1000;
	timeouts.ReadTotalTimeoutConstant = 1000;
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = 1000;
	timeouts.WriteTotalTimeoutMultiplier = 0;
	success = SetCommTimeouts(port, &timeouts);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to set comm timeouts.  Error code 0x%x.\n", GetLastError());
		CloseHandle(port);
		return INVALID_HANDLE_VALUE;
	}

	/* Set the baud rate. */
	success = GetCommState(port, &commState);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to get comm state.  Error code 0x%x.\n", GetLastError());
		CloseHandle(port);
		return INVALID_HANDLE_VALUE;
	}
	commState.BaudRate = baudRate;
	success = SetCommState(port, &commState);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to set comm state.  Error code 0x%x.\n", GetLastError());
		CloseHandle(port);
		return INVALID_HANDLE_VALUE;
	}

	/* Flush out any bytes received from the device earlier. */
	success = FlushFileBuffers(port);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to flush port buffers.  Error code 0x%x.\n", GetLastError());
		CloseHandle(port);
		return INVALID_HANDLE_VALUE;
	}

	return port;
}

/** Implements the Maestro's Get Position serial command.
* channel: Channel number from 0 to 23
* position: A pointer to the returned position value (for a servo channel, the units are quarter-milliseconds)
* Returns 1 on success, 0 on failure.
* For more information on this command, see the "Serial Servo Commands"
* section of the Maestro User's Guide: http://www.pololu.com/docs/0J40 */
BOOL maestroGetPosition(HANDLE port, unsigned char channel, unsigned short * position)
{
	unsigned char command[2];
	unsigned char response[2];
	BOOL success;
	DWORD bytesTransferred;

	// Compose the command.
	command[0] = 0x90;
	command[1] = channel;

	// Send the command to the device.
	success = WriteFile(port, command, sizeof(command), &bytesTransferred, NULL);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to write Get Position command to serial port.  Error code 0x%x.", GetLastError());
		return 0;
	}
	if (sizeof(command) != bytesTransferred)
	{
		fprintf(stderr, "Error: Expected to write %d bytes but only wrote %d.", sizeof(command), bytesTransferred);
		return 0;
	}

	// Read the response from the device.
	success = ReadFile(port, response, sizeof(response), &bytesTransferred, NULL);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to read Get Position response from serial port.  Error code 0x%x.", GetLastError());
		return 0;
	}
	if (sizeof(response) != bytesTransferred)
	{
		fprintf(stderr, "Error: Expected to read %d bytes but only read %d (timeout). "
			"Make sure the Maestro's serial mode is USB Dual Port or USB Chained.", sizeof(command), bytesTransferred);
		return 0;
	}

	// Convert the bytes received in to a position.
	*position = response[0] + 256 * response[1];

	return 1;
}

/** Implements the Maestro's Set Target serial command.
* channel: Channel number from 0 to 23
* target: The target value (for a servo channel, the units are quarter-milliseconds)
* Returns 1 on success, 0 on failure.
* Fore more information on this command, see the "Serial Servo Commands"
* section of the Maestro User's Guide: http://www.pololu.com/docs/0J40 */
BOOL maestroSetTarget(HANDLE port, unsigned char channel, unsigned short target)
{
	unsigned char command[4];
	DWORD bytesTransferred;
	BOOL success;

	// Compose the command.
	command[0] = 0x84;
	command[1] = channel;
	command[2] = target & 0x7F;
	command[3] = (target >> 7) & 0x7F;

	// Send the command to the device.
	success = WriteFile(port, command, sizeof(command), &bytesTransferred, NULL);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to write Set Target command to serial port.  Error code 0x%x.", GetLastError());
		return 0;
	}
	if (sizeof(command) != bytesTransferred)
	{
		fprintf(stderr, "Error: Expected to write %d bytes but only wrote %d.", sizeof(command), bytesTransferred);
		return 0;
	}

	return 1;
}

/** Make sure to initialize speed otherwise you get a really violent robotic arm. */

BOOL maestroSetSpeed(HANDLE port, unsigned char channel, unsigned short speed)
{
	unsigned char command[4];
	//unsigned char response[4];
	BOOL success;
	DWORD bytesTransferred;

	// Compose the command.
	command[0] = 0x87; // setting the speed
	command[1] = channel;
	int lowerbits = speed % 32;
	int higherbits = speed / 32;
	command[2] = lowerbits << 2;
	command[3] = higherbits;

	// Send the command to the device.
	success = WriteFile(port, command, sizeof(command), &bytesTransferred, NULL);

	return 1;
	// Read the response from the device.
	//success = ReadFile(port, response, sizeof(response), &bytesTransferred, NULL);

}
//This function takes in the radian angle given by the trig functions and converts to uSec input needed for 

unsigned short thetaRadToUSec(double radians, double min, double max) {
	unsigned short sec = (unsigned short) 4*((radians*(max-min)/(2*3.1415926))+min); // the *4 is needed
	return sec;
}

BOOL moveArm(HANDLE port)
{
	//take current arm data
	Vector4& lh = skeletonPosition[NUI_SKELETON_POSITION_HAND_LEFT];

	const Vector4& le = skeletonPosition[NUI_SKELETON_POSITION_ELBOW_LEFT];
	const Vector4& ls = skeletonPosition[NUI_SKELETON_POSITION_SHOULDER_LEFT];
	const Vector4& rh = skeletonPosition[NUI_SKELETON_POSITION_HAND_RIGHT];
	const Vector4& re = skeletonPosition[NUI_SKELETON_POSITION_ELBOW_RIGHT];
	const Vector4& rs = skeletonPosition[NUI_SKELETON_POSITION_SHOULDER_RIGHT];
	const Vector4& cs = skeletonPosition[NUI_SKELETON_POSITION_SHOULDER_CENTER];

	//Left hand is the origin
	//Right hand is the target
	lh.x = -.5219;
	lh.y = .300888;
	lh.z = .96446;

	//Right hand
	double des_x = rh.x - lh.x;
	double des_y = rh.z - lh.z; // not a typo, Kinect has a flipped x,y,z 
	double des_z = rh.y - lh.y;

	double r = sqrt(pow(des_x, 2) + pow(des_y, 2)); //radius length
	double h = sqrt(pow((des_z - l_1), 2) + pow(r, 2));		//distance from joint1 to wrist joint


	double angle_1 = atan2(des_y, des_x);
	double angle_2 = atan2(r, (des_z - l_1)) + acos((pow(l_3, 2) - pow(l_2, 2) - pow(h, 2) / (-2 * l_1*h)));
	double angle_3 = 2*(3.1415926) - acos((pow(h,2) - pow(l_1,2) - pow(l_2, 2)) / (-2 * l_1*l_1));

	double theta_1 = atan2(des_y, des_x); //  lh.x and lh.y WRT to 
	double theta_3 = acos((pow(des_z - l_1, 2) + pow(des_x, 2) + pow(des_y, 2) - pow(l_2, 2) - pow(l_3, 2)) / (2 * l_2*l_3));
	double theta_2 = atan2(r, des_x - l_1) - atan2(l_2 + l_3*cos(theta_3), l_3 * sin(theta_3)); //theta 2 is somehow depedent on theta 3? 
	
	// from points, determine other portions
	
	maestroSetTarget(port, 8, thetaRadToUSec(theta_1, 992.0, 2368.0 ));
	maestroSetTarget(port, 4, thetaRadToUSec(theta_3, 992, 2368));
	maestroSetTarget(port, 6, thetaRadToUSec(theta_2, 1296, 2704));
	
	double test = thetaRadToUSec(theta_3, 992, 2368);

	

	/* //Here goes nothing... Arm recognition:
	length_1=sqrt((cs.x-ls.x)^2 + (cs.y-ls.y)^2); // center shoulder and left shoulder
	length_2=sqrt((ls.x-le.x)^2 + (ls.x-le.y)^2); // left shoulder and left elbow
	length_3=sqrt((cs.x-le.x)^2 + (cs.y-le.y)^2); // center shoulder and left elbow
	length_4=sqrt((le.x-lh.x)^2+(le.y-lh.y)^2);  // left elbow, left hand
	length_5=sqrt((ls.x-lh.x)^2+(ls.y-lh.y)^2);   // left shoulder, left hand

	theta_1= acos((length_1^2 + length_2^2 - length_3^2)/(2*length_1 *length_2);
	maestroSetTarget(port, 4 , thetaRadtoUSec(theta_1));
	theta_2= acos((length_2^2 + length_4^2 - length_5^2)/(2*length_2 *length_4);
	maestroSetTarget(port, 6 , thetaRadtoUSec(theta_2));

	*/

	return 1;

}

void initSpeed(HANDLE port) {
	maestroSetSpeed(port, 0, 5);
	maestroSetSpeed(port, 1, 5);
	maestroSetSpeed(port, 2, 5);
	maestroSetSpeed(port, 3, 5);
	maestroSetSpeed(port, 4, 5);
	//maestroSetSpeed(port, 5, 5); Channel 5 isn't connected. Go figure.
	maestroSetSpeed(port, 6, 5);
	maestroSetSpeed(port, 7, 5);
	maestroSetSpeed(port, 8, 5);
	maestroSetSpeed(port, 9, 5);
}



int main(int argc, char* argv[]) {
	if (!init(argc, argv)) return 1;
	if (!initKinect()) return 1;

	/*Initialiaze the port that is used for the Motor controller*/
	HANDLE port;
	char * portName;
	int baudRate;
	BOOL success;
	unsigned short target, position;

	portName = "COM4";  // Each double slash in this source code represents one slash in the actual name.
	baudRate = 9600;    // bits per second

	/* Open the Maestro's serial port. */
	port = openPort(portName, baudRate);

						//Set any servo positions here. delay(x) in millisec...


						// OpenGL setup
	glClearColor(0, 0, 0, 0);
	glClearDepth(1.0f);

	// Set up array buffers
	const int dataSize = width*height * 3 * 4;
	glGenBuffers(1, &vboId);
	glBindBuffer(GL_ARRAY_BUFFER, vboId);
	glBufferData(GL_ARRAY_BUFFER, dataSize, 0, GL_DYNAMIC_DRAW);
	glGenBuffers(1, &cboId);
	glBindBuffer(GL_ARRAY_BUFFER, cboId);
	glBufferData(GL_ARRAY_BUFFER, dataSize, 0, GL_DYNAMIC_DRAW);

	// Camera setup
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, width / (GLdouble)height, 0.1, 1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 0, 0, 0, 1, 0, 1, 0);

	//initialize speed for every channel, we only really need channel 0,1, 4 and 6
	//initSpeed(port);

	// Main loop
	//execute();
	while (1) {
		//Take in Kinect data
		getKinectData();
		if (skeletonPosition[NUI_SKELETON_POSITION_HAND_RIGHT].z != 0) {
			moveArm(port);
		}


	}

	//After the execute function is terminated via 'x', use those coordinates.

	//Close the port handle for motor controller
	//CloseHandle(port);
	return 0;
}

