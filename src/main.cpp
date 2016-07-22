#include "serial.hpp"
#include "crc16.hpp"
#include "recttools.hpp"
#include "ramp.hpp"
#include "fd.hpp"
#include "hogdetector.hpp"
#include "kcftracker.hpp"
#include "mytim.hpp"
#include "mythread.hpp"
#include "myevent.hpp"
#include "mylock.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <unistd.h>
#include <stdio.h>

using namespace std;
using namespace cv;

#define YES 1
#define NO 0

#define DEBUG_SONAR YES
#define DEBUG_VISION NO
#define DEBUG_ROBOT NO
#define DEBUG_ACTION NO

/**************************** HELPER FUNCTIONS ******************************/
#define scale(num,min1,max1,min2,max2) ((num-min1)*(max2-min2)/(max1-min1)+min2)
#define distance(x,y) (sqrt((x)*(x)+(y)*(y)))
#define limit(val,min,max) (val=(val<min?min:(val>max?max:val)))
#define centerX(rect) (rect.x + rect.width/2.0f)
#define centerY(rect) (rect.y + rect.height/2.0f)


/********************************* SONAR ************************************/
Serial sonar("/dev/ttyACM0");

#define SONAR_NUM 8
#define SONAR_RX_MSG_HEADER 0x00
#define SONAR_RX_MSG_INIT_CRC16  0x7000

#pragma pack(1)
typedef struct SonarRxMsg
{
	uint8_t header : 8;
	uint8_t id : 8;
	uint16_t data : 16;
	uint16_t crc16_check_sum : 16;
}SonarRxMsg;

#define SONAR_RX_MSG_LEN() sizeof(SonarRxMsg)

const uint16_t OBSTACLE[SONAR_NUM] = {45, 45, 45, 45, 45, 45, 45, 45};
uint16_t obstacle[SONAR_NUM]  = {0};

SonarRxMsg sonarRxMsg;

#define SONAR_RX_MSG_BUF_LEN 256

uint8_t obstacleMask = 0x00;
const uint8_t OBSTACLE_MASK[SONAR_NUM] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

void *SonarThreadFunction(void* param)
{
	uint8_t buf[SONAR_RX_MSG_BUF_LEN];
	uint8_t tmp[SONAR_RX_MSG_BUF_LEN - 1];
	uint32_t msg_len = 0, new_msg_len = 0;
	while (1)
	{
		msg_len += sonar.rx(buf + msg_len, SONAR_RX_MSG_BUF_LEN - msg_len);
		if (msg_len >= SONAR_RX_MSG_LEN())
		{
			int index = 0, last_index = 0;
			while (index < msg_len - SONAR_RX_MSG_LEN())
			{
				//find header
				if (buf[index] == SONAR_RX_MSG_HEADER)
				{
					//check msg
					if(CRC16Check(buf, SONAR_RX_MSG_LEN(), SONAR_RX_MSG_INIT_CRC16))
					{
						//pop data out
						memcpy(&sonarRxMsg, buf+index, SONAR_RX_MSG_LEN());
						//put it in the coordinate position
						obstacle[sonarRxMsg.id] = sonarRxMsg.data;
						if(DEBUG_SONAR == YES) 
							printf( "id=%d,data=%d\n", sonarRxMsg.id, sonarRxMsg.data);
						last_index = index;
						index += SONAR_RX_MSG_LEN();
					}
					else
					{
						last_index = index;
						index++;
					}
				}
				else
				{
					last_index = index;
					index++;
				}
			}
			new_msg_len = msg_len - last_index;
			memcpy(tmp, buf+last_index, new_msg_len);
			memcpy(buf, tmp, new_msg_len);
			msg_len = new_msg_len;
		}
	}
}

void ScanObstacle()
{
	obstacleMask = 0x00;
	for(int orientation = 0; orientation < SONAR_NUM; orientation++)
	{
		if(obstacle[sonarRxMsg.id] < OBSTACLE[sonarRxMsg.id])
			obstacleMask |= OBSTACLE_MASK[sonarRxMsg.id];
	}
}

/********************************* VISION ************************************/
VideoCapture capture(0);

#define VISION_FIFO_SIZE 5
#define VISION_FRAME_WIDTH 640
#define VISION_FRAME_HEIGHT 480
#define VISION_TARGET_STD_SIZE Size(200,100)
#define VISION_FIRE_ON_ACCURACY 180
#define VISION_RETARGET_ON_SPEED 3
#define FRAME_DELAY_FROM_LOST_T0_SCOUT 0
#define VISION_LOST_FRAME_COUNTER 4

typedef struct VisionResult
{
	uint8_t lost;
	int dx;
	int dy;
	int ds;
	float stdRatio;
}VisionResult;

VisionResult visionResult = {1, 0, 0, 0, 0};

void *VisionThreadFunction(void* param)
{
	FD motion;
	HOGDetector detector;
	detector.load("model.xml");
	KCFTracker tracker;
	static Rect roi, last_roi, obj;
	static int trackingDeltaX, trackingDeltaY;
	static double trackingDeltaS = 0;
	static uint32_t lost_counter = 0;
	
	Mat frame;
	capture.grab();	
	capture.retrieve(frame);
	motion.init(frame);
	Rect rect(0, 0, frame.cols, frame.rows);
	Vec<int, 2 > center = RectTools::center(rect);
	vector<Rect> objs;
	double t = 0;
	while(1)
	{
		//cout<<"*********************retrieve***********************\n";
		t = cvGetTickCount();
		capture.grab();	
		capture.retrieve(frame); 
		t =  cvGetTickCount() - t;
		//cout << t/cvGetTickFrequency()/1000 << endl;
	
		//t = cvGetTickCount() ;

		if(visionResult.lost)
		{
			//motion detection
			//cout << "motion" << endl;
			roi = motion.detect(frame);
			//cout << roi.width << "," << roi.height << endl;
			if (roi.width >80 && roi.height>40 && roi.width < 320 && roi.height < 240)
			{
				//appearence detection
				t = cvGetTickCount();
				objs = detector.detect(frame(roi));
				//cout<<"*******detect*****"<<((cvGetTickCount()-t)/cvGetTickFrequency()/1000.0)<<endl;
				if (objs.size() > 0) 
				{
					RectTools::getMostCenteredObj(objs, center, obj);
					RectTools::adjRoi(roi, obj);
					tracker.init(roi, frame); //important
					trackingDeltaS = VISION_RETARGET_ON_SPEED; //important
					rectangle(frame, roi, Scalar(255,0,0), 3);
					visionResult.lost = false;
				}
			 }
		}
	    else 
		{
			last_roi = roi;
			roi = tracker.update(frame); 
			if(roi.width < 80 || roi.height < 40 || roi.x+roi.width > frame.cols || roi.y+roi.height > frame.rows)
			{
				visionResult.lost = true;
			}
			else
			{
				trackingDeltaX = centerX(roi) - centerX(last_roi);
				trackingDeltaY = centerY(roi) - centerY(last_roi);
				trackingDeltaS = distance(trackingDeltaX, trackingDeltaY);
				if(trackingDeltaS < VISION_RETARGET_ON_SPEED)
				{
					//cout << "DS = " << trackingDeltaS << endl;
					lost_counter++;
					if(lost_counter > VISION_LOST_FRAME_COUNTER)
					{
						Rect verification_roi = roi;
						RectTools::resize(verification_roi, 1.2, 1.2);
						objs = detector.detect(frame(verification_roi));
						if (objs.size() > 0) 
						{
							RectTools::getMostCenteredObj(objs, center, obj);
							RectTools::adjRoi(roi, obj);
							tracker.init(roi, frame); //important
							trackingDeltaS = VISION_RETARGET_ON_SPEED; //important
							rectangle(frame, roi, Scalar(255,0,0), 3);
							visionResult.lost = false;
						}
						else
						{
							lost_counter = 0;
							visionResult.lost = true;
							//cout << "lost = true" << endl;
						}
					}
				}
				else
				{
					lost_counter = 0;
				}
			}
		}
		if (!visionResult.lost)
		{
			visionResult.dx = roi.x - frame.cols/2.f;
			visionResult.dy = roi.y - frame.rows/2.f;
			visionResult.ds= distance(visionResult.dx, visionResult.dy);
			visionResult.stdRatio = roi.area() / VISION_TARGET_STD_SIZE.area();
			//cout << centerX(roi) << "," << centerY(roi) << endl;
			rectangle(frame, roi, Scalar(0,255,0), 3);
			
			//cout << "SIZE:     " <<  roi.width << "," << roi.height << endl;
		}
		imshow("vision",frame);
		waitKey(1);
		t = cvGetTickCount() - t;
		//cout << "**************************** VISION **********************************"<<endl;
		//cout <<  fixed << "TIME COST                                        " << (t/cvGetTickFrequency()/1000)  << endl;
		//cout << "*********************************************************************"<<endl;
	}
}


/********************************* ROBOT ************************************/
Serial robot("/dev/ttyS4");

#define ROBOT_TX_MSG_HEADER 0xff
#define ROBOT_TX_MSG_INIT_CRC16 0x3692
#define ROBOT_SPEED_MIN -127
#define ROBOT_SPEED_MAX  127
#define ROBOT_SPEED_OFF 127
#define ROBOT_SPEED_HIGH 127
#define ROBOT_SPEED_LOW  50
#define ROBOT_SPEED_NORMAL 60
#define ROBOT_SPEED_ORIENTATION_COEFF_FORWARD 1
#define ROBOT_SPEED_ORIENTATION_COEFF_BACKWARD -1
#define ROBOT_SPEED_ORIENTATION_COEFF_LEFT -1
#define ROBOT_SPEED_ORIENTATION_COEFF_RIGHT 1

#pragma pack(1)
typedef struct RobotTxMsg
{
	uint8_t header : 8;
	uint8_t speed_fb : 8;
	uint8_t speed_lr : 8;
	uint8_t speed_rotate : 8;
	uint8_t speed_pan : 8;
	uint8_t speed_tilt : 8;
	uint8_t fire : 8;
	uint16_t crc16_check_sum : 16;
}RobotTxMsg;

#define ROBOT_TX_MSG_LEN() sizeof(RobotTxMsg)

RobotTxMsg robotTxMsg;

void RobotNewState(float speed_fb, float speed_lr, float speed_rotate, float speed_pan, float speed_tilt, uint8_t fire)
{
	limit(speed_fb, ROBOT_SPEED_MIN, ROBOT_SPEED_MAX);
	limit(speed_lr, ROBOT_SPEED_MIN, ROBOT_SPEED_MAX);
	limit(speed_rotate, ROBOT_SPEED_MIN, ROBOT_SPEED_MAX);
	limit(speed_pan, ROBOT_SPEED_MIN, ROBOT_SPEED_MAX);
	limit(speed_tilt, ROBOT_SPEED_MIN, ROBOT_SPEED_MAX);
	robotTxMsg.speed_fb = ROBOT_SPEED_OFF + speed_fb;
	robotTxMsg.speed_lr = ROBOT_SPEED_OFF + speed_lr;
	robotTxMsg.speed_rotate = ROBOT_SPEED_OFF + speed_rotate;
	robotTxMsg.speed_pan = ROBOT_SPEED_OFF + speed_pan;
	robotTxMsg.speed_tilt = ROBOT_SPEED_OFF + speed_tilt;
	robotTxMsg.fire = fire;
}

void RobotTxCmd()
{
	robotTxMsg.header = ROBOT_TX_MSG_HEADER;
	CRC16Append((uint8_t*)&robotTxMsg, ROBOT_TX_MSG_LEN(), ROBOT_TX_MSG_INIT_CRC16);
	robot.tx((uint8_t*)&robotTxMsg, ROBOT_TX_MSG_LEN());
	if(DEBUG_ROBOT)
		printf("fb=%d,lr=%d,rotate=%d,pan=%d,tilt=%d,fire=%d\n",robotTxMsg.speed_fb, robotTxMsg.speed_lr, robotTxMsg.speed_rotate, robotTxMsg.speed_pan, robotTxMsg.speed_tilt, robotTxMsg.fire);
}

/********************************* APP ************************************/
typedef enum
{
	APP_INIT_OK = 0x00,
	ROBOT_CONNECTION_ERROR = 0x01,
	SONAR_CONNECTION_ERROR = 0x02,
	CAMERA_CONNECTION_ERROR = 0x04
}AppError;

int AppInit()
{
	if(!robot.connect())
	{
		cout << "app init error: ROBOT_CONNECTION_ERROR" << endl;
		return ROBOT_CONNECTION_ERROR;
	}
	if(!sonar.connect())
	{
		cout << "app init error: SONAR_CONNECTION_ERROR" << endl;
		return SONAR_CONNECTION_ERROR;
	}
	if(!capture.isOpened())
	{
		cout << "app init error: CAMERA_CONNECTION_ERROR" << endl;
		return  CAMERA_CONNECTION_ERROR;
	}
	return APP_INIT_OK;
}

/*********************************  OBSTACLE_AVOIDANCE STATE ************************************/
#define HEADING_ORIRENTATION_NUM 9
typedef enum
{
	FORWARD = 0,
	FORWARD_RIGHT = 1,
	RIGHT = 2,
	RIGHT_BACKWARD = 3,
	BACKWARD = 4,
	BACKWARD_LEFT = 5,
	LEFT = 6,
	LEFT_FORWARD = 7,
	NOWHERE = 8
}Heading;

/*
const HEADING[SONAR_NUM] = 
Heading headingProposal[HEADING_ORIRENTATION_NUM] = {NOWHERE};

Heading headingHistory[HEADING_ORIRENTATION_NUM] = {NOWHERE};
*/

const Heading HEADING[HEADING_ORIRENTATION_NUM] = {FORWARD, FORWARD_RIGHT, RIGHT, RIGHT_BACKWARD, BACKWARD, BACKWARD_LEFT, LEFT, LEFT_FORWARD, NOWHERE};

Heading heading = NOWHERE;

bool ObstaclePercept(Heading orientation)
{
	return obstacle[orientation] < OBSTACLE[orientation];
}

void ObstacleAvoidanceDecFunc(Heading obstacleOrientation)
{
	//if(DEBUG_SONAR)
	//cout << "ObstacleOrientation: " << obstacleOrientation << endl;
	int orientation = 0;
	for(orientation = 0; orientation < SONAR_NUM; orientation++)
	{
		if(orientation == obstacleOrientation) continue;
		if(!ObstaclePercept((Heading)orientation))
		{
			heading = HEADING[orientation];
			break;
		}
	}
	heading = HEADING[orientation];
}

bool getObstacle(Heading orientation)
{
	return obstacleMask&orientation;
}

Heading findClearOrientation(Heading orientation)
{
	int o1,o2,o3;
	if(orientation == FORWARD)
	{
		o1 = LEFT_FORWARD;
		o2 = FORWARD_RIGHT;
		o3 = RIGHT;
	}
	else if(orientation == LEFT_FORWARD)
	{
		o1 = LEFT;
		o2 = FORWARD;
		o3 = FORWARD_RIGHT;
	}
	else
	{
		o1 = orientation - 1;
		o2 = orientation + 1;
		o3 = (o2 + 1)%8;
	}
	for(int i = o3; i != o2; i=(i+1)%8)
	{
		if(!getObstacle(HEADING[i]))
			return HEADING[i];
	}
	return NOWHERE;
}

void HeadingSM()
{
	if(heading == NOWHERE)
	{
		for(int i = 0; i < SONAR_NUM; i++)
		{
			if(!getObstacle(HEADING[i])
			{
				heading = HEADING[i];
				return;
			}
		}
	}
	else if(getObstacle(heading))
		heading = findClearOrientation(FORWARD);
	
}

#define ROBOT_CTL_STATE_NUM 9
typedef struct RobotCtrl
{
	float speed_fb;
	float speed_lr;
	float speed_rotate;
	float speed_pan;
	float speed_tilt;
	uint8_t fire;
}RobotCtrl;

#define ROBOT_CTRL_HEADING_FORWARD {ROBOT_SPEED_NORMAL * ROBOT_SPEED_ORIENTATION_COEFF_FORWARD, 0, 0, 0, 0, 0}
#define ROBOT_CTRL_HEADING_FORWARD_RIGHT {ROBOT_SPEED_NORMAL * ROBOT_SPEED_ORIENTATION_COEFF_FORWARD, ROBOT_SPEED_NORMAL * ROBOT_SPEED_ORIENTATION_COEFF_RIGHT, 0, 0, 0, 0}
#define ROBOT_CTRL_HEADING_RIGHT {0, ROBOT_SPEED_NORMAL * ROBOT_SPEED_ORIENTATION_COEFF_RIGHT, 0, 0, 0, 0}
#define ROBOT_CTRL_HEADING_RIGHT_BACKWARD {ROBOT_SPEED_NORMAL * ROBOT_SPEED_ORIENTATION_COEFF_BACKWARD, ROBOT_SPEED_NORMAL * ROBOT_SPEED_ORIENTATION_COEFF_RIGHT, 0, 0, 0, 0}
#define ROBOT_CTRL_HEADING_BACKWARD {ROBOT_SPEED_NORMAL * ROBOT_SPEED_ORIENTATION_COEFF_BACKWARD, 0, 0, 0, 0, 0}
#define ROBOT_CTRL_HEADING_BACKWARD_LEFT {ROBOT_SPEED_NORMAL * ROBOT_SPEED_ORIENTATION_COEFF_BACKWARD, ROBOT_SPEED_NORMAL * ROBOT_SPEED_ORIENTATION_COEFF_LEFT, 0, 0, 0, 0}
#define ROBOT_CTRL_HEADING_LEFT {0, ROBOT_SPEED_NORMAL * ROBOT_SPEED_ORIENTATION_COEFF_LEFT, 0, 0, 0, 0}
#define ROBOT_CTRL_HEADING_LEFT_FORWARD {ROBOT_SPEED_NORMAL * ROBOT_SPEED_ORIENTATION_COEFF_FORWARD, ROBOT_SPEED_NORMAL * ROBOT_SPEED_ORIENTATION_COEFF_LEFT, 0, 0, 0, 0}
#define ROBOT_CTRL_HEADING_NOWHERE {0, 0, 0, 0, 0, 0}

const RobotCtrl ROBOT_CTRL[ROBOT_CTL_STATE_NUM] = {
	ROBOT_CTRL_HEADING_FORWARD, 
	ROBOT_CTRL_HEADING_FORWARD_RIGHT, 
	ROBOT_CTRL_HEADING_RIGHT, 
	ROBOT_CTRL_HEADING_RIGHT_BACKWARD, 
	ROBOT_CTRL_HEADING_BACKWARD,
	ROBOT_CTRL_HEADING_BACKWARD_LEFT,
	ROBOT_CTRL_HEADING_LEFT,
	ROBOT_CTRL_HEADING_LEFT_FORWARD,
	ROBOT_CTRL_HEADING_NOWHERE
};

RobotCtrl robotCtrl;

void RobotTxCtrlCmd()
{
	RobotNewState(robotCtrl.speed_fb, robotCtrl.speed_lr, robotCtrl.speed_rotate, robotCtrl.speed_pan, robotCtrl.speed_tilt, robotCtrl.fire);
	RobotTxCmd();
}

/*********************************  STOP ************************************/
void Stop()
{
	robotCtrl = ROBOT_CTRL_HEADING_NOWHERE;
}

/*********************************  OBSTACLE AVOIDANCE ************************************/
void ObstacleAvoidance()
{
	HeadingSM();
	robotCtrl = ROBOT_CTRL[heading];
}

/*********************************  SCOUT ************************************/
#define SCOUT_DUTY_CYCLE 5000
#define SCOUT_PERIOD 40000
Mytim mytim(SCOUT_PERIOD);
void Scout()
{
	Stop();
	if(mytim.remaining() > SCOUT_PERIOD - SCOUT_DUTY_CYCLE)
	{
		robotCtrl.speed_rotate = 30;
	}
	if(mytim.timeout()) mytim.reset(SCOUT_PERIOD);
	robotCtrl.speed_tilt = 2;
}


/*********************************  TRACKING ************************************/
#define RAMP_LEN 60
Ramp trackingRamp(RAMP_LEN);
double trackingRampVal = 0;
void Tracking()
{
	Stop();
	trackingRampVal = trackingRamp.ramp();
	robotCtrl.speed_rotate = 0.5*scale(trackingRampVal*visionResult.dx, -320, 320,ROBOT_SPEED_MIN, ROBOT_SPEED_MAX);
	robotCtrl.speed_tilt = -0.09*scale(trackingRampVal*visionResult.dy, -240, 240,ROBOT_SPEED_MIN, ROBOT_SPEED_MAX);
	//printf("dy=%d\n,tilt=%f\n", visionResult.dy, robotCtrl.speed_tilt);
	if(visionResult.ds < VISION_FIRE_ON_ACCURACY)
		robotCtrl.fire = 0x07;
}


/*********************************  WORKING STATE MACHINE ************************************/
typedef enum
{
	STOP = 0,
	OBSTACLE_AVOIDANCE = 1,
	SCOUT = 2,
	TRACKING = 3
}WorkingState;
WorkingState workingState = STOP;

void WorkingStateSM()
{
	ScanObstacle();
	switch(workingState)
	{
		case STOP:
		if(!obstacleMask)
		{
			//STOP->TRACKING
			if(!visionResult.lost)
			{
				workingState = TRACKING;
				trackingRamp.reset();
			}
			//STOP->SCOUT
			else
			{
				workingState = SCOUT;
			}
		}
		else
		{
			//STOP->OBSTACLE_AVOIDANCE
			workingState = OBSTACLE_AVOIDANCE;
		}
		break;
		
		case OBSTACLE_AVOIDANCE:
		if(!obstacleMask)
		{
			//OBSTACLE_AVOIDANCE->TRACKING
			if(!visionResult.lost)
			{
				trackingRamp.reset();
				workingState = TRACKING;
			}
			//OBSTACLE_AVOIDANCE->SCOUT
			else
			{
				workingState = SCOUT;
			}
		}
		else
		{
			//OBSTACLE_AVOIDANCE->OBSTACLE_AVOIDANCE
			workingState = OBSTACLE_AVOIDANCE;
		}
		break;
		
		case SCOUT:
		if(!obstacleMask)
		{
			//SCOUT->TRACKING
			if(!visionResult.lost)
			{
				trackingRamp.reset();
				workingState = TRACKING;
			}
			//SCOUT->SCOUT
			else
			{
				workingState = SCOUT;
			}
		}
		else
		{
			//SCOUT->OBSTACLE_AVOIDANCE
			workingState = OBSTACLE_AVOIDANCE;
		}
		break;
		
		case TRACKING:
		if(!obstacleMask)
		{
			//TRACKING->TRACKING
			if(!visionResult.lost)
			{
				workingState = TRACKING;
			}
			//TRACKING->SCOUT
			else
			{
				workingState = SCOUT;
			}
		}
		else
		{
			//TRACKING->OBSTACLE_AVOIDANCE
			workingState = OBSTACLE_AVOIDANCE;
		}
		break;
		
	}
}

/*********************************  ACTION ************************************/
void Action()
{
	switch(workingState)
	{
	case STOP:
	if(DEBUG_ACTION == YES)cout << "STOP"  << endl;
	Stop();
	break;
	
	case OBSTACLE_AVOIDANCE:
	if(DEBUG_ACTION == YES)cout << "OBSTACLE_AVOIDANCE"  << endl;
	ObstacleAvoidance();
	break;
	
	case SCOUT:
	if(DEBUG_ACTION == YES)cout << "SCOUT"  << endl;
	Scout();
	break;
	
	case TRACKING:
	if(DEBUG_ACTION == YES)cout << "TRACKING"  << endl;
	Tracking();
	break;
	
	default:
	if(DEBUG_ACTION == YES)cout << "STOP"  << endl;
	Stop();
	break;
	}
}

int main(int argc, char* argv[])
{
	int appInitError = AppInit() ;
	if(appInitError != APP_INIT_OK) return appInitError;
	MyThread sonarThread;
	MyThread visionThread;
	sonarThread.create(SonarThreadFunction,NULL);
	visionThread.create(VisionThreadFunction,NULL);
	
	//double t = 0;
	
	while(1)
	{
		WorkingStateSM();
		Action();
		RobotTxCtrlCmd();
		//t = cvGetTickCount() - t;
		//cout << "**************************** ROBOT **********************************"<<endl;
		//cout << "TIME COST                                        " << (t/(cvGetTickFrequency()*1000)) << endl;
		//cout << "*********************************************************************"<<endl;
		//printf("dy=%d\n,tilt=%f\n", visionResult.dy, robotCtrl.speed_tilt);
		usleep(2000);
	}
	
	//visionThread.join();
	//grabThread.join();
	return 0;
}
