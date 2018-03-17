#ifndef DATASHARE_H
#define DATASHARE_H

#include <QObject>
#include<sys/types.h>  
#include<sys/stat.h>  
#include<fcntl.h>  
#include<termios.h>  

class Datashare : public QObject
{
    Q_OBJECT
public:
    explicit Datashare(QObject *parent = NULL);

    int fd;                                 	//serial port file identify，WiFi串口文件标识符
    int fd1,fd2,fd3,fd4,fd5,fd6;                    //serial port T1-T4，fd1-fd4舵机串口文件标识符，fd5为扬声器、电池485文件标识符  fd6 navigation part
    int i2c_fd1,i2c_fd2,i2c_fd3;				//I2C 文件标识符
    const int i2c_device1 = 0x26;				//I2C三个设备地址
    const int i2c_device2 = 0x21;
    const int i2c_device3 = 0x23;
    const int and_buf[16]={0x0001,0x0002,0x0004,0x0008,0x0010,0x0020,0x0040,0x0080,0x0100,0x0200,0x0400,0x0800,0x1000,0x2000,0x4000,0x8000};//IO数据转换用数组
	unsigned int speed_arr[5] = {B115200,B57600 ,B38400, B19200, B9600 };  //波特率设置数组
	int name_arr[5] = {115200,57600,38400, 19200,  9600 }; 					//波特率设置数组
    char seri_send_buzzer1[7]={0x01,0x51,0x01,0x00,0x1c,0x4d,0x02};			//normal	扬声器正常报文
    char seri_send_buzzer2[7]={0x01,0x51,0x02,0x00,0x1c,0x4e,0x02};			//alarm		扬声器报警报文
    char seri_send_buzzer3[7]={0x01,0x51,0x03,0x00,0x1c,0x4f,0x02};			//bar		扬声器警告报文

    int state_input[2][16];													//I/O口读取外部信息
    int state_output[16];													//I/O输出控制

    unsigned char gainAccess[12]={0xA5, 0x3F, 0x02, 0x07, 0x00, 0x01, 0xB3, 0xE7, 0x0F, 0x00, 0x10, 0x3E};						//获取舵机权限报文
    unsigned char enableBridgeCommand[12]={0xA5, 0x3F, 0x02, 0x01, 0x00, 0x01, 0x01, 0x47, 0x00, 0x00, 0x00, 0x00};				//使能舵机报文
    unsigned char disableBridgeCommand[12]={0xA5, 0x3F, 0x02, 0x01, 0x00, 0x01, 0x01, 0x47, 0x01, 0x00, 0x33, 0x31};			//断连舵机报文
    unsigned char readCurrentData[8] = {0xA5,0x3F,0x01,0x10,0x03,0x01,0xBB,0x9B};												//读取电流报文
    unsigned char readSpeedData[8] = {0xA5,0x3F,0x01,0x11,0x02,0x8F,0xF9};														//读取速度报文
    unsigned char readPositionData[8] = {0xA5,0x3F,0x01,0x12,0x00,0x02,0xB0,0xCB};												//读取角度报文

    unsigned char readInertialBuff[8]={0x04,0x03, 0x00, 0x04, 0x00, 0x04, 0x05, 0x9d};          //telegram for reading the navigation part

    char write0RPM[14];								//舵机停车，0速报文存放地址																							
    char commanData[14];							//舵机写速度报文存放数组
    char commanDataReverse[14];						//舵机反响写速度报文存放数组
    float AGVSpeed;									//AGV速度
    float getSpeedString;							//放弃不用
    float wheelMoveSpeedSet;						//舵机设定速度
    float wheelMoveSpeedSetMax;						//舵机设定速度上限
    float wheelMoveSpeedReadFront;					//舵机前轮速度（读取值）
    float wheelMoveSpeedReadRear;					//舵机后轮速度（读取值）
    float wheelAngle;								//舵机角度
    float wheelAngle2=0;								
    float wheelAngle4=0;
    float wheelFrontAngle;			
    float wheelRearAngle;
	int wheelFrontAngleOffset;						//舵机前轮偏移量
    int wheelRearAngleOffset;						//舵机后轮偏移量
    unsigned int accum;								//CRC校验
    unsigned int Gr1;								//CRC校验
    int wheelAddress;								//舵机地址选择
    int delayTimeSet;								//延时数据
    
    bool initialReady = false;                                          //check whether the system has iniitaled,true=ready,flase=not ready 初始化完成标志
    bool breakFlag = true;                                              //judge MainWindow::systemOn() loop whether to stop					手自切换标志
    bool calibrationFlag = false;					//舵机校零完成标志

    bool wheelCommunicationErrorFlag = false;		//舵机通讯错误标志
    bool emergencyFlag = false;						//紧急情况标志（目前未使用）
    bool sickWarningSpaceAlert = false;				//sick警报区1标志
    bool sickFalse = false;							//sick警报区2标志
    bool steerFrontLimitDetect = false;				//前轮左极限
    bool steerFrontLimitDetect2 = false;			//前轮右极限
    bool steerBackLimitDetect = false;				//后轮右极限
    bool steerBackLimitDetect2 = false;				//后轮左极限
    bool steerLimitDetectFlag = false;				//舵轮极限标志位（放弃）
    bool chargeContectorConnect, reserve,conveyorDriveOKFlag, liftSwitchFlag;						//bool型变量为IO口标志位，变量表中详细讲述
    bool powerOn, chargeConnectReady, chargeOn, chargeComplete, chargerFalse, liftFrontUpDetect, liftFrontDownDetect,
        liftBacktUpDetect, liftBackDownDetect, conveyorFrontArriveDetect, convryorFrontInterfaceDetect, conveyorBackArriveDetect,
        conveyorBackInterfaceDetect, steerFrontZeroPositionDetect, steerBackZeroPositionDetect, wheelMoveFrontFalseFlag,
        wheelSteerFrontFalseFlag, wheelMoveBackFalseFlag, wheelSteerBackFalseFlag;

    bool reserve1,reserve2,reserve3,chargeContactorPickup,wheelMoveFrontBrake,wheelMoveBackBrake,conveyorForward,
        conveyorBack,conveyorBrake,liftFrontOn,liftBackOn,systemOnLight,alarmLight,warmingLight,batteryChargeCircuitOn,chargeStart,batteryChargeComplete;

    bool sickA = 1;
    bool sickB = 1;
    bool sickC = 1;
    bool systemOnFlag = true;

    float KP=0;                                        //PID coefficient
    float KI = 0;
    float KD = 0;

    float yaw = 0;
	float yawLast = 0;
    float yawTarget = 0;
	bool yawFlag = false;

     unsigned char accessData[12];						//报文存放数组
     unsigned char enableData[12];
     unsigned char disableData[12];
     unsigned char writeCurrentData[14];
     //unsigned char readCurrentData[8];
     unsigned char writeSpeedData[14];
     //unsigned char readSpeedData[8];
     unsigned char writePositionData[14];
     unsigned char writePositionFrontData[14];
     unsigned char writePositionBackData[14];
     //unsigned char readPositionData[8];
     QString testStr;

public:
    void gainAccessAndEnableWheel(void);

    void writeWheelSpeed(float speedREV, int inputArea, char commandData[14]);

    void writeAccessToDrive(int address);

    void enableBridge(int address);

    void disableBridge(int address);

    void writeWheelCurrent(int inputArea, float currentAMPS);

    void readWheelCurrent(int address);

    void writeWheelSpeed(int inputArea, float speedREV);

    void readWheelSpeed(int address);

    float convertTelegramHex2Speed(unsigned char array[]);

    void writeWheelPosition(int inputArea, float positonANGLE);

    void readWheelPositon(int address);

    float convertTelegramHex2Angle(unsigned char array[]);

    void crunchCRC(char x);

    void delayTimeMsecs(int msecs);

    QString checkWheelCommunication(int filedestiny);

    void writeWheelSpeed(float speedREV);

    void sendSignal(void);

    void bufTOvariable();

    void variableTobuf();

    int i2c_trans_in(int i2c_v,int num);                        //将所要的数据位移到最右边，i2c_v是所需要转换的数据

    int i2c_trans_out1(int *p);                                 //低位

    int i2c_trans_out2(int *p);                                  //高位

    void readIO(void);

    void checkIO(void);

    void writeIO(void);
	
	void set_speed(int fd, int speed);
	
	int set_Parity(int fd,int databits,int stopbits,int parity);
	
    int openSerial (const char *device, const char *device2, const char *device3, const char *device4, const char *device5, const char *device6, const char *device7, const int baud);

    int Incremental_PI (int Encoder,int Target);

    int Position_PID (int Encoder,int Target);

    float angle_trans(unsigned char low, unsigned char high);

signals:

    void timingbeginSignal();

public slots:

    void timingNow();
};

#endif // DATASHARE_H
