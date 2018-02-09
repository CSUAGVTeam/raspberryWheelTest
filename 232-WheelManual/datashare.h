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

    int fd;                                 //serial port file identify
    int fd1,fd2,fd3,fd4,fd5;                    //serial port T1-T4
    int i2c_fd1,i2c_fd2,i2c_fd3;
    const int i2c_device1 = 0x26;//三个设备地址
    const int i2c_device2 = 0x21;
    const int i2c_device3 = 0x27;
    const int and_buf[16]={0x0001,0x0002,0x0004,0x0008,0x0010,0x0020,0x0040,0x0080,0x0100,0x0200,0x0400,0x0800,0x1000,0x2000,0x4000,0x8000};
	unsigned int speed_arr[5] = {B115200,B57600 ,B38400, B19200, B9600 };  
	int name_arr[5] = {115200,57600,38400, 19200,  9600 }; 
    char seri_send_buzzer1[7]={0x01,0x51,0x01,0x00,0x1c,0x4d,0x02};//normal
    char seri_send_buzzer2[7]={0x01,0x51,0x02,0x00,0x1c,0x4e,0x02};//alarm
    char seri_send_buzzer3[7]={0x01,0x51,0x03,0x00,0x1c,0x4f,0x02};//bar

    int state_input[2][16];	//I/O口读取外部信息
    int state_output[16];//I/O输出控制
    //void value_input;//将变量输入state_input数组

    unsigned char gainAccess[12]={0xA5, 0x3F, 0x02, 0x07, 0x00, 0x01, 0xB3, 0xE7, 0x0F, 0x00, 0x10, 0x3E};
    unsigned char enableBridgeCommand[12]={0xA5, 0x3F, 0x02, 0x01, 0x00, 0x01, 0x01, 0x47, 0x00, 0x00, 0x00, 0x00};
    unsigned char disableBridgeCommand[12]={0xA5, 0x3F, 0x02, 0x01, 0x00, 0x01, 0x01, 0x47, 0x01, 0x00, 0x33, 0x31};
    unsigned char readCurrentData[8] = {0xA5,0x3F,0x01,0x10,0x03,0x01,0xBB,0x9B};
    unsigned char readSpeedData[8] = {0xA5,0x3F,0x01,0x11,0x02,0x8F,0xF9};
    unsigned char readPositionData[8] = {0xA5,0x3F,0x01,0x12,0x00,0x02,0xB0,0xCB};
//    unsigned char gainAccess1[12]={0xa5,0x01,0x02,0x07,0x00,0x01,0x70,0xa1,0x0f,0x00,0x01,0x3e};
//    unsigned char gainAccess3[12]={0xa5,0x03,0x02,0x07,0x00,0x01,0x34,0x22,0x0f,0x00,0x01,0x3e};

//    char write500RPM[14]={0xa5,0x00,0x02,0x45,0x00,0x02,0x99,0x5e,0x55,0x55,0x03,0x00,0x29,0x13};
//    char write500RPMreverse[14]={0xa5,0x00,0x02,0x45,0x00,0x02,0xf0,0x49,0xab,0xaa,0xfc,0xff,0xc6,0x68};
    char write0RPM[14];
    char commanData[14];
    char commanDataReverse[14];
    float AGVSpeed;
    float getSpeedString;
    float wheelMoveSpeedSet;
    float wheelMoveSpeedSetMax;
    float wheelMoveSpeedReadFront;
    float wheelMoveSpeedReadRear;
    float wheelAngle;
    float wheelAngle2=0;
    float wheelAngle4=0;
    float wheelFrontAngle;
    float wheelRearAngle;
    unsigned int accum;
    unsigned int Gr1;
    int wheelAddress;
    int delayTimeSet;
    int wheelFrontAngleOffset;
    int wheelRearAngleOffset;

    bool initialReady = false;                                          //check whether the system has iniitaled,true=ready,flase=not ready
    bool breakFlag = true;                                              //judge MainWindow::systemOn() loop whether to stop
    bool calibrationFlag = false;

    bool wheelCommunicationErrorFlag = false;
    bool emergencyFlag = false;
    bool sickWarningSpaceAlert = false;
    bool sickFalse = false;
    bool steerFrontLimitDetect = false;
    bool steerFrontLimitDetect2 = false;
    bool steerBackLimitDetect = false;
    bool steerBackLimitDetect2 = false;
    bool steerLimitDetectFlag = false;
    bool chargeContectorConnect, reserve,conveyorDriveOKFlag, liftSwitchFlag;
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


     unsigned char accessData[12];
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
	
    int openSerial (const char *device, const char *device2, const char *device3, const char *device4, const char *device5, const char *device6, const int baud);

signals:

    void timingbeginSignal();

public slots:

    void timingNow();
};

#endif // DATASHARE_H
