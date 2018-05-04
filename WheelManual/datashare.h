#ifndef DATASHARE_H
#define DATASHARE_H

#include <QObject>
#include<sys/types.h>  
#include<sys/stat.h>  
#include<fcntl.h>  
#include<termios.h>  

#include <QTcpSocket>

#define deltT 0.05                             //time unit


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

    int Angle_QRtoCar=0;                                                    //车身与二维码的夹角
    int QR_Code_Number=-999;                                                //二维码对应的编号
    //double Dis_Cen_Two=0;                                                   //扫码枪与车中心的偏差距离；

    int state_input[2][16];													//I/O口读取外部信息
    int state_output[16];													//I/O输出控制

    unsigned char gainAccess[12]={0xA5, 0x3F, 0x02, 0x07, 0x00, 0x01, 0xB3, 0xE7, 0x0F, 0x00, 0x10, 0x3E};						//获取舵机权限报文
    unsigned char enableBridgeCommand[12]={0xA5, 0x3F, 0x02, 0x01, 0x00, 0x01, 0x01, 0x47, 0x00, 0x00, 0x00, 0x00};				//使能舵机报文
    unsigned char disableBridgeCommand[12]={0xA5, 0x3F, 0x02, 0x01, 0x00, 0x01, 0x01, 0x47, 0x01, 0x00, 0x33, 0x31};			//断连舵机报文
    unsigned char readCurrentData[8] = {0xA5,0x3F,0x01,0x10,0x03,0x01,0xBB,0x9B};												//读取电流报文
    unsigned char readSpeedData[8] = {0xA5,0x3F,0x01,0x11,0x02,0x02,0x8F,0xF9};														//读取速度报文
    unsigned char readPositionData[8] = {0xA5,0x3F,0x01,0x12,0x00,0x02,0xB0,0xCB};												//读取角度报文

    unsigned char readInertialBuff[8]={0x04,0x03, 0x00, 0x04, 0x00, 0x04, 0x05, 0x9d};          //telegram for reading the navigation part

    QString frontTelegram;
    QString backTelegram;
    char write0RPM[14];								//舵机停车，0速报文存放地址
    char commanData[14];							//舵机写速度报文存放数组
    char commanDataReverse[14];						//舵机反响写速度报文存放数组
    double AGVSpeed;							    //AGV速度
    int num=0;                                      //path number
    int numberOfStaEnd = 0;                                 //起点终点的数量
    //double Distance_QR_CarCenter=0.48;              //用不到了
    //QTcpSocket *tcpSocket;                          //二维码TCP/IP通信套接字
    QString buf=NULL;                               //二维码解读信息存储字符串
    QString bufWifi=NULL;                           //wifi信息存储字符串
    double a=0;                                      //直线位置PID输出量
    double b=0;                                      //直线角度PID输出量
    double yaw_error=0;                              //惯导飘移角度
    int Num_Turn = 0;                                //转弯的数量（左加右减）
    int numberOfTurnCentre = 0;                      //圆心的数量
    int countOfTurnCentre = 0;                       //到第几个圆心的计数

    QString buf_last=NULL;                       //上次二维码信息
    double delta_s;                                 //path planning deviation
    double getSpeedString;							//放弃不用
    double wheelMoveSpeedSet;						//舵机设定速度
    double wheelMoveSpeedSetMax;						//舵机设定速度上限
    double wheelMoveSpeedReadFront = 1;					//舵机前轮速度（读取值）
    double wheelMoveSpeedReadRear =1;					//舵机后轮速度（读取值）
    double wheelAngle;								//舵机角度
    double wheelAngle2=0;
    double wheelAngle4=0;
    double wheelFrontAngle;
    double wheelRearAngle;
    double wheelFrontAngleOffset;						//舵机前轮偏移量
    double wheelRearAngleOffset;						//舵机后轮偏移量
    unsigned int accum;								//CRC校验
    unsigned int Gr1;								//CRC校验
    int wheelAddress;								//舵机地址选择
    int delayTimeSet;								//延时数据

    int batteryCollum=0;                            //电池容量
    int batteryVoltage=0;                           //电池电压
    int batteryCurrent=0;                           //电池电流

    bool turn_flag=false;
    bool direction_flag=true;                       //方向标志位（正向为true，反向为false）
    bool direction_flag_last=true;
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

    bool TCPconnectFlag = 0;

    bool sickA = 1;
    bool sickB = 1;
    bool sickC = 1;
    bool systemOnFlag = true;

    bool Flag_Forward = true; //正向标志位
    bool Flag_SpeedDe = false;//减速标志
    bool Flag_SpeedAdd = false;//加速标志
    bool Flag_Stop = false;
    int num_AddSpeed = 29;  //记录出弯点，到出弯后一个点加速

    //**********   目标值设定 ************
        double Td_SpeedSet=0;      //目标速度
        double Delta_Angle1 = 0;  //目标角度和车身角度差
        double Pos_Target = -8;    //目标位置

    //**********   速度TD Parameters ************
        double Speed_r=1;
        double Speed_h=0.03;
        double Speed_Td_x1=0;
        double Speed_Td_x2=0;

    //**********   位置TD Parameters ************
        double Pos_r0 = 1;
        double Pos_h0 = 0.08;    //Pos_h0 和 z1_Position初始值要相等
        double Pos_Td_x1=0;
        double Pos_Td_x2=0;
    //**********   位置ESO Parameters ************
        double beta1 = 2;
        double beta2 = 4;
        double beta3 = 8;
        double z1_Position=0;
        double z2_Position=0;
        double z3_Position=0;
        double Pos_b0=3;     //重点调节

    //**********   位置NLSEF Parameters ************
        double NL_e1 = 0;
        double NL_e2 = 0;
        double NL_c = 0.4;
        double NL_r = 900;
        double NL_h1 = 0.4;
        double Pos_u0 = 0;

    double KP = 10.0;                                        //PID coefficient
    double KI = 0;
    double KD = 1250.0;//1250.0

    double KP_Angle = 1.0;                                        //PID coefficient
    double KI_Angle = 0;
    double KD_Angle = 0.2;


    double KP_turn=60.0;
    double KI_turn=0;
    double KD_turn=30.0;

    double yaw = 0;
    double yawLast = 0;
    double yawTarget = 0;
    double yawInt = 0;
    double yawTarget_Last=0;

	bool yawFlag = false;
    bool QR_Flag = false;

    struct Position
    {
        double X;
        double Y;
    };
    struct PositionStaEnd
    {
        int X;
        int Y;
    };
    struct Vector
    {
        double X;
        double Y;
        double Z;
    };
    struct Speed
    {
        double X;
        double Y;
    };
    struct Curve_Planning
    {
        double X_Start;
        double Y_Start;
        double X_Center;
        double Y_Center;
        double Radius;
    };
     Position AGVLocation={0,0};
     Position P_Centre={0,0};

     Position P_TwoCode[120]={{0,0},{0,1},{0,2},{0,3},{0,4},{0,5},{0,6},{0,7},{0,8},{0,9},
                              {-1,9},{-2,9},{-3,9},{-4,9},{-5,9},{-6,9},{-7,9},{-8,9},{-8,8},{-8,7},
                              {-8,6},{-8,5},{-8,4},{-8,3},{-8,2},{-8,1},{-8,0},{-7,0},{-6,0},{-5,0},
                              {-4,0},{-3,0},{-2,0},{-1,0},{-1,4},{-2,4},{-7.951,7},{-8.049,7},{-6,0.049},{-3,4},
                              {-6,-0.049},{-7,4},{-6,4},{-5,4},{-2,3.951},{-4,4},{-2,8.951},{-2,9.049},{-2,4.049},{-6,4.049},
                              {-0.049,7},{0.049,7},{-0.098,7},{0.098,7},{-2,0.049},{-2,-0.049},{-6,3.951},{-7.951,6},{-7.951,2},{-8.049,2},
                              {-8.049,6},{0.049,6},{-6,8.951},{-0.049,2},{-6,9.049},{0.049,2},{-6,8.951},{-6,8.902},{-6,9.049},{-0.049,6}};//二维码坐标位置
     /*
          Position P_Target[30] = { {0,0},{0,1},{0,2},{0,3},{0,4},{0,5},{0,6},{0,7},
                                    {-2,9},{-3,9},{-4,9},{-5,9},{-6,9},
                                    {-8,7},{-8,6},{-8,5},{-8,4},{-8,3},{-8,2},
                                    {-6,0},{-5,0},{-4,0},{-3,0},{-2,0},{0,2}};//路径坐标
     */
     //{2,4},{3,4},{4,4},
     Position P_Target[30] = { {0,0},{0,1},{0,2},
                               {-2,4},{-3,4},{-4,4},{-5,4},{-6,4},
                               {-8,6},{-8,7},{-6,9},{-5,9},{-4,9} };//路径坐标
     /*
         Position P_Target[30] = { {0,0},{0,1},{0,2},
                                   {2,4},{3,4},{4,4} };  //test
     */
        // Position P_Target3[100]={{0,7},{-6,9},{-8,2},{-2,0}};//弯道坐标(正向入弯点)
         // Position P_Target4[100]={{-2,9},{-8,7},{-6,0},{0,2}};//弯道坐标（正向出弯点）

     Position P_Target3[100]={{0,2},{-6,4},{-8,7}};//弯道坐标(正向入弯点) test
     Position P_Target4[100]={{-6,9},{-8,6},{-2,4}};//弯道坐标（正向出弯点）
/*
     Position P_Target2[100]={{0,0},{0,1},{0,2},{0,3},{0,4},{0,5},{0,6},{0,7},{0,8},{0,9},
                              {0,10},{0,11},{0,12},{0,13},{0,14},{0,15},{0,16},{0,17},{0,18},{0,19},
                              {0,20},{0,21},{0,22},{0,23},{0,24},{0,25},{0,26},{0,27},{0,28},{0,29},
                              {0,30},{0,31},{0,32},{0,33},{0,34},{0,35},{0,36},{0,37},{0,38},{0,39}};//路径坐标
 */
     Position P_Target2[100]={{0,0}};
     Position P_Stop = {0,5}; //停车点
     PositionStaEnd start_end[100] = {{0,0}};               //从TCP/IP信息中解出的首末点存放数组
     PositionStaEnd turnCenter[100]={{0,0}};
     //Position P_Target3[100]={{0,1000}};//弯道坐标
     Curve_Planning P_Curve[100]={{0,1,-2,3,2}};//********************************
     Position Image_Center={800,600};                                            //图像中心点像素坐标
     Position QR_Point[5]={{0,0},{0,0},{0,0},{0,0},{0,0}};                   //二维码5个点在图像上的位置
     Speed AGVSpeeds={0,0};
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

    double Go2 (Position P_Target );

    double Information_Corrective();

    bool Two_bar_codes_Pro2(QString information);

    bool Two_bar_codes_Pro(QString information);

    double Position_PID2 (double delta,bool flag);

    double Straight_Line (Position P_Now,Position P_Start,Position P_Target);

    double Go (Position P_Target );

    double angle_tran (double Yaw_Target,double Yaw);

    double Position_Turn2 (Position P_Now,Position P_Target,double Yaw_Target);

    double Position_Turn_crol (Position P_Centre,Position P_Target,Position P_Now,double Radius_turn_sq);

    void gainAccessAndEnableWheel(void);

    void writeWheelSpeed(double speedREV, int inputArea, char commandData[14]);

    void writeAccessToDrive(int address);

    void enableBridge(int address);

    void disableBridge(int address);

    void writeWheelCurrent(int inputArea, double currentAMPS);

    void readWheelCurrent(int address);

    void writeWheelSpeed(int inputArea, double speedREV);

    void readWheelSpeed(int address);

    double convertTelegramHex2Speed(unsigned char array[]);

    void writeWheelPosition(int inputArea, double positonANGLE);

    void readWheelPositon(int address);

    double convertTelegramHex2Angle(unsigned char array[]);

    void crunchCRC(char x);

    void delayTimeMsecs(int msecs);

    QString checkWheelCommunication(int filedestiny);

    void writeWheelSpeed(double speedREV);

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

    double Position_PID (double Encoder,double Target);

    double angle_trans(unsigned char low, unsigned char high);
    //TD函数
    float fhan(float x1,float x2,float r,float h);
    //位置ESO函数
    void ESO(float u,float b,float output,float h);
    //速度设定
    void Speed_Adj(void);

    int Trace(int x1,int y1,int x2,int y2, int init );  //自动生成路径坐标函数
signals:

    void timingbeginSignal();

public slots:

    void timingNow();
};

#endif // DATASHARE_H