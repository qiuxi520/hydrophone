#ifndef STREAMINGBUFFEREDAI_H
#define STREAMINGBUFFEREDAI_H

#include "ui_streamingbufferedai.h"
#include "configuredialog.h"

#include <QDebug>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QDir>
#include <QPalette>
#include <QTimer>
#include <QUdpSocket>
#include <QHostAddress>

#define DOcnt 8
#define AIcnt 7
#define AOcnt 1
#define STATEcnt 4

#define DO_V1 0
#define DO_V2 1
#define DO_CH2 2
#define DO_CH3 3
#define DO_CH4 4
#define DO_CH5 5
#define DO_CH6 6
#define DO_CH7 7

#define DI_CH0 0
#define DI_CH1 1
#define DI_CH2 2
#define DI_CH3 3


#define AI_B0 0
#define AI_B1 1
#define AI_B2 2
#define AI_B3 3
//#define AI_CH4 4

#define AI_B5 4
#define AI_B6 5
#define AI_B7 6

#define AO_Q3M 0

#define STOP_STATE 0
#define RESET_STATE 1
#define RUN_STATE 2

static double csv_time;
static int fre;
static int SAVE_DATA=0;

typedef struct {
    float buffer[50];
    int capacity;
    int size;
    int head;
    int tail;
    float sum;
} MovingAverage;

class AI_StreamingBufferedAi : public QDialog
{
    Q_OBJECT
public:
    Ui::AI_StreamingBufferedAiClass ui;

    QPixmap *closeState;
    QPixmap *openState;

    //创建ui更新定时器
    QTimer* UIupdateTimer;
    //创建udp发送定时器
    QTimer *TimerUdp;

    QUdpSocket      *UDP;
    QHostAddress    Local_hostAddr;     //本机的地址对象
    quint16         Local_port;         //本机的端口号

    QString targetIP = "192.168.137.201";  // STM32 IP（远程设备）
    quint16 targetPort = 7000;  // 目标端口

    int STM32_DO_WRITE[DOcnt]={0};
    int STM32_AO_WRITE[AOcnt]={0};
    int STM32_STATE_WRITE[STATEcnt]={0};

    int STM32_DO_READ[DOcnt]={0};
    int STM32_AO_READ[AOcnt]={0};
    int STM32_AI_READ[AIcnt]={0};
    int STM32_STATE_READ[STATEcnt]={0};

    QByteArray UDP_SEND_BUFFER;
    QByteArray UDP_RECV_BUFFER;

    //采集卡数据转运缓存
    double AO[1000]={0};
    double X1[1000]={0};
    double X2[1000]={0};
    double Xsp[1000]={0};
    double P1[1000]={0};
    double P2[1000]={0};

    int PlotIdx=0;
    //每 Timer_out ms调用定时器，系统时钟，与发送报文频率、图像分辨率、延时有关
    int Timer_out=100;

    //是否连接采集卡
    int DEVICE_CONNECT=0;
    void recv_connnect_flag(int flag){DEVICE_CONNECT=flag;}

    //AI
    WaveformAiCtrl * waveformAiCtrl;
    double *scaledData;
    static const int sectionCount = 0;
    int rawDataBufferLength;
    bool m_waveSeled[2];

    // 创建 data 目录
    QString dirPath;
    QString csvFilePath;

    ConfigureDialog *configureDialog;
    ConfigureParameter configure;

    int STATE=0;
    int subState=0;

    MovingAverage ma,mb;
    double x_bar = 0;
    double x_bar2 = 0;
    AI_StreamingBufferedAi(QDialog *parent = 0, Qt::WindowFlags flags = 0);
    ~AI_StreamingBufferedAi();

    //参数配置相关，main.cpp中调用
    void SetConfigureDialog(ConfigureDialog * dialog){configureDialog = dialog;}
    void SetConfigureParameter(ConfigureParameter value){configure = value;}

    void Init();
    void Initialize();
    void ConfigureDevice();

    double trans(double in_min, double in_max, double out_min, double out_max, double input);
    void PrintArr(unsigned char *buf, int Len);
    void PrintArr(QByteArray *buf);
    void PrintArr(int *buf, int Len);
    void GreenLight(QLabel *label);
    void RedLight(QLabel *label);
    void CheckError(ErrorCode errorCode);

    //回调函数
    static void BDAQCALL OnDataReadyEvent(void * sender, BfdAiEventArgs * args, void * userParam);

    void PlotInit();
    void RefreshPlot();

    void UIupdate();
    void UDPsend();
    void Process_udp_Packet();

    void MovingAverage_Init(MovingAverage *ma);
    void MovingAverage_Add(MovingAverage *ma, float value);
    float MovingAverage_Get(MovingAverage *ma);
private slots:
    void Timerfunction();
    void onDataReceived(const QVector<double> &data);

    void on_readyReadData();

    void on_btn_DO0_clicked();
    void on_btn_DO1_clicked();
    void on_btn_DO2_clicked();

    void on_btn_setName_clicked();
    void on_sld_AO_valueChanged(int value);
    void on_dsb_AO0_valueChanged(double arg1);
    void on_btn_opflag_clicked();
    void on_sb_f_valueChanged(int arg1);
    void on_sld_f_valueChanged(int value);
    void ButtonStartClicked();
    void ButtonStopClicked();

    void on_btn_stop_clicked();
    void on_btn_reset_clicked();
    void on_btn_run_clicked();

    void on_btn_paint_clicked();
    void on_btn_startSave_clicked();
    void on_btn_stopSave_clicked();


signals:
    void dataReady(const QVector<double>& data);
};

#endif // STREAMINGBUFFEREDAI_H
