#include "streamingbufferedai.h"

AI_StreamingBufferedAi::AI_StreamingBufferedAi(QDialog *parent, Qt::WindowFlags flags)
    : QDialog(parent, flags | Qt::WindowMinMaxButtonsHint)  // 添加最小化最大化按钮提示
{
    ui.setupUi(this);

    Init();

    //AI相关
    rawDataBufferLength = 0;
    scaledData = NULL;
    waveformAiCtrl  = WaveformAiCtrl::Create();
    waveformAiCtrl->addDataReadyHandler(OnDataReadyEvent, this);//绑定回调函数

    // udp报文接收
    connect(UDP, &QUdpSocket::readyRead, this, &AI_StreamingBufferedAi::on_readyReadData);


    connect(this, &AI_StreamingBufferedAi::dataReady,this, &AI_StreamingBufferedAi::onDataReceived);

    setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX); // 允许窗口扩大到最大
    setWindowFlags(windowFlags() | Qt::WindowTitleHint | Qt::WindowSystemMenuHint | Qt::WindowMinimizeButtonHint | Qt::WindowMaximizeButtonHint);



    MovingAverage_Init(&ma);
    MovingAverage_Init(&mb);

}
AI_StreamingBufferedAi::~AI_StreamingBufferedAi()
{
   if (waveformAiCtrl != NULL)
   {
      waveformAiCtrl->Dispose();
      waveformAiCtrl = NULL;
   }

    if (scaledData != NULL)
    {
        delete []scaledData;
        scaledData = NULL;
    }
}


//=================初始化=================
void AI_StreamingBufferedAi::Init()
{
    ui.btn_startSave->setEnabled(false);
    ui.btn_stopSave->setEnabled(false);

    //设置按钮和背景
    this->setWindowFlags(Qt::WindowFlags(Qt::WindowSystemMenuHint | Qt::WindowTitleHint | Qt::WindowCloseButtonHint));
    this->setAutoFillBackground(true);
    QPixmap pixMap(":/AI_StreamingBufferedAi/Resources/bg2.jpg");
    QPalette backPalette;
    backPalette.setBrush(this->backgroundRole(), QBrush(pixMap));
    this->setPalette(backPalette);

    closeState=new QPixmap(":/AI_StreamingBufferedAi/Resources/close.png");
    openState=new QPixmap(":/AI_StreamingBufferedAi/Resources/open.png");

    // 绑定本机 IP (监听端口)
    Local_hostAddr.setAddress("192.168.100.233");
    Local_port = 7000;

    UDP = new QUdpSocket(this);
    UDP->bind(Local_hostAddr, Local_port);

    TimerUdp=new QTimer(this);
    TimerUdp->start(Timer_out);

    //电压幅值spinbox
    ui.dsb_AO0->setRange(-10.00, 10.00);  // 设置最小值和最大值
    ui.dsb_AO0->setSingleStep(0.01);  // 设置步进值
    ui.dsb_AO0->setValue(0.0);  // 设置初始值
    ui.dsb_AO0->setSuffix(" V");  // 后缀

    //电压幅值滑动条
    ui.sld_AO->setRange(-1000, 1000);
    ui.sld_AO->setValue(0);
    ui.sld_AO->setTickInterval(200);  // 刻度间隔
    ui.sld_AO->setTickPosition(QSlider::TicksBelow);  // 刻度显示位置

    //频率滑动条
    ui.sld_f->setRange(10, 100);
    ui.sld_f->setValue(10);
    ui.sld_f->setTickInterval(10);  // 刻度间隔
    ui.sld_f->setTickPosition(QSlider::TicksBelow);  // 刻度显示位置

    //频率spinbox
    ui.sb_f->setRange(10, 100);  // 设置最小值和最大值
    ui.sb_f->setSingleStep(1);  // 设置步进值
    ui.sb_f->setValue(10);  // 设置初始值
    ui.sb_f->setSuffix(" Hz");  // 后缀

    for(int i=0;i<DOcnt;i++)
        STM32_DO_WRITE[i]=0;
    for(int i=0;i<AOcnt;i++)
        STM32_AO_WRITE[i]=int(trans(-10.0,10.0,0,3700,0));
//        STM32_AO_WRITE[i]=trans(-10.0,12.0,0,4095,0);

    //连接槽函数
    connect(TimerUdp, SIGNAL(timeout()), this, SLOT(Timerfunction()));
    connect(ui.btnStart, SIGNAL(clicked()), this, SLOT(ButtonStartClicked()));
    connect(ui.btnStop, SIGNAL(clicked()), this, SLOT(ButtonStopClicked()));

    // 创建 data 目录，文件默认命名为时间戳
    dirPath = QDir::currentPath() + "/../../hydrophone/data";
    QDir dir;
    if (!dir.exists(dirPath))
        dir.mkpath(dirPath);  // 递归创建目录
    // 生成唯一的 CSV 文件路径（只生成一次）
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    csvFilePath = dirPath + QString("/data_%1.csv").arg(timestamp);

    PlotInit();
}
void AI_StreamingBufferedAi::Initialize()
{
    setWindowTitle(tr("输出采集一体化上位机软件"));//设置窗口标题栏
    ConfigureDevice();

    if(DEVICE_CONNECT==1)
    {
        m_waveSeled[0]=true;

        ui.btnStart->setEnabled(true);
        ui.btnStop->setEnabled(false);
        ui.btn_setName->setEnabled(true);
        ui.led_fileName->setText("如不指定名称，则使用默认的时间戳对文件进行命名");
    }
    else
    {
        ui.btnStart->setEnabled(false);
        ui.btnStop->setEnabled(false);
        ui.btn_setName->setEnabled(false);
        ui.led_fileName->setText("采集卡未连接，无法采集数据,请连接采集卡后重启软件");
    }

}
void AI_StreamingBufferedAi::ConfigureDevice()//IO配置
{

    if(DEVICE_CONNECT==1)
    {
        ErrorCode errorCode = Success;
        if (scaledData != NULL)
        {
            delete []scaledData;
            scaledData = NULL;
        }

        //原始数据缓存区总长度=单通道缓存长度*通道数
        rawDataBufferLength = configure.sectionLength * configure.channelCount;

        //动态分配数据采集缓存数据
        scaledData = new double[rawDataBufferLength];
        if (scaledData == NULL)
        {
            QMessageBox::information(this, tr("Warning Information"),tr("Sorry! Error in allocating memory...."));
            return;
        }

        // Select a device with AccessWrite/AccessWriteWithReset mode with device number or device description.
        std::wstring description = configure.deviceName.toStdWString();
        DeviceInformation selected(description.c_str());

        errorCode = waveformAiCtrl->setSelectedDevice(selected);
        CheckError(errorCode);
        errorCode = waveformAiCtrl->LoadProfile(configure.profilePath);
        CheckError(errorCode);

        //设置流模式
        errorCode = waveformAiCtrl->getConversion()->setChannelCount(configure.channelCount);
        CheckError(errorCode);
        errorCode = waveformAiCtrl->getConversion()->setChannelStart(configure.channelStart);
        CheckError(errorCode);
        errorCode = waveformAiCtrl->getConversion()->setClockRate(configure.clockRatePerChan);
        CheckError(errorCode);
        errorCode = waveformAiCtrl->getRecord()->setSectionLength(configure.sectionLength);
        CheckError(errorCode);
        errorCode = waveformAiCtrl->getRecord()->setSectionCount(sectionCount);//The 0 means setting 'streaming' mode.
        CheckError(errorCode);

        //*******************************Ai配置量程*******************************
        for (int i = 0; i < waveformAiCtrl->getChannels()->getCount(); i++)//为AI0-AI15通道配置量程
        {
            errorCode = waveformAiCtrl->getChannels()->getItem(i).setValueRange(configure.valueRange);
        }

        errorCode = waveformAiCtrl->Prepare();
        CheckError(errorCode);

    }
    else
    {

    }

}

//=================工具函数=================
void AI_StreamingBufferedAi::CheckError(ErrorCode errorCode)
{
    if (BioFailed(errorCode))
    {
        QString message = tr("Sorry, there are some errors occurred, Error Code: 0x") +
        QString::number(errorCode, 16).right(8).toUpper();
        QMessageBox::information(this, "Warning Information", message);
    }
}
double AI_StreamingBufferedAi::trans(double in_min, double in_max, double out_min, double out_max,double input)//值变换函数
{
    return out_min + (out_max-out_min)*(input-in_min)/(in_max-in_min);
}
void AI_StreamingBufferedAi::PrintArr(unsigned char *buf,int Len)//打印数组
{
    QString Str;
    Str="";
    for(int i=0;i<Len;i++)
        Str=Str+" 0x"+QString::number(buf[i],16);
    qDebug()<<Str;

}
void AI_StreamingBufferedAi::PrintArr(QByteArray *buf)
{
    if (!buf || buf->isEmpty()) {
        qDebug() << "数据为空";
        return;
    }

    QString Str;
    for (int i = 0; i < buf->size(); i++)
        Str += QString(" 0x%1").arg(static_cast<uint8_t>(buf->at(i)), 2, 16, QChar('0')).toUpper();

    qDebug() << Str;
}
void AI_StreamingBufferedAi::PrintArr(int *buf,int Len)//打印数组
{
    QString Str;
    Str="";
    for(int i=0;i<Len;i++)
        Str=Str+" "+QString::number(buf[i]);
    qDebug()<<Str;

}
void AI_StreamingBufferedAi::GreenLight(QLabel* label)
{
    label->setPixmap(openState->scaled(25, 25, Qt::KeepAspectRatio, Qt::SmoothTransformation));
}
void AI_StreamingBufferedAi::RedLight(QLabel* label)
{
    label->setPixmap(closeState->scaled(25, 25, Qt::KeepAspectRatio, Qt::SmoothTransformation));
}
void AI_StreamingBufferedAi::MovingAverage_Init(MovingAverage *ma) {
    ma->capacity = 50;
    ma->size = 0;
    ma->head = 0;
    ma->tail = 0;
    ma->sum = 0;
}
void AI_StreamingBufferedAi::MovingAverage_Add(MovingAverage *ma, float value) {
    if (ma->size < ma->capacity) {
        ma->buffer[ma->tail] = value;
        ma->sum += value;
        ma->size++;
    } else {
        ma->sum -= ma->buffer[ma->head];
        ma->buffer[ma->tail] = value;
        ma->sum += value;
        ma->head = (ma->head + 1) % ma->capacity;
    }
    ma->tail = (ma->tail + 1) % ma->capacity;
}
float AI_StreamingBufferedAi::MovingAverage_Get(MovingAverage *ma) {
    if (ma->size == 0) return 0;
    return ma->sum / ma->size;
}

//=================定时器函数=================
void AI_StreamingBufferedAi::Timerfunction()
{
    UDPsend();
    RefreshPlot();
on_btn_paint_clicked();

    MovingAverage_Add(&ma, trans(2420,1740,-3.4,3.34,STM32_AI_READ[1]));
    MovingAverage_Add(&mb, trans(2170,3970,-3.08,3.06,STM32_AI_READ[2]));
    x_bar = MovingAverage_Get(&ma);
    x_bar2 = MovingAverage_Get(&mb);

    if(x_bar2<-2.4 || x_bar2>3.3)
        STATE=1;
    qDebug()<<"x_bar ="<<x_bar
            <<"x_bar2 ="<<x_bar2;


//    if()

}

//=================图像绘制=================
void AI_StreamingBufferedAi::PlotInit()//图像初始化
{
    //--------------图1--------------共有4条线
    ui.widget_Plot0->addGraph();
    ui.widget_Plot0->addGraph();
    ui.widget_Plot0->addGraph();
    ui.widget_Plot0->addGraph();
    ui.widget_Plot0->addGraph();
    ui.widget_Plot0->addGraph();

    ui.widget_Plot0->graph(0)->data().data()->clear();
    ui.widget_Plot0->graph(1)->data().data()->clear();
    ui.widget_Plot0->graph(2)->data().data()->clear();
    ui.widget_Plot0->graph(3)->data().data()->clear();
    ui.widget_Plot0->graph(4)->data().data()->clear();
    ui.widget_Plot0->graph(5)->data().data()->clear();

    //设置坐标轴标签
    ui.widget_Plot0->xAxis->setLabel("时间/s");
    ui.widget_Plot0->yAxis->setLabel("位移/mm");

    //设置坐标轴范围，以便我们可以看到全部数据
    ui.widget_Plot0->xAxis->setRange(-0.02,0.12);
    ui.widget_Plot0->yAxis->setRange(-5,5);

    //棕色
    ui.widget_Plot0->graph(0)->setPen(QPen(QColor(255,0,255)));
    ui.widget_Plot0->graph(1)->setPen(QPen(QColor(255,0,0)));
    ui.widget_Plot0->graph(2)->setPen(QPen(QColor(0,255,0)));
    ui.widget_Plot0->graph(3)->setPen(QPen(QColor(0,0,255)));

    ui.widget_Plot0->graph(4)->setPen(QPen(QColor(0,0,0)));
    ui.widget_Plot0->graph(5)->setPen(QPen(QColor(0,0,0)));


    //--------------图2--------------共有3条线
    ui.widget_Plot1->addGraph();
    ui.widget_Plot1->addGraph();

    ui.widget_Plot1->graph(0)->data().data()->clear();
    ui.widget_Plot1->graph(1)->data().data()->clear();


    //设置坐标轴标签
    ui.widget_Plot1->xAxis->setLabel("时间/s");
    ui.widget_Plot1->yAxis->setLabel("压力/bar");

    //设置坐标轴范围，以便我们可以看到全部数据
    ui.widget_Plot1->xAxis->setRange(0,300);
    ui.widget_Plot1->yAxis->setRange(-10,150);

    ui.widget_Plot1->graph(0)->setPen(QPen(QColor(255,0,0)));
    ui.widget_Plot1->graph(1)->setPen(QPen(QColor(0,255,0)));

    PlotIdx=0;
    //设置图的拖动缩放
    ui.widget_Plot0->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui.widget_Plot1->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

}
void  AI_StreamingBufferedAi::RefreshPlot()//画图函数
{
    //--------------图2--------------共有3条线
    ui.widget_Plot1->graph(0)->addData(PlotIdx*(Timer_out/1000.0),trans(0,4095,0,400,STM32_AI_READ[AI_B5])); //油缸压力传感器1
    ui.widget_Plot1->graph(1)->addData(PlotIdx*(Timer_out/1000.0),trans(0,4095,0,400,STM32_AI_READ[AI_B6])); //油缸压力传感器2

    PlotIdx++;
    ui.widget_Plot1->replot();

}
void AI_StreamingBufferedAi::on_btn_paint_clicked()
{
    ui.widget_Plot0->graph(0)->data().data()->clear();
    ui.widget_Plot0->graph(1)->data().data()->clear();
    ui.widget_Plot0->graph(2)->data().data()->clear();
    ui.widget_Plot0->graph(3)->data().data()->clear();

    //--------------图1--------------共有4条线
    for(int i=0;i<1000;i++)
    {
        ui.widget_Plot0->graph(0)->addData(i/10000.0,AO[i]); //AO

        ui.widget_Plot0->graph(1)->addData(i/10000.0,-X1[i]*2); //基恩士激光位移传感器1
        ui.widget_Plot0->graph(2)->addData(i/10000.0,trans(0,10,-3.4,3.4,X2[i])); //松下激光位移传感器2
        ui.widget_Plot0->graph(3)->addData(i/10000.0,Xsp[i]); //阀芯位移

        ui.widget_Plot0->graph(4)->addData(i/10000.0,3); //阀芯位移
        ui.widget_Plot0->graph(5)->addData(i/10000.0,-3); //阀芯位移

//        qDebug() << "X1[" <<i<<"]"<<X1[i];
    }
    ui.widget_Plot0->replot();
}

//=================UDP通信=================

void AI_StreamingBufferedAi::UDPsend()
{
    //DO    [0]
    char DO_Byte=0;//把DO八个字节凑成一个字节
    for(int j=0;j<8;j++)
        DO_Byte|=STM32_DO_WRITE[j]>0?1<<j:0x00;
    UDP_SEND_BUFFER.append(DO_Byte);


    //AO    [1]~[2]
    for(int i=0;i<AOcnt;i++)
    {
        UDP_SEND_BUFFER.append(STM32_AO_WRITE[i]/256);
        UDP_SEND_BUFFER.append(STM32_AO_WRITE[i]%256);
    }

    //状态量   [3]~[4]
    UDP_SEND_BUFFER.append(STM32_STATE_WRITE[0]);//opflag
    UDP_SEND_BUFFER.append(STM32_STATE_WRITE[1]);//f
    UDP_SEND_BUFFER.append(STATE);//STATE


    //打印发送的报文
//    PrintArr(&UDP_SEND_BUFFER);
    UDP->writeDatagram(UDP_SEND_BUFFER, QHostAddress(targetIP), targetPort);
    UDP_SEND_BUFFER.clear();

}
void AI_StreamingBufferedAi::on_readyReadData()//udp报文接收函数
{
    while (UDP->hasPendingDatagrams())
    {
        UDP_RECV_BUFFER.resize(UDP->pendingDatagramSize());
        QHostAddress senderIP;
        quint16 senderPort;

        // 读取 UDP 数据
        UDP->readDatagram(UDP_RECV_BUFFER.data(), UDP_RECV_BUFFER.size(), &senderIP, &senderPort);
    }
    //打印接收报文
//    PrintArr(&UDP_RECV_BUFFER);
    //拆报文
    Process_udp_Packet();
}
void AI_StreamingBufferedAi::Process_udp_Packet()//拆报文函数
{
    //DO数据    [0]
    for(int i = 0; i < 8; i++)
        STM32_DO_READ[i] = ((quint8)UDP_RECV_BUFFER[0] & (1 << i)) ? 1 : 0;
//    PrintArr(STM32_DO_READ,8);

    //AI数据   五路电压AI    [1]~[8]     |    六路电流AI    [9]~[14]
    for(int i = 1;i<1+AIcnt*2;i+=2)
        STM32_AI_READ[(i-1)/2] = (quint8)UDP_RECV_BUFFER[i]*256+(quint8)UDP_RECV_BUFFER[i+1];
//    PrintArr(STM32_AI_READ,6);

    //AO数据    [15]~[16]
    for(int i = 1+AIcnt*2;i<1+AIcnt*2+AOcnt*2;i+=2)
        STM32_AO_READ[(i-1-AIcnt*2)/2] = (quint8)UDP_RECV_BUFFER[i]*256+(quint8)UDP_RECV_BUFFER[i+1];
//    PrintArr(STM32_AO_READ,1);

//    //STATE数据    [17]~[25]
    STM32_STATE_READ[0]=(quint8)UDP_RECV_BUFFER[17];//OUTPUT_FALG
    STM32_STATE_READ[1]=(quint8)UDP_RECV_BUFFER[18];//f
    STM32_STATE_READ[2]=(quint8)UDP_RECV_BUFFER[19];//STATE
    STM32_STATE_READ[3]=(quint8)UDP_RECV_BUFFER[20];//SUB_STATE
    PrintArr(STM32_STATE_READ,4);


    //更新ui显示
    UIupdate();

}
void AI_StreamingBufferedAi::UIupdate()
{
    ui.led_filePath->setText(csvFilePath);

    // 状态机
    if(STM32_STATE_READ[2]==STOP_STATE) ui.led_State->setText("停机");
    if(STM32_STATE_READ[2]==RESET_STATE) ui.led_State->setText("复位");
    if(STM32_STATE_READ[2]==RUN_STATE) ui.led_State->setText("运行");

    // AO
    if(STM32_STATE_READ[0]==1) ui.led_opflag->setText("开启方波输出");else ui.led_opflag->setText("关闭方波输出");

    ui.led_AOreal->setText(QString::number(trans(0,3700,-10.0,10.0,STM32_AO_READ[0]),'f', 2)+" V");
    ui.led_freal->setText(QString::number(STM32_STATE_READ[1])+" Hz");
//    ui.led_x1->setText(QString::number(-X1[0]*2));
//    ui.led_x2->setText(QString::number(trans(0,10,-3.4,3.4,X2[0])));

    ui.led_x1->setText(QString::number(trans(2420,1740,-3.4,3.34,STM32_AI_READ[1])));
    ui.led_x2->setText(QString::number(trans(2170,3970,-3.08,3.06,STM32_AI_READ[2])));

    // DO
    if(STM32_DO_READ[0]==1) GreenLight(ui.label_DO0); else RedLight(ui.label_DO0);
    if(STM32_DO_READ[1]==1) GreenLight(ui.label_DO1); else RedLight(ui.label_DO1);
    if(STM32_DO_READ[2]==1) GreenLight(ui.label_DO2); else RedLight(ui.label_DO2);
}

//=================数据记录=================
void AI_StreamingBufferedAi::ButtonStartClicked()
{
    ErrorCode errorCode = waveformAiCtrl->Start();
    CheckError(errorCode);

    ui.btnStart->setEnabled(false);
    ui.btnStop->setEnabled(true);
    ui.btn_setName->setEnabled(true);
    ui.btn_startSave->setEnabled(true);
    ui.btn_stopSave->setEnabled(true);
    ui.State->setText("采集启动");
}
void AI_StreamingBufferedAi::ButtonStopClicked()
{
    ErrorCode errorCode = waveformAiCtrl->Stop();
    CheckError(errorCode);

    ui.btnStart->setEnabled(true);
    ui.btnStop->setEnabled(false);
    ui.btn_setName->setEnabled(true);

    ui.btn_startSave->setEnabled(false);
    ui.btn_stopSave->setEnabled(false);
    ui.State->setText("采集停止");
    ui.saveState->setText("保存数据停止");

    csv_time=0;
    SAVE_DATA=0;
}

void AI_StreamingBufferedAi::on_btn_startSave_clicked()
{
    SAVE_DATA=1;
    ui.saveState->setText("保存数据启动");
}
void AI_StreamingBufferedAi::on_btn_stopSave_clicked()
{
    csv_time=0;
    SAVE_DATA=0;
    ui.saveState->setText("保存数据停止");

}
void AI_StreamingBufferedAi::on_btn_setName_clicked()
{
    csvFilePath = dirPath + QString("/%1.csv").arg(ui.led_fileName->text());
}

//=================数据采集+转运=================
//流输入的回调函数，单独一个线程
void AI_StreamingBufferedAi::OnDataReadyEvent(void * sender, BfdAiEventArgs * args, void * userParam)
{
    //    AI_StreamingBufferedAi * uParam = (AI_StreamingBufferedAi *)userParam;
    AI_StreamingBufferedAi *uParam = static_cast<AI_StreamingBufferedAi *>(userParam);

    int32 getDataCount = ((uParam->configure.sectionLength * uParam->configure.channelCount) < args->Count)?(uParam->configure.sectionLength * uParam->configure.channelCount) : args->Count;
    ErrorCode ret = ((WaveformAiCtrl*)sender)->GetData(getDataCount, uParam->scaledData, 0, NULL, NULL, NULL, NULL);
    if (ret != Success && ret != WarningRecordEnd)
    {
        QString message = tr("Sorry, there are some errors occurred, Error Code: 0x") +QString::number(ret, 16).right(8);
        QMessageBox::information(uParam, "Warning Information", message);
        return;
    }


    // 创建 QVector 并复制数据
    QVector<double> dataVector(getDataCount);
    std::copy(uParam->scaledData, uParam->scaledData + getDataCount, dataVector.begin());
    // 发射信号
    emit uParam->dataReady(dataVector);


    //保存
    // 追加数据到唯一的 CSV 文件
    if(SAVE_DATA)
    {
        QFile file(uParam->csvFilePath);
        bool isNewFile = !file.exists();  // 仅在文件首次创建时写入表头
        if (file.open(QIODevice::Append | QIODevice::Text))
        {
            QTextStream out(&file);

            // 如果是新文件，则写入表头
            if (isNewFile)
                out << "Time,Control Signal,Displacement_1,Displacement_2,Spool_Displacement,Pressure_1,Pressure_2,Frequency = "<<fre<<" Hz\n";

            // 遍历数据并写入 CSV
            for (int i = 0; i < getDataCount; i++)
            {
                if(i%6==0)//控制量
                {
                    out << csv_time << "," << uParam->scaledData[i] << ",";
                    csv_time+=1/10000.0;
                }
                if(i%6==1)//激光位移1
                    out << uParam->scaledData[i]*2 << ",";
                if(i%6==2)//激光位移2
                    out << uParam->scaledData[i] << ",";
                if(i%6==3)//阀芯位移
                    out << uParam->scaledData[i] << ",";
                if(i%6==4)//油缸压力1
                    out << uParam->scaledData[i] << ",";
                if(i%6==5)//油缸压力2
                    out << uParam->scaledData[i] << "\n";
            }
            file.close();
        }
        else
            qDebug() << "Failed to open CSV file!";
    }


}
void AI_StreamingBufferedAi::onDataReceived(const QVector<double>& data)
{
//    qDebug() << "data.size() =" <<data.size();
    //ui显示
    for (int i = 0; i < data.size(); i++)
    {
        // 处理接收到的数据
        if(i%6==0)//控制量
            AO[i/6]=data[i];
        if(i%6==1)//激光位移1
            X1[i/6]=data[i];
        if(i%6==2)//激光位移2
            X2[i/6]=data[i];
        if(i%6==3)//阀芯位移
            Xsp[i/6]=data[i];
        if(i%6==4)//油缸压力1
            P1[i/6]=data[i];
        if(i%6==5)//油缸压力2
            P2[i/6]=data[i];

//        qDebug() << "data[" <<i<<"]"<<data[i];
//        if(i==0)
//            qDebug() << "=======================";
    }
}

//=================DO设置=================
void AI_StreamingBufferedAi::on_btn_DO0_clicked()
{
    STM32_DO_WRITE[DO_V1]=!STM32_DO_WRITE[DO_V1];
}
void AI_StreamingBufferedAi::on_btn_DO1_clicked()
{
    STM32_DO_WRITE[DO_V2]=!STM32_DO_WRITE[DO_V2];
}
void AI_StreamingBufferedAi::on_btn_DO2_clicked()
{
    STM32_DO_WRITE[DO_CH2]=!STM32_DO_WRITE[DO_CH2];
}

//=================AO设置=================
void AI_StreamingBufferedAi::on_sld_AO_valueChanged(int value)
{
    ui.dsb_AO0->setValue(value/100.0);
}
void AI_StreamingBufferedAi::on_dsb_AO0_valueChanged(double arg1)
{
//    STM32_AO_WRITE[AO_Q3M]=int(trans(-10.0,10.0,0,4095,arg1));
    STM32_AO_WRITE[AO_Q3M]=int(trans(-10.0,10.0,0,3700,arg1));
}

void AI_StreamingBufferedAi::on_sb_f_valueChanged(int arg1)
{
    STM32_STATE_WRITE[1]=arg1;
    fre=arg1;
}
void AI_StreamingBufferedAi::on_sld_f_valueChanged(int value)
{
    STM32_STATE_WRITE[1]=value;
}

void AI_StreamingBufferedAi::on_btn_opflag_clicked()
{
    STM32_STATE_WRITE[0]=!STM32_STATE_WRITE[0];
}

//=================状态机=================
void AI_StreamingBufferedAi::on_btn_stop_clicked()
{
    STATE=0;
}
void AI_StreamingBufferedAi::on_btn_reset_clicked()
{
    STATE=1;
}
void AI_StreamingBufferedAi::on_btn_run_clicked()
{
    STATE=2;
    on_btn_paint_clicked();
}

