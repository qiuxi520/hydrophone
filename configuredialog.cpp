#include "configuredialog.h"

#define MAXCLOCKRATE 500000000

ConfigureDialog::ConfigureDialog(QDialog *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
    //设置最小化和关闭按钮
	this->setWindowFlags(Qt::WindowFlags(Qt::WindowSystemMenuHint | Qt::WindowTitleHint | Qt::WindowCloseButtonHint));

	connect(ui.cmbDevice, SIGNAL(currentIndexChanged(int)), this, SLOT(DeviceChanged(int)));
	connect(ui.btnOK, SIGNAL(clicked()), this, SLOT(ButtonOKClicked()));
	connect(ui.btnCancel, SIGNAL(clicked()), this, SLOT(ButtonCancelClicked()));
	connect(ui.btnBrowse, SIGNAL(clicked()), this, SLOT(ButtonBrowseClicked()));

    //设置每个通道的最大时钟频率
	ui.edtClockRatePerChan->setValidator(new QDoubleValidator(1, MAXCLOCKRATE, 2, this));

	Initailization();
}

ConfigureDialog::~ConfigureDialog()
{

}

void ConfigureDialog::Initailization()
{
   WaveformAiCtrl * waveformAiCtrl = WaveformAiCtrl::Create();
   BufferedAoCtrl * bfdAoCtrl = BufferedAoCtrl::Create();

    //supportedDevice是存放可用设备的数组
	Array<DeviceTreeNode> *supportedDevice = waveformAiCtrl->getSupportedDevices();

    if (supportedDevice->getCount() == 0)//getCount()返回supportedDevice的长度，没有可用设备直接退出程序
	{
        DEVICE_CONNECT=0;
	}
	else
	{
        DEVICE_CONNECT=1;
        ui.cmbDevice->blockSignals(true);
        for (int i = 0; i < supportedDevice->getCount(); i++)//遍历每一个可用设备，并列在选择框中
		{
			DeviceTreeNode const &node = supportedDevice->getItem(i);
			qDebug("%d, %ls\n", node.DeviceNumber, node.Description);
            QString description = QString::fromWCharArray(node.Description);
			ui.cmbDevice->addItem(description);
		}		
         ui.cmbDevice->blockSignals(false);
         ui.cmbDevice->setCurrentIndex(0);//默认为第一个设备
         DeviceChanged(0);
	}
	configure.profilePath = L"";

    //初始化完毕释放对象
    bfdAoCtrl->Dispose();
	waveformAiCtrl->Dispose();
	supportedDevice->Dispose();
}

//提示错误信息
void ConfigureDialog::CheckError(ErrorCode errorCode)
{
	if (errorCode >= 0xE0000000 && errorCode != Success)
	{
        QString message = tr("!!Sorry, there are some errors occurred, Error Code: 0x") +
			QString::number(errorCode, 16).right(8).toUpper();
		QMessageBox::information(this, "Warning Information", message);
	}
}

//******************按钮槽函数******************
void ConfigureDialog::DeviceChanged(int index)
{
	ui.cmbChannelCount->clear();
	ui.cmbChannelStart->clear();
	ui.cmbValueRange->clear();

    std::wstring description = ui.cmbDevice->currentText().toStdWString();//获取 QComboBox 当前选中的文本，并将其转换为 std::wstring
    DeviceInformation selected(description.c_str());

   //==============================配置AI==============================
    WaveformAiCtrl * waveformAiCtrl = WaveformAiCtrl::Create();
    ErrorCode errorCode = waveformAiCtrl->setSelectedDevice(selected);
	ui.btnOK->setEnabled(true);
    if (errorCode != 0)
    {
		QString str;
        QString des = QString::fromStdWString(description);
		str.sprintf("Error:the error code is 0x%x\n\
                   The %s is busy or not exit in computer now.\n\
                   Select other device please!", errorCode, des.toUtf8().data());
		QMessageBox::information(this, "Warning Information", str);
		ui.btnOK->setEnabled(false);
		return;
	}

    //****************设置cmbChannelCount，通道数最大为16****************
    int channelCount = (waveformAiCtrl->getChannelCount() < 16) ? waveformAiCtrl->getChannelCount() : 16;
	int logicChannelCount = waveformAiCtrl->getChannelCount();

    for (int i = 0; i < logicChannelCount; i++)//从哪个通道开始
	{
		ui.cmbChannelStart->addItem(QString("%1").arg(i));
	}
    for (int i = 0; i < channelCount; i++)//读取几个通道
	{
		ui.cmbChannelCount->addItem(QString("%1").arg(i + 1));
	}

    //****************设置cmbValueRange****************
    Array<ValueRange> * ValueRanges = waveformAiCtrl->getFeatures()->getValueRanges();
	wchar_t			vrgDescription[128];
    MathInterval	ranges;//限幅
    ValueUnit		valueUnit;//单位
	for(int i = 0; i < ValueRanges->getCount(); i++)
	{
		errorCode = AdxGetValueRangeInformation(ValueRanges->getItem(i), sizeof(vrgDescription), vrgDescription, &ranges, &valueUnit);
        CheckError(errorCode);

		//we filter the Celsius degree for the buffered AI can not support this function.
		if (valueUnit == CelsiusUnit)
		{
			continue;
		}

		QString str = QString::fromWCharArray(vrgDescription);
		ui.cmbValueRange->addItem(str);
	}
    waveformAiCtrl->Dispose();
    //************************************************

    //开始采样通道、通道数量、量程初始值
    ui.cmbChannelStart->setCurrentIndex(0);
    ui.cmbChannelCount->setCurrentIndex(5);
    ui.cmbValueRange->setCurrentIndex(4);

}
void ConfigureDialog::ButtonOKClicked()
{
	if (ui.cmbDevice->count() == 0)
	{
//		QCoreApplication::quit();

	}

    if(DEVICE_CONNECT==1)
    {
        //设置输入的时钟频率
        double clockRate = ui.edtClockRatePerChan->text().toDouble();
        if (clockRate < 1 || clockRate > MAXCLOCKRATE)
        {
            QMessageBox::information(this, tr("Warning Information"), tr("Sorry, the clock rate per channel is invalid"));
            return;
        }

        std::wstring description = ui.cmbDevice->currentText().toStdWString();
        DeviceInformation selected(description.c_str());

        WaveformAiCtrl * waveformAiCtrl = WaveformAiCtrl::Create();
        ErrorCode errorCode = waveformAiCtrl->setSelectedDevice(selected);
        CheckError(errorCode);

        Array<ValueRange> * ValueRanges = waveformAiCtrl->getFeatures()->getValueRanges();
        configure.deviceName = ui.cmbDevice->currentText();
        configure.channelCount = ui.cmbChannelCount->currentText().toInt();
        configure.channelStart = ui.cmbChannelStart->currentText().toInt();
        configure.valueRange = ValueRanges->getItem(ui.cmbValueRange->currentIndex());
        configure.clockRatePerChan = ui.edtClockRatePerChan->text().toDouble();
        configure.sectionLength = ui.edtSectionLength->text().toInt();

       waveformAiCtrl->Dispose();

       this->accept();
    }
    else
    {
        this->accept();
    }
}
void ConfigureDialog::ButtonCancelClicked()
{
	this->reject();
}
void ConfigureDialog::ButtonBrowseClicked()
{
	QString str = QFileDialog::getOpenFileName(this, tr("Open Profile"), "../../profile", tr("Image Files(*.xml)")); 
	ui.txtProfilePath->setText(str);
    configure.profilePath = str.toStdWString().c_str();
}


void ConfigureDialog::RefreshConfigureParameter()
{
    //让 selected 存储当前选中的设备信息，以便后续使用
    std::wstring description = ui.cmbDevice->currentText().toStdWString();
    DeviceInformation selected(description.c_str());

    //将selected设置为AI对象
    WaveformAiCtrl * waveformAiCtrl = WaveformAiCtrl::Create();
    ErrorCode errorCode = waveformAiCtrl->setSelectedDevice(selected);
    CheckError(errorCode);

    ui.edtClockRatePerChan->setText(QString::number(waveformAiCtrl->getConversion()->getClockRate(), 'f',0));
    ui.edtSectionLength->setText(QString::number(waveformAiCtrl->getRecord()->getSectionLength(), 'f', 0));

    waveformAiCtrl->Dispose();
}
