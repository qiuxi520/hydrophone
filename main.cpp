#include "streamingbufferedai.h"
#include "configuredialog.h"
#include <QtWidgets/QApplication>


int main(int argc, char *argv[])
{
	QApplication a(argc, argv);

    qRegisterMetaType<QVector<double>>("QVector<double>");

	AI_StreamingBufferedAi w;
	ConfigureDialog dialog;
	
	int resultDialog = dialog.exec();
	if (resultDialog == QDialog::Rejected)
	{
		a.exit(0);
		return 0;
	}
	else if (resultDialog == QDialog::Accepted)
	{
        //配置参数

        w.SetConfigureParameter(dialog.GetConfigureParameter());
        w.SetConfigureDialog(&dialog);

        w.recv_connnect_flag(dialog.connnect_flag());


        //
		w.Initialize();
		w.show();
	}
	return a.exec();
}


