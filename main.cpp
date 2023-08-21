#include "DataFusion.h"
#include <QtWidgets/QApplication>

int main(int argc, char* argv[])
{
	vtkObject::GlobalWarningDisplayOff();
	QApplication a(argc, argv);
	DataFusion w;
	w.show();
	return a.exec();
}