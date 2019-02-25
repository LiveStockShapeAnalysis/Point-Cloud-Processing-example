#include "RGlib_Export.h"
#include "IinitalRG.h"
 #include <QCoreApplication>
#include "Registration.h"
#include <boost/thread.hpp>

using namespace RG;

 int main(int argc, char *argv[])
{

	QApplication app (argc, argv);

	cout<<QThread::currentThreadId ()<<endl;
	cout<<"main(int argc, char *argv[])()"<<endl;
	Registration mytest;

	mytest.show ();
	return (app.exec ());
	
	/*	QCoreApplication a(argc, argv);
		QThread *inital_alignment(new QThread());
		IinitalRG *rg;

		QObject::connect(inital_alignment,SIGNAL(started()),rg, SLOT(run()));
		QObject::connect(rg,SIGNAL(Initial_alignment_Done()),inital_alignment,SLOT(quit()));
		rg->moveToThread(inital_alignment);
		inital_alignment->start();
	
		
		a.exec();*/


}