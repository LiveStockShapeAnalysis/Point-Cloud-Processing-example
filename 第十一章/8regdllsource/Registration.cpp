#include "Registration.h"

  Registration::Registration(QWidget *parent) :
  QMainWindow (parent),
	  ui (new Ui::MainWindow)
  {
	  ui->setupUi (this);
	  QString str;
	  str = str.fromLocal8Bit("点云库PCL学习从入门到精通案例");
	  this->setWindowTitle (str);

	  progressBar=new QProgressBar();
	  progressBar->setWindowTitle(str);
	  progressBar->setMinimum(0);
	  progressBar->setMaximum(100);
	  progressBar->setTextVisible(true);

	  inital_alignment=new QThread();
	  rg=new IinitalRG(0.05,2,5,10,10,1.0,10000);
	  QObject::connect(inital_alignment,SIGNAL(started()),rg, SLOT(run()));
	  QObject::connect(rg,SIGNAL(Initial_alignment_Done()),inital_alignment,SLOT(quit()));
	  QObject::connect(inital_alignment,SIGNAL(finished()),this, SLOT(finished_slot()));
	  QObject::connect(rg,SIGNAL(Initial_alignment_Done()),this,SLOT(alignment_done_showresult_slot()));

	  rg->moveToThread(inital_alignment);
	 
	  
	  
	   // Set up the QVTK window
	  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
	  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
	  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
	  ui->qvtkWidget->update ();

	  connect(ui->action_load_source,SIGNAL(triggered()),this,SLOT(Open_source()));
	  connect(ui->action_load_target,SIGNAL(triggered()),this,SLOT(Open_target()));
	  connect(ui->action_rg,SIGNAL(triggered()),this,SLOT(rg_slot()));

	  connect(rg,SIGNAL(DownS_Done()),this,SLOT(Downsamp_slot()));
	  connect(rg,SIGNAL(NoramlE_Done()),this,SLOT(NormalDone_slot()));

  }

  Registration::~Registration ()
  {

  }
  void Registration::Open_source()
  {
	 QString _qt_ply_file = QFileDialog::getOpenFileName (this,
                                                            "Select one ply files to open",
                                                            "/home",
                                                            "3d Model (*.ply)");
	 if(_qt_ply_file!=NULL)
	 {

		 if (pcl::io::loadPLYFile (_qt_ply_file.toStdString(), cloud_src) < 0)
		 {
			 cout << "bad thing happened in loadPLYFile!,please check your ply format" << "\n";
		 }
		 else
		 {
			 rg->Set_source(cloud_src);
			 pcl::PointCloud<pcl::PointXYZ>::Ptr P_cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
			 *P_cloud_src=cloud_src;
			 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green (P_cloud_src, 0,255,0); 
			 this->viewer->addPointCloud(P_cloud_src,green,"cloud_src_orignal");
			 viewer->resetCamera ();
			 ui->qvtkWidget->update ();

		 } 
	 }


  }

  void Registration::Open_target()
  {
	  QString _qt_ply_file = QFileDialog::getOpenFileName (this,
                                                            "Select one ply files to open",
                                                            "/home",
                                                            "3d Model (*.ply)");
	 if(_qt_ply_file!=NULL)
	 {

		 if (pcl::io::loadPLYFile (_qt_ply_file.toStdString(), cloud_tgt) < 0)
		 {
			 cout << "bad thing happened in loadPLYFile!,please check your ply format" << "\n";
		 }
		 else
		 { 
			 rg->Set_target(cloud_tgt);
			  pcl::PointCloud<pcl::PointXYZ>::Ptr P_cloud_tgt(new pcl::PointCloud<pcl::PointXYZ>);
			 *P_cloud_tgt=cloud_tgt;
			 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (P_cloud_tgt, 255,0,0); 
			 this->viewer->addPointCloud(P_cloud_tgt,red,"cloud_tgt_orignal");
			 viewer->resetCamera ();
			 ui->qvtkWidget->update ();
		 }
			
	 }

  }
  void Registration::rg_slot()
  {
	  inital_alignment->start();
	  progressBar->setValue(0);
	  progressBar->show();
  }
  void Registration::Downsamp_slot()
  {
	  progressBar->setValue(20);
	  ui->statusbar->showMessage(QString("downsamping is done"));
  }

  void Registration::NormalDone_slot()
  {
	   progressBar->setValue(40);
	  cout<<QThread::currentThreadId ()<<endl;
	  cout<<"Registration::NormalDone_slot()"<<endl;
	  ui->statusbar->showMessage(QString("normal estimation is done"));
  }

   void Registration::finished_slot()
   {
	    progressBar->setValue(100);
	   cout<<QThread::currentThreadId ()<<endl;
	   cout<<"Registration::finished_slot()"<<endl;
	   ui->statusbar->showMessage(QString("thread is done"));
   }

  void Registration::alignment_done_showresult_slot()
   {
	    progressBar->setValue(80);
	   rg->Get_alignment_result(result);
	   cout<<result<<endl;
	   //progressBar->close();

   }