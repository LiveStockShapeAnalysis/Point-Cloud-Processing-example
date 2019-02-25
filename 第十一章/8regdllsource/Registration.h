#ifndef REGISTATIONH
#define REGISTATIONH

#include <QMainWindow>
#include <QFileDialog>
 #include <QString>
#include <QProgressBar>

#include <pcl/io/ply_io.h>

#include "ui_MainWindow.h"
#include "header/stdafx.h"
#include <pcl/visualization/pcl_visualizer.h>
#include "vtkRenderWindow.h"
#include "header/IinitalRG.h"
using namespace RG;

namespace Ui 
{
    class MainWindow;
} // namespace Ui

class Registration:public QMainWindow
{
	 Q_OBJECT

public:
  explicit Registration (QWidget *parent = 0);
  ~Registration ();
  public slots:
	  void Open_source();
	  void Open_target();
	  void rg_slot();
	  void Downsamp_slot();
	  void NormalDone_slot();
	  void finished_slot();
	  void alignment_done_showresult_slot();

	protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

private:
	IinitalRG *rg;
	Ui::MainWindow *ui;
	pcl::PointCloud<pcl::PointXYZ> cloud_src; 
	pcl::PointCloud<pcl::PointXYZ> cloud_tgt; 
	QThread *inital_alignment;
	Eigen::Matrix4f result;
	 QProgressBar *progressBar;


};



#endif