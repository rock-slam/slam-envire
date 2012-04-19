
#include <vizkit/Vizkit3DWidget.hpp>
#include <QApplication>
#include <QMainWindow>
#include <QDockWidget>
#include <QLayout>
#include <QHBoxLayout>

#include "TreeViewListener.hpp"
#include "EnvireVisualization.hpp"

using namespace envire;

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    
    if(argc != 2)
    {
	std::cout << "Usage : ./envireViewer 'path to env'" << std::endl;
	exit(1);
    }

    //create main window
    QMainWindow a;

    //load environment
    envire::Environment *env = envire::Environment::unserialize( std::string(argv[1]) );

    //create vizkit plugin for showing envire
    EnvireVisualization *envViz = new EnvireVisualization();    
    envViz->updateData(env);

    //create tree widget for showing nodes
    QTreeWidget *qtw = new QTreeWidget();
    envViz->attachTreeWidget(qtw);
    
    //dock tree widget
    QDockWidget *qdw = new QDockWidget();
    qdw->setWidget(qtw);    
    a.addDockWidget(Qt::LeftDockWidgetArea, qdw);
    
    //create vizkit widget
    vizkit::Vizkit3DWidget *widget = new vizkit::Vizkit3DWidget();
    
    //set envire as central dock widget
    a.setCentralWidget(widget);

    //add envire plugin
    widget->addPlugin(envViz);
    
    a.show();
    return app.exec();

}
