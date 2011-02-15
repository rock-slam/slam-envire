
#include <vizkit/QVizkitWidget.hpp>
#include <QApplication>
#include <QMainWindow>
#include <QDockWidget>
#include <QLayout>
#include <QHBoxLayout>

#include <viz/TreeViewListener.hpp>
#include <viz/EnvireVisualization.hpp>

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
    envire::Serialization so;
    envire::Environment *env = so.unserialize( std::string(argv[1]) );

    //create vizkit plugin for showing envire
    vizkit::EnvireVisualization *envViz = new vizkit::EnvireVisualization();    
    envViz->updateData(env);

    //create tree widget for showing nodes
    QTreeWidget *qtw = new QTreeWidget();
    envViz->attachTreeWidget(qtw);
    
    //dock tree widget
    QDockWidget *qdw = new QDockWidget();
    qdw->setWidget(qtw);    
    a.addDockWidget(Qt::LeftDockWidgetArea, qdw);
    
    //create vizkit widget
    vizkit::QVizkitWidget *widget = new vizkit::QVizkitWidget();
    
    //set envire as central dock widget
    a.setCentralWidget(widget);

    //add envire plugin
    widget->addDataHandler(envViz);
    
    a.show();
    return app.exec();

}