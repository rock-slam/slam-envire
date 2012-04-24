#ifndef __ENVIEW_APPLICATIONWINDOW_HPP__
#define __ENVIEW_APPLICATIONWINDOW_HPP__

#include "ui_MainWindow.h"

#include <Eigen/Core>
#include "../EnvireVisualization.hpp"

namespace envire 
{
class LaserScan;
class Environment;
}

namespace enview {

class ICPHandler;

class ApplicationWindow : public QMainWindow {
    Q_OBJECT

protected:
    std::string envFileName;

public:
    explicit ApplicationWindow( envire::Environment* _env = NULL );
    /*
    void addNode(osg::Node *node);
    void removeNode(osg::Node *node);
    
    void addPlugin(enview::BaseDataNode *node);
    void removePlugin(enview::BaseDataNode *node);

    void registerSequenceData(SequenceData *node);
    void updateSequenceData();

    TreeViewListener &getTreeViewListener();
    */
    //osg::ref_ptr<ViewQOSG> getView() { return view1; }
   
    void createTriMesh(envire::LaserScan* scan);
    void setEnvironment(envire::Environment* env);
    envire::Environment* getEnvironment();
    envire::EnvironmentItem* getSelectedItem() { return envViz->getSelectedItem(); }

public slots:
    void updateOperators();

    void addFromScanFileDialog();
    void addFromCsvDialog();
    void loadEnvironment();
    void saveEnvironment();
    void saveAsEnvironment();
    /*
    void savePluginData(const std::string &path);
    void loadPluginData(const std::string &path);
    */

    void pickedPoint(const Eigen::Vector3d& coord);

public:  // for now
    ICPHandler *icpHandler;
    //SequenceHandler *sequenceHandler;
    //TreeViewListener *twl;
    Ui::MainWindow ui;

    boost::shared_ptr<envire::EnvireVisualization> envViz;
    boost::shared_ptr<envire::Environment> env;
};

}

#endif // MAINWINDOWINSTANCE_H
