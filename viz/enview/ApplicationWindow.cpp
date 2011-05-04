#include "ApplicationWindow.hpp"

#include <QFileDialog>
#include <envire/maps/LaserScan.hpp>
#include <envire/maps/TriMesh.hpp>
#include <envire/maps/Pointcloud.hpp>
#include <envire/operators/ScanMeshing.hpp>
#include <yaml.h>

#include <osg/PositionAttitudeTransform>
#include <osg/LightModel>
#include <osg/Depth>

#include "ICPHandler.hpp"
#include <boost/bind.hpp>

namespace enview {

ApplicationWindow::ApplicationWindow() 
    : QMainWindow()
{
    // connect menu items to functions
    ui.setupUi( this );
    this->setCentralWidget(ui.graphicsArea);
    connect( ui.actionQuit, SIGNAL(activated(void)), this, SLOT(close()) );
    connect( ui.actionAdd_From_Scanfile, SIGNAL(activated(void)), this, SLOT(addFromScanFileDialog()) );
    connect( ui.actionAdd_From_Csv, SIGNAL(activated(void)), this, SLOT(addFromCsvDialog()) );
    connect( ui.actionOpen_Envire, SIGNAL(activated(void)), this, SLOT(loadEnvironment()) );
    connect( ui.actionSave_Envire, SIGNAL(activated(void)), this, SLOT(saveEnvironment()) );
    connect( ui.actionSave_As, SIGNAL(activated(void)), this, SLOT(saveAsEnvironment()) );
    connect( ui.updateOperatorsButton, SIGNAL(clicked(void)), this, SLOT(updateOperators()) );
    

    // installing handlers for different functionalities
    icpHandler = new ICPHandler(this, ui, boost::bind( &ApplicationWindow::getSelectedItem, this ) );
    //sequenceHandler = new SequenceHandler(this, ui);

    // seting up envire plugin and creating an empty environment
    envViz = boost::shared_ptr<vizkit::EnvireVisualization>(new vizkit::EnvireVisualization());
    ui.vizkitWidget->addDataHandler( envViz.get() );
    envViz->attachTreeWidget( ui.treeWidget );

    setEnvironment(new envire::Environment());

    this->show();
}

void ApplicationWindow::setEnvironment( envire::Environment* _env )
{
    envViz->updateData( _env );
    env = boost::shared_ptr<envire::Environment>( _env );
}

envire::Environment* ApplicationWindow::getEnvironment()
{
    return env.get();
}

/*
TreeViewListener& ApplicationWindow::getTreeViewListener()
{
    return *twl;
}
*/

void ApplicationWindow::updateOperators()
{
    env->updateOperators();
}

void ApplicationWindow::createTriMesh(envire::LaserScan* ls)
{
    if( ls != NULL )
    {
	// create a TriMesh Layer and attach it to the root Node.
	envire::TriMesh* mesh = new envire::TriMesh();
	env->attachItem( mesh );
	env->setFrameNode( mesh, ls->getFrameNode() );

	// set up a meshing operator on the output mesh. Add then an input
	// and parametrize the meshing operation. 
	envire::ScanMeshing* mop = new envire::ScanMeshing();
	env->attachItem( mop );

	mop->setMaxEdgeLength(0.5);

	mop->addInput(ls);
	mop->addOutput(mesh);

	mop->updateAll();
	
	env->itemModified(mesh);
    }
}

void ApplicationWindow::addFromScanFileDialog()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Add Scan File"),
            ".",
            tr("Scans (*.scan)"));

    if(fileName != NULL) {
	envire::LaserScan* scan = 
	    envire::LaserScan::importScanFile(fileName.toStdString(), env->getRootNode() );
	
	createTriMesh( scan );
    }
}

void ApplicationWindow::addFromCsvDialog()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Add Csv File"),
            ".",
            tr("Csv (*.txt)"));

    envire::FrameNode *fn = dynamic_cast<envire::FrameNode*>(getSelectedItem());
    if( !fn )
	fn = env->getRootNode();

    if(fileName != NULL) {
	envire::Pointcloud::importCsv(fileName.toStdString(), fn);
    }
}

void ApplicationWindow::loadEnvironment()
{
    QString fileName = QFileDialog::getExistingDirectory(this, tr("Load Environment"),
            ".",
	    QFileDialog::ShowDirsOnly);

    if(fileName != NULL) {
	envire::Serialization so;
	envire::Environment *env = so.unserialize( fileName.toStdString() );

	setEnvironment(env);
    }
}

void ApplicationWindow::saveEnvironment()
{
    if(envFileName.empty())
	saveAsEnvironment();

    envire::Serialization so;
    so.serialize( 
	env.get(),
	envFileName );
}

void ApplicationWindow::saveAsEnvironment()
{
    QString fileName = QFileDialog::getExistingDirectory(this, tr("Save Environment"),
            ".",
	    QFileDialog::ShowDirsOnly);

    if(fileName != NULL) {
	envFileName = fileName.toStdString();
	saveEnvironment();
    }

}

void ApplicationWindow::pickedPoint(const Eigen::Vector3d& coord)
{
    envire::Pointcloud *pc = dynamic_cast<envire::Pointcloud*>(getSelectedItem());
    if( pc )
    {
	pc->vertices.push_back( coord );
	pc->getEnvironment()->itemModified( pc );
    }
}

/*
void ApplicationWindow::addNode ( osg::Node* node )
{
    ui.enviewWidget->getRootNode()->addChild(node);
}

void ApplicationWindow::registerSequenceData ( enview::SequenceData* node )
{
    sequenceHandler->registerNode( node );
}

void ApplicationWindow::updateSequenceData()
{
    sequenceHandler->updateRange();
}

void ApplicationWindow::removeNode ( osg::Node* node )
{
    ui.enviewWidget->getRootNode()->removeChild(node);
}


void ApplicationWindow::addPlugin(BaseDataNode* node)
{
    std::vector<BaseDataNode*>::iterator it = std::find(plugins.begin(), plugins.end(), node);
    if(it != plugins.end()) 
        throw std::runtime_error("Error trying to add Plugin twice.");
    plugins.push_back(node);
}

void ApplicationWindow::removePlugin(BaseDataNode* node)
{
    std::vector<BaseDataNode*>::iterator it = std::find(plugins.begin(), plugins.end(), node);
    if(it == plugins.end()) 
        throw std::runtime_error("Error trying remove not added Plugin.");
        
    plugins.erase(it);
}

void ApplicationWindow::loadPluginData(const std::string& path)
{
    std::ifstream fin(path.c_str());
    if (fin.is_open()) 
    {
        try 
        {
            YAML::Parser parser(fin);
            YAML::Node yamlNode;
            parser.GetNextDocument(yamlNode);
            for(std::vector<BaseDataNode *>::iterator it = plugins.begin(); it != plugins.end(); it++) 
            {
                BaseDataNode* dataNode = *it;
                if (dataNode) 
                {
                    if (const YAML::Node* yamlSubNode = yamlNode.FindValue(dataNode->getPluginName())) 
                    {
                        dataNode->loadData(*yamlSubNode);
                    }
                }
                
            }
        } catch (YAML::ParserException& e1) {
            std::cerr << "Fehler beim Parsen der YAML-Datei: " << e1.what() << std::endl;
        } catch (YAML::RepresentationException& e2) {
            std::cerr << "Ein Attribut wurde nicht gefunden: " << e2.what() << std::endl;
        }
        fin.close();
    }
}

void ApplicationWindow::savePluginData(const std::string& path)
{
    std::ofstream fout(path.c_str());
    if (fout.is_open()) 
    {
        YAML::Emitter emitter;
        emitter << YAML::BeginMap;
        for(std::vector<BaseDataNode *>::iterator it = plugins.begin(); it != plugins.end(); it++)
        {
            BaseDataNode* dataNode = *it;
            if (dataNode) 
            emitter << YAML::Key << dataNode->getPluginName();
            emitter << YAML::Value;
            dataNode->saveData(emitter);
        }
        emitter << YAML::EndMap;
        fout << emitter.c_str();
        fout.close();
    }
}
*/


}
