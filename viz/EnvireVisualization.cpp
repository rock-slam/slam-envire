#include "EnvireVisualization.hpp"
#include <boost/bind.hpp>

#include "LaserScanVisualization.hpp"
#include "FrameNodeVisualization.hpp"
#include "TriMeshVisualization.hpp"
#include "PointcloudVisualization.hpp"
#include "ElevationGridVisualization.hpp"
#include "MLSVisualization.hpp"
#include "ImageRGB24Visualization.hpp"
#include "TraversabilityGridVisualization.hpp"
#include "MapSegmentVisualization.hpp"
#include "GridVisualization.hpp"

#include "ItemManipulator.hpp"

#include <QTreeWidget>
#include <QString>
#include <QAction>

using namespace envire;

EnvireVisualization::EnvireVisualization()
    : m_handleDirty( true ), m_ownsEnvironment( false ), env( NULL )
{
    ownNode = new osg::Group();

    // setup eventlistener
    eventListener = boost::shared_ptr<EnvireEventListener>(
	    new EnvireEventListener( 
            [this](osg::Node* n) { ownNode->asGroup()->addChild(n);},
            [this](osg::Node* n) { ownNode->asGroup()->removeChild(n);}
         ) );

    // create and register visualizers
    // NOTE: the visualizers at the back have higher priority 
    visualizers.push_back( boost::shared_ptr<GridVisualization>(new GridVisualization()));
    visualizers.push_back( boost::shared_ptr<LaserScanVisualization>(new LaserScanVisualization() ) );
    visualizers.push_back( boost::shared_ptr<FrameNodeVisualization>(new FrameNodeVisualization() ) );
    visualizers.push_back( boost::shared_ptr<TriMeshVisualization>(new TriMeshVisualization() ) );
    visualizers.push_back( boost::shared_ptr<PointcloudVisualization>(new PointcloudVisualization() ) );
    visualizers.push_back( boost::shared_ptr<ElevationGridVisualization>(new ElevationGridVisualization() ) );
    visualizers.push_back( boost::shared_ptr<MLSVisualization>(new MLSVisualization() ) );
    visualizers.push_back( boost::shared_ptr<ImageRGB24Visualization>(new ImageRGB24Visualization() ) );
    visualizers.push_back( boost::shared_ptr<TraversabilityGridVisualization>(new TraversabilityGridVisualization()));
    visualizers.push_back( boost::shared_ptr<MapSegmentVisualization>(new MapSegmentVisualization()));
    
    // attach visualizers
    for(std::vector<boost::shared_ptr<EnvironmentItemVisualizer> >::iterator it = visualizers.begin(); it != visualizers.end(); it++)
    {
        (*it)->setParent(this);
	eventListener->addVisualizer( (*it).get() );
        QObject::connect((*it).get(), SIGNAL(propertyChanged(QString)), eventListener.get(), SLOT(propertyChangedInVizualization(QString)));
    }
}

EnvireVisualization::~EnvireVisualization()
{
    if (m_ownsEnvironment)
        delete env;
}

/** Load the environment from disk and display it */
void EnvireVisualization::load(std::string const& path)
{
    updateData(envire::Environment::unserialize(path));
    m_ownsEnvironment = true;
}

void EnvireVisualization::attachTreeWidget( QTreeWidget *treeWidget )
{
    // TODO fix the widget handling stuff
    //create context menu for TreeWidget
    treeWidget->setContextMenuPolicy(Qt::ActionsContextMenu);
    QAction* hideItem = new QAction(QString("hide item"), treeWidget);
    QAction* unhideItem = new QAction(QString("unhide item"), treeWidget);
    QAction* removeItem = new QAction(QString("remove item"), treeWidget);
    treeWidget->addAction(hideItem);
    treeWidget->addAction(unhideItem);
    treeWidget->addAction(removeItem);
    
    twl = boost::shared_ptr<TreeViewListener>(new TreeViewListener(treeWidget));
    ItemManipulator *im = new ItemManipulator(eventListener.get(), twl.get());
    QObject::connect(treeWidget, SIGNAL(itemActivated ( QTreeWidgetItem *, int )), im, SLOT(itemActivated ( QTreeWidgetItem *, int)));
    QObject::connect(treeWidget, SIGNAL(itemClicked ( QTreeWidgetItem *, int )), im, SLOT(itemClicked ( QTreeWidgetItem *, int)));
    QObject::connect(hideItem, SIGNAL(triggered()), im, SLOT(hideSelectedItems()));
    QObject::connect(unhideItem, SIGNAL(triggered()), im, SLOT(unhideSelectedItems()));
    QObject::connect(treeWidget, SIGNAL(itemChanged ( QTreeWidgetItem *, int )), im, SLOT(itemChanged ( QTreeWidgetItem *, int)));
    QObject::connect(removeItem, SIGNAL(triggered()), im, SLOT(removeSelectedItems()));
    //view->installEventFilter(im);
    if( env )
	env->addEventHandler(twl.get());
}

bool EnvireVisualization::isDirty() const
{
    // TODO instead of going through the list to find dirty events
    // the event listener could actually keep a list. But for now, 
    // assume we are dirty, if not proven otherwise :)
    if( m_handleDirty )
	return true;
    else
	return VizPluginBase::isDirty();
}
    
void EnvireVisualization::updateMainNode(osg::Node* node)
{
    // since we are in the update call here, we can modify the osg tree
    // we'll call the apply method of the eventlistener, to perform back/front
    // swapping on all nodes that have been modified since the last call.
    eventListener->apply();
}

void EnvireVisualization::updateBinaryEvents( std::vector<envire::BinaryEvent> const& events )
{
    for( std::vector<envire::BinaryEvent>::const_iterator it = events.begin(); 
	    it != events.end(); it++ )
    {
	updateBinaryEvent( *it );
    }
}

void EnvireVisualization::updateBinaryEvent( envire::BinaryEvent const& binary_event )
{
    // the only thing that needs to be locked against the osg thread
    // is the actual generation of the environment
    // everything else can happen in the current thread, due
    // to the double buffering mechanism in the visualization handler
    if( !env )
    {
	updateData( new envire::Environment() );
	m_ownsEnvironment = true;
    }
    else if( !m_ownsEnvironment )
	throw std::runtime_error("BinaryEvents are only supported on environments owned by the visualization");


    // see if we need to deserialize the binary event
    envire::EnvironmentItem* item = 0;
    if( binary_event.type == envire::event::ITEM 
	    && (binary_event.operation == envire::event::ADD 
		|| binary_event.operation == envire::event::UPDATE ))
    {
	// unserialize item
	item = serialization.unserializeBinaryEvent( binary_event );
    }

    // set up event
    envire::EnvironmentItem::Ptr item_ptr(item);
    envire::Event event(binary_event.type, binary_event.operation, item_ptr);
    event.id_a = binary_event.id_a;
    event.id_b = binary_event.id_b;

    // apply event
    event.apply(env);
}

void EnvireVisualization::updateDataIntern( envire::Environment* const& data )
{
    // detach old root node
    if( env )
    {
	ownNode->asGroup()->removeChild( eventListener->getNodeForItem( env->getRootNode() ) );
    }

    if (env && m_ownsEnvironment)
        delete env;

    m_ownsEnvironment = false;
    env = data;
    env->addEventHandler( eventListener.get() );
    if( twl )
	env->addEventHandler( twl.get() );
}

osg::ref_ptr< osg::Node > EnvireVisualization::createMainNode()
{
    return ownNode;
}

QObject* EnvireVisualization::getVisualizer(QString name){
	for (std::vector<boost::shared_ptr<EnvironmentItemVisualizer> >::iterator it = visualizers.begin();it !=visualizers.end();it++){
		if ( name == (*it)->getPluginName()){
			return it->get();
		}
	}
	perror("Visualizer not found\n");
	return NULL;
}

namespace envire
{
VizkitQtPluginImpl(EnvireVisualization);
}
