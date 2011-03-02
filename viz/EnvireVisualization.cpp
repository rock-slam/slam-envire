#include "EnvireVisualization.hpp"
#include <boost/bind.hpp>

#include "LaserScanVisualization.hpp"
#include "FrameNodeVisualization.hpp"
#include "TriMeshVisualization.hpp"
#include "PointcloudVisualization.hpp"
#include "ElevationGridVisualization.hpp"
#include "MLSVisualization.hpp"
#include "ImageRGB24Visualization.hpp"

#include "ItemManipulator.hpp"
#include <vizkit/PickHandler.hpp>

#include <QTreeWidget>
#include <QString>
#include <QAction>

using namespace vizkit;

EnvireVisualization::EnvireVisualization()
    : m_handleDirty( true ), env( NULL )
{
    ownNode = new osg::Group();
    setMainNode( ownNode );

    // setup eventlistener
    eventListener = boost::shared_ptr<EnvireEventListener>(
	    new EnvireEventListener( 
		boost::bind( &osg::Group::addChild, ownNode->asGroup(), _1 ),
		boost::bind( &osg::Group::removeChild, ownNode->asGroup(), _1 ) ) );

    // create and register visualizers
    visualizers.push_back( boost::shared_ptr<LaserScanVisualization>(new LaserScanVisualization() ) );
    visualizers.push_back( boost::shared_ptr<FrameNodeVisualization>(new FrameNodeVisualization() ) );
    visualizers.push_back( boost::shared_ptr<TriMeshVisualization>(new TriMeshVisualization() ) );
    visualizers.push_back( boost::shared_ptr<PointcloudVisualization>(new PointcloudVisualization() ) );
    visualizers.push_back( boost::shared_ptr<ElevationGridVisualization>(new ElevationGridVisualization() ) );
    visualizers.push_back( boost::shared_ptr<MLSVisualization>(new MLSVisualization() ) );
    visualizers.push_back( boost::shared_ptr<ImageRGB24Visualization>(new ImageRGB24Visualization() ) );

    // attach visualizers
    for(std::vector<boost::shared_ptr<EnvironmentItemVisualizer> >::iterator it = visualizers.begin(); it != visualizers.end(); it++)
    {
	eventListener->addVisualizer( (*it).get() );
    }
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
    
void EnvireVisualization::operatorIntern( osg::Node* node, osg::NodeVisitor* nv )
{
    // since we are in the update call here, we can modify the osg tree
    // we'll call the apply method of the eventlistener, to perform back/front
    // swapping on all nodes that have been modified since the last call.
    eventListener->apply();
}

void EnvireVisualization::updateDataIntern( envire::Environment* const& data )
{
    // detach old root node
    if( env )
    {
	ownNode->asGroup()->removeChild( eventListener->getNodeForItem( env->getRootNode() ) );
    }

    env = data;
    env->addEventHandler( eventListener.get() );
    if( twl )
	env->addEventHandler( twl.get() );
}

