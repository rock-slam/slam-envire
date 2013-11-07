#include "ItemManipulator.hpp"
#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <QEvent>
#include <QMouseEvent>
#include <envire/Core.hpp>
#include <Eigen/LU> 
#include <osgManipulator/TranslateAxisDragger>
#include "FrameNodeVisualization.hpp"
#include <osgManipulator/CommandManager>
#include <osgManipulator/TrackballDragger>
#include "FrameNodeManipulator.hpp"

#include <base/Pose.hpp>

#include <iostream>

using namespace envire;

ItemManipulator::ItemManipulator()
{
    activeItem = 0;
}

void ItemManipulator::itemClicked(QTreeWidgetItem* item, int column)
{
    std::cout << "set selectedItem" << std::endl;
    //mw->getInstance()->selectedItem = tvl->getItemForWidget(item);
}

void ItemManipulator::itemActivated(QTreeWidgetItem* item, int column)
{
    std::cout << "Got Activated " << item <<  std::endl;
    static FrameNodeManipulator *frameManipulator = NULL;
    
    
    if(activeItem) {
	EnvironmentItemVisualizer *viz = eel->getVisualizerForItem(activeItem);
	if(viz) {
	    osg::Group *osgGroup = eel->getNodeForItem(activeItem);
	    viz->unHighlightNode(activeItem, osgGroup);
	    if(frameManipulator) {
		delete frameManipulator;
		frameManipulator = NULL;
	    }
	}
    }
    
    if(item) {
	activeItem = tvl->getItemForWidget(item);
	if(activeItem) {
	    EnvironmentItemVisualizer *viz = eel->getVisualizerForItem(activeItem);
	    if(viz) {
		osg::Group *osgGroup = eel->getNodeForItem(activeItem);		
		viz->highlightNode(activeItem, osgGroup);
		
		FrameNode *fn = dynamic_cast<envire::FrameNode *>(activeItem);
		if( fn )
		{
		    frameManipulator = new FrameNodeManipulator(activeItem, eel->getParentNodeForItem(activeItem));

		    base::Pose pose( fn->getTransform() );
		    std::cout << "FrameNode selected:" << std::endl
			<< ":position => ["
			    << pose.position.x() << ", " 
			    << pose.position.y() << ", " 
			    << pose.position.z() << "], " 
			<< ":orientation => ["
			    << pose.orientation.w() << ", " 
			    << pose.orientation.x() << ", " 
			    << pose.orientation.y() << ", " 
			    << pose.orientation.z() << "], " 
			<< ":id => '" << fn->getUniqueIdSuffix() << "'" << std::endl;
		}
	    }
	}
    }
}

void ItemManipulator::itemChanged(QTreeWidgetItem* item, int column)
{
    if(column == 0) 
    {
        if (item->checkState(0) == Qt::Checked) 
        {
            unhideItem(item);
        }
        else if (item->checkState(0) == Qt::Unchecked)
        {
            hideItem(item);
        }
    }
}

/**
* This function hides all in TreeWidget selectet items using the hideItem function.
*/
void ItemManipulator::hideSelectedItems()
{
    QList<QTreeWidgetItem*> itemList = tvl->getSelectetWidgets();
    for(QList<QTreeWidgetItem*>::iterator it = itemList.begin(); it != itemList.end(); it++) 
    {
        hideItem(*it);
    }
}

/**
* This function unhides all in TreeWidget selectet items using the unhideItem function.
*/
void ItemManipulator::unhideSelectedItems()
{
    QList<QTreeWidgetItem*> itemList = tvl->getSelectetWidgets();
    for(QList<QTreeWidgetItem*>::iterator it = itemList.begin(); it != itemList.end(); it++) 
    {
        unhideItem(*it);
    }
}

/**
* Hides the osg representation of the given QTreeWidgetItem.
*/
void ItemManipulator::hideItem(QTreeWidgetItem* widgetItem)
{
    envire::EnvironmentItem* item = tvl->getItemForWidget(widgetItem);
    envire::FrameNode* frameNode = dynamic_cast<envire::FrameNode *>(item);
    if(frameNode) 
    {
        widgetItem->setCheckState(0,Qt::Unchecked);
        EnvironmentItemVisualizer *viz = eel->getVisualizerForItem(item);
        if (viz) 
        {
            osg::Group *osgGroup = eel->getParentNodeForItem(item);
            viz->hideNode(item, osgGroup);
        }
    }
}

/**
* Unhides the osg representation of the given QTreeWidgetItem.
*/
void ItemManipulator::unhideItem(QTreeWidgetItem* widgetItem)
{
    envire::EnvironmentItem* item = tvl->getItemForWidget(widgetItem);
    envire::FrameNode* frameNode = dynamic_cast<envire::FrameNode *>(item);
    if(frameNode) 
    {
        widgetItem->setCheckState(0,Qt::Checked);
        EnvironmentItemVisualizer *viz = eel->getVisualizerForItem(item);
        if (viz) 
        {
            osg::Group *osgGroup = eel->getParentNodeForItem(item);
            viz->unHideNode(item, osgGroup);
        }
    }
}


/**
 * Detaches the selected item and all its childs from the environment.
 * The root node can't be removed.
 */
void ItemManipulator::removeSelectedItems()
{
    QList<QTreeWidgetItem*> itemList = tvl->getSelectetWidgets();
    for(QList<QTreeWidgetItem*>::iterator it = itemList.begin(); it != itemList.end(); it++) 
    {
        envire::EnvironmentItem* item = tvl->getItemForWidget(*it);
        envire::FrameNode* fn = dynamic_cast<envire::FrameNode*>(item);
        if(item && !(fn && fn->isRoot()))
            item->getEnvironment()->detachItem(item, true);
    }
}
