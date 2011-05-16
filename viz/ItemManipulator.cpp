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

vizkit::ItemManipulator::ItemManipulator()
{
    activeItem = 0;
}

void vizkit::ItemManipulator::itemClicked(QTreeWidgetItem* item, int column)
{
    std::cout << "set selectedItem" << std::endl;
    //mw->getInstance()->selectedItem = tvl->getItemForWidget(item);
}

void vizkit::ItemManipulator::itemActivated(QTreeWidgetItem* item, int column)
{
    std::cout << "Got Activated " << item <<  std::endl;
    static vizkit::FrameNodeManipulator *frameManipulator = NULL;
    
    
    if(activeItem) {
	vizkit::EnvironmentItemVisualizer *viz = eel->getVisualizerForItem(activeItem);
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
	    vizkit::EnvironmentItemVisualizer *viz = eel->getVisualizerForItem(activeItem);
	    if(viz) {
		osg::Group *osgGroup = eel->getNodeForItem(activeItem);		
		viz->highlightNode(activeItem, osgGroup);
		
		if(dynamic_cast<envire::FrameNode *>( activeItem))
		    frameManipulator = new FrameNodeManipulator(activeItem, eel->getParentNodeForItem(activeItem), osgGroup);
	    }
	}
    }
}

void vizkit::ItemManipulator::itemChanged(QTreeWidgetItem* item, int column)
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

/*
* This function hides all in TreeWidget selectet items using the hideItem function.
*/
void vizkit::ItemManipulator::hideSelectedItems()
{
    QList<QTreeWidgetItem*> itemList = tvl->getSelectetWidgets();
    for(QList<QTreeWidgetItem*>::iterator it = itemList.begin(); it != itemList.end(); it++) 
    {
        hideItem(*it);
    }
}

/*
* This function unhides all in TreeWidget selectet items using the unhideItem function.
*/
void vizkit::ItemManipulator::unhideSelectedItems()
{
    QList<QTreeWidgetItem*> itemList = tvl->getSelectetWidgets();
    for(QList<QTreeWidgetItem*>::iterator it = itemList.begin(); it != itemList.end(); it++) 
    {
        unhideItem(*it);
    }
}

/*
* Hides the osg representation of the given QTreeWidgetItem.
*/
void vizkit::ItemManipulator::hideItem(QTreeWidgetItem* widgetItem)
{
    envire::EnvironmentItem* item = tvl->getItemForWidget(widgetItem);
    envire::FrameNode* frameNode = dynamic_cast<envire::FrameNode *>(item);
    if(frameNode) 
    {
        widgetItem->setCheckState(0,Qt::Unchecked);
        vizkit::EnvironmentItemVisualizer *viz = eel->getVisualizerForItem(item);
        if (viz) 
        {
            osg::Group *osgGroup = eel->getParentNodeForItem(item);
            viz->hideNode(item, osgGroup);
        }
    }
}

/*
* Unhides the osg representation of the given QTreeWidgetItem.
*/
void vizkit::ItemManipulator::unhideItem(QTreeWidgetItem* widgetItem)
{
    envire::EnvironmentItem* item = tvl->getItemForWidget(widgetItem);
    envire::FrameNode* frameNode = dynamic_cast<envire::FrameNode *>(item);
    if(frameNode) 
    {
        widgetItem->setCheckState(0,Qt::Checked);
        vizkit::EnvironmentItemVisualizer *viz = eel->getVisualizerForItem(item);
        if (viz) 
        {
            osg::Group *osgGroup = eel->getParentNodeForItem(item);
            viz->unHideNode(item, osgGroup);
        }
    }
}

void vizkit::ItemManipulator::removeSelectedItems()
{
    QList<QTreeWidgetItem*> itemList = tvl->getSelectetWidgets();
    for(QList<QTreeWidgetItem*>::iterator it = itemList.begin(); it != itemList.end(); it++) 
    {
        
    }
}
