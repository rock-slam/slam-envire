#include "TreeViewListener.hpp"

#include <string>
#include <iostream>
#include <boost/lexical_cast.hpp>

namespace envire {


void TreeViewListener::itemClicked(QTreeWidgetItem* item, int column)
{
    selected = getItemForWidget(item);
}

void TreeViewListener::setRootNode(envire::FrameNode* root)
{
    getWidgetForEnvItem(root, FrameNode, true);    
}

void TreeViewListener::removeRootNode(envire::FrameNode* root)
{
    QTreeWidgetItem *rootW = nodeToWidget[root];
    tw->removeItemWidget(rootW, 0);
    widgetToNode.erase(rootW);
    delete rootW;
    
    nodeToWidget.erase(root);
}

void TreeViewListener::childAdded(envire::FrameNode* parent, envire::FrameNode* child)
{
    QTreeWidgetItem *parentW = getWidgetForEnvItem(parent, FrameNode, true);    
    QTreeWidgetItem *childW = getWidgetForEnvItem(child, FrameNode);
    
    parentW->addChild(childW);    
}

QTreeWidgetItem* TreeViewListener::getWidgetForEnvItem(envire::EnvironmentItem* item, TreeViewListener::ItemTypes type, bool topLevel )
{
    QTreeWidgetItem *itemW = 0;
    
    if(nodeToWidget.count(item) == 0) {
	std::string name = item->getClassName()+"["+boost::lexical_cast<std::string>(item->getUniqueId())+"]";
	QStringList sl(QString::fromStdString(name));
	itemW = new QTreeWidgetItem(sl, type);
        
        //acivate QCheckBoxes for FrameNodes 
        if(dynamic_cast<envire::FrameNode *>(item)) 
        {
            QFlags<Qt::ItemFlag> flg=Qt::ItemIsUserCheckable | Qt::ItemIsSelectable | Qt::ItemIsEnabled;
            itemW->setFlags(flg);
            itemW->setCheckState(0,Qt::Checked);
        }
        
	nodeToWidget[item] = itemW;
	widgetToNode[itemW] = item; 
	if(topLevel)
	    tw->addTopLevelItem(itemW);
    } else {
	itemW = nodeToWidget[item];
    }
    
    return itemW;
}

void TreeViewListener::frameNodeSet(envire::CartesianMap* map, envire::FrameNode* node)
{
    QTreeWidgetItem *mapW = getWidgetForEnvItem(map, CartesianMap);
    QTreeWidgetItem *nodeW = getWidgetForEnvItem(node, FrameNode);
    nodeW->addChild(mapW);    
}

void TreeViewListener::frameNodeDetached(envire::CartesianMap* map, envire::FrameNode* node)
{
    QTreeWidgetItem *nodeW = getWidgetForEnvItem(node, FrameNode);
    QTreeWidgetItem *mapW = getWidgetForEnvItem(map, FrameNode);
    
    nodeW->removeChild(mapW);
    
    nodeToWidget.erase(map);
    widgetToNode.erase(mapW);
    
    delete mapW;
}

void TreeViewListener::childRemoved(envire::FrameNode* parent, envire::FrameNode* child)
{
    QTreeWidgetItem *childW = getWidgetForEnvItem(child, FrameNode);
    QTreeWidgetItem *parentW = getWidgetForEnvItem(parent, FrameNode);
    
    parentW->removeChild(childW);
    
    nodeToWidget.erase(child);
    widgetToNode.erase(childW);
    
    delete childW;
}

void TreeViewListener::itemDetached(EnvironmentItem* item)
{
    // for the case that the root node gets replaced
    envire::FrameNode* fn = dynamic_cast<envire::FrameNode*>(item);
    if(fn && fn->isRoot())
        removeRootNode(fn);
}

TreeViewListener::TreeViewListener(QTreeWidget* tw)
    : selected( NULL )
{
    assert( tw );
    this->tw = tw;
    connect(tw, SIGNAL(itemClicked ( QTreeWidgetItem *, int )), this, SLOT(itemClicked ( QTreeWidgetItem *, int)));
}

/*
* returns the selected TreeWidgetItems
*/
QList<QTreeWidgetItem*> TreeViewListener::getSelectetWidgets()
{
    return this->tw->selectedItems();
}

}
