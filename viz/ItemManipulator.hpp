#ifndef ITEMMANIPULATOR_H
#define ITEMMANIPULATOR_H

#include <QTreeWidgetItem>
#include "TreeViewListener.hpp"
#include "EnvireEventListener.hpp"
#include <QObject>
#include <QPoint>

namespace envire {

class ItemManipulator : public QObject
{
    Q_OBJECT
    public:
	ItemManipulator();
	ItemManipulator(EnvireEventListener *eel, TreeViewListener *tvl) : activeItem(0), eel(eel), tvl(tvl) {};
    public slots:
	void itemActivated ( QTreeWidgetItem * item, int column );
	void itemClicked ( QTreeWidgetItem * item, int column );
        void itemChanged ( QTreeWidgetItem * item, int column );
        void hideSelectedItems();
        void unhideSelectedItems();
        void removeSelectedItems();
        
        void createNewCutPCBox();
        void removeCutPCBox();
        /*
        void currentItemChanged(QTreeWidgetItem *current, QTreeWidgetItem *previous);
	void itemCollapsed ( QTreeWidgetItem * item );
	void itemDoubleClicked ( QTreeWidgetItem * item, int column );
	void itemEntered ( QTreeWidgetItem * item, int column );
	void itemExpanded ( QTreeWidgetItem * item );
        void itemPressed ( QTreeWidgetItem * item, int column );
	void itemSelectionChanged (); 
        */
    private:
	envire::EnvironmentItem *activeItem;
	EnvireEventListener *eel;
	TreeViewListener *tvl;
        void hideItem(QTreeWidgetItem*);
        void unhideItem(QTreeWidgetItem*);
};

}
#endif // ITEMMANIPULATOR_H
