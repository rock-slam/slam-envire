#ifndef ENVIRE_GRIDVISUALIZATION_HPP
#define ENVIRE_GRIDVISUALIZATION_HPP

#include "GridVisualizationBase.hpp"

namespace envire 
{
    class GridVisualization : public GridVisualizationBase
    {
    Q_OBJECT

    Q_PROPERTY(bool cycle_grid_color READ isGridColorCycled WRITE setCycleGridColor)
    Q_PROPERTY(bool show_empty_cells READ areEmptyCellsShown WRITE setShowEmptyCells)

    enum ColorID
    {
        White = 0,
        Red, Green, Blue,
        Cyan, Yellow, Magenta
    };

    public:
        GridVisualization();
        virtual bool handlesItem(envire::EnvironmentItem *item) const;
        virtual void updateNode(envire::EnvironmentItem *item, osg::Group *node) const;

    public slots:
        bool isGridColorCycled() {return cycleGridColor;}
        void setCycleGridColor(bool enabled) {cycleGridColor = enabled; emit propertyChanged("cycle_grid_color");}
        bool areEmptyCellsShown() {return showEmptyCells;}
        void setShowEmptyCells(bool enabled) {showEmptyCells = enabled; emit propertyChanged("show_empty_cells");}

    private:
        float colors[7][3];
        bool cycleGridColor;
        bool showEmptyCells;
    };
}

#endif 
