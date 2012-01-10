#ifndef __ENVIEW_QMLSVISUALIZATION__
#define __ENVIEW_QMLSVISUALIZATION__

#include <envire/Core.hpp>
#include <vizkit/Vizkit3DPlugin.hpp>
#include "MLSVisualization.hpp"
#include <iostream>

namespace vizkit 
{
    class QMLSVisualization :public VizPluginBase, public MLSVisualization
    {
        Q_OBJECT
            Q_ENUMS(colorModeType)
            Q_PROPERTY(colorModeType color_mode READ getColorMode WRITE setColorMode USER true)
            Q_PROPERTY(bool show_uncertainty READ isUncertaintyEnabled WRITE showUncertainty USER true)
            Q_PROPERTY(bool estimate_normals READ isEstimateNormalsEnabled WRITE estimateNormals USER true)

        public:
            enum colorModeType{HEIGHT,CELL};

            QMLSVisualization(QObject *parent):
                VizPluginBase(parent),
                MLSVisualization(){};

            const QString getPluginName()const{return "MLSVisualization";};
            void updateMainNode(osg::Node*){};

            void setPluginEnabled(bool value)
            {
                VizPluginBase::setPluginEnabled(value);
                setEnabled(value);
            }

            colorModeType getColorMode()
            {
                switch(MLSVisualization::getColorMode())
                {
                case MLSVisualization::HEIGHT:
                    return HEIGHT;
                case MLSVisualization::CELL:
                    return CELL;
                default:
                    return HEIGHT;
                }
            }

            void setColorMode(colorModeType color_mode)
            {
                switch(color_mode)
                {
                case HEIGHT: 
                    MLSVisualization::setColorMode(MLSVisualization::HEIGHT);
                    break;
                case CELL:
                    MLSVisualization::setColorMode(MLSVisualization::CELL);
                    break;
                default:
                    MLSVisualization::setColorMode(MLSVisualization::HEIGHT);
                }
            }
    };

}
#endif 
