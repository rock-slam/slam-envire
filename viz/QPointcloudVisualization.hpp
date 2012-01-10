#ifndef __ENVIEW_QPOINTCLOUDVISUALIZATION__
#define __ENVIEW_QPOINTCLOUDVISUALIZATION__

#include <envire/Core.hpp>
#include <vizkit/Vizkit3DPlugin.hpp>
#include "PointcloudVisualization.hpp"

namespace vizkit 
{

class QPointcloudVisualization :public VizPluginBase, public PointcloudVisualization
{
    Q_OBJECT
    Q_PROPERTY(bool show_normals READ isNormalsEnabled WRITE setShowNormals USER true)
    Q_PROPERTY(bool show_features READ isFeaturesEnabled WRITE setShowFeatures USER true)

    public:
        QPointcloudVisualization(QObject *parent):VizPluginBase(parent){};
        const QString getPluginName()const{return "PCLVisualization";};
        void updateMainNode(osg::Node*){};

        void setPluginEnabled(bool value)
        {
            VizPluginBase::setPluginEnabled(value);
            setEnabled(value);
        }
};

}
#endif 
