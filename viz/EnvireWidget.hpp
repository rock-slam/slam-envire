#ifndef __ENVIRE_VIZ_ENVIREWIDGET_HPP__
#define __ENVIRE_VIZ_ENVIREWIDGET_HPP__

#include <vizkit/QVisualizationTestWidget.hpp>
#include <vizkit/EnvireVisualization.hpp>
#include <envire/Core.hpp>

namespace vizkit
{

class EnvireWidget : 
    public QVisualizationTestWidget<EnvireVisualization, envire::Environment*>
{
    envire::Environment *env;

public:
    EnvireWidget()
    {
	env = new envire::Environment();
	updateData( env );
    }

    ~EnvireWidget()
    {
	delete env;
    }

    envire::Environment* getEnvironment()
    {
	return env;
    }
};

}

#endif
