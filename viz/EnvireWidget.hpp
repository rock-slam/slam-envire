#ifndef __ENVIRE_VIZ_ENVIREWIDGET_HPP__
#define __ENVIRE_VIZ_ENVIREWIDGET_HPP__

#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/EnvireVisualization.hpp>
#include <envire/Core.hpp>

namespace envire
{

class EnvireWidget : public vizkit3d::Vizkit3DWidget
{
    envire::Environment *env;
    EnvireVisualization *viz;

public:
    EnvireWidget()
    {
	env = new envire::Environment();
        viz = new EnvireVisualization;
        addPlugin(viz);
	viz->updateData(env);
    }

    ~EnvireWidget()
    {
	delete env;
    }

    envire::Environment* getEnvironment()
    {
	return env;
    }

    void setEnvironment(envire::Environment* env)
    {
        this->env = env;
	viz->updateData(env);
    }
};

}

#endif
