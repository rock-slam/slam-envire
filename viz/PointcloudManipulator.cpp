#include "PointcloudManipulator.hpp"
#include <osg/Version>
#include <envire/core/EventHandler.hpp>
#include <osg/ShapeDrawable>
#include <osgManipulator/TabBoxDragger>
#include <envire/operators/CutPointcloud.hpp>
#include "EnvireEventListener.hpp"

namespace envire 
{
    
class TabBoxUpdate : public osgManipulator::DraggerCallback
{
public:
    TabBoxUpdate(osgManipulator::TabBoxDragger* dragger, osg::MatrixTransform* transform, std::list<CutPointcloud*> operators)
        : dragger(dragger), transform(transform), operators(operators)
    {
        // create exclusion box
        box.reset(new ExclusionBox());
        for(std::list<CutPointcloud*>::const_iterator it = this->operators.begin(); it != this->operators.end(); it++)
        {
            (*it)->addBox(box.get());
        }
        
        updateBoxCoordinates();
    }
    
    void detachBox()
    {
        for(std::list<CutPointcloud*>::const_iterator it = operators.begin(); it != operators.end(); it++)
        {
            (*it)->removeBox(box.get());
        }
    }
    
    bool receive( const osgManipulator::MotionCommand& cmd )
    {
        updateBoxCoordinates();
        return true;
    }
    
    void updateBoxCoordinates()
    {
        osg::Vec3d p1(0.5,0.5,0.5);
        osg::Vec3d p2(-0.5,-0.5,-0.5);
        
        p1 = p1 * transform->getMatrix();
        p2 = p2 * transform->getMatrix();
        
        box->box.setNull();
        box->box.extend(Eigen::Vector3d(p1.x(),p1.y(),p1.z()));
        box->box.extend(Eigen::Vector3d(p2.x(),p2.y(),p2.z()));
    }
    
private:
    osg::ref_ptr<osgManipulator::TabBoxDragger> dragger;
    osg::ref_ptr<osg::MatrixTransform> transform;
    std::list<CutPointcloud*> operators;
    boost::shared_ptr<envire::ExclusionBox> box;
};
    
class TabBoxManipulator
{
public:
    TabBoxManipulator(osg::Group* parentNode, std::list<CutPointcloud*> operators)
    {
        assert(parentNode);
        this->parentNode = parentNode;
        
        // create matrix transform
        transform = new osg::MatrixTransform;
        parentNode->addChild(transform);

        // set up dragger
        dragger = new osgManipulator::TabBoxDragger();
        dragger->setupDefaultGeometry();
        dragger->setActivationKeyEvent(osgGA::GUIEventAdapter::KEY_A);
        dragger->setHandleEvents(true);
        dragger->addTransformUpdating(transform);
        parentNode->addChild(dragger);
        
        callback = new TabBoxUpdate(dragger, transform, operators);
        dragger->addDraggerCallback(callback);
    }
    
    ~TabBoxManipulator()
    {
        callback->detachBox();
        dragger->removeDraggerCallback(callback);
        parentNode->removeChild(dragger);
        parentNode->removeChild(transform);
    }

private:
    osg::ref_ptr<osgManipulator::TabBoxDragger> dragger;
    osg::ref_ptr<osg::MatrixTransform> transform;
    osg::ref_ptr<TabBoxUpdate> callback;
    osg::Group* parentNode;
};

PointcloudManipulator::PointcloudManipulator(envire::EnvironmentItem* item, osg::Group* parentNode )
{
    this->parentNode = parentNode;
    pointcloud = dynamic_cast<envire::Pointcloud*>(item);

    assert(pointcloud && parentNode);
    
    std::list<Operator*> ops = pointcloud->getEnvironment()->getGenerators(pointcloud);
    // add operators
    for(std::list<Operator*>::const_iterator it = ops.begin(); it != ops.end(); it++)
    {
        CutPointcloud* cut_op = dynamic_cast<CutPointcloud*>(*it);
        if(cut_op)
            operators.push_back(cut_op);
    }
    
    // if no cut pointcloud operator could be found add one
    if(operators.size() == 0)
    {
        CutPointcloud* cut_op = new CutPointcloud();
        pointcloud->getEnvironment()->addInput(cut_op, pointcloud);
        pointcloud->getEnvironment()->addOutput(cut_op, pointcloud);
        operators.push_back(cut_op);
    }
}

PointcloudManipulator::~PointcloudManipulator()
{
    for(std::list< TabBoxManipulator* >::iterator it = manipulators.begin(); it != manipulators.end();)
    {
        TabBoxManipulator* manipulator = *it;
        it = manipulators.erase(it);
        delete manipulator;
    }
}

void PointcloudManipulator::addManipulationBox()
{
    TabBoxManipulator* manipulator = new TabBoxManipulator(parentNode, operators);
    manipulators.push_back(manipulator);
}

void PointcloudManipulator::removeManipulationBox()
{
    // TODO add parameters
    if(!manipulators.empty())
    {
        TabBoxManipulator* manipulator = manipulators.front();
        manipulators.pop_front();
        delete manipulator;
    }
}

}