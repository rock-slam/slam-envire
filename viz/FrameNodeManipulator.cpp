#include "FrameNodeManipulator.hpp"


vizkit::FrameNodeManipulator::FrameNodeManipulator(envire::EnvironmentItem* item, osg::Group* group)
{
    fr = dynamic_cast<envire::FrameNode *>(item);
    transform = dynamic_cast<osg::PositionAttitudeTransform *>(group);

    osg::Group *pGroup = transform->getParent(0)->asGroup();
    
    selectionForFrameNode = new vizkit::FrameNodeSelection(fr);
    pGroup->addChild(selectionForFrameNode.get());
    
    translateDragger = new osgManipulator::TranslateAxisDragger();
    translateDragger->setupDefaultGeometry();

    rotateDragger = new osgManipulator::TrackballDragger();
    rotateDragger->setupDefaultGeometry();

    selectionForTrackball = new osgManipulator::Selection();
    pGroup->addChild(selectionForTrackball.get());

    selectionForTrackball->addChild(rotateDragger.get());
    pGroup->addChild(translateDragger.get());
    
    // Starting matrix for the Dragger 
    //float scale = transform->getBound().radius() * 1.5f;
    float scale = 1.5;
    
    Eigen::Vector3d framePos = fr->getTransform().translation();
    osg::Matrix mat = osg::Matrix::scale(scale, scale, scale) * osg::Matrix::translate(osg::Vec3d(framePos.x(), framePos.y(), framePos.z()));
    //dragger->setMatrix(mat);
    translateDragger->setMatrix(mat);
    scale = 1;
    //scale = transform->getBound().radius();
    mat = osg::Matrix::scale(scale, scale, scale) * osg::Matrix::translate(osg::Vec3d(framePos.x(), framePos.y(), framePos.z()));
    rotateDragger->setMatrix(mat);
    
    cm = new osgManipulator::CommandManager();
    
    // Command Manager - connects Dragger objects with Selection objects 
    cm->connect(*(translateDragger.get()), *(selectionForFrameNode.get()));
    cm->connect(*(rotateDragger.get()), *(selectionForFrameNode.get()));
    cm->connect(*(translateDragger.get()), *(selectionForTrackball.get())); 
}

vizkit::FrameNodeManipulator::~FrameNodeManipulator()
{
    osg::Group *pGroup = transform->getParent(0)->asGroup();

    cm->disconnect(*(translateDragger.get()));
    cm->disconnect(*(rotateDragger.get()));
    cm->disconnect(*(translateDragger.get())); 

    selectionForTrackball->removeChild(rotateDragger);
    pGroup->removeChild(rotateDragger);
    pGroup->removeChild(translateDragger);
    pGroup->removeChild(selectionForTrackball);
    pGroup->removeChild(selectionForFrameNode);
    
}
