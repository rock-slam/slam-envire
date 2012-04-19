#ifndef __ENVIEW_ENVIREVISUALIZATION__
#define __ENVIEW_ENVIREVISUALIZATION__

#include <vizkit/Vizkit3DPlugin.hpp>
#include <osg/Geometry>
#include <envire/Core.hpp>
#include <envire/core/Serialization.hpp>

#include <boost/thread/recursive_mutex.hpp>
#include <vizkit/EnvireEventListener.hpp>
#include <vizkit/TreeViewListener.hpp>

class QTreeWidget;

namespace envire 
{

class EnvireVisualization : 
    public vizkit::Vizkit3DPlugin<envire::Environment*>
{
    Q_OBJECT

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EnvireVisualization();    
    ~EnvireVisualization();    

    void attachTreeWidget( QTreeWidget *treeWidget );
    bool isDirty() const;

    /** Load the environment from disk and display it */
    Q_INVOKABLE void load(std::string const& path);

    /** set to false if you want to manually handle the dirty flag by calling
     * setDirty() directly 
     */
    void handleDirty( bool handleDirty ) { m_handleDirty = handleDirty; }

    /** @return the currently selected item, or NULL if there is no selection
     */
    envire::EnvironmentItem* getSelectedItem() { return twl ? twl->selected : NULL; }

    void setFilter( envire::EventFilter *filter ) { eventListener->setFilter( filter ); }

    Q_INVOKABLE void updateBinaryEvent( envire::BinaryEvent const& data );
    Q_INVOKABLE void updateBinaryEvents( std::vector<envire::BinaryEvent> const& data );

protected:
    virtual void updateMainNode(osg::Node* node);
    virtual void updateDataIntern( envire::Environment* const& data );
    virtual osg::ref_ptr<osg::Node> createMainNode();

private:
    bool m_handleDirty;
    /** If true, the object pointed-to by @c env is owned by this object */
    bool m_ownsEnvironment;

    osg::ref_ptr<osg::Group> ownNode;
    envire::Environment *env;
    boost::recursive_mutex envLock;

    std::vector<boost::shared_ptr<EnvironmentItemVisualizer> > visualizers;

    boost::shared_ptr<EnvireEventListener> eventListener;
    boost::shared_ptr<TreeViewListener> twl;

    envire::BinarySerialization serialization;
};

}
#endif 
