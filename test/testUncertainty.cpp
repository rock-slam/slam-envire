#include <Eigen/Geometry>
#include <boost/scoped_ptr.hpp>

#include <vizkit/QVizkitWidget.hpp>
#include <vizkit/QtThreadedWidget.hpp>
#include "EnvireVisualization.hpp"
#include "maps/MultiLevelSurfaceGrid.hpp"

#include "Core.hpp"
#include "Uncertainty.hpp"

using namespace envire;

namespace vizkit
{

class UncertaintyVisualization : public VizPlugin<envire::PointWithUncertainty>
{
public:
    osg::ref_ptr<osg::Node> createMainNode()
    {
	m_uncertainty = new Uncertainty();
	m_uncertainty->setNumSamples( 10000 );
	return m_uncertainty;
    }

    void updateMainNode(osg::Node* node)
    {
	m_uncertainty->setMean( point.getPoint() );
	m_uncertainty->setCovariance( point.getCovariance() );
    }

    void updateDataIntern(const envire::PointWithUncertainty &point)
    {
	this->point = point;
    }

private:
    envire::PointWithUncertainty point;
    osg::ref_ptr<Uncertainty> m_uncertainty;
};

class QEnvireTestWidget : public QVizkitWidget
{
    public:
	QEnvireTestWidget( QWidget* parent = 0, Qt::WindowFlags f = 0 )
	    : QVizkitWidget( parent, f ), viz( new UncertaintyVisualization() )
	{
	    addDataHandler( viz.get() );
	}

	~QEnvireTestWidget()
	{
	    removeDataHandler( viz.get() );
	}

	void setData( const envire::PointWithUncertainty &point )
	{
	    viz->updateData( point );
	}

    private:
	boost::shared_ptr<UncertaintyVisualization> viz;
};
}

int main( int argc, char **argv )
{
    QtThreadedWidget<vizkit::QEnvireTestWidget> app;
    app.start();

    for(int i=0;i<5000 && app.isRunning();i++)
    {
	double r = i/5000.0;
	PointWithUncertainty p(
		Eigen::Vector3d( 0.0, 0, 0 ),
		Eigen::Vector3d( 0.01, 0.01, 0.01 ).cwise().square().asDiagonal() );
	
	Eigen::Matrix<double,6,6> lt1; 
	lt1 <<
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	TransformWithUncertainty t1(
		Eigen::Transform3d(Eigen::Translation3d(Eigen::Vector3d(r,0,0))),
		lt1.cwise().square() );

	Eigen::Matrix<double,6,6> lt2; 
	lt2 <<
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	    0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	TransformWithUncertainty t2(
		Eigen::Transform3d(Eigen::Transform3d::Identity()),
		lt2.cwise().square() );

	PointWithUncertainty pp = t2 * t1 * p;

	app.widget->setData( pp );

	usleep(1000);
    }
}

