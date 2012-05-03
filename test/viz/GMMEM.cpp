#define BOOST_TEST_MODULE GMMEMTest 
#include <boost/test/included/unit_test.hpp>

#include <envire/tools/ExpectationMaximization.hpp>

#include <boost/random/normal_distribution.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

#include <Eigen/QR>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

using namespace std;
using namespace envire;

#include <QtGui>

typedef GaussianMixture<double, 2> GMM;

class Particles : public QGraphicsItem
{
public:
    GMM &gmm;
    GaussianMixtureSampling<GMM> sampling;
    ExpectationMaximization<GMM> em;
    std::vector<GMM::Vector> particles;
    std::vector<GMM::Scalar> weights;

    Particles( GMM& gmm ) : gmm( gmm ), sampling( gmm ) 
    {
	for( int i=0; i<500; i++ )
	    particles.push_back( sampling.sample() );
	em.initialize( 3, particles );
	/*
	for( double x=-2.0; x<2.0; x+=0.1 )
	{
	    for( double y=-2.0; y<2.0; y+=0.1 )
	    {
		GMM::Vector v( x, y );
		particles.push_back( v );
		weights.push_back( gmm.eval(v) );
	    }
	}
	em.initialize( 2, particles, weights );
	*/

	update();
    }

    void advance( int step )
    {
	double res = em.step();
	std::cout << "step : " << res << std::endl;
	update();
    }

    QRectF boundingRect() const
    {
	return QRectF(-1000, -1000, 1000, 1000);
    }

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
    {
	painter->setBrush(Qt::black);
	for( size_t i=0; i<particles.size(); i++ )
	{
	    // draw particles
	    GMM::Vector &v( particles[i] );
	    painter->drawEllipse(QRectF( v.x()*100, v.y()*100, 2, 2));

	    // draw ellipses
	    for( size_t j=0; j<em.gmm.params.size(); j++ )
	    {
		GMM::Parameter &param( em.gmm.params[j] );

		Eigen::SelfAdjointEigenSolver<GMM::Matrix> e( param.dist.cov );
		for( int n=0; n<2; n++ )
		{
		    painter->save();
		    painter->setPen(Qt::red);
		    painter->setBrush(Qt::NoBrush);

		    painter->translate( param.dist.mean.x() * 100, param.dist.mean.y() * 100 );
		    Eigen::Rotation2D<GMM::Scalar> rm(0);
		    rm.fromRotationMatrix( e.eigenvectors() );
		    painter->rotate( rm.angle() * 180.0 / M_PI );

		    double sigma = 1.0;
		    painter->drawEllipse(
			    sqrt(e.eigenvalues()(0)) * -sigma * 100, 
			    sqrt(e.eigenvalues()(1)) * -sigma * 100,
			    sqrt(e.eigenvalues()(0)) * 2*sigma * 100, 
			    sqrt(e.eigenvalues()(1)) * 2*sigma * 100 );
		    painter->restore();
		}
	    }
	}
    }
};


BOOST_AUTO_TEST_CASE( qt_vis )
{
    GMM gmm;

    GMM::Vector mu;
    GMM::Matrix sigma;

    mu << 0.5, 0.1;
    sigma << 0.01, 0.01,
	     0.01,  .05;
    gmm.params.push_back( GMM::Parameter( 0.2, mu, sigma ) ); 

    mu << -0.3, 0.2;
    sigma << 0.01, -0.005,
	     -.0005, 0.0425;
    gmm.params.push_back( GMM::Parameter( 0.5, mu, sigma ) ); 

    mu << -0.1, 0.0;
    sigma << 0.08, -0.001,
	     -.0001, 0.0025;
    gmm.params.push_back( GMM::Parameter( 0.3, mu, sigma ) ); 

    int argc = 0;
    char* argv[0];

    QApplication app(argc, argv);
    QGraphicsScene scene;
    scene.setSceneRect(-300, -300, 600, 600);
    scene.setItemIndexMethod(QGraphicsScene::NoIndex);

    Particles *particles = new Particles( gmm );
    scene.addItem( particles );

    QGraphicsView view(&scene);
    view.setRenderHint(QPainter::Antialiasing);
    view.setCacheMode(QGraphicsView::CacheBackground);
    view.setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
    view.setDragMode(QGraphicsView::ScrollHandDrag);
    view.setWindowTitle(QT_TRANSLATE_NOOP(QGraphicsView, "Colliding Mice"));
    view.resize(400, 300);
    view.show();

    QTimer timer;
    QObject::connect(&timer, SIGNAL(timeout()), &scene, SLOT(advance()));
    timer.start(1000);

    app.exec();
}

BOOST_AUTO_TEST_CASE( eval )
{
    GMM gmm;

    GMM::Vector mu;
    GMM::Matrix sigma;

    mu << 0.5, 0.1;
    sigma << 0.01, 0.01,
	     0.01,  .05;
    gmm.params.push_back( GMM::Parameter( 0.3, mu, sigma ) ); 

    mu << -0.3, 0.2;
    sigma << 0.01, -0.005,
	     -.0005, 0.0425;
    gmm.params.push_back( GMM::Parameter( 0.7, mu, sigma ) ); 

    mu << -0.1, 0.0;
    sigma << 0.08, -0.005,
	     -.0005, 0.0425;
    gmm.params.push_back( GMM::Parameter( 0.7, mu, sigma ) ); 

    GaussianMixtureSampling<GMM> sampling(gmm);
    for( int i=0; i<500; i++ )
    {
	GMM::Vector s = sampling.sample();
	GMM::Scalar v = gmm.eval( s );
	if( false )
	    std::cout << s.transpose() << " " << v << std::endl;
    }
}
