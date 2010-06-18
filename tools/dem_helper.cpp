#include "icp/icp.hpp"
#include "envire/Core.hpp"
#include "envire/TriMesh.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

#include <boost/scoped_ptr.hpp>


using namespace std;

int main(int argc, const char* argv[])
{
    Eigen::Quaterniond q( Eigen::AngleAxisd(-0.15*M_PI, Eigen::Vector3d::UnitZ()) );
    cout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << endl;

    Eigen::Vector3d v(12, 85, 0);
    cout << (q*(v*.5) + Eigen::Vector3d(-9, -3, -2 )).transpose() << endl;


    envire::Serialization so;
    boost::scoped_ptr<envire::Environment> env(so.unserialize( argv[1] ));

    // update the operators
    env->updateOperators();

    ICP icp;

    // and load all the pointcloud data into the icp model
    std::vector<envire::TriMesh*> items = env->getItems<envire::TriMesh>();
    for(std::vector<envire::TriMesh*>::iterator it=items.begin();it!=items.end();it++)
    {
	icp.updateTree( *it, 1.0 );
    }


    //Eigen::Translation3d(-9,-3,-2)*;
}
