#include "icp.hpp"

USING_PART_OF_NAMESPACE_EIGEN

ICP::ICP() :
    rand_gen( 42u ),
    rand( rand_gen, boost::uniform_real<>(0,1) )
{
}

ICP::Result ICP::align( envire::TriMesh* measurement, int max_iter, double min_error)
{
    int n=0;
    double avg_error;

    // TODO come up with a real algorithm to estimate starting values
    double density = 0.1, threshold = 1.0;

    while(n<max_iter)
    {
	// update the model
	for(int i=0;i<modelVec.size();i++)
	{
	    updateTree( modelVec[i], density );
	}

	std::cout << std::endl;
	std::cout << "n:" << n << " dens:" << density << " thresh:" << threshold << std::endl;
	avg_error = updateAlignment( measurement, threshold, density );
	if( avg_error < min_error )
	    break;

	// TODO come up with a real algoritm for estimating
	// density and threshold
	//density = 1.0-(1.0-density)*.7;
	//threshold = threshold*.7;
	n++;
    }

    return ICP::Result( n, avg_error );
}


ICP::Result ICP::align( int max_iter, double min_error)
{
    throw std::runtime_error("not yet implemented");
}

void ICP::updateTree( envire::TriMesh* model, double density )
{
    // get the transformation between the local frame and the global
    // frame of the environment 
    envire::FrameNode::TransformType
	C_l2g = model->getEnvironment()->relativeTransform(
		model->getFrameNode(),
		model->getEnvironment()->getRootNode() );

    std::vector<Eigen::Vector3d>& points(model->vertices);
    std::vector<unsigned int>& attrs(model->getData<unsigned int>(envire::TriMesh::VERTEX_ATTRIBUTES));

    // insert the model into the tree
    for(int i=0;i<points.size();i++) {
        if( rand() <= density ) {
            kdtree.insert( 
		    TreeNode( 
			C_l2g * points[i], 
			attrs[i] & (1 << envire::TriMesh::SCAN_EDGE)
			) );
        }
    }
}

void ICP::addToModel( envire::TriMesh* model )
{
    modelVec.push_back( model );
}

void ICP::clearModel()
{
    modelVec.clear();
}

void ICP::clearTree()
{
    kdtree.clear();
}

double ICP::updateAlignment( envire::TriMesh* measurement, double threshold, double density )
{
    // for each point in the measurement, try to find a point in the model given
    // the current threshold and store the in the X and P
    //
    // ICP implementation based on the paper by Besl and McKay
    // Besl P, McKay H. A method for registration of 3-D shapes. IEEE Transactions on pattern... 1992. 
    // Available at: http://doi.ieeecomputersociety.org/10.1109/34.121791.
  
    x.clear();
    p.clear();

    // get the transformation between the local frame and the global
    // frame of the environment 
    envire::FrameNode::TransformType
	C_l2g = measurement->getEnvironment()->relativeTransform(
		measurement->getFrameNode(),
		measurement->getEnvironment()->getRootNode() );

    std::vector<Eigen::Vector3d>& points(measurement->vertices);
    std::vector<unsigned int>& attrs(measurement->getData<unsigned int>(envire::TriMesh::VERTEX_ATTRIBUTES));

    int stat_edges = 0;

    // find matching point pairs between measurement and model
    for(int i=0;i<points.size();i++) {
	if( rand() <= density ) {
	    TreeNode tn( 
		    C_l2g * points[i], 
		    attrs[i] & (1 << envire::TriMesh::SCAN_EDGE) );

	    std::pair<tree_type::const_iterator,double> found = kdtree.find_nearest(tn, threshold);
	    if( found.first != kdtree.end() )
	    {
		// really ignore anything to do with scan edges
		if(!found.first->edge && !tn.edge)
		{
		    x.push_back( found.first->point );
		    p.push_back( tn.point );
		}
		else
		{
		    stat_edges++;
		}
	    }
	}
    }

    int n = x.size();
    std::cout << "found pairs:" << n << " discarded edges:" << stat_edges << std::endl;

    Vector3d mu_p(Vector3d::Zero()), 
	     mu_x(Vector3d::Zero());
    Matrix3d sigma_px(Matrix3d::Zero());
    double mu_d = 0;

    // calculate the mean and covariance values of x and p
    for(int i=0;i<n;i++) {
	mu_d += (p[i] - x[i]).norm();

        mu_p += p[i];
        mu_x += x[i];

        sigma_px += p[i] * x[i].transpose();
   }
    mu_p /= static_cast<double>(n);
    mu_x /= static_cast<double>(n);
    mu_d /= static_cast<double>(n);

    sigma_px = sigma_px / static_cast<double>(n) - mu_p*mu_x.transpose();

    std::cout << "mu_d: " << mu_d << std::endl;
    //std::cout << "mu_p: " << mu_p << "mu_x:  " << mu_x << std::endl << "sigma_px: " << sigma_px << std::endl;

    // form the symmetric 4x4 matrix Q
    Matrix3d A = sigma_px-sigma_px.transpose();
    Vector3d delta = Vector3d( A(1,2), A(2,0), A(0,1) );

    Matrix4d q_px;
    q_px << sigma_px.trace(), delta.transpose(), 
         delta, A - Matrix3d::Identity() * sigma_px.trace();

    // do an eigenvalue decomposition of Q
    Eigen::Quaterniond q_R;
    Eigen::EigenSolver<Matrix4d> eigenSolver(q_px);
    double max_eig = eigenSolver.eigenvalues().real().maxCoeff();
    for(int i=0;i<eigenSolver.eigenvalues().rows();i++) {
        if(eigenSolver.eigenvalues()(i) == max_eig) {
            Vector4d max_eigv = eigenSolver.eigenvectors().col(i).real();
            q_R = Eigen::Quaterniond( max_eigv(0), max_eigv(1), max_eigv(2), max_eigv(3) );
        }
    }

    // resulting transformation in global frame
    Eigen::Vector3d q_T = mu_x - q_R * mu_p;
    Eigen::Transform3d t;
    t = Eigen::Translation3d( q_T );
    t *= q_R;

    // TODO: find the framenode that should be updated really as this might not
    // be necessarily the framenode associated with the trimesh
    envire::FrameNode* fm = measurement->getFrameNode();

    // get the transformation to the root framenode
    envire::FrameNode::TransformType
	C_fm2g = measurement->getEnvironment()->relativeTransform(
		fm,
		measurement->getEnvironment()->getRootNode() );

    // TODO check this
    fm->setTransform( C_fm2g * fm->getTransform() );

    // TODO return a real measurement of quality here
    return mu_d;
}

