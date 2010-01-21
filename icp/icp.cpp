#include "icp.hpp"

USING_PART_OF_NAMESPACE_EIGEN

ICP::ICP( const std::vector<Eigen::Vector3f>& model, 
        const std::vector<Eigen::Vector3f>& measurement, 
        const Eigen::Transform3f alignment,
        float threshold, int density ) : 
    model(model), measurement(measurement), t(alignment), threshold(threshold), density(density)  {

    // initialise seed
    srand ( time(NULL) );

    // insert the model into the tree
    std::cout << "model size: " << model.size() << "     " << std::endl;
    for(int i=0;i<model.size();i++) {
        if( rand() % density == 0 ) {
            kdtree.insert( model[i] );
        }
    }
    //    std::cout << std::endl << "building tree done " << std::endl;
    //    std::cout << "optimising tree" << std::endl;
    //    std::vector<Vector3f> model_copy(model);
    //    kdtree.efficient_replace_and_optimise( model_copy );
    //    std::cout << "optimising tree done" << std::endl;
}

float ICP::iterate() {
    // for each point in the measurement, try to find a point in the model given
    // the current threshold and store the in the X and P
    //
    // ICP implementation based on the paper by Besl and McKay
    // Besl P, McKay H. A method for registration of 3-D shapes. IEEE Transactions on pattern... 1992. 
    // Available at: http://doi.ieeecomputersociety.org/10.1109/34.121791.
  
//    std::vector<Vector3f> x, p;
    x.clear();
    p.clear();

    std::cout << t.matrix() << std::endl;
    std::cout << "start find pairs " << measurement.size() << " threshold: " << threshold << std::endl;
    for(int i=0;i<measurement.size();i+=1) {
        if( rand() % density == 0 ) {
            Vector3f point = t * measurement[i];
            std::pair<tree_type::const_iterator,float> found = kdtree.find_nearest(point, threshold);

            if( found.first != kdtree.end() ) { 
                x.push_back( *found.first );
                p.push_back( point );
            }
        }

        if( i%1000 == 0 ) {
            std::cout << "\rfind pairs " << i << "     " << std::flush;
        }
    }

    int n = x.size();
    std::cout << std::endl << "found pairs " << n << std::endl;

    Vector3f mu_p(Vector3f::Zero()), mu_x(Vector3f::Zero());
    Matrix3f sigma_px(Matrix3f::Zero());

    // calculate the mean and covariance values of x and p
    for(int i=0;i<n;i++) {
        mu_p += p[i];
        mu_x += x[i];

        sigma_px += p[i] * x[i].transpose();
   }
    mu_p /= static_cast<float>(n);
    mu_x /= static_cast<float>(n);

    sigma_px = sigma_px / static_cast<float>(n) - mu_p*mu_x.transpose();

    std::cout << "mu_p: " << mu_p << "mu_x:  " << mu_x << std::endl << "sigma_px: " << sigma_px << std::endl;

    // form the symmetric 4x4 matrix Q
    Matrix3f A = sigma_px-sigma_px.transpose();
    Vector3f delta = Vector3f( A(1,2), A(2,0), A(0,1) );

    Matrix4f q_px;
    q_px << sigma_px.trace(), delta.transpose(), 
         delta, A - Matrix3f::Identity() * sigma_px.trace();

    // do an eigenvalue decomposition of Q
    Eigen::Quaternionf q_R;
    Eigen::EigenSolver<Matrix4f> eigenSolver(q_px);
    float max_eig = eigenSolver.eigenvalues().real().maxCoeff();
    for(int i=0;i<eigenSolver.eigenvalues().rows();i++) {
        if(eigenSolver.eigenvalues()(i) == max_eig) {
            Vector4f max_eigv = eigenSolver.eigenvectors().col(i).real();
            q_R = Eigen::Quaternionf( max_eigv(0), max_eigv(1), max_eigv(2), max_eigv(3) );
        }
    }

    Eigen::Vector3f q_T = mu_x - q_R * mu_p;

    t = Eigen::Translation3f( q_T );
    t *= q_R;
}

