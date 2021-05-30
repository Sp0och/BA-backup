#include "../include/parameters.h"

using namespace Eigen;

int main(int argc, char **argv){
    ros::init(argc, argv, "cloud_projection");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
    while(ros::ok()){
        MatrixXd prev(3,50);
        MatrixXd cur(3,50);
        Matrix3d transform_R;
         transform_R << 0.511853,  0.806518,  0.295864,
        0.754284, -0.257086, -0.604121,
        0.411172, -0.532387,  0.739934;
        Vector3d transform_t(0,0,1);
        Matrix4d transform;
        transform.setIdentity();
        transform.block<3,3>(0,0) = transform_R;
        transform.block<3,1>(0,3) = transform_t;
        //build points
        for(int i = 0; i < prev.cols(); i++){
            Vector4d storage;
            storage(3) = 1;
            prev(0,i) = rand()%1024;
            prev(1,i) = rand()%128;
            prev(2,i) = rand()%1024;
            storage(0) = prev(0,i);
            storage(1) = prev(1,i);
            storage(2) = prev(2,i);
            storage = transform * storage;
            cur(0,i) = storage(0);
            cur(1,i) = storage(1);
            cur(2,i) = storage(2);
        }

        //points initialized correctly

        //create sum of points
        Vector3d sum_prev(0,0,0);
        Vector3d sum_cur(0,0,0);
        for(int i = 0;i < prev.cols();i++){
            sum_prev(0) += prev(0,i);
            sum_prev(1) += prev(1,i);
            sum_prev(2) += prev(2,i);
            sum_cur(0) += cur(0,i);
            sum_cur(1) += cur(1,i);
            sum_cur(2) += cur(2,i);
        }
        //create center of points and subtract it from both sets
        Vector3d mean_prev = sum_prev/prev.cols();
        Vector3d mean_cur = sum_cur/cur.cols();        
        // std::cout << std::endl << prev << std::endl;
        for(int i = 0; i < prev.cols();i++){
            prev(0,i) -= mean_prev(0);
            prev(1,i) -= mean_prev(1);
            prev(2,i) -= mean_prev(2);
            cur(0,i) -= mean_cur(0);
            cur(1,i) -= mean_cur(1);
            cur(2,i) -= mean_cur(2);
        }
        // std::cout << std::endl << mean_prev << std::endl;
        // std::cout << std::endl << prev << std::endl;
        MatrixXd W;
        W.resize(3,3);
        W.setZero();
        //W is of rank 3, that I checked.
        for(int i = 0; i < prev.cols();i++){
            Vector3d prevp(prev(0,i),prev(1,i),prev(2,i));
            Vector3d curp(cur(0,i),cur(1,i),cur(2,i));
            auto CPT = curp.transpose();
            W += prevp*CPT;
        }
        // W = cur*prev.transpose();
        // std::cout << std::endl << W << std::endl;
        JacobiSVD<MatrixXd> svd(W, ComputeThinU | ComputeThinV);
        auto VT = svd.matrixV().transpose();
        Matrix3d R = svd.matrixU()*VT;
        Vector3d t = mean_prev - R*mean_cur;
        std::cout << "The rotation matrix is: " << std::endl << R << std::endl;
        std::cout << "The translation vector is: " << std::endl << t << std::endl;
        // std::cout << svd.singularValues() << std::endl;

    }
    ros::spinOnce();
    loop_rate.sleep();
    
    return 0;
}