#include "parameters.h"

using namespace Eigen;

int main(int argc, char **argv){
    ros::init(argc, argv, "cloud_projection");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
    while(ros::ok()){
        MatrixXd prev(3,50);
        MatrixXd cur(3,50);
        //build points
        for(int i = 0; i < prev.cols(); i++){
            prev(0,i) = rand()%1024;
            prev(1,i) = rand()%128;
            prev(2,i) = rand()%1024;
            cur(0,i) = -prev(0,i);
            cur(1,i) = -prev(1,i);
            cur(2,i) = prev(2,i)+1;
        }
        //create center of mass
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
        W = cur*prev.transpose();
        // std::cout << std::endl << W << std::endl;
        JacobiSVD<MatrixXd> svd(W, ComputeThinU | ComputeThinV);
        auto VT = svd.matrixV().transpose();
        MatrixXd R = svd.matrixU()*VT;
        MatrixXd t = mean_cur - R*mean_prev;
        std::cout << "The rotation matrix is: " << std::endl << R << std::endl;
        std::cout << "The translation vector is: " << std::endl << t << std::endl;
        // std::cout << svd.singularValues() << std::endl;

    }
    ros::spinOnce();
    loop_rate.sleep();
    
    return 0;
}