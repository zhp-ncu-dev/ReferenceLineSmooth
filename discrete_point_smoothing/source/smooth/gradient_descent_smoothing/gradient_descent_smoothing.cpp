#include "smooth/gradient_descent_smooth/gradient_descent_smoothing.h"
using namespace HybridAStar;

//###################################################
//                                SMOOTHING ALGORITHM
//###################################################
void Smoother::smoothPath() {

    // current number of iterations of the gradient descent smoother
    int iterations = 0;
    // the maximum iterations for the gd smoother
    int maxIterations = 1000;
    // the lenght of the path in number of nodes
    int pathLength = 0;

    // path objects with all nodes oldPath the original, newPath the resulting smoothed path
    pathLength = path.size();
    std::vector<ADC::planning::dPoint3d> newPath = path;

    // descent along the gradient untill the maximum number of iterations has been reached
    double totalWeight = wSmoothness + wCurvature ;
    //迭代 500 次，使得优化解稳定
    while (iterations < maxIterations) {

        // choose the first three nodes of the path
        for (int i = 2; i < pathLength - 2; ++i) {

            // 以 5 个点优化中间的点，即 i-2 , i-1 , i , i+1 , i+2 优化 i
            Vector2D xim2(newPath[i - 2].getX(), newPath[i - 2].getY());
            Vector2D xim1(newPath[i - 1].getX(), newPath[i - 1].getY());

            Vector2D xi(newPath[i].getX(), newPath[i].getY());

            Vector2D xip1(newPath[i + 1].getX(), newPath[i + 1].getY());
            Vector2D xip2(newPath[i + 2].getX(), newPath[i + 2].getY());
            Vector2D correction;

            correction = correction - smoothnessTerm(xim2, xim1, xi, xip1, xip2);
            correction = correction - curvatureTerm(xim1, xi, xip1);

            // 存储平滑后的 path
            xi = xi + alpha * correction; // /totalWeight;

            newPath[i].setX(xi.getX());
            newPath[i].setY(xi.getY());
            Vector2D Dxi = xi - xim1;
            newPath[i - 1].setT(std::atan2(Dxi.getY(), Dxi.getX()));
        }

        iterations++;
    }

    path = newPath;
}

// 将 Hybrid-A* 求出的路径传输至 Smoother 中 【迭代法】
void Smoother::tracePath(const std::vector<ADC::planning::dPoint3d>& node) {

    path.clear();
    int iNum = node.size();
    path.reserve(iNum);       // 分配空间，防止数据存放错误
    for(int i = 0; i < iNum; ++i){
        path.push_back(node[i]);
    }
}

//###################################################
//                                     CURVATURE TERM
//###################################################
// 曲率梯度 计算
Vector2D Smoother::curvatureTerm(Vector2D xim1, Vector2D xi, Vector2D xip1) {
    Vector2D gradient;
    // the vectors between the nodes
    Vector2D Dxi = xi - xim1;
    Vector2D Dxip1 = xip1 - xi;
    // orthogonal complements vector
    Vector2D p1, p2;

    // the distance of the vectors
    double absDxi = Dxi.length();
    double absDxip1 = Dxip1.length();

    // ensure that the absolute values are not null
    if (absDxi > 0 && absDxip1 > 0) {

        // the angular change at the node
        // TODO ： 此处有错误，与原版paper不一样
        Vector2D Dxi_T;
        Dxi_T.setX(Dxi.getX());
        Dxi_T.setY(Dxi.getY());
        double Dphi = std::acos(Helper::clamp(Dxi_T.dot(Dxip1) / (absDxi * absDxip1), -1, 1));

        //double Dphi = std::acos(Helper::clamp(Dxi.dot(Dxip1) / (absDxi * absDxip1), -1, 1));

        double kappa = Dphi / absDxi;

        //std::cout << "kappa = " << kappa << std::endl;

        // if the curvature is smaller then the maximum do nothing
        if (kappa <= kappaMax) {
            Vector2D zeros;
            return zeros;
        } else {
            double absDxi1Inv = 1 / absDxi;
            double PDphi_PcosDphi = -1 / std::sqrt(1 - std::pow(std::cos(Dphi), 2));
            double u = -absDxi1Inv * PDphi_PcosDphi;
            // calculate the p1 and p2 terms
            p1 = xi.ort(-xip1) / (absDxi * absDxip1);
            p2 = -xip1.ort(xi) / (absDxi * absDxip1);
            // calculate the last terms
            double s = Dphi / (absDxi * absDxi);
            Vector2D ones(1, 1);
            Vector2D ki = u * (-p1 - p2) - (s * ones);
            Vector2D kim1 = u * p2 - (s * ones);
            Vector2D kip1 = u * p1;

            // calculate the gradient
            //gradient = wCurvature * (0.25 * kim1 + 0.5 * ki + 0.25 * kip1);
            gradient =  ( 0.25*kim1 +  0.5*ki +  0.25*kip1);

            if (std::isnan(gradient.getX()) || std::isnan(gradient.getY())) {
                std::cout << "nan values in curvature term" << std::endl;
                Vector2D zeros;
                return zeros;
            }
                // return gradient of 0
            else {
                return gradient;
            }
        }
    }
        // return gradient of 0
    else {
        //std::cout << "abs values not larger than 0" << std::endl;
        Vector2D zeros;
        return zeros;
    }
}

//###################################################
//                                    SMOOTHNESS TERM
//###################################################
Vector2D Smoother::smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2) {
    return 1.0 * (xim2 - 4 * xim1 + 6 * xi - 4 * xip1 + xip2);
}
