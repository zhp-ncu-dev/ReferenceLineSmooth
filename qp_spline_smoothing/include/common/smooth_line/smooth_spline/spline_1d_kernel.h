//
// Created by wangzihua on 19-8-5.
//

#ifndef PLANNING_SPLINE_1D_KERNEL_H
#define PLANNING_SPLINE_1D_KERNEL_H

#include "spline_1d.h"
#include "spline_seg_kernel.h"

namespace planning{
    class Spline1dKernel{
    public:
        explicit Spline1dKernel(const Spline1d& spline1d);
        Spline1dKernel(const std::vector<double>& x_knots,
                       const uint32_t spline_order);

        // customized input / output method
        void AddRegularization(const double regularized_param);
        bool AddKernel(const Eigen::MatrixXd& kernel, const Eigen::MatrixXd& offset,
                       const double weight);
        bool AddKernel(const Eigen::MatrixXd& kernel, const double weight);

        Eigen::MatrixXd* mutable_kernel_matrix();
        Eigen::MatrixXd* mutable_offset();

        const Eigen::MatrixXd& kernel_matrix() const;
        const Eigen::MatrixXd& offset() const;

        // build-in kernel methods
        void AddDerivativeKernelMatrix(const double weight);
        void AddSecondOrderDerivativeMatrix(const double weight);
        void AddThirdOrderDerivativeMatrix(const double weight);
        void AddDerivativeKernelMatrixForSplineK(const uint32_t k,
                                                 const double weight);
        void AddSecondOrderDerivativeMatrixForSplineK(const uint32_t k,
                                                      const double weight);
        void AddThirdOrderDerivativeMatrixForSplineK(const uint32_t k,
                                                     const double weight);

        // reference line kernel, x_coord in strictly increasing order (for path
        // optimizer)
        bool AddReferenceLineKernelMatrix(const std::vector<double>& x_coord,
                                          const std::vector<double>& ref_fx,
                                          const double weight);

        // reference line kernel, x_coord in strictly increasing order (for path
        // optimizer)
        bool addSecDerivativeReferenceLineKernelMatrix(const std::vector<double>& x_coord,
                                                      const std::vector<double>& ref_fx,
                                                      const double weight);
        // distance offset (for speed optimizer, given time optimize the distance can
        // go)
        void AddDistanceOffset(const double weight);

    private:
        void AddNthDerivativekernelMatrix(const uint32_t n, const double weight);
        void AddNthDerivativekernelMatrixForSplineK(const uint32_t n,
                                                    const uint32_t k,
                                                    const double weight);
        uint32_t FindIndex(const double x) const;

    private:
        Eigen::MatrixXd kernel_matrix_;
        Eigen::MatrixXd offset_;
        std::vector<double> x_knots_;
        uint32_t spline_order_;
        uint32_t total_params_;
    };
}

#endif //PLANNING_SPLINE_1D_KERNEL_H
