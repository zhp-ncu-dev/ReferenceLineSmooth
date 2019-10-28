//
// Created by gaoyang on 18-12-3.
//
#include "common/smooth_line/smooth_spline/spline_seg_kernel.h"

namespace planning
{
    SplineSegKernel *SplineSegKernel::splineSegKernel_ = NULL;

    SplineSegKernel *SplineSegKernel::instance()
    {
        if(splineSegKernel_ == NULL)
        {
            splineSegKernel_ = new SplineSegKernel;
        }
        return splineSegKernel_;
    }

    void SplineSegKernel::destroy()
    {
        if(splineSegKernel_ != NULL)
        {
            delete splineSegKernel_;
        }
        splineSegKernel_ = NULL;
    }

    SplineSegKernel::SplineSegKernel() {
        const int reserved_num_params = reserved_order_ + 1;
        CalculateFx(reserved_num_params);
        CalculateDerivative(reserved_num_params);
        CalculateSecondOrderDerivative(reserved_num_params);
        CalculateThirdOrderDerivative(reserved_num_params);
    }

    Eigen::MatrixXd SplineSegKernel::NthDerivativeKernel(
            const uint32_t n, const uint32_t num_params, const double accumulated_x) {
        if (n == 1) {
            return DerivativeKernel(num_params, accumulated_x);
        } else if (n == 2) {
            return SecondOrderDerivativeKernel(num_params, accumulated_x);
        } else if (n == 3) {
            return ThirdOrderDerivativeKernel(num_params, accumulated_x);
        } else {
            return Eigen::MatrixXd::Zero(num_params, num_params);
        }
    }

    Eigen::MatrixXd SplineSegKernel::DerivativeKernel(const uint32_t num_params,
                                                      const double accumulated_x) {
        if (num_params > reserved_order_ + 1) {
            CalculateDerivative(num_params);
        }
        Eigen::MatrixXd term_matrix;
        IntegratedTermMatrix(num_params, accumulated_x, "derivative", &term_matrix);
        return kernel_derivative_.block(0, 0, num_params, num_params)
                .cwiseProduct(term_matrix);
    }

    Eigen::MatrixXd SplineSegKernel::SecondOrderDerivativeKernel(
            const uint32_t num_params, const double accumulated_x) {
        if (num_params > reserved_order_ + 1) {
            CalculateSecondOrderDerivative(num_params);
        }
        Eigen::MatrixXd term_matrix;
        IntegratedTermMatrix(num_params, accumulated_x, "second_order", &term_matrix);
        return kernel_second_order_derivative_.block(0, 0, num_params, num_params)
                .cwiseProduct(term_matrix);
    }

    Eigen::MatrixXd SplineSegKernel::ThirdOrderDerivativeKernel(
            const uint32_t num_params, const double accumulated_x) {
        if (num_params > reserved_order_ + 1) {
            CalculateThirdOrderDerivative(num_params);
        }
        Eigen::MatrixXd term_matrix;
        IntegratedTermMatrix(num_params, accumulated_x, "third_order", &term_matrix);
        return (kernel_third_order_derivative_.block(0, 0, num_params, num_params))
                .cwiseProduct(term_matrix);
    }

    void SplineSegKernel::CalculateFx(const uint32_t num_params) {
        kernel_fx_ = Eigen::MatrixXd::Zero(num_params, num_params);
        for (int r = 0; r < kernel_fx_.rows(); ++r) {
            for (int c = 0; c < kernel_fx_.cols(); ++c) {
                kernel_fx_(r, c) = 1.0 / (r + c + 1.0);
            }
        }
    }

    void SplineSegKernel::CalculateDerivative(const uint32_t num_params) {
        kernel_derivative_ = Eigen::MatrixXd::Zero(num_params, num_params);
        for (int r = 1; r < kernel_derivative_.rows(); ++r) {
            for (int c = 1; c < kernel_derivative_.cols(); ++c) {
                kernel_derivative_(r, c) = r * c / (r + c - 1.0);
            }
        }
    }

    void SplineSegKernel::CalculateSecondOrderDerivative(
            const uint32_t num_params) {
        kernel_second_order_derivative_ =
                Eigen::MatrixXd::Zero(num_params, num_params);
        for (int r = 2; r < kernel_second_order_derivative_.rows(); ++r) {
            for (int c = 2; c < kernel_second_order_derivative_.cols(); ++c) {
                kernel_second_order_derivative_(r, c) =
                        (r * r - r) * (c * c - c) / (r + c - 3.0);
            }
        }
    }

    void SplineSegKernel::CalculateThirdOrderDerivative(const uint32_t num_params) {
        kernel_third_order_derivative_ =
                Eigen::MatrixXd::Zero(num_params, num_params);
        for (int r = 3; r < kernel_third_order_derivative_.rows(); ++r) {
            for (int c = 3; c < kernel_third_order_derivative_.cols(); ++c) {
                kernel_third_order_derivative_(r, c) =
                        (r * r - r) * (r - 2) * (c * c - c) * (c - 2) / (r + c - 5.0);
            }
        }
    }

    void SplineSegKernel::IntegratedTermMatrix(const uint32_t num_params,
                                               const double x,
                                               const std::string& type,
                                               Eigen::MatrixXd* term_matrix) const {
        if (term_matrix->rows() != term_matrix->cols() ||
            term_matrix->rows() != static_cast<int>(num_params)) {
            term_matrix->resize(num_params, num_params);
        }

        std::vector<double> x_pow(2 * num_params + 1, 1.0);
        for (uint32_t i = 1; i < 2 * num_params + 1; ++i) {
            x_pow[i] = x_pow[i - 1] * x;
        }

        if (type == "fx") {
            for (uint32_t r = 0; r < num_params; ++r) {
                for (uint32_t c = 0; c < num_params; ++c) {
                    (*term_matrix)(r, c) = x_pow[r + c + 1] * kernel_fx_(r,c);
                }
            }

        } else if (type == "derivative") {
            for (uint32_t r = 1; r < num_params; ++r) {
                for (uint32_t c = 1; c < num_params; ++c) {
                    (*term_matrix)(r, c) = x_pow[r + c - 1] * kernel_derivative_(r , c);
                }
            }
            (*term_matrix).block(0, 0, num_params, 1) =
                    Eigen::MatrixXd::Zero(num_params, 1);
            (*term_matrix).block(0, 0, 1, num_params) =
                    Eigen::MatrixXd::Zero(1, num_params);

        } else if (type == "second_order") {
            for (uint32_t r = 2; r < num_params; ++r) {
                for (uint32_t c = 2; c < num_params; ++c) {
                    (*term_matrix)(r, c) = x_pow[r + c - 3] * kernel_second_order_derivative_(r,c);
                }
            }
            (*term_matrix).block(0, 0, num_params, 2) =
                    Eigen::MatrixXd::Zero(num_params, 2);
            (*term_matrix).block(0, 0, 2, num_params) =
                    Eigen::MatrixXd::Zero(2, num_params);

        } else {
            for (uint32_t r = 3; r < num_params; ++r) {
                for (uint32_t c = 3; c < num_params; ++c) {
                    (*term_matrix)(r, c) = x_pow[r + c - 5] * kernel_third_order_derivative_(r,c);
                }
            }
            (*term_matrix).block(0, 0, num_params, 3) =
                    Eigen::MatrixXd::Zero(num_params, 3);
            (*term_matrix).block(0, 0, 3, num_params) =
                    Eigen::MatrixXd::Zero(3, num_params);
        }
    }
}