//
// Created by wangzihua on 19-8-5.
//

#include <limits>
#include "common/smooth_line/smooth_spline/piecewise_linear_constraint.h"

namespace planning{
    namespace {

        Eigen::MatrixXd MergeMaxtrices(const std::vector<Eigen::MatrixXd>& matrices) {
            if (matrices.size() == 0) {
                return Eigen::MatrixXd::Zero(1, 2);
            }
            int32_t d = 0;
            for (const auto& mat : matrices) {
                d += mat.rows();
            }
            int32_t col = matrices.front().cols();
            Eigen::MatrixXd res = Eigen::MatrixXd::Zero(d, col);
            int32_t index = 0;
            for (const auto& mat : matrices) {
                res.block(index, 0, mat.rows(), mat.cols()) = mat;
                index += mat.rows();
            }
            return res;
        }
    }

    PiecewiseLinearConstraint::PiecewiseLinearConstraint(const uint32_t dimension,
                                                         const double unit_segment)
            : dimension_(dimension), unit_segment_(unit_segment) {}

    Eigen::MatrixXd PiecewiseLinearConstraint::inequality_constraint_matrix()
    const {
        return MergeMaxtrices(inequality_matrices_);
    }

    Eigen::MatrixXd PiecewiseLinearConstraint::inequality_constraint_boundary()
    const {
        return MergeMaxtrices(inequality_boundaries_);
    }

    Eigen::MatrixXd PiecewiseLinearConstraint::equality_constraint_matrix() const {
        return MergeMaxtrices(equality_matrices_);
    }

    Eigen::MatrixXd PiecewiseLinearConstraint::equality_constraint_boundary()
    const {
        return MergeMaxtrices(equality_boundaries_);
    }

    bool PiecewiseLinearConstraint::AddBoundary(
            const std::vector<uint32_t>& index_list,
            const std::vector<double>& lower_bound,
            const std::vector<double>& upper_bound) {
        if (index_list.size() != lower_bound.size() ||
            index_list.size() != upper_bound.size()) {
            return false;
        }
        Eigen::MatrixXd inequality_matrix =
                Eigen::MatrixXd::Zero(2 * index_list.size(), dimension_);
        Eigen::MatrixXd inequality_boundary =
                Eigen::MatrixXd::Zero(2 * index_list.size(), 1);

        for (uint32_t i = 0; i < index_list.size(); ++i) {
            uint32_t index = index_list[i];

            // x(i) < upper_bound[i] ==> -x(i) > -upper_bound[i]
            inequality_matrix(2 * i, index) = -1.0;
            inequality_boundary(2 * i, 0) = -upper_bound[i];

            // x(i) > lower_bound[i]
            inequality_matrix(2 * i + 1, index) = 1.0;
            inequality_boundary(2 * i + 1, 0) = lower_bound[i];
        }
        inequality_matrices_.push_back(inequality_matrix);
        inequality_boundaries_.push_back(inequality_boundary);
        return true;
    }

    bool PiecewiseLinearConstraint::AddDerivativeBoundary(
            const std::vector<uint32_t>& index_list,
            const std::vector<double>& lower_bound,
            const std::vector<double>& upper_bound) {
        if (index_list.size() != lower_bound.size() ||
            index_list.size() != upper_bound.size()) {
            return false;
        }
        Eigen::MatrixXd inequality_matrix =
                Eigen::MatrixXd::Zero(2 * index_list.size(), dimension_);
        Eigen::MatrixXd inequality_boundary =
                Eigen::MatrixXd::Zero(2 * index_list.size(), 1);

        for (uint32_t i = 0; i < index_list.size(); ++i) {
            uint32_t index = index_list[i];

            if (index > 0) {
                inequality_matrix(2 * i, index - 1) = 1.0;
                inequality_matrix(2 * i + 1, index - 1) = -1.0;
            }
            inequality_matrix(2 * i, index) = -1.0;
            inequality_boundary(2 * i, 0) = -unit_segment_ * upper_bound[i];

            inequality_matrix(2 * i + 1, index) = 1.0;
            inequality_boundary(2 * i + 1, 0) = unit_segment_ * lower_bound[i];
        }
        inequality_matrices_.push_back(inequality_matrix);
        inequality_boundaries_.push_back(inequality_boundary);
        return true;
    }

    bool PiecewiseLinearConstraint::AddSecondDerivativeBoundary(
            const double init_derivative, const std::vector<uint32_t>& index_list,
            const std::vector<double>& lower_bound,
            const std::vector<double>& upper_bound) {
        if (index_list.size() != lower_bound.size() ||
            index_list.size() != upper_bound.size()) {
            return false;
        }
        Eigen::MatrixXd inequality_matrix =
                Eigen::MatrixXd::Zero(2 * index_list.size(), dimension_);
        Eigen::MatrixXd inequality_boundary =
                Eigen::MatrixXd::Zero(2 * index_list.size(), 1);

        for (uint32_t i = 0; i < index_list.size(); ++i) {
            uint32_t index = index_list[i];
            const double upper = upper_bound[i];
            const double lower = lower_bound[i];

            if (index == 0) {
                inequality_matrix(2 * i, 0) = -1.0;
                inequality_boundary(2 * i, 0) = -(upper * unit_segment_ * unit_segment_ +
                                                  init_derivative * unit_segment_);
                inequality_matrix(2 * i + 1, 0) = 1.0;
                inequality_boundary(2 * i + 1, 0) =
                        lower * unit_segment_ * unit_segment_ +
                        init_derivative * unit_segment_;
            } else if (index == 1) {
                inequality_matrix(2 * i, 0) = 2.0;
                inequality_matrix(2 * i, 1) = -1.0;
                inequality_boundary(2 * i, 0) = -upper * unit_segment_ * unit_segment_;

                inequality_matrix(2 * i + 1, 0) = -2.0;
                inequality_matrix(2 * i + 1, 1) = 1.0;
                inequality_boundary(2 * i + 1, 0) = lower * unit_segment_ * unit_segment_;
            } else {
                inequality_matrix(2 * i, index - 2) = -1.0;
                inequality_matrix(2 * i, index - 1) = 2.0;
                inequality_matrix(2 * i, index) = -1.0;
                inequality_boundary(2 * i, 0) = -upper * unit_segment_ * unit_segment_;

                inequality_matrix(2 * i + 1, index - 2) = 1.0;
                inequality_matrix(2 * i + 1, index - 1) = -2.0;
                inequality_matrix(2 * i + 1, index) = 1.0;
                inequality_boundary(2 * i + 1, 0) = lower * unit_segment_ * unit_segment_;
            }
        }
        inequality_matrices_.push_back(inequality_matrix);
        inequality_boundaries_.push_back(inequality_boundary);
        return true;
    }

    bool PiecewiseLinearConstraint::AddPointConstraint(const uint32_t index,
                                                       const double val) {
        Eigen::MatrixXd equality_matrix = Eigen::MatrixXd::Zero(1, dimension_);
        Eigen::MatrixXd equality_boundary = Eigen::MatrixXd::Zero(1, 1);

        equality_matrix(0, index) = 1.0;
        equality_boundary(0, 0) = val;

        inequality_matrices_.push_back(equality_matrix);
        inequality_boundaries_.push_back(equality_boundary);
        return true;
    }

    bool PiecewiseLinearConstraint::AddPointDerivativeConstraint(
            const uint32_t index, const double val) {
        Eigen::MatrixXd equality_matrix = Eigen::MatrixXd::Zero(1, dimension_);
        Eigen::MatrixXd equality_boundary = Eigen::MatrixXd::Zero(1, 1);

        equality_matrix(0, index) = -1.0;
        equality_matrix(0, index + 1) = 1.0;
        equality_boundary(0, 0) = val;

        equality_matrices_.push_back(equality_matrix);
        equality_boundaries_.push_back(equality_boundary);

        return true;
    }

    bool PiecewiseLinearConstraint::AddMonotoneInequalityConstraint() {
        Eigen::MatrixXd inequality_matrix =
                Eigen::MatrixXd::Zero(dimension_, dimension_);
        Eigen::MatrixXd inequality_boundary = Eigen::MatrixXd::Zero(dimension_, 1);

        for (uint32_t i = 0; i < dimension_; ++i) {
            if (i == 0) {
                inequality_matrix(0, 0) = 1.0;
            } else {
                inequality_matrix(i, i - 1) = -1.0;
                inequality_matrix(i, i) = 1.0;
            }
        }
        inequality_matrices_.push_back(inequality_matrix);
        inequality_boundaries_.push_back(inequality_boundary);

        return true;
    }
}
