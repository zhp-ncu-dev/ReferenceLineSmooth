//
// Created by wangzihua on 19-8-5.
//

#ifndef PLANNING_SPLINE_1D_CONSTRAINT_H
#define PLANNING_SPLINE_1D_CONSTRAINT_H

#include <algorithm>
#include <functional>
#include <vector>

#include "spline_1d.h"
#include "affine_constraint.h"

namespace planning{
    class Spline1dConstraint{
    public:
        explicit Spline1dConstraint(const Spline1d& pss);
        Spline1dConstraint(const std::vector<double>& x_knots, const uint32_t order);

        // direct methods
        bool AddInequalityConstraint(const Eigen::MatrixXd& constraint_matrix,
                                     const Eigen::MatrixXd& constraint_boundary);
        bool AddEqualityConstraint(const Eigen::MatrixXd& constraint_matrix,
                                   const Eigen::MatrixXd& constraint_boundary);

        // preset method
        /**
         * @brief: inequality boundary constraints
         * if no boundary, do specify either by std::infinity or
         * let vector.size() = 0
         **/
        bool AddBoundary(const std::vector<double>& x_coord,
                         const std::vector<double>& lower_bound,
                         const std::vector<double>& upper_bound);

        bool AddDerivativeBoundary(const std::vector<double>& x_coord,
                                   const std::vector<double>& lower_bound,
                                   const std::vector<double>& upper_bound);

        bool AddSecondDerivativeBoundary(const std::vector<double>& x_coord,
                                         const std::vector<double>& lower_bound,
                                         const std::vector<double>& upper_bound);

        bool AddThirdDerivativeBoundary(const std::vector<double>& x_coord,
                                        const std::vector<double>& lower_bound,
                                        const std::vector<double>& upper_bound);

        /**
         * @brief: equality constraint to guarantee joint smoothness
         * boundary equality constriant constraint on fx, dfx, ddfx ... in vector
         * form; upto third order
         **/
        bool AddPointConstraint(const double x, const double fx);
        bool AddPointDerivativeConstraint(const double x, const double dfx);
        bool AddPointSecondDerivativeConstraint(const double x, const double ddfx);
        bool AddPointThirdDerivativeConstraint(const double x, const double dddfx);

        bool AddPointConstraintInRange(const double x, const double fx,
                                       const double range);
        bool AddPointDerivativeConstraintInRange(const double x, const double dfx,
                                                 const double range);
        bool AddPointSecondDerivativeConstraintInRange(const double x,
                                                       const double ddfx,
                                                       const double range);
        bool AddPointThirdDerivativeConstraintInRange(const double x,
                                                      const double dddfx,
                                                      const double range);
        // guarantee upto values are joint
        bool AddSmoothConstraint();

        // guarantee upto derivative are joint
        bool AddDerivativeSmoothConstraint();

        // guarantee upto second order derivative are joint
        bool AddSecondDerivativeSmoothConstraint();

        // guarantee upto third order derivative are joint
        bool AddThirdDerivativeSmoothConstraint();

        /**
         * @brief: Add monotone constraint inequality, guarantee the monotone city at
         * evaluated point. customized monotone inequality constraint at x_coord
         **/
        bool AddMonotoneInequalityConstraint(const std::vector<double>& x_coord);

        // default inequality constraint at knots
        bool AddMonotoneInequalityConstraintAtKnots();

        /**
         * @brief: output interface inequality constraint
         **/
        const AffineConstraint& inequality_constraint() const;
        const AffineConstraint& equality_constraint() const;

    private:
        uint32_t FindIndex(const double x) const;

        bool FilterConstraints(const std::vector<double>& x_coord,
                               const std::vector<double>& lower_bound,
                               const std::vector<double>& upper_bound,
                               std::vector<double>* const filtered_lower_bound_x,
                               std::vector<double>* const filtered_lower_bound,
                               std::vector<double>* const filtered_upper_bound_x,
                               std::vector<double>* const filtered_upper_bound);
        void GeneratePowerX(const double x, const uint32_t order,
                            std::vector<double>* const power_x) const;

        using AddConstraintInRangeFunc =
        std::function<bool(const std::vector<double>&, const std::vector<double>&,
                           const std::vector<double>&)>;

        bool AddConstraintInRange(AddConstraintInRangeFunc func, const double x,
                                  const double val, const double range);

    private:
        AffineConstraint inequality_constraint_;
        AffineConstraint equality_constraint_;
        std::vector<double> x_knots_;
        uint32_t spline_order_;
    };
}

#endif //PLANNING_SPLINE_1D_CONSTRAINT_H
