//
// Created by gaoyang on 18-11-26.
//
#include "common/polynomial_xd/curve_math.h"
#include "common/math/math_utils.h"
#include "reference_line/qp_spline_reference_line_smooth.h"

namespace planning
{
    QpSplineReferenceLineSmooth::QpSplineReferenceLineSmooth()
    {
        spline_solver_.reset(new Spline2dSolver(t_knots_,FLAGS_SPLINE_ORDER));
    }
    void QpSplineReferenceLineSmooth::setAnchorPoints(const std::vector<AnchorPoint> &anchor_points,bool dif_time_smooth)
    {
        anchor_points_ = anchor_points;
        dif_time_smooth_ = dif_time_smooth;
    }

    void QpSplineReferenceLineSmooth::clear()
    {
        t_knots_.clear();
    }

    double QpSplineReferenceLineSmooth::getSpacingDis(const double &speed)const
    {
        return 0.1;
    }

    bool QpSplineReferenceLineSmooth::smooth(const ReferenceLine &raw_reference_line,
                                             const double &longitudinalSpeed,
                                             ReferenceLine *const smoothed_reference_line)
    {
        clear();
        const double kEpsilon = 1e-6;
        if (!sample())
        {
            std::cout << "Fail to sample reference line smoother points!";
            return false;
        }
        spline_solver_->Reset(t_knots_, FLAGS_SPLINE_ORDER);
        if (!addConstraint())
        {
            std::cout << "Add constraint for spline smoother failed";
            return false;
        }
        if (!AddKernel())
        {
            std::cout << "Add kernel for spline smoother failed.";
            return false;
        }
        if (!Solve())
        {
            std::cout << "Solve spline smoother problem failed";
            return false;
        }
        std::cout << "Solve spline smoother problem suceess !!!"<<std::endl;
        const double start_t = t_knots_.front();
        const double end_t = t_knots_.back();
        double spacingDis  = longitudinalSpeed;
        const std::uint32_t num_of_total_points = static_cast<std::uint32_t>((anchor_points_.back().abs_s- anchor_points_.front().abs_s) / spacingDis + 1);
        const double resolution = (end_t - start_t) / (num_of_total_points - 1);

        double t = start_t;
        std::vector<ReferencePoint> ref_points;
        std::vector<double> ref_s;
        const auto &spline = spline_solver_->spline();

//        had_map::MapPoint orginPoint;
//        orginPoint.set_x(vehicleState.vehicleInsData().gussLongtitude());
//        orginPoint.set_y(vehicleState.vehicleInsData().gussLatitude());
//        orginPoint.setHeading(vehicleState.vehicleInsData().heading());

        for (std::uint32_t i = 0; i < num_of_total_points && t < end_t;
             ++i, t += resolution)
        {
            const double heading =
                    math::getAngle(spline.DerivativeX(t), spline.DerivativeY(t));
            const double kappa = CurveMath::ComputeCurvature(
                    spline.DerivativeX(t), spline.SecondDerivativeX(t),
                    spline.DerivativeY(t), spline.SecondDerivativeY(t));
            const double dkappa = CurveMath::ComputeCurvatureDerivative(
                    spline.DerivativeX(t), spline.SecondDerivativeX(t),
                    spline.ThirdDerivativeX(t), spline.DerivativeY(t),
                    spline.SecondDerivativeY(t), spline.ThirdDerivativeY(t));
            const double xds = spline.DerivativeX(t);
            const double yds = spline.DerivativeY(t);
            const double xseconds = spline.SecondDerivativeX(t);
            const double yseconds = spline.SecondDerivativeY(t);
            std::pair<double, double> xy = spline(t);

            xy.first += ref_x_;
            xy.second += ref_y_;
            had_map::MapPoint point_info;
            point_info.set_x(xy.first);
            point_info.set_y(xy.second);
            point_info.setHeading(heading);
            double left_width;
            double right_width;
            if(!raw_reference_line.getLeftAndRightWidth(point_info,left_width,right_width))
            {
                continue;
            }
            double minSpeed;
            double maxSpeed;
            if(!raw_reference_line.getSpeed(point_info,maxSpeed,minSpeed))
            {
                continue;
            }
//            had_map::MapPoint goalPoint;
//            math::gussPointToRFU(orginPoint,point_info,goalPoint);
//            double x = goalPoint.x();
//            double y = goalPoint.y();
//            double heading1 = goalPoint.heading();
//            printf("x = %f,y= %f,heading = %f\r\n",x,y,heading1);

            double s = t * ((anchor_points_.back().abs_s- anchor_points_.front().abs_s)) / (end_t - start_t);
            LinkLaneSegment linkLaneSegment = raw_reference_line.getLinkLaneByS(s + anchor_points_.front().abs_s);
            ref_s.emplace_back(s);
//            printf("s = %f,xds = %f, yds = %f, heading = %f,kappa = %f\r\n", s, xds,yds, heading, kappa);
            ref_points.emplace_back(ReferencePoint(point_info,kappa,dkappa,xds,yds,xseconds,yseconds,left_width,right_width,maxSpeed,minSpeed,linkLaneSegment));
        }
        if(ref_points.size() < 2)
        {
            return false;
        }
        *smoothed_reference_line = ReferenceLine(ref_points,ref_s);
        smoothed_reference_line->setSpacingDis(spacingDis);
        smoothed_reference_line->reCalculateSegments();
        return true;
    }

    bool QpSplineReferenceLineSmooth::sample()
    {
        const double length = anchor_points_.back().abs_s -
                              anchor_points_.front().abs_s;
        uint32_t num_spline =
                std::max(1u, static_cast<uint32_t>(
                        length / FLAGS_max_spline_length + 0.5));
        for (std::uint32_t i = 0; i <= num_spline; ++i) {
            t_knots_.emplace_back(i * 1.0);
        }
        // normalize point xy
        ref_x_ = anchor_points_.front().pointInfo.x();
        ref_y_ = anchor_points_.front().pointInfo.y();
        return true;
    }

    bool QpSplineReferenceLineSmooth::addConstraint()
    {
        std::vector<double> headings;
        std::vector<double> longitudinal_bound;
        std::vector<double> lateral_bound;
        std::vector<math::Vec2d> xy_points;
        for (const auto& point : anchor_points_) {
            headings.emplace_back(point.pointInfo.heading());
            longitudinal_bound.emplace_back(point.longitudinal_bound);
            lateral_bound.emplace_back(point.lateral_bound);
            math::Vec2d point1;
//            printf("需要平滑的点x = %f,y = %f,heading = %f\r\n",point.pointInfo.position.x,point.pointInfo.position.y,point.pointInfo.heading);
            point1.set_x(point.pointInfo.x() - ref_x_);// = point.pointInfo.x() - ref_x_;
            point1.set_y(point.pointInfo.y() - ref_y_);//y = point.pointInfo.y() - ref_y_;
            xy_points.emplace_back(point1);
        }
        const double scale = (anchor_points_.back().abs_s -
                              anchor_points_.front().abs_s) /
                             (t_knots_.back() - t_knots_.front());
        std::vector<double> evaluated_t;
        for (const auto& point : anchor_points_) {
//            printf("s = %f\r\n",point.abs_s);
            evaluated_t.emplace_back(point.abs_s / scale);
        }
        auto* spline_constraint = spline_solver_->mutable_constraint();

        // all points (x, y) should not deviate anchor points by a bounding box
        if (!spline_constraint->Add2dBoundary(evaluated_t, headings, xy_points,
                                              longitudinal_bound, lateral_bound)) {
            std::cout << "Add 2d boundary constraint failed." << std::endl;
            return false;
        }

        // the heading of the first point should be identical to the anchor point.
        if(!dif_time_smooth_)
        {
            if (!spline_constraint->AddPointAngleConstraint(evaluated_t.front(),
                                                            headings.front())) {
                std::cout << "Add 2d point angle constraint failed." << std::endl;
                return false;
            }
        }
        else
        {
//            printf("xds = %f,yds = %f,xseconds = %f,yseconds = %f\r\n",anchor_points_.front().xds,anchor_points_.front().yds,anchor_points_.front().xseconds,anchor_points_.front().yseconds);
            if(!spline_constraint->AddPointSecondDerivativeSmoothConstraint(evaluated_t.front(),
                                                                            anchor_points_.front().xds,
                                                                            anchor_points_.front().yds,
                                                                            anchor_points_.front().xseconds,
                                                                            anchor_points_.front().yseconds))
            {
                std::cout << "Add 2d point derivative and second derivative failed." << std::endl;
                return false;
            }
        }

        // all spline should be connected smoothly to the second order derivative.
        if (!spline_constraint->AddSecondDerivativeSmoothConstraint()) {
            std::cout << "Add jointness constraint failed." << std::endl;
            return false;
        }

        return true;
    }

    bool QpSplineReferenceLineSmooth::AddKernel() {
        Spline2dKernel* kernel = spline_solver_->mutable_kernel();

        // add spline kernel
        if (FLAGS_second_derivative_weight > 0.0) {
            kernel->AddSecondOrderDerivativeMatrix(
                    FLAGS_second_derivative_weight);
        }
        if (FLAGS_third_derivative_weight > 0.0) {
            kernel->AddThirdOrderDerivativeMatrix(
                    FLAGS_third_derivative_weight);
        }

        kernel->AddRegularization(FLAGS_regularization_weight);
        return true;
    }

    bool QpSplineReferenceLineSmooth::Solve() { return spline_solver_->Solve(); }
}