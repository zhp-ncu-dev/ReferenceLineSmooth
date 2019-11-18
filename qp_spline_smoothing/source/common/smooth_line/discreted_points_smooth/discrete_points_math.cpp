#include <cmath>
#include "common/smooth_line/discreted_points_smooth/discrete_points_math.h"
#include <iostream>

namespace planning
{
    bool DiscretePointsMath::ComputePathProfile(
            const std::vector<std::pair<double, double>>& xy_points,
            std::vector<double>* headings, std::vector<double>* accumulated_s,
            std::vector<double>* kappas, std::vector<double>* dkappas)
    {
        //CHECK_NOTNULL(headings);
        //CHECK_NOTNULL(kappas);
        //CHECK_NOTNULL(dkappas);
        headings->clear();
        kappas->clear();
        dkappas->clear();

        size_t pointSize = xy_points.size();
/*
        // accumulated_s calculation
        double distance = 0.0;
        accumulated_s->push_back(distance);
        double fx = xy_points[0].first;
        double fy = xy_points[0].second;
        double nx = 0.0;
        double ny = 0.0;
        for (std::size_t i = 1; i < pointSize; ++i)
        {
            nx = xy_points[i].first;
            ny = xy_points[i].second;
            double end_segment_s =
                    std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
            accumulated_s->push_back(end_segment_s + distance);
            distance += end_segment_s;
            fx = nx;
            fy = ny;
        }

        for (std::size_t i = 0; i < pointSize; ++i)
        {
            double x_delta = 0.0;
            double y_delta = 0.0;
            if (i == 0)
            {
                x_delta = (xy_points[i + 1].first - xy_points[i].first);
                y_delta = (xy_points[i + 1].second - xy_points[i].second);
            }
            else if (i == pointSize - 1)
            {
                x_delta = (xy_points[i].first - xy_points[i - 1].first);
                y_delta = (xy_points[i].second - xy_points[i - 1].second);
            }
            else
            {
                x_delta = 0.5 * (xy_points[i + 1].first - xy_points[i - 1].first);
                y_delta = 0.5 * (xy_points[i + 1].second - xy_points[i - 1].second);
            }
            headings->push_back(std::atan2(y_delta, x_delta));
        }

        // Kappa calculation
        double kappa = 0.0;
        double kappaLast = 0.0;
        for(size_t i = 0; i < pointSize; ++i)
        {
            double deltaAngle = 0.0;
            double deltaS = 0.0;

            if(i == 0)
            {
                deltaAngle = headings->at(i+1) - headings->at(i);
                deltaS = accumulated_s->at(i+1) - accumulated_s->at(i);
            }
            else if(i == pointSize - 1)
            {
                deltaAngle = headings->at(i) - headings->at(i-1);
                deltaS = accumulated_s->at(i) - accumulated_s->at(i-1);
            }
            else
            {
                deltaAngle = headings->at(i+1) - headings->at(i-1);
                deltaS = accumulated_s->at(i+1) - accumulated_s->at(i-1);
            }

            kappa = deltaAngle / (deltaS + 1.0e-6);
            if(i > 5)
            {
                if(std::abs(kappa - kappaLast) > 0.1 && std::abs(kappaLast) < 0.5)
                {
                    kappa = kappaLast;
                }
            }
            kappaLast = kappa;
            kappas->emplace_back(kappa);
        }

        // Dkappa calculation
        for (std::size_t i = 0; i < pointSize; ++i)
        {
            double dkappa = 0.0;
            if (i == 0)
            {
                dkappa = (kappas->at(i + 1) - kappas->at(i)) /
                         (accumulated_s->at(i + 1) - accumulated_s->at(i));
            }
            else if (i == pointSize - 1)
            {
                dkappa = (kappas->at(i) - kappas->at(i - 1)) /
                         (accumulated_s->at(i) - accumulated_s->at(i - 1));
            }
            else
            {
                dkappa = (kappas->at(i + 1) - kappas->at(i - 1)) /
                         (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
            }
            dkappas->push_back(dkappa);
        }

*/

        if (xy_points.size() < 2)
        {
            return false;
        }
        std::vector<double> dxs;
        std::vector<double> dys;
        std::vector<double> y_over_s_first_derivatives;
        std::vector<double> x_over_s_first_derivatives;
        std::vector<double> y_over_s_second_derivatives;
        std::vector<double> x_over_s_second_derivatives;

        // Get finite difference approximated dx and dy for heading and kappa
        // calculation
        std::size_t points_size = xy_points.size();
        for (std::size_t i = 0; i < points_size; ++i)
        {
            double x_delta = 0.0;
            double y_delta = 0.0;
            if (i <= 1)
            {
                x_delta = (xy_points[i + 1].first - xy_points[i].first);
                y_delta = (xy_points[i + 1].second - xy_points[i].second);
            }
            else if (i >= points_size - 2)
            {
                x_delta = (xy_points[i].first - xy_points[i - 1].first);
                y_delta = (xy_points[i].second - xy_points[i - 1].second);
            }
            else
            {
                x_delta = 0.5 * (xy_points[i + 1].first - xy_points[i - 1].first);
                y_delta = 0.5 * (xy_points[i + 1].second - xy_points[i - 1].second);
            }
            dxs.push_back(x_delta);
            dys.push_back(y_delta);
        }

        // Heading calculation
        for (std::size_t i = 0; i < points_size; ++i)
        {
            headings->push_back(std::atan2(dys[i], dxs[i]));
        }

        // Get linear interpolated s for dkappa calculation
        double distance = 0.0;
        accumulated_s->push_back(distance);
        double fx = xy_points[0].first;
        double fy = xy_points[0].second;
        double nx = 0.0;
        double ny = 0.0;
        for (std::size_t i = 1; i < points_size; ++i)
        {
            nx = xy_points[i].first;
            ny = xy_points[i].second;
            double end_segment_s =
                    std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
            accumulated_s->push_back(end_segment_s + distance);
            distance += end_segment_s;
            fx = nx;
            fy = ny;
        }

        // Get finite difference approximated first derivative of y and x respective
        // to s for kappa calculation
        for (std::size_t i = 0; i < points_size; ++i)
        {
            double xds = 0.0;
            double yds = 0.0;
            if (i == 0)
            {
                xds = (xy_points[i + 1].first - xy_points[i].first) /
                      (accumulated_s->at(i + 1) - accumulated_s->at(i));
                yds = (xy_points[i + 1].second - xy_points[i].second) /
                      (accumulated_s->at(i + 1) - accumulated_s->at(i));
            }
            else if (i == points_size - 1)
            {
                xds = (xy_points[i].first - xy_points[i - 1].first) /
                      (accumulated_s->at(i) - accumulated_s->at(i - 1));
                yds = (xy_points[i].second - xy_points[i - 1].second) /
                      (accumulated_s->at(i) - accumulated_s->at(i - 1));
            }
            else
            {
                xds = (xy_points[i + 1].first - xy_points[i - 1].first) /
                      (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
                yds = (xy_points[i + 1].second - xy_points[i - 1].second) /
                      (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
            }
            x_over_s_first_derivatives.push_back(xds);
            y_over_s_first_derivatives.push_back(yds);
        }

        // Get finite difference approximated second derivative of y and x respective
        // to s for kappa calculation
        for (std::size_t i = 0; i < points_size; ++i)
        {
            double xdds = 0.0;
            double ydds = 0.0;
            if (i == 0)
            {
                xdds =
                        (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
                        (accumulated_s->at(i + 1) - accumulated_s->at(i));
                ydds =
                        (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
                        (accumulated_s->at(i + 1) - accumulated_s->at(i));
            }
            else if (i == points_size - 1)
            {
                xdds =
                        (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
                        (accumulated_s->at(i) - accumulated_s->at(i - 1));
                ydds =
                        (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
                        (accumulated_s->at(i) - accumulated_s->at(i - 1));
            }
            else
            {
                xdds = (x_over_s_first_derivatives[i + 1] -
                        x_over_s_first_derivatives[i - 1]) /
                       (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
                ydds = (y_over_s_first_derivatives[i + 1] -
                        y_over_s_first_derivatives[i - 1]) /
                       (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
            }
            x_over_s_second_derivatives.push_back(xdds);
            y_over_s_second_derivatives.push_back(ydds);
        }

        for (std::size_t i = 0; i < points_size; ++i)
        {
            double xds = x_over_s_first_derivatives[i];
            double yds = y_over_s_first_derivatives[i];
            double xdds = x_over_s_second_derivatives[i];
            double ydds = y_over_s_second_derivatives[i];
            double kappa =
                    (xds * ydds - yds * xdds) /
                    (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
            kappas->push_back(kappa);
        }


        // Dkappa calculation
        for (std::size_t i = 0; i < points_size; ++i)
        {
            double dkappa = 0.0;
            if (i == 0)
            {
                dkappa = (kappas->at(i + 1) - kappas->at(i)) /
                         (accumulated_s->at(i + 1) - accumulated_s->at(i));
            }
            else if (i == points_size - 1)
            {
                dkappa = (kappas->at(i) - kappas->at(i - 1)) /
                         (accumulated_s->at(i) - accumulated_s->at(i - 1));
            }
            else
            {
                dkappa = (kappas->at(i + 1) - kappas->at(i - 1)) /
                         (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
            }
            dkappas->push_back(dkappa);
        }

        return true;
    }

    void DiscretePointsMath::ComputeOriginalPathKappaProfile(
            const planning::ReferenceLine &referenceLine,
            std::vector<double> *kappas)
    {
        std::vector<double> headings;
        std::vector<double> accumulateS;
        std::vector<double> y_over_s_first_derivatives;
        std::vector<double> x_over_s_first_derivatives;
        std::vector<double> y_over_s_second_derivatives;
        std::vector<double> x_over_s_second_derivatives;

        // get headings
        std::vector<ReferencePoint> referencePoints = referenceLine.referencePoints();
        double dis = 0.0;
        size_t pointSize = referencePoints.size();
        for (std::size_t i = 0; i < pointSize; ++i)
        {
            headings.emplace_back(referencePoints[i].pointInfo().heading());
            if(i == 0)
            {
                accumulateS.emplace_back(dis);
            }
            else
            {
                dis += std::sqrt(
                        (referencePoints[i].pointInfo().x() - referencePoints[i-1].pointInfo().x()) *
                        (referencePoints[i].pointInfo().x() - referencePoints[i-1].pointInfo().x()) +
                        (referencePoints[i].pointInfo().y() - referencePoints[i-1].pointInfo().y()) *
                        (referencePoints[i].pointInfo().y() - referencePoints[i-1].pointInfo().y()));
                accumulateS.emplace_back(dis);
            }
        }

        double kappa = 0.0;
        double kappaLast = 0.0;
        for(size_t i = 0; i < pointSize; ++i)
        {
            double deltaAngle = 0.0;
            double deltaS = 0.0;

            if(i == 0)
            {
                deltaAngle = headings[i + 1] - headings[i];
                deltaS = accumulateS[i + 1] - accumulateS[i];
            }
            else if(i == pointSize)
            {
                deltaAngle = headings[i] - headings[i - 1];
                deltaS = accumulateS[i] - accumulateS[i - 1];
            }
            else
            {
                deltaAngle = headings[i + 1] - headings[i - 1];
                deltaS = accumulateS[i + 1] - accumulateS[i - 1];
            }

            kappa = -deltaAngle / (deltaS + 1.0e-6);
            if(i > 5)
            {
                if(std::abs(kappa - kappaLast) > 0.1 && std::abs(kappaLast) < 0.5)
                {
                    kappa = kappaLast;
                }
            }
            kappaLast = kappa;
            kappas->emplace_back(kappa);
        }

/*
        // Get finite difference approximated first derivative of y and x respective
        // to s for kappa calculation
        for (std::size_t i = 0; i < pointSize; ++i)
        {
            double xds = 0.0;
            double yds = 0.0;
            if (i == 0)
            {
                xds = (referencePoints[i + 1].pointInfo().x() - referencePoints[i].pointInfo().x()) /
                      (accumulateS.at(i + 1) - accumulateS.at(i));
                yds = (referencePoints[i + 1].pointInfo().y() - referencePoints[i].pointInfo().y()) /
                      (accumulateS.at(i + 1) - accumulateS.at(i));
            }
            else if (i == pointSize - 1)
            {
                xds = (referencePoints[i].pointInfo().x() - referencePoints[i - 1].pointInfo().x()) /
                      (accumulateS.at(i) - accumulateS.at(i - 1));
                yds = (referencePoints[i].pointInfo().y() - referencePoints[i - 1].pointInfo().y()) /
                      (accumulateS.at(i) - accumulateS.at(i - 1));
            }
            else
            {
                xds = (referencePoints[i + 1].pointInfo().x() - referencePoints[i - 1].pointInfo().x()) /
                      (accumulateS.at(i + 1) - accumulateS.at(i - 1));
                yds = (referencePoints[i + 1].pointInfo().y() - referencePoints[i - 1].pointInfo().y()) /
                      (accumulateS.at(i + 1) - accumulateS.at(i - 1));
            }
            x_over_s_first_derivatives.push_back(xds);
            y_over_s_first_derivatives.push_back(yds);
        }

        // Get finite difference approximated second derivative of y and x respective
        // to s for kappa calculation
        for (std::size_t i = 0; i < pointSize; ++i)
        {
            double xdds = 0.0;
            double ydds = 0.0;
            if (i == 0)
            {
                xdds =
                        (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
                        (accumulateS.at(i + 1) - accumulateS.at(i));
                ydds =
                        (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
                        (accumulateS.at(i + 1) - accumulateS.at(i));
            }
            else if (i == pointSize - 1)
            {
                xdds =
                        (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
                        (accumulateS.at(i) - accumulateS.at(i - 1));
                ydds =
                        (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
                        (accumulateS.at(i) - accumulateS.at(i - 1));
            }
            else
            {
                xdds = (x_over_s_first_derivatives[i + 1] -
                        x_over_s_first_derivatives[i - 1]) /
                       (accumulateS.at(i + 1) - accumulateS.at(i - 1));
                ydds = (y_over_s_first_derivatives[i + 1] -
                        y_over_s_first_derivatives[i - 1]) /
                       (accumulateS.at(i + 1) - accumulateS.at(i - 1));
            }
            x_over_s_second_derivatives.push_back(xdds);
            y_over_s_second_derivatives.push_back(ydds);
        }

        for (std::size_t i = 0; i < pointSize; ++i)
        {
            double xds = x_over_s_first_derivatives[i];
            double yds = y_over_s_first_derivatives[i];
            double xdds = x_over_s_second_derivatives[i];
            double ydds = y_over_s_second_derivatives[i];
            double kappa =
                    (xds * ydds - yds * xdds) /
                    (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
            kappas->push_back(kappa);
        }
*/
    }

}// end namespace

