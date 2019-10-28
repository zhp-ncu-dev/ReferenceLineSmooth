#pragma once

#include <cmath>
#include <vector>

#include "common/gradient_descent/dynamicvoronoi.h"
#include "common/gradient_descent/node3d.h"
#include "common/gradient_descent/vector2d.h"
#include "common/gradient_descent/helper.h"
#include "common/gradient_descent/constants.h"
#include "proto/pnc_point.h"

using namespace ADC;

namespace HybridAStar {
/*!
   \brief This class takes a path object and smoothes the nodes of the path.

   It also uses the Voronoi diagram as well as the configuration space.
*/
    class Smoother {
    public:
        Smoother() {}

        /*!
           \brief This function takes a path consisting of nodes and attempts to iteratively smooth the same using gradient descent.

           During the different interations the following cost are being calculated
           obstacleCost
           curvatureCost
           smoothnessCost
           voronoiCost
        */
        void smoothPath();

        /*!
           \brief Given a node pointer the path to the root node will be traced recursively
           \param node a 3D node, usually the goal node
           \param i a parameter for counting the number of nodes
        */
        void tracePath(const std::vector<ADC::planning::dPoint3d>& node);

        /// returns the path of the smoother object
        std::vector<ADC::planning::dPoint3d> getPath() {return path;}

        /// curvatureCost - forces a maximum curvature of 1/R along the path ensuring drivability
        Vector2D curvatureTerm(Vector2D xi0, Vector2D xi1, Vector2D xi2);

        /// smoothnessCost - attempts to spread nodes equidistantly and with the same orientation
        Vector2D smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2);

    private:
        /// maximum possible curvature of the non-holonomic vehicle
        double kappaMax = 1.0 / (Constants::r * 1.1);

        /// falloff rate for the voronoi field
        double alpha = 0.0;

        /// weight for the curvature term
        double wCurvature = 1.0e10;

        /// weight for the smoothness term
        double wSmoothness = 1.0e1;

        std::vector<ADC::planning::dPoint3d> path;
    };
}

