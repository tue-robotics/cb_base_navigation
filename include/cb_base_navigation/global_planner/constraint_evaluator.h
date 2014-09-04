/*******************************
 *                             *
 *  Author: Rein Appeldoorn    *
 *  Date:   2013-03-06         *
 *                             *
 *******************************/

#ifndef cb_global_planner_CONSTRAINT_EVALUATOR_H_
#define cb_global_planner_CONSTRAINT_EVALUATOR_H_

#include "exprtk.h"
#include <tf/transform_datatypes.h>

namespace cb_global_planner
{

/**
 * @class Class which is used to evaluate a set of constraints.
 * @brief A set of position constraints can be specified which can be evaluated for arbitrary x,y values.
 */
class ConstraintEvaluator
{

public:
    /**
     * @brief  Constructor for the ConstraintEvaluator Object
     */
    ConstraintEvaluator();

    /**
     * @brief   Init function which initializes the constraint that has to be evaluated.
     * @param   constraint  The position constraint that has to be evaluated.
     */
    bool init(const std::string& constraint);

    /**
     * @brief   Evaluates the constraint for an x,y value
     * @param   x   x value
     * @param   y   y value
     */
    bool evaluate(const double& x, const double& y);

    /**
     * @brief   Returns the initialized constraint.
     */
    inline std::string getConstraint() { return constraint_; }

private:
    std::string constraint_;
    exprtk::symbol_table<double> symbol_table_;
    exprtk::expression<double> expression_;
    double x_,y_;
    bool initialized_;

};

}
#endif

