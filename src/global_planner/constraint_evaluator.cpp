#include "cb_base_navigation/global_planner/constraint_evaluator.h"

namespace cb_global_planner {

ConstraintEvaluator::ConstraintEvaluator() {}

bool ConstraintEvaluator::init(const std::string& constraint)
{
    symbol_table_.clear();
    symbol_table_.add_variable("x",x_);
    symbol_table_.add_variable("y",y_);
    expression_.register_symbol_table(symbol_table_);

    exprtk::parser<double> parser;

    if(!parser.compile(constraint,expression_)) {
        printf("Error: %s\tExpression: %s\n", parser.error().c_str(), constraint.c_str());
        return false;
    }

    constraint_ = constraint;
    initialized_ = true;
    return true;
}

bool ConstraintEvaluator::evaluate(const double& x, const double& y) {
    if (!initialized_) return false;
    x_ = x; y_ = y;
    return expression_.value();
}

}
