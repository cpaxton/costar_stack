#include <predicator_msgs/PredicateStatement.h>
#include <predicator_planning/PredicatePlan.h>

#include "predicator.h"

namespace predicator_planning {


  struct Planner {

    // context contains information about the world and will produce new predicates
    PredicateContext * context;

    unsigned int max_iter; // maximum iterations to attempt to find destination
    unsigned int children; // number of children to create at each step


  };


}
