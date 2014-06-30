# Defines the set of values for which a predicate is true
#
###

# The statement defining this predicate
predicator_msgs/PredicateStatement statement

# The set of values for which this statement is true, if applicable
# Only true predicates will be published, so if there are no free variables in the statement this will be empty
predicator_msgs/PredicateStatement[] values
