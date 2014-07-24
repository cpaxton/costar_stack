
######
# Statement of a predicate
# ex: left_of(Ovaltine_1, Ovaltine_2, ur5)
# This predicate would mean "Ovaltine_2 is to the left of Ovaltine_1 from the point of view of the ur5 robot".
#######

# name of the predicate
# ex: left_of
string predicate

# value of the predicate
# this is not really used right now
float64 value

# probability that this predicate is true
float64 confidence

# adding truth
bool enabled

# number of parameters used
# TODO: remove this
int32 num_params

# list of params
# ex: [Ovaltine_1, Ovaltine_2, ur5]
# TODO: remove the fixed number of parameters from this
string[3] params

# class of the parameter
string[] param_classes

# first parameter value
# ex: Ovaltine_1
#string param1

# name of parameter 2
# ex: Ovaltine_2
#string param2

# name of the frame of reference
# ex: ur5
#string reference_frame
