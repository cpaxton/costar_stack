# Statement of a predicate
# 
# ex: left_of(Ovaltine_1, Ovaltine_2, ur5)
# 
# This predicate would mean "Ovaltine_2 is to the left of Ovaltine_1 from the point of view of the ur5 robot".
#######

# name of the predicate
# ex: left_of
string predicate

# value of the predicate
# this is not really used right now
# could be for feature values or for truth value (probability)
float64 value

# number of parameters used
int32 num_params

# list of params
# ex: [Ovaltine_1, Ovaltine_2, ur5]
string[3] params

# first parameter value
# ex: Ovaltine_1
#string param1

# name of parameter 2
# ex: Ovaltine_2
#string param2

# name of the frame of reference
# ex: ur5
#string reference_frame
