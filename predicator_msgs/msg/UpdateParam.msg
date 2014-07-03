# UPDATE PARAMS
# Message type used to set params in things like the predicator_params package.
# Set the "operation" field to say whether the node should start/stop publishing this message.

byte operation

byte PUBLISH_PREDICATE=0
byte REMOVE_PREDICATE=1

predicate_msgs/PredicateStatement statement

