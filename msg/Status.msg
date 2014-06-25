# constant byte indicating result of query
byte result

# what error did we get
byte error

# string describing what happened
string info

byte SUCCESS=1
byte FAILURE=0

byte NO_ERROR=0
byte FILE_EXISTS=1
byte FILE_MISSING=2
byte TYPE_MISSING=3
byte NO_SUCH_TYPE=4
