uint8 ACK=0
uint8 INFO=1
uint8 WARN=2
uint8 ERROR=3
uint8 UNKNOWN=4

uint8 type
string category
string msg

# If type is error, retrieve error message from error code
string error