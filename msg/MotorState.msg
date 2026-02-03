uint16 motor_id
string name
uint8 mode
bool enabled
bool fault
int32 position
int16 velocity
int16 current

# Common mode constants so topics/services share the same enum semantics.
uint8 MODE_UNKNOWN=0
uint8 MODE_POSITION=1
uint8 MODE_VELOCITY=2
