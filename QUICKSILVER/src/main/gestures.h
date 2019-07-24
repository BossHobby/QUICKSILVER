
int gestures2( void);
int gesture_sequence( int gesture);
void gestures( void);

// warning: using if x>GESTURE_UDR to check if pid gestures used
enum gestures_enum{
    GESTURE_NONE = 0, 
    GESTURE_DDD,
    GESTURE_UUU,
    GESTURE_LLD,
    GESTURE_RRD,
    GESTURE_UDU,
    GESTURE_UDD,
    GESTURE_UDR,
    GESTURE_UDL,
		GESTURE_RRR,
		GESTURE_LLL,
		GESTURE_DUD
    
};


