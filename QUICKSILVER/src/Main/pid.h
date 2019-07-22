void rotateErrors(void);
float pid( int x );
int next_pid_term( void); // Return value : 0 - p, 1 - i, 2 - d
int next_pid_axis( void); // Return value : 0 - Roll, 1 - Pitch, 2 - Yaw
int increase_pid( void );
int decrease_pid( void );
void pid_precalc( void);



