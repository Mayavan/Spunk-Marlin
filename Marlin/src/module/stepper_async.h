#define INITIAL_X 0
#define INITIAL_Y 180

#define GEAR_RATIO_X 2.33
#define GEAR_RATIO_Y 1

#define MAX_VELOCITY_X 36.0*GEAR_RATIO_X  // Angular velocity in degrees/second
#define MAX_VELOCITY_Y 36.0*GEAR_RATIO_Y  // Angular velocity in degrees/second
#define TIME_TO_MAX_VELOCITY 0.5  // in seconds

#define ACCELERATION_X MAX_VELOCITY_X/TIME_TO_MAX_VELOCITY
#define ACCELERATION_Y MAX_VELOCITY_Y/TIME_TO_MAX_VELOCITY

#define MICROSTEPS_PER_STEP 32.0
#define FULL_STEPS_PER_ROTATION 200.0
#define TOLERENCE 1.0
#define INTEGRAL_FREQUENCY 100.0

#define DEGREE_PER_MICROSTEP 360.0/(MICROSTEPS_PER_STEP*FULL_STEPS_PER_ROTATION)

/* Singleton class */
class stepper_async{
   int direction;
   float target_x_state;
   float target_y_state;
   
   float current_x_state;
   float current_y_state;
   double current_x_velocity;
   double current_y_velocity;
   float max_velocity_x;
   float max_velocity_y;
   float acceleration_x;
   float acceleration_y;

   double step_time_period;

   float amount;
   unsigned long interval;
   unsigned long previous_step_time_us;
    
   // Private constructor so that no objects can be created.
   stepper_async() {}

   stepper_async(stepper_async const&);              // Don't Implement
   void operator=(stepper_async const&);             // Don't implement

 public:
   static stepper_async& getInstance() {
       static stepper_async instance;
      return instance;
   }
   
   void init();
   void set_async_target_x(float x);
   void set_async_target_y(float y);
   void set_velocity_x(float x);
   void set_velocity_y(float y);
   void set_acceleration_x(float x);
   void set_acceleration_y(float y);
   void set_current_x(float x);
   void set_current_y(float y);
   void controller_thread();

   void print_log(float z);

};


