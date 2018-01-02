// 2.4
//  - Reduce guided approach power from 0.30 to 0.20 
// 2.3
//  - Add X for controlled drift
// 2.2
//  - Switch wheel PWM id's to match wiring
//  - Switch zones to allow diagonal movement
//  - Add Precision mode
// 2.1
//  - Add dampening to prevent rapid reversal

// 2.0
//  - Use new method of movement (veer instead of diagonal)
// 1.1
//  - handled cae where magnitude was 0
//  - switch joysticks to take raw data
// 1.0 Created



package robot;




import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Encoder;




/**
 *
 * @author Andrew Rowles
 */
public class Drive {
    
    public final int WHEEL_LF = 1;
    public final int WHEEL_RF = 2;
    public final int WHEEL_LR = 3;
    public final int WHEEL_RR = 4;
    
    
 
    public VictorSP  wheel_lf;
    public VictorSP  wheel_rf;
    public VictorSP  wheel_lr;
    public VictorSP  wheel_rr;
    
    
    public Encoder left_encoder;
    public Encoder right_encoder;
    
    
    public Joystick driver;
    public Joystick shooter;
    public Tracker  tracker;
    public CameraFeed camera;
    public Timer      timer;
    
    //Intermediate values 
    //- Sign correct
    //- Smoothed and scaled
    //- Limited to range
    public double y  = 0;   //Forward  (+1) Reverse (-1)
    public double x  = 0;   //Right    (+1) Left    (-1) 
    public double ccw = 0;  //Spin CCW (+1) CW (-1) 
    
    //Mecanum directions
    public double mag   = 0;   //Amount of power 0-1
    public int    zone  = 0;   //1-4
    public double rot   = 0;   
    public double bias  = 0;   
    
    
    //Speed settings
    public double speed_LF  = 0.0;
    public double speed_RF  = 0.0;
    public double speed_LR  = 0.0;
    public double speed_RR  = 0.0;
    
    public double SPEED_IDLE  = 0.0;
    public double SPEED_LAUNCH = 0.5;
    public double SPEED_RETURN = -0.1;
    
    //Automated routine flags
    public boolean back_up  = false;  //If true, back away (cleared on any command)

    //States 
    public final int STATE_IDLE     = 0;
    public final int STATE_BACKUP   = 1;
    public final int STATE_COMPLETE = 9;
    public int state = STATE_IDLE;
     
    
    //This is maximum ramping rate to increment 
    // power when reversing wheel direction
    // Used to limit brownouts
    // Expressed as number of seconds to go from
    // max rev to max forward
    // 1.0  = 1.0 second to go from -1 to +1
    // 0.5 current setting
    // 5.0 for testing
    public final double MAX_RAMP = 0.6;
    public long   time_prev = 0; //Previous time (nanoseconds) 
    public long   time_now  = 0; //Current time (nanoseconds) 
    
    
    public boolean PRESSED_UP_POWER = false;
    public boolean PRESSED_DOWN_POWER = false;
     
  
    public Drive () {
        wheel_lf   = new VictorSP(WHEEL_LF);
        wheel_rf   = new VictorSP(WHEEL_RF);
        wheel_lr   = new VictorSP(WHEEL_LR);
        wheel_rr   = new VictorSP(WHEEL_RR);
        
        left_encoder  = new Encoder(3,4);
        right_encoder = new Encoder(5,6);
        timer         = new Timer ();    
    }
    
    public void update ()
        {
        //Update tick count for ramp function
        time_prev = time_now;
        time_now  = System.nanoTime();     
        
        //Reset wheel encoders
        if (driver.getRawButton(XboxMap.BTN_START)==true)
        	{resetEncoders ();}
        
        
        //Load in raw values
        y   =  getFRPower ();   //Forward = +1 Reverse = -1
        x   =  getRLPower ();   //Right   = +1 Left    = -1  
        ccw =  getCCWPower ();  //Positive=CCW, Negative=CW
        
        //Reverse controls
        //y = -y;
        //x = -x;
        
        
        //Convert the smoothed values into a vector
        //This is always positive, range 0-1.0
        mag =  Math.sqrt(y*y + x*x);
        if (mag > 1.0) mag=1.0;

        //If steady speed is pressed, then override mag
        if (IsSteadyMode ()) mag = 0.65;

        //Use doBackUp routine if
        // - IsBackUpPressed (manual buttons or auto flag set)
        // - state is not idle
        if (IsBackUpPressed() || state != STATE_IDLE)
        	{
        	doBackUp();
        	return;
        	}


        //If drift is pressed, then process request
        //and return.  Other features not used 
        if (IsDriftPressed())
            {
            //Drift standard drift
            applyDrift (mag, x, y);
            setPower (speed_LF,speed_RF,speed_LR,speed_RR);
            return;
            }
        
        //If JTurn pressed, then process request
        //and return.  Other features not used 
        if (IsJTurnPressed())
            {
            applyJTurn (mag, x, y);
            setPower (speed_LF,speed_RF,speed_LR,speed_RR);
            return;
            }


        //If steady ahead is pressed, then do that
        //ApplyDirections sets the speed values (but not the motors)
        //setPower applies values to the motor 
        if (driver.getRawButton(XboxMap.BTN_A))
            {
            ApplyDirection (0.30,0.30,0.30, 0.30);
            adjustPower (getLeftPosition(),getRightPosition());
            setPower (speed_LF,speed_RF,speed_LR,speed_RR);
            return;
            }
        
        //If guided approach is pressed, then do it
        //If camera 1 is used, reverse power
        //This only is used if tracking is successful
        if (driver.getRawButton(XboxMap.BTN_RIGHT_BUMPER))
            {
            double power = 0.20;
            if (camera.cam_id == 1) power = -0.20;
            tracker.update (power,camera.procTarget.center, camera.procTarget.target_ok);
            if (camera.procTarget.target_ok)
            	{
            	ApplyDirection (tracker.power_lf,tracker.power_rf,tracker.power_lr,tracker.power_rr);
            	setPower (speed_LF,speed_RF,speed_LR,speed_RR);
            	return;
            	}
            }
        
        //If guided approach (gear) is pressed, then do it
        if (driver.getRawButton(XboxMap.BTN_B))
            {
        	tracker.update (0.30,camera.procTarget.center, camera.procTarget.target_ok);
            if (camera.procTarget.target_ok)
            	{
                ApplyDirection (tracker.power_lf,tracker.power_rf,tracker.power_lr,tracker.power_rr);
                setPower (speed_LF,speed_RF,speed_LR,speed_RR);
                return;
            	}
            }
            
        
       
        
        //Determine which zone of movement
        // Quadrant
        // 1 - Top/Right
        // 2 - Bottom/Right
        // 3 - Bottom/Left
        // 4 - Top/Left
        // 0 - Stop
        zone = 0;
        if (mag > 0.01)
        {
        	if ((y >=  0) && (x >=  0)) zone = 1;
            if ((y <   0) && (x >=  0)) zone = 2;
            if ((y <   0) && (x <   0)) zone = 3;
            if ((y >=  0) && (x <   0)) zone = 4;
        }
        
        //If using a zone, then calculate angle
    	double theta = 0; //Radians 
        double k     = 0;
    	if (zone > 0)
        	{
        	theta = Math.asin(x/mag);
        	k     = Math.cos(2.0 * theta);
        	}
        
        //Calculate Bias values, that is, how much to slow
        //down one wheel.  Bias is always positive
        switch (zone)
        {
            case 0:
            	//Stop
                ApplyDirection(mag, mag, mag, mag);
                break;            	
            	
         	case 1: 
                // Forward
                ApplyDirection(mag, k*mag, k*mag, mag);
                break;
        
            case 2:
                //Right
                ApplyDirection(-k*mag, -mag,-mag, -k*mag);
                break;
            
            case 3: 
                //Reverse
                ApplyDirection(-mag,-k*mag,-k*mag,-mag);
                break;
                
            case 4: 
                ApplyDirection(k*mag, mag, mag, k*mag);
                break;
        }

        
        
        //The last step is to calculate rotation.  To make this work
        //smoothly there are a few factors to consider
        // - If standing still, rotate quickly.  
        // - If moving fast in a particular direction, rotate more slowly
        // - To rotate CCW, speed up right wheels, slow down left wheels
        // - To rotate CW, slow down right wheels, speed up left wheels
        
        double spin_effect = (0.5 - 0.25 * mag);  //Slower when moving fast
        spin_effect *= ccw;                       //Postive ccw, negative cw
        
        //Add effect to each wheel
        speed_LF = AddEffect (speed_LF, -spin_effect); //Slow down left wheels
        speed_LR = AddEffect (speed_LR, -spin_effect); //Slow down left wheels
        
        speed_RF = AddEffect (speed_RF,  spin_effect); //Speed up right wheels 
        speed_RR = AddEffect (speed_RR,  spin_effect); 
        
        
        //If the precision button is pressed, scale all values by
        // 0.25
        double scale = getPrecisionValue();
        setPower (scale*speed_LF,scale*speed_RF,scale*speed_LR,scale*speed_RR);
       
        }        

    //Apply spin is used to set targets for cross pattern
    //At this point, the diagonal wheels move the same amount
    //That is, there is no rotation
    public void ApplyDirection (double lf, double rf,double lr, double rr)
    {
        
        speed_LF = XboxMap.setMax(lf);
        speed_RR = XboxMap.setMax(rr);
        speed_RF = XboxMap.setMax(rf);
        speed_LR = XboxMap.setMax(lr);
  
    }
    
        
       
    //It is a good idea to convert user actions into simple functions.
    //Why?  Because sometimes switches return 0 when pressed, also
    //to use the same functions in auto mode makes it easier to override
    //the actual value
    public boolean IsTriggerPressed () 
    {
        boolean ret_val = false;
        if (driver.getTrigger()) ret_val = true;
        return ret_val;
    }
  public boolean IsManualPressed () 
    {
        boolean ret_val = false;
   //     if (joystick.getRawButton(BUTTON_MANUAL)) ret_val = true;
        return ret_val;
    }

public boolean IsDriftPressed () 
{
    return (driver.getRawButton(XboxMap.BTN_X));
}

// Steady mode holds speed at specific level
public boolean IsSteadyMode () 
{
    return (driver.getRawAxis(XboxMap.AXIS_L_TRIGGER) >= 0.5);
}

public boolean IsJTurnPressed () 
{
    return (driver.getRawButton(XboxMap.BTN_Y));
}

//Backup can be triggered by
// - Shooter 
// - Driver
// - Gear Sensor (back_up flag)
public boolean IsBackUpPressed () 
{
     if (back_up) return true;
     if (driver.getRawButton(XboxMap.BTN_LEFT_BUMPER)) return true;
     if (shooter.getRawButton(XboxMap.BTN_LEFT_BUMPER)) return true;
     return false; 
}

public void adjustPower (int left_pos, int right_pos)
	{
	double delta = left_pos - right_pos;
	double adj   = (delta * 0.015/17)/2.0;
	speed_LF -= adj;
   	speed_LR -= adj;

    speed_RR += adj;
    speed_RF += adj;
	}

//BackUp moves back at 10% power for 1 second
public void doBackUp ()
{
	if (state == STATE_IDLE)
		    {
			state = STATE_BACKUP;
			timer.reset();
			timer.start();
			}
	if (state == STATE_BACKUP)
            {
           	double power = -0.10;
            if (timer.get() > 0.80) 
            	{
            	state = STATE_COMPLETE;
            	power = 0;
            	}
			setRawPower (power,power,power,power);
            }

	if (state == STATE_COMPLETE)
			{
			back_up = false; //Clear flag
			if (!IsBackUpPressed()) state=STATE_IDLE;
			} 			

}

  public double getFRPower ()
  {
	  return -XboxMap.smoothPower(driver.getRawAxis (XboxMap.AXIS_LEFT_Y));
  }

  public double getRLPower ()
  {
	  return  XboxMap.smoothPower(driver.getRawAxis (XboxMap.AXIS_LEFT_X));
  }

public double getCCWPower ()
  {
	  return -XboxMap.smoothPower(driver.getRawAxis (XboxMap.AXIS_RIGHT_X));
  }

public int getLeftPosition () {return -left_encoder.get();}
public int getRightPosition () {return right_encoder.get();}

public void resetEncoders ()
  {
  left_encoder.reset ();
  right_encoder.reset ();
  }


//Given a distance in inches, convert to number
//of encoder values to produce that distance
public int convertDistance (double inches)
{
return (int)(40.34 * inches);
}

public void setPower (double lf, double rf, double lr, double rr)
{
	//Apply to CIM motors
    // Forward is Positive (Right Side)
    // Forward is Negative (Left Side)
   wheel_lf.set (rampPower(wheel_lf.get(), -lf));
   wheel_rf.set (rampPower(wheel_rf.get(),  rf));
   wheel_lr.set (rampPower(wheel_lr.get(), -lr));
   wheel_rr.set (rampPower(wheel_rr.get(),  rr));

}

public void setRawPower (double lf, double rf, double lr, double rr)
{
	//Apply to CIM motors
    // Forward is Positive (Right Side)
    // Forward is Negative (Left Side)
   wheel_lf.set ( -lf);
   wheel_rf.set (  rf);
   wheel_lr.set ( -lr);
   wheel_rr.set (  rr);

}


public double getPowerLF ()  {return -wheel_lf.get();}
public double getPowerRF ()  {return  wheel_rf.get();}
public double getPowerLR ()  {return -wheel_lr.get();}
public double getPowerRR ()  {return  wheel_rr.get();}



// Set drift values (does not apply values - caller must set motors)
// Drift2 is a very tight turn to pick up gear
// mag is always positive
// if y is positive, moving forward
// if x is positive, drift to right
public void applyDrift (double mag, double x, double y)
 {
        // Always Drift Right
        speed_LF = 0.75 * mag;
        speed_RF = 0.25 * mag;
        speed_LR = mag;
        speed_RR = 0;
      
       //Reverse (switch left/right sides and direction)
       //This also drifts to right 
       if (y<0)
       {
        speed_RF = -0.75 * mag;
        speed_LF = -0.25 * mag;
        speed_RR = -mag;
        speed_LR = 0;
       }
       
 }


// Set J-Turn values (does not apply to motors - caller must set motors)
// mag is always positive
// if y is positive, moving forward
// if x is positive, drift to right
public void applyJTurn (double mag, double x, double y)
 {
 
        //Beginning of Turn
        if (y < 0)
        {
        speed_LF = -1.00 * mag;
        speed_RF = -0.50 * mag;
        speed_LR = -0.90 * mag;
        speed_RR = -0.70 * mag;
        }
        
        //End of Turn
        if (y > 0)
        {
        speed_LF = -1.00 * mag;
        speed_RF = +1.00 * mag;
        speed_LR = +0.50 * mag;
        speed_RR = -0.50 * mag;
        }
 
       
 }

  
 public double AddEffect (double src, double bias)
 {
     src += bias;
     if (src < -1.0) src = -1.0;
     if (src > 1.0) src = 1.0;
     return src; 
 }


//Designed to ramp inputs to the controllers to prevent
//extreme changes that can cause brownouts
//MAX_RAMP is a value in seconds that is the length
//of time to go from full reverse to full forward
// MAX_RAMP = 0.5 
  
public double rampPower (double old_power, double new_power)
{
    //Maximum power transmission
    // MAX_RAMP = 0.5 (1/2 sec for 2 unit change)
    // delta_t  = 0.1 second
    // max_inc  = 2.0 / 0.5 * 0.1 = 0.4
    double max_inc = 2.0 / MAX_RAMP * ((time_now - time_prev)/ 1000000000.0);

    //If it exceeds the ramp amount, clip it to max
    if (new_power - old_power >  max_inc) new_power = old_power + max_inc;
    if (new_power - old_power < -max_inc) new_power = old_power - max_inc;
	return new_power;
}


public double getPrecisionValue ()
{
	//factor is amount of power reduction
	// 0 = None
	// 1 = Full Reduction (to 25%)
	double factor = 0;
	
	//Default is value from Driver's Right Trigger
	if ((driver.getRawAxis(XboxMap.AXIS_R_TRIGGER)> 0.50) || (shooter.getRawAxis(XboxMap.AXIS_R_TRIGGER)> 0.50))
	factor= 1.00; 

	
	// If tracking mode is pressed, then set to 1.0
	// so that net effect is 0.25
	// Note: does not have to have a lock for precision mode
	// to be in effect (just pushing button)
	// Either Gear or Rope
	if (driver.getRawButton(XboxMap.BTN_RIGHT_BUMPER)) factor = 1.0;
    if (driver.getRawButton(XboxMap.BTN_B)) factor = 1.0;
	
	return (1.0  - 0.75 * factor);
	
}
 
}