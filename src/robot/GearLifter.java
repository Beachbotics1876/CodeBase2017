//Emily 
// Rev
// 5 - Disable reduced power settings when close (using brake now)
//   - Decreased error range from 5 to 2
// 4 - Add Raise and Look command (RIGHT_BUMPER)
//   - Separate raise and lower max power values
//   - Add Kieran's formula for scaling back power
// 3 - Use Robot joysticks, don't create own version
// 2 - Switch to POV commands for lifter


package robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DigitalInput;

public class GearLifter {

	public int PRESET_Raised = 0;   
	public int PRESET_Hang   = 85;  //was 110, then 100, now 85
	public int PRESET_Floor  = 365; //was 323, now 365
	public final int STATE_IDLE    = 0; 
	public final int STATE_RETRACT = 1;
	public final int STATE_HANG    = 2;
	public final int STATE_COMPLETE =3;

	public int position;         //Current position (0=Fully raised)
	public int DIST_STOP  = 2;   //Maximum error in target position 
	public int DIST_CLOSE = 20;  //Close distance (slow down) 
	public int vibrate    = 0;   //+1 = Raise cyle, -1=Lower cycle
	public int state      = STATE_IDLE;
	
	public double MAX_RAISE   = 0.75;  //Maximum when raising 
	public double MAX_LOWER   = -0.50; //Maximum when lowering 
	public boolean raise_lock = false; //True=Raise lifter and keep locked 
	public boolean auto_lift  = false; //True=Automated gear lift 
	public boolean kill_lift  = false; //True=Ignore Auto lift 
	
	

	public VictorSP lifter;
	public Joystick shooter;
	public Joystick driver;
	public DigitalInput raisedswitch; 
	public DigitalInput loweredswitch; 
	public DigitalInput gear_in;          //Detects gear in position 
	public Encoder      encoder; 
	
	
public GearLifter () {

		lifter 	      = new VictorSP (7);  //PWM 7
		raisedswitch  = new DigitalInput (1); //DIO #1 
		loweredswitch = new DigitalInput (2); //DIO #2
		gear_in       = new DigitalInput (9); //DIO #0
		encoder       = new Encoder(7,8);     //DIO #7,8
		encoder.reset ();
	    
}

public void resetPosition ()
{
if (state == STATE_IDLE)
		    {
			state = STATE_RETRACT;
			}
if (state == STATE_RETRACT)
            {
           gotoPosition(PRESET_Raised);
           if (isRaised()) state = STATE_HANG;
            }
if (state == STATE_HANG)
			{
			if (gotoPosition(PRESET_Hang)) state = STATE_COMPLETE;
			}

if (state == STATE_COMPLETE)
			{
			gotoPosition(PRESET_Hang);
            if (!shooter.getRawButton(XboxMap.BTN_X)) state= STATE_IDLE;
			} 			
}			



public void update () {	

    double power = 0;
    if (shooter.getRawButton(XboxMap.BTN_START))  encoder.reset ();  
 	
    //Update position
    getPosition ();

    //If raise and lock is used set the flag
    //Keep that power, unless overruled elsewhere
    if (shooter.getRawButton(XboxMap.BTN_RIGHT_BUMPER))  raise_lock = true;

    //If gear is in place (and raised) 
    //then this is a problem - kill lift option
    //auto_lift is set in Robot module loop
    if (inPlace() && isRaised()) 
	    {
	    kill_lift = true;
	    auto_lift = false;
	    }
    		
	
	//If a preset is applied, then go to that preset
	// B = Vibrate Mode
	if (shooter.getRawButton(XboxMap.BTN_B))
		{
		clearFlags();
		if (vibrate == 0) vibrate = -1; //Begin with lowering
		gotoPosition(PRESET_Hang, true);
		return;
		}
	// Y = Hang Position
	if (shooter.getRawButton(XboxMap.BTN_Y))
		{
		clearFlags();
		gotoPosition(PRESET_Hang);
		return;
		}

	// X = Reset position and hang
	if (shooter.getRawButton(XboxMap.BTN_X) || state != STATE_IDLE)
		{
		clearFlags();
		resetPosition ();
		return;
		} 
    
  
    //If no preset, then use the default
    //Get desired power and apply to motor    
    //If power less than zero, disable raise_lock
    power = getLiftPower ();
    if (power < 0 && raise_lock) raise_lock = false;
    if (power < 0 && auto_lift)  auto_lift  = false;
    if (raise_lock) power = MAX_RAISE; 
	if (auto_lift)  power = MAX_RAISE; 
	setLiftPower (power); 
	
	
} 


//Gets current position
//Note: 0 = fully raised, then increases
//When fully raised, encoder is reset to 0
public int getPosition () 
{
	if (isRaised()) encoder.reset();
	position = -encoder.get();  
	return position;
}

//Current options are to use either
//Left Y or POV commands
// +1 = Raise lifter
// -1 = Lower lifter
public double getLiftPower () {

    double power = 0;

     
    //If driver is using control that takes priority
    //Use 1/2 speed because that is in a tight spot
    if (driver.getRawButton(XboxMap.BTN_LEFT_BUMPER))
    	{
        return 0.85;
    	}
    if (shooter.getRawButton(XboxMap.BTN_LEFT_BUMPER))
    	{
        return 0.85;
    	}

    //Get default value from Joystick
    //If not zero, then return it
    power = -XboxMap.smoothPower(shooter.getRawAxis(XboxMap.AXIS_LEFT_Y));
    if (!XboxMap.isZero(power))
    {
    	//Raising, multiply by max raise
    	//MAX_LOWER is negative, so need to adjust sign since power is negative
    	if (power > 0)  return MAX_RAISE * power;
    	else return -MAX_LOWER * power;
    }
    //Not using joystick, then look at POV
    //Up is Positive 
    int pov = shooter.getPOV();
    if (pov == 315 || pov == 0   || pov == 45)  power = MAX_RAISE;
    if (pov == 135 || pov == 180 || pov == 225) power = MAX_LOWER;
    return power;
}
     
 
 //Called to set lift power
 // Slow down if getting close to extreme values
 public void setLiftPower (double power){
 	
    if (XboxMap.isZero(power))
    	{ 
    	lifter. set(power);
    	return;
    	}
    	
 	//If raising (+1)
 	// - if switch triggered, stop
 	// - if close, set max at 20% power
 	if (power > 0)
 		{
 		if (isRaised()) power = 0;
 		}
    // If moving fast and within 10 clicks of fully raised
    // Dial it back to 0.30
 	// Obsolete - using brake mode
 	/*
 	if (power > 0.30)
 		{
 		if (position <= PRESET_Raised + 10) power = 0.30;
 		}
	*/
	
 	//If lowering (-1)
 	// if switch triggered, stop
 	// clear raise_lock
 	if (power < 0)
 		{
 		clearFlags();
	 	if (isLowered()) power = 0;
 		}

    // If moving fast down and within 10 clicks of fully lowered
    // Dial it back to -0.30
 	// Obsolete - using brake mode
 	/*
 	if (power < -0.30)
 		{
 		if (position >= PRESET_Floor - 10) power = -0.30;
 		}
    */
    
 	lifter. set(power); 
 }

// Position starts at 0 and increases as lifter is lowered
// So,
// if position > target, need to raise (+1)
// if position < target, need to lower (-1)
// Future - put a variable if overshoots
public boolean gotoPosition(int target) {return gotoPosition(target, false);}
public boolean gotoPosition(int target, boolean isVibrate) 
	{
	double  power  = 0;
    if (!isVibrate) vibrate = 0;
	
	//Extreme values use special functions
	if (target == PRESET_Raised) return gotoRaised();
	if (target == PRESET_Floor ) return gotoFloor ();
	
	
	//If in range (and not vibrating), then exit
   	if (inRange(target, position) && !isVibrate)
	   		{
     		setLiftPower (0);
    		return (true);
       		}	
	      
	//If too high
    //  - Assume max lowering power
	//  - If close
	//     - If no vibrate then scale power
    //     - If vibrate, then continue same direction  
    //  - If not close and vibrate on, then go to direction
	if (position <= target)
		{
	   	power= MAX_LOWER;
	   	if (isClose (target,position)) 
	   	     {
	   	     power *= getCloseScale (target,position);
             //Lowering sequence of vibrate  - keep lowering
             if (vibrate == -1) power = MAX_LOWER;
             //Raising sequence of vibrate  - keep raising
             if (vibrate == +1) power = MAX_RAISE;
	   	     }
	   	//If not close to target anymore, force lower 
	   	else if (vibrate != 0) vibrate = -1;
	   	      	   	     
    	setLiftPower (power);
   		return (false);
		}
	  
	//If too low
    //  - Assume max raising power
	//  - If close (and no vibrate) then scale 
	if (position >= target)
		{
	   	power= MAX_RAISE;
	   	if (isClose (target,position)) 
	   	     {
	   	     power *= getCloseScale (target,position);
             //Lowering sequence of vibrate  - keep lowering
             if (vibrate == -1) power = MAX_LOWER;
             //Raising sequence of vibrate  - keep raising
             if (vibrate == +1) power = MAX_RAISE;
	   	     }
	   	//If not close to target anymore, force raise 
	   	else if (vibrate != 0) vibrate = 1;
	   	     
    	setLiftPower (power);
   		return (false);
		}

	//Backup, should not get here but in case 
	//turn off motors
   	setLiftPower (power);
	return (false);
	}

// gotoRaised raises scoop to fully upright position
// Power is positive
public boolean gotoRaised() 
	{
	double  power  = MAX_RAISE;
    if (isRaised()) power = 0;

	//If close to raised, reduce power to 1/2 speed
    //Using brake - disabling reduced power settings
	//if (position < PRESET_Raised + DIST_CLOSE) power  = MAX_RAISE / 2; 
   	setLiftPower (power);
	return isRaised();
	}

// gotoFloor lowers scoop to fully down position
// Power is negative
public boolean gotoFloor() 
	{
	double  power  = MAX_LOWER;
    if (isLowered()) power = 0;
	//If close to raised, reduce power to 1/2 speed
    //Using brake - disabling reduced power settings
	//if (position > PRESET_Floor - DIST_CLOSE) power  = MAX_LOWER / 2; 
   	setLiftPower (power);
	return isLowered();
	}

//clearFlags removes all the sticky flags
public void clearFlags ()
	{
	auto_lift = false;
	raise_lock = false;
	}

//Returns true when fully raised
//The switch changes to false when the 
//magnet is activated 
public boolean isRaised ()
{
return (raisedswitch.get () == false); 
}

//Returns true when fully lowered
//Switch changes to false when the 
//magnet is activated 
public boolean isLowered ()
{
return (loweredswitch.get () == false); 
}

//Return TRUE when close to target
public boolean isClose (int target, int position)
	{
	return (Math.abs(target-position)<=DIST_CLOSE);
	}

//Return TRUE when at target (within error)
public boolean inRange (int target, int position)
	{
	return (Math.abs(target-position)<=DIST_STOP);
	}

//Return TRUE if Gear is in place (actually in place - not the flag)
//However, if kill lift is set, then always returns false
public boolean inPlace ()
	{
    //if (shooter.getRawButton(XboxMap.BTN_START)) kill_lift=true;
    //kill_lift =  !gear_in.get();
    //if (kill_lift) return false;
    //return (gear_in.get());
    return false;
	}

//getCloseScale 
//returns factor depending on how close to target
// < DIST_STOP  => 0.0
// > DIST_CLOSE => 1.0
// else:   
public double getCloseScale (int target, int position)
	{
  	double d = Math.abs(target-position);
  	if (d < DIST_STOP)  return 0.0;
  	if (d > DIST_CLOSE) return 1.0;
  	return ((d - DIST_STOP) /(DIST_CLOSE - DIST_STOP));
	}


}




		