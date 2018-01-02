/* Emily
Revision
3- corrected the state mistake 
2 - corrected brace errors
1 - Created (w/ brace errors)
 
 */

package robot;

import edu.wpi.first.wpilibj.Timer;


public class Autonomous {
	
	// Available States
	public final int STATE_IDLE    = 0; 
	public final int STATE_FORWARD1 =2;
	public final int STATE_TURN = 3;
	public final int STATE_FORWARD2 =4;
	public final int STATE_MOVING  = 1;
	public final int STATE_COMPLETE = 9;


    //Label variables 
	public int  state = STATE_IDLE;
	public double travel_time;  
	public Timer timer = new Timer();
	public int  travel_dist; 
	
	//Available modes (more than 1 possible)
	public boolean  MODE_STRAIGHT_AHEAD = false; 
	public boolean  MODE_CURVE          = false;
	public boolean  MODE_POSITION_RIGHT = false;  
	public boolean  MODE_POSITION_LEFT  = false; 


    // Depends on these classes
    public Drive      drive;  //Get reference to Drive class
	public GearLifter gear;
    public Tracker    tracker;

public void init ()
{
	state       = STATE_IDLE;
	travel_time = 0;  
	travel_dist = 0; 
	
	//Available modes (more than 1 possible)
	MODE_STRAIGHT_AHEAD = false; 
	MODE_CURVE          = false;
	MODE_POSITION_RIGHT = false;  
	MODE_POSITION_LEFT  = false; 


}

public void update () {
	
if (MODE_STRAIGHT_AHEAD)
        {
		AutoMode_StraightAhead();
	    return; 
	    }
	    
if (MODE_POSITION_LEFT)
       {
		AutoMode_Angle  (false);
		return;  
       }   
 if (MODE_POSITION_RIGHT) 		
		{
		AutoMode_Angle (true);
		return;  
		}    
}
       
public void AutoMode_Angle (boolean isRight)
{
	double t = 2.2;   //Time allocated to each stage       
	double power_left = 0; //value given to wheels before time is started 
	double power_right = 0;
	int d= drive.convertDistance (6 * 12); //encoder value to exit stage 
	//------------------
	// STATE_IDLE
	//------------------
	if (state == STATE_IDLE)
		    {
			timer.reset();
			drive. resetEncoders (); 
			timer.start();
			state = STATE_FORWARD1;
			}
			
	//-------------------
	// STATE_FORWARD1
	//------------------		
	if (state == STATE_FORWARD1)
	    	{
			t = 3.0;
			gear.gotoPosition(gear.PRESET_Raised) ;
		 d= drive.convertDistance (60);    //original was 54, then 50 ,now 60
			power_left  = +0.30;
			power_right = +0.30;
			// - stop moving (set power to 0)
			// - record travel time 
			
			travel_time = timer.get ();
			travel_dist = drive.getLeftPosition (); 
			if (travel_time >= t || travel_dist >= d)  
				{
				power_left  = 0;
				power_right = 0;
   				timer.reset();
   				drive. resetEncoders (); 
				state = STATE_TURN;
				}
			drive.setRawPower (power_left,power_right,power_left,power_right);
		
	    	}
	 //-------------------
	 //STATE_TURN
	 //-------------------
if (state == STATE_TURN)
	    	{
			//Default position is to turn left
			// isRight == true
			t = 2.0;
			power_left  = -0.10;
			power_right = +0.30;
			travel_dist = drive.getRightPosition (); 
			gear.gotoPosition(gear.PRESET_Hang) ; 
			// - stop moving (set power to 0)
			if (isRight == false) 
				{
				power_left = +0.30;
				power_right = -0.10;
				travel_dist = drive.getLeftPosition ();
				} 
			travel_time = timer.get ();
			d = drive.convertDistance (38);  
			if (travel_time >= t || travel_dist >= d) 
				{
				power_left  = 0;
				power_right = 0;
				timer.reset();
				drive. resetEncoders ();  
				state = STATE_FORWARD2;
				}
			drive.setRawPower (power_left,power_right,power_left,power_right);
	    	}	
	   //----------------
	   //STATE_FORWARD2
	   //-----------------    	
if (state == STATE_FORWARD2)
	    	{
			t = 2.0;
			gear.gotoPosition(gear.PRESET_Hang) ; 
			d = drive.convertDistance (40);
			power_left  = +0.30;
			power_right = +0.30;
			// - stop moving (set power to 0)
			//Tracking routine
			power_left = tracker.power_lf;
			power_right = tracker.power_rf;
			
			
			
			travel_time = timer.get ();
			travel_dist = drive.getLeftPosition();
			if (travel_time >= t || travel_dist>=d)
				{
				power_left  = 0;
				power_right = 0;
				timer.reset();
				drive. resetEncoders (); 
				state = STATE_COMPLETE;
				}
			drive.setRawPower (power_left,power_right,power_left,power_right);
	    	}	 
	   //-------------
	   //STATE_COMPLETE
	   //---------------   
if (state == STATE_COMPLETE)
		{
			state = STATE_IDLE;
			MODE_POSITION_LEFT = false;
			MODE_POSITION_RIGHT = false;
		}	    		

}

	
public void AutoMode_StraightAhead ()		
{
    double t = 5.0;        //Maximum time to allow for stage to complete   
	double power_left = 0; //value given to wheels before time is started 
	double power_right = 0;
	int d= drive.convertDistance (68); //encoder value to exit stage 
	//-----------------
	// STATE_IDLE
	//------------------
	if (state == STATE_IDLE)
		    {
				timer.reset();
				timer.start();
				drive. resetEncoders ();  
				state = STATE_MOVING;
			}
	//------------------
	// STATE_MOVING
	//------------------
	if (state == STATE_MOVING)
	    	{
			power_left  = +0.3;
			power_right = +0.3;
			// - stop moving (set power to 0)
			// - record travel time
			//Tracking routine
			power_left = tracker.power_lf;
			power_right = tracker.power_rf;

			travel_time = timer.get ();
			travel_dist = drive.getLeftPosition();
			gear.gotoPosition(gear.PRESET_Hang) ;
			if (travel_time >= t || travel_dist >= d)
				{
				//travel_time = timer.get ();
				power_left  = 0;
				power_right = 0;
				state = STATE_COMPLETE;

				}
			drive.setRawPower (power_left,power_right,power_left,power_right);
	    	}
	    	
	    	
		if (state == STATE_COMPLETE)
		{
			state = STATE_IDLE;
			MODE_STRAIGHT_AHEAD = false;
		}
		
}
		

			
	
	    	
		
			
					    	

}




