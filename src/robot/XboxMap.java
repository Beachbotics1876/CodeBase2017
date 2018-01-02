// Revision 4:
// - add isZero function 
// Revision 3:
// - Change members to static
// Revision 2:
// - Correct typo on b smoothing value
// - Annotate AXIS_LEFT_X/Y

package robot;


public class XboxMap 
{
    /* Button IDs */
    public static final int BTN_A = 1;
    public static final int BTN_B = 2;
    public static final int BTN_X = 3;
    public static final int BTN_Y = 4;
    public static final int BTN_LEFT_BUMPER  = 5;
    public static final int BTN_RIGHT_BUMPER = 6;
    public static final int BTN_BACK = 7;
    public static final int BTN_START = 8;
    public static final int BTN_LEFT_STICK = 9;
    public static final int BTN_RIGHT_STICK = 10;

    /* Axis IDs */
    public static final int AXIS_LEFT_X    = 0; //=1 on cRio Robots
    public static final int AXIS_LEFT_Y    = 1; //=2 in cRio Robots
    public static final int AXIS_L_TRIGGER = 2;
    public static final int AXIS_R_TRIGGER = 3;
    public static final int AXIS_RIGHT_X   = 4;
    public static final int AXIS_RIGHT_Y   = 5;

    /* Settings */
    public static double dead_zone = 0.15;  //Anything less is 0
    public static double max_zone  = 0.95;  //Anything more is 1

    //Very small numbers are cleared
    //Don't use dead_zone - some functions use 10%
    public static boolean isZero (double x)
    	{
 		return (x > -0.05 && x < 0.05 );    
    	}
 
    public static double setMax (double x)
    	{
 		if (x>1.0)  x = 1.0;
 		if (x<-1.0) x = -1.0;    
    	return x;
    	}
 
    
    public static double smoothPower (double x)
    {
    	double direction = (x>=0 ? 1 : -1);
    	x = Math.abs(x);
    	if (x <= dead_zone) return 0;
    	if (x >= max_zone)  return direction;
    	
    	
    	//Uses a smoothing curve
    	double m = Math.PI / (max_zone - dead_zone);
    	double b = -Math.PI/2.0 - m * dead_zone;
    	double y = 0.5 *(1 + Math.sin(m*x+b));
    	return direction * y;
    	
    	
    	
    }
    
    
}
