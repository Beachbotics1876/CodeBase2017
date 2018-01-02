/*
Kieran M Ashton
Tracker 2.0
*/

//This class is used to track a target by 
//updating the wheel values to turn towards the target

package robot;

public class Tracker 
	{
	public double a_left;
	public double a_right;
	public double a_max;

	public double c_min    = 150;
	public double c_max    = 170;
	public double c_target = 160;

	public double s;
	public double s_max;
	public double s_min;

    //Current wheel power settings
	public double power_lf;
	public double power_rf;
	public double power_lr;
	public double power_rr;

	public double power_interval;


// This version uses continous tracking (instead
// of outside of a range
// Also works correctly in reverse (power is negative)
// Negative values are used for rope tracking
public void update (double power, double center, boolean tracking) {

	
	//Set power values to the default value  
	power_lf=power;
	power_rf=power;
	power_lr=power;
	power_rr=power;
    	
    //If not tracking, then exit
    //Leave tracking values at default
    if (!tracking) return;

    //If either target is too close, the stop 
    //moving forward
    double delta  = c_target - center;
    
    //if delta > 0 then c is too small (move to left)
    //if delta < 0 then c is too large (move to right)
    // 40 units  = 0.20 percent (original)
    // 60 units, 30% power = 0.18 (d * p / 100)   
    //if delta > 0 (backwards) 
    //  left = -0.30  

    power_interval = Math.abs(power) * delta/100.0;
    
     
   	power_lf -= power_interval;
    power_lr -= power_interval;
	
	power_rf += power_interval;
	power_rr += power_interval;
		
   }
/*
 * This version is obsolete
 * New version uses continuous tracking
public void update (double power, boolean tracking) {

	//power_interval=-3625a+3725; //??
	power_interval = 0.10; //Assume 10%
	
	//Set power values to the default value  
	power_lf=power;
	power_rf=power;
	power_lr=power;
	power_rr=power;
    	
    //If not tracking, then exit
    //Leave tracking values at default
    if (!tracking) return;

    //If either target is too close, the stop 
    //moving forward
    if (a_left > a_max || a_right > a_max)
    	{
		power_lf=0;
		power_rf=0;
		power_lr=0;
		power_rr=0;
    	}

    //If center of target is too far right (>max)
    //then need to turn to the right to bring it 
    //back into range  
	if (c>c_max)
	   	{
		power_lf += +power_interval;
		power_rf += -power_interval;
		power_lr += +power_interval;
		power_rr += -power_interval;
		}
	
    //If center of target is too far left (<min)
    //then need to turn to left to bring target 
    //back towards center  
	
	if (c<c_min)
	    {
		power_lf += -power_interval;
		power_rf += +power_interval;
		power_lr += -power_interval;
		power_rr += +power_interval;
*/
/*
Hold off beyond this until vision tracking is tested

	if (s>s_max){
		power_lf+=power_interval;
		power_rf+=0;
		power_lr+=0;
		power_rr+=power_interval;
	}
	if (s<s_min){
		power_lf+=0;
		power_rf+=power_interval;
		power_lr+=power_interval;
		power_rr+=0;
	}
*/

/*
	//In case any values exceed max, then trim it
    //Not likely, since the default power is 30% 
    //and should not change by more than 10%
	if (power_lf> 1) {power_lf=1;}
	if (power_lf<-1) {power_lf=-1;}
	if (power_rf> 1) {power_rf=1;}
	if (power_rf<-1) {power_rf=-1;}
	if (power_lr> 1) {power_lr=1;}
	if (power_lr<-1) {power_lr=-1;}
	if (power_rr>1)  {power_rr=1;}
	if (power_rr<-1) {power_rr=-1;}
}
*/


}









