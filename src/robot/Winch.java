/* 
 Kieran
 
 V1.4
 Switched joysticks to reference Joysticks created by Robot
 instead of creating own version
 
 V1.3
 Improved program flow
 
 V1.2
 Change Log:
 Made edits to IsAtTop function to make it also work in autonomuos mode
 Added a slow mode which can be activated if you press the A button
 */
//



package robot;

//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;

public class Winch {
	
	
	public final int WINCH1 = 5;
	public final int WINCH2 = 6;
	
	public final int LEFT_SWITCH = 7;
	public final int RIGHT_SWITCH = 8;
	
	public VictorSP winch1;
	public VictorSP winch2;
	
	public Joystick shooter;
	
	
public Winch() {
		winch1 	= new VictorSP (WINCH1);
		winch2 	= new VictorSP (WINCH2);
	}
	
public void update() {
		
		//By default, get power from joystick
		double power = getWinchPower();
		if (power>=-0.2 && power<=0.2){power=0;}

        //If slow mode is on, override current power
     	if (isSlowModeOn()) {power = 0.5;}
		
		//Apply the desired power to the winch
		setWinchPower(power);	//positive is up & negative is down	
	}
	
	
public double getWinchPower() {
		
     // Positive (Edward)
     // Negative (Bryan)
 	 int dir = -1;
 	return dir * shooter.getRawAxis(XboxMap.AXIS_RIGHT_Y);
	}
	
public boolean isSlowModeOn(){
		
		return shooter.getRawButton(XboxMap.BTN_A);
	}
	
public void setWinchPower(double power) {
		
		if (power>0 && IsAtTop()) power = 0;
		winch1.set(power);
		winch2.set(power);
	}
	

public boolean IsAtTop() {
	
	// Until switches are in place, disable
	// and always return false
	//if (LEFT_SWITCH=true || RIGHT_SWITCH==true) return false;
    return false; 
	
}

}
