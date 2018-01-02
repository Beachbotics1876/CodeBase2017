//Emily 
package robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;

public class Roller {

	public final int ROLLER = 8;
	public VictorSP roller;
	public Joystick shooter;
					
public Roller() {

	roller 	= new VictorSP (ROLLER);
}
			
public void update() {

double power = getRollerPower();
setRollerPower(power);
}

public double getRollerPower() {
		
		double r = 0; //shooter.getRawAxis(XboxMap.AXIS_R_TRIGGER); //forward 
		double l = shooter.getRawAxis(XboxMap.AXIS_L_TRIGGER); //reverse
	return (r-l);
	}
	
public void setRollerPower(double power) {
		roller.set (-power); 

}	




}






