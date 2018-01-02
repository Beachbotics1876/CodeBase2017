


package robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



//import edu.wpi.first.wpilibj.CameraServer;

/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */


	
public class Robot extends SampleRobot {

	
	Drive drive = new Drive();
    Winch winch = new Winch();
    GearLifter gear  = new GearLifter ();
    Tracker  tracker = new Tracker   ();
    Autonomous auto = new Autonomous ();
    Roller roller  = new Roller ();
    //Relay  led     = new Relay (0);
    
	DriverStation driverStation = DriverStation.getInstance();
	SendableChooser <String> chooser;
	CameraFeed camera = new CameraFeed ();
	
    public final int JOYSTICK1 = 1;
    public final int JOYSTICK2 = 2;
    
    public Joystick driver = new Joystick (JOYSTICK1);
    public Joystick shooter = new Joystick (JOYSTICK2);

    //Autonomous Modes
    public final String AM_Nothing  = "Nothing";
    public final String AM_Straight = "Straight Ahead";
    public final String AM_LeftPos  = "Position Left";
    public final String AM_RightPos = "Position Right";
    
         
    public Robot() 
    {   	
    	
        //Assign class joysticks
        drive.driver  = driver;
        drive.shooter = shooter;
        winch.shooter = shooter;
        gear.shooter  = shooter;
        gear.driver   = driver;
        camera.shooter = shooter;
        roller.shooter = shooter;
       

    	//Provide reference to drive to autonomous
    	auto.drive = drive;
  		auto.gear  = gear;
		auto.tracker = tracker;
		
		//Additional references to classes
        drive.tracker = tracker;
        drive.camera  = camera;
		
 
    	 

    }
//Overall, one-time call here
@Override
public void robotInit() 
{    	


    	//Launch the camera and begin capture
    	camera.cameraInit();
    	camera.show_align = true;

    	//Populate autonomous modes
        //chooser = new SendableChooser ();
        //String mode1 = "Nothing";
        //String mode2 = "Straight Ahead";
        chooser = new SendableChooser <String> ();
    	chooser.addDefault ("Auto: Nothing",              AM_Nothing);
    	chooser.addObject  ("Auto: Pos 1 (Turn Right)",   AM_LeftPos);
    	chooser.addObject  ("Auto: Pos 2 (Straight)",     AM_Straight);
    	chooser.addObject  ("Auto: Pos 3 (Turn Left)"  ,  AM_RightPos);
    	SmartDashboard.putData("Autonomous mode chooser", chooser);
      
        //Initialize motors
        drive.setPower(0,0,0,0);
        gear.setLiftPower  (0);
        winch.setWinchPower(0);
        

}

//Autonomous functions go here
@Override
public void autonomous() {    	

    //Initialize autonomous values
    //Initialize values
   	camera.show_align = false;
    auto.init ();
 
    String auto_mode = (String) chooser.getSelected (); 
    auto.MODE_STRAIGHT_AHEAD = (auto_mode == AM_Straight);
    auto.MODE_POSITION_LEFT  = (auto_mode == AM_LeftPos);
    auto.MODE_POSITION_RIGHT = (auto_mode == AM_RightPos);

    if (auto_mode == AM_Nothing)  SmartDashboard.putString("Auto Selection", AM_Nothing);
    if (auto.MODE_STRAIGHT_AHEAD) SmartDashboard.putString("Auto Selection", AM_Straight);
    if (auto.MODE_POSITION_RIGHT) SmartDashboard.putString("Auto Selection", AM_RightPos);
    if (auto.MODE_POSITION_LEFT ) SmartDashboard.putString("Auto Selection", AM_LeftPos);


    
    
    //Turn on targeting
    camera.target_id  = camera.TARGET_Peg;
    //led.set(Relay.Value.kForward);

    while (isAutonomous() && isEnabled()){

    	//Get current target position and feed to Tracker
    	tracker.update (0.30,camera.procTarget.center, camera.procTarget.target_ok);
    		
    	//Update Autonomous class (using new Tracker values)
    	auto.update ();
        
  	
        //Update display
       	updateDashboard ();
        Timer.delay(0.005);
    	}

    camera.target_id  = camera.TARGET_None;
   	camera.show_align = true;
    //led.set(Relay.Value.kOff);
  }
    		
    		
//Operator Control Functions go here
@Override
public void operatorControl() {

        //Turn off alignment mark
       	camera.show_align = false;
        //led.set(Relay.Value.kForward);
        gear.clearFlags ();
        while (isOperatorControl() && isEnabled())
        	{
        	//If targeting, turn it on
        	//Right bumper can either be rope or peg,
        	// depending on camera 
        	camera.target_id  = camera.TARGET_None;
        	if (driver.getRawButton(XboxMap.BTN_RIGHT_BUMPER))
        		{
        		//If 0, this is the forward facing camera - target peg
        		//If 1, this is the read facing camera - target rope
        		if (camera.cam_id == 0) camera.target_id = camera.TARGET_Peg;
        		if (camera.cam_id == 1) camera.target_id = camera.TARGET_Rope;
        		}

        	if (driver.getRawButton(XboxMap.BTN_B))
        		{
           		camera.target_id  = camera.TARGET_Gear;
        		}
        	
        	//If gear detected,
        	// set backup flag 
        	// set auto_lift flag
        	if (gear.inPlace())
        		{
             	drive.back_up        = true;
        	 	gear.auto_lift       = true;
        		}
        	
        	//Turn on/off alert depending on if backup mode is in use
        	camera.show_backup    = (drive.state == drive.STATE_BACKUP);
        	camera.show_scoopdown = (gear.isLowered());
        	
        	
        	//Each assembly/module is updated here
       		drive.update();
       		winch.update ();
       		gear.update ();
       		roller.update ();

        	updateDashboard ();       	
        	Timer.delay(0.005);
        	}
    
  	camera.target_id   = 0;
  	camera.show_align  = true;
    camera.show_backup = false;
    }

public void updateDashboard ()
{
       	SmartDashboard.putNumber("Power LF", drive.getPowerLF());
        SmartDashboard.putNumber("Power RF", drive.getPowerRF());
        SmartDashboard.putNumber("Power LR", drive.getPowerLR());
        SmartDashboard.putNumber("Power RR", drive.getPowerRR());
        
        SmartDashboard.putNumber("Wheel Right Position", drive.getRightPosition());
		SmartDashboard.putNumber("Wheel Left Position", drive.getLeftPosition());

 		//Gear Detected is instantaneous switch value (direct from sensor - not the flag)
 		//Gear Power is value from the motor (not the target power)
        SmartDashboard.putNumber("Gear Lift Pos", gear.getPosition());
        SmartDashboard.putNumber("Gear Power",    gear.lifter.get());
 		SmartDashboard.putBoolean("Gear Detected", gear.inPlace());
//	SmartDashboard.putBoolean("Gear Detected", gear.gear_in.get());
  
        SmartDashboard.putNumber("Target1: area", camera.procTarget.rect1.area());
        SmartDashboard.putNumber("Target2: area", camera.procTarget.rect2.area());
        SmartDashboard.putNumber("Target Rope: ", camera.procTarget.rect_rope.area());
        SmartDashboard.putNumber("Target Center", camera.procTarget.center);
  
  
 		SmartDashboard.putNumber("State", auto.state);
  
}



//Test functions
 @Override
 public void test() {
    
    //Test locating gear
    //Turn off alignment mark
   	camera.show_align = false;
   	camera.target_id  = 0;
    while (isTest() && isEnabled())
        	{
            //Camera runs in separate process, nothing to do
        	camera.target_id  = camera.TARGET_None;
        	if (driver.getRawButton(XboxMap.BTN_RIGHT_BUMPER))
        		{
        		//If 0, this is the forward facing camera - target peg
        		//If 1, this is the read facing camera - target rope
        		if (camera.cam_id == 0) camera.target_id = camera.TARGET_Peg;
        		if (camera.cam_id == 1) camera.target_id = camera.TARGET_Rope;
        		}

        	if (driver.getRawButton(XboxMap.BTN_B))
        		{
           		camera.target_id  = camera.TARGET_Gear;
        		}

        	//Each assembly/module is updated here
        	updateDashboard ();       	
        	Timer.delay(0.01);
        	}
    
  	camera.target_id  = 0;
  	camera.show_align = true;
    //led.set(Relay.Value.kOff);
 
    
    }
   
}
    
