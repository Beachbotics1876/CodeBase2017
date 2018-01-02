//Revision
// 12/01/2016 - Convert to single camera
// 11/18/2016 - Streamline and document

package robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.CameraServer;






import edu.wpi.first.wpilibj.Timer;

//OpenCV classes
import org.opencv.core.Mat;


//Vision Processing Functions
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;




//Supports 2 cameras 
public class CameraFeed {
	
	//--------
	//List of possible targets
	//--------
    final int TARGET_None       = 0;
    final int TARGET_Peg        = 1; //Target gear placement peg
    final int TARGET_Gear       = 2; //Target gear on floor
    final int TARGET_Rope       = 3; //Target hanging rope
	
	int    cam_id         = 0;
	Mat    source         = new Mat(); //Original Source image (color)
	Mat    output         = new Mat(); //Processed Output image 
	
	int     source_id     = 0;     //Source for imaged camera
	boolean cameraActivef = true;  //True when camera is Active
	boolean cameraKillf   = false; //True to force kill camera process
	boolean switch_ok     = true;  //True if okay to switch cameras 
    int     target_id     = 0;     //Object currently targeting
    boolean show_align    = false;  //True to display camera alignment target
    boolean show_backup   = false;  //True to display backup alert
    boolean show_scoopdown   = false;  //True to display backup alert
    
  
	ProcessTarget procTarget;      //Handle image processing
    Joystick      shooter;         //Shooter joystick
    
	public CameraFeed() 
	    {
		procTarget = new ProcessTarget();
	    }
	
	

	
//Initiate Camera Capture
//Create a new thread and start it
public void cameraInit() {


		    Thread captureThread = new Thread(new Runnable() {
		      @Override
		      public void run() {
		        capture();
		      }
		    });
		    captureThread.setName("Camera Capture Thread");
		    captureThread.start();
}



protected void capture() {

    // Delay to allow joysticks to initialize
    // Does not seem to work - always has a problem
    // Timer.delay(5);
    
    //Initialize cameras and settings
    //320x1240 seems to be the best
    //blend between resolution and lag-time
    //20fps gets rid of occasional bandwidth errors
    // 25/15 still produces bandwidth errors
    // 30/10 good
    // 20/20 
 	//Try setting buffer size to 2 images 



 	//UsbCamera cam0 = CameraServer.getInstance().startAutomaticCapture(0);
 	UsbCamera cam0 = new UsbCamera ("USB Camera 0", 0);
 	cam0.setResolution(320,240);
 	cam0.setBrightness(5);
 	cam0.setExposureManual(35);
 	cam0.setFPS(20);
 	
 	//Should match driver station settings
 	//UsbCamera cam1 = CameraServer.getInstance().startAutomaticCapture(1);
 	UsbCamera cam1 = new UsbCamera ("USB Camera 1", 1);  //CameraServer.getInstance().startAutomaticCapture(0);
 	cam1.setResolution(320,240);
 	cam1.setFPS(20);
 	cam1.setBrightness(5);
    
    CvSink   cvSink   = CameraServer.getInstance().getVideo(cam0);
    CvSource cvSource = CameraServer.getInstance().putVideo("Current View", 320, 240);
    
    //Set brightness to 5%
    //Setting it using UsbCamera.setBrightness does not seem to work 

    //
                
    while(!Thread.interrupted() && !cameraKillf)
    	{
       	//Grab multiple images
       	//display last image
       	//Score: poor
       	//cvSink.grabFrame(buffer, 0.02);
       	//cvSink.grabFrame(source, 0.02);
       	//if (source.empty()) source = buffer;
       	//if (source.empty()) continue;             
        
       	//cvSink.grabFrame(source, 0.04);
       	//if (source.empty()) continue;             
 
        //Use old version (no timeouts)
       	cvSink.grabFrameNoTimeout(source);
        if (source.empty()) continue;
       	
        //If cam switch pressed, switch cams
        if (switch_ok && shooter.getRawButton (XboxMap.BTN_BACK))
          	{ 
          //	cvSink.setEnabled (false);
          	if (cam_id == 0) 
          		{
//             	cam0.setFPS(20);
  	      		cvSink.setSource(cam1);
//           	    cam1.setFPS(20);
  	      		cam_id = 1;
          		}
          	else 
          		{
           		//cam0.setBrightness(5); Restore brightness to desired level
//         		cam1.setFPS(20);
  	      		cvSink.setSource(cam0);
//           	    cam0.setFPS(20);
  	      		cam_id = 0;
          		}  	
          
          //cvSink.setEnabled (true);
          switch_ok = false;
          continue;
          }    
     
        //Once button released, enable switching again
        if (!shooter.getRawButton (XboxMap.BTN_BACK)) switch_ok = true;
        
        //Default is to set output to source
        //directly
        output = source;

        //This block is used to target 2 boxes
        //next to gear peg
        if (cam_id == 0 && target_id == TARGET_Peg) 
        	{
        	//Process target 
        	// - Threshold image and extract binary image
        	output = procTarget.processTarget(source);
        	
        	//Locate target in image and set location values
        	procTarget.locateTarget (output);
       	
        	//Draw target on the source image
        	output = procTarget.drawTarget (source);
        	} 

        //If attempting to target a gear, then do it
        if (cam_id == 0 && target_id == TARGET_Gear) 
        	{
        	//Perform processing on image, updating targets
        	//Process target 
        	// - Threshold image and extract binary image
        	output = procTarget.processGear(source);
        	
        	//Locate target in image and set location values
        	procTarget.locateGear (output);
     		
        	//Draw target on the source image
        	output = procTarget.drawGear (source);
        	} 

        //If attempting to target a rope, then do it
        if (cam_id == 1 && target_id == TARGET_Rope) 
        	{
        	//Perform processing on image, updating targets
        	//Process target 
        	// - Threshold image and extract binary image
        	output = procTarget.processRope(source);
        	
        	//Locate target in image and set location values
        	procTarget.locateRope (output);
     		
        	//Draw target on the source image
        	output = procTarget.drawRope (source);
        	} 



        //If displaying alignment target
        if (cam_id == 0 && show_align) 
        	{
        	output = procTarget.drawAlign (source);
        	}

        //If displaying backup alert
        if (cam_id == 0)
        	{
        	if (show_backup || show_scoopdown) 
        	    output = procTarget.drawAlert (source,show_backup,show_scoopdown);
        	}

        //Display result to screen
      	cvSource.putFrame(output);
          
     	Timer.delay(0.005);
    	}  

    } 





}

	

