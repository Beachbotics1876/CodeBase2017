package robot;

//OpenCV classes
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;



public class ProcessTarget {

    Rect        rect1     = new Rect();           //Bounding rectangle of target 1
    Rect        rect2     = new Rect();           //Bounding rectangle of target 2
    RotatedRect rect_gear = new RotatedRect();    //Bounding rectangle of gear
    Rect        rect_rope = new Rect();           //Bounding rectangle of rope
   
    Scalar  color = new Scalar(0,255, 0); //Color of selection
	boolean target_ok     = false;       //True if a valid target is found
    double     center        = 160;                //Center of targets
	
// Process image, applying filters
// and other cleanup operations
// Output should become input for 
// Locating targets
/*
Seemed to be more affected by background, switching to HSL
public Mat processTarget  (Mat mat)
	{
	//Threshold image and split
	Mat rgbThresholdInput = mat;
	Mat rgbThresholdOutput = new Mat();
    //Note: do not use 256 as high end - will not accept
	double[] rgbThresholdRed =   {235.0, 255.0};
	double[] rgbThresholdGreen = {235.0, 255.0};
	double[] rgbThresholdBlue =  {235.0, 255.0};
	rgbThreshold(rgbThresholdInput, rgbThresholdRed, rgbThresholdGreen, rgbThresholdBlue, rgbThresholdOutput);

    return rgbThresholdOutput;
	}
*/
//Looking for vision targets using HSL
public Mat processTarget  (Mat mat)
	{
	//Threshold image and split
	Mat hslThresholdInput = mat;
	Mat hslThresholdOutput = new Mat();

    //Note: do not use 256 as high end - will not accept
	double[] hslThresholdH =   {  0.0, 180.0};
	double[] hslThresholdS =   {  0.0, 255.0};
	double[] hslThresholdL =   {220.0, 255.0};
	hslThreshold(hslThresholdInput, hslThresholdH, hslThresholdS, hslThresholdL, hslThresholdOutput);
    return hslThresholdOutput;
	}


public Mat processGear  (Mat mat)
	{
	//Threshold image and split
	Mat hslThresholdInput = mat;
	Mat hslThresholdOutput = new Mat();
    //Note: do not use 256 as high end - will not accept
	//double[] hslThresholdH =   {0.0, 28.0};
	//double[] hslThresholdS =   {181.0, 222.0};
	//double[] hslThresholdL =   {  5.0, 144.0};
	double[] hslThresholdH =   {0.0, 28.0};
	double[] hslThresholdS =   {131.0, 255.0};
	double[] hslThresholdL =   {  28.0, 255.0};
	hslThreshold(hslThresholdInput, hslThresholdH, hslThresholdS, hslThresholdL, hslThresholdOutput);
    return hslThresholdOutput;
	}

public Mat processRope  (Mat mat)
	{
	//Threshold image and split
	Mat hslThresholdInput = mat;
	Mat hslThresholdOutput = new Mat();
    //Note: do not use 256 as high end - will not accept
	//double[] hslThresholdH =   {0.0,    40.0};
	//double[] hslThresholdS =   {213.0, 255.0};
	//double[] hslThresholdL =   {183.0, 255.0};
	double[] hslThresholdH =   {0.0,    46.0};
	double[] hslThresholdS =   {177.0, 255.0};
	double[] hslThresholdL =   {11.0, 194.0};
	hslThreshold(hslThresholdInput, hslThresholdH, hslThresholdS, hslThresholdL, hslThresholdOutput);
    return hslThresholdOutput;
	}


//Given an image that has been processed (and is binary),
// attempt to locate gear in image
// returns true if successful
public boolean locateGear  (Mat mat)
{
	//Find Contours
	Mat findContoursInput = mat;
	ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
	boolean findContoursExternalOnly = false;
	findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

	//Find bounding box of largest contour
	int numContours  = findContoursOutput.size();
	RotatedRect rc   = new RotatedRect();
	double area      = 0; 
	double max_area  = 0; 
 	target_ok        = false;
    center           = 160;
	for (int i = 0; i < numContours; i++)
		{
		MatOfPoint seg = findContoursOutput.get(i);
        if (seg.total() < 100 )  continue;
	    MatOfPoint2f points = new MatOfPoint2f (seg.toArray());
	  	rc  = Imgproc.fitEllipse(points);
	    area = rc.size.area();
		if (area > max_area || max_area == 0) 
		    {
		   	max_area   = area;
		   	rect_gear  = rc;
        	target_ok = true;
            center = rect_gear.center.x;
		    }	
     	}
	
    return target_ok;
    }

// Given an image that has been processed (and is binary),
// attempt to locate ROPE in image
// returns true if successful
public boolean locateRope (Mat mat)
{
	//Find Contours
	Mat findContoursInput = mat;
	ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
	boolean findContoursExternalOnly = false;
	findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

	//Find bounding box of largest contour
	int numContours  = findContoursOutput.size();
	Rect rc          = new Rect();
	double area_max  = 0;
	double area      = 0; 
 	target_ok        = false;
    center           = 160;
	for (int i = 0; i < numContours; i++)
		{
		rc    = Imgproc.boundingRect(findContoursOutput.get(i)); 
    	area  = rc.area();
	    //Check if it is biggest area
	    // and the height must be at least 
	    // 4 times width
	    if (area >= area_max)
			{
	        if (rc.height >= 4*rc.width)
	        	{
   	    		area_max     = area;
   	    		rect_rope    = rc;
            	}
			}	
	    }
    //Size must be greater than 60 (4x15)
    //Center is left edge, plus 1/2 width
    if (area_max >= 60)
     	{
       	target_ok = true;
        center = rect_rope.x + 0.5 * rect_rope.width;
     	}
	
    return target_ok;
    }

//Given an image that has been processed,
//attempt to locate target in image
// returns true if successful
public boolean locateTarget  (Mat mat)
{
	//Find Contours
	Mat findContoursInput = mat;
	ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
	boolean findContoursExternalOnly = false;
	findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

	//Find bounding box of largest contour
	int numContours  = findContoursOutput.size();
	Rect rc          = new Rect();
	double area1     = 0;
	double area2     = 0;
	double area      = 0; 
 	target_ok        = false;
	center           = 160.0;
	for (int i = 0; i < numContours; i++)
		{
		rc    = Imgproc.boundingRect(findContoursOutput.get(i)); 
    	area  = rc.area();
	    //Area 1 gets first choice
	    //If greater than area1, move area1 to area2
	    if (rc.height < 1.2 * rc.width) continue ;
	    if (area >= area1)
			{
	        area2     = area1;
	    	rect2     = rect1;
	    	area1     = area;
	    	rect1     = rc;
	        }
	    //If not, then try area2
	    else 
	        {
    	    if (area > area2)
    	    	{
    	      	area2     = area;
    	      	rect2     = rc;
    	    	}
    	    }
	    
	    }

	double HeightDif = Math.abs(rect1.y - rect2.y);
    if ((area2 >= 100) && (HeightDif < 10))
     	{
     	target_ok = true;
        center = (rect1.x + rect2.x) / 2.0;
     	}
	
    return target_ok;
    }

    
//Draw a rectangular target on the captured image
//But only if the target is valid
public Mat drawTarget  (Mat mat)
		{
        if (target_ok)
          	{
          	Imgproc.rectangle(mat, rect1.tl(), rect1.br(), color,2);
          	Imgproc.rectangle(mat, rect2.tl(), rect2.br(), color,2);
          	}
        return mat;
		}

//Draw a rectangular gear target on the captured image
//But only if the target is valid
public Mat drawGear  (Mat mat)
		{
        if (target_ok)
          	{
          	Imgproc.ellipse(mat, rect_gear, color,2);
          	}
        return mat;
		}



//Draw a rectangular gear target on the captured image
//But only if the target is valid
public Mat drawRope  (Mat mat)
		{
        if (target_ok)
          	{
          	Imgproc.rectangle(mat, rect_rope.tl(), rect_rope.br(), color,2);
          	}
        return mat;
		}

//Draw alerts on screen 
//- backup enabled
//- scoop down 
public Mat drawAlert  (Mat mat, boolean show_backup, boolean show_scoopdown)
	{
    Scalar color  = new Scalar(0,255,0);
    int thickness = 4;
	//DownScoop is a Down arrow 
    if (show_scoopdown)
        {
       	Point  pos1 = new Point (300,40);
    	Point  pos2 = new Point (300,70);
    	int line_type = Imgproc.LINE_4;
    	int shift = 0;
    	double tipLength = 0.5; 
    	Imgproc.arrowedLine(mat, pos1, pos2, color, thickness, line_type, shift, tipLength);
        }
	//Draw Diamond in upper right hand corner 
    if (show_backup)
		{
        Point  pos    = new Point (300,20);
        int size      = 20;
        Imgproc.drawMarker(mat, pos, color, Imgproc.MARKER_DIAMOND, size, thickness, Imgproc.LINE_AA);
		}
    
    return mat;
	}

//Draw alignement rectangle to make sure
// camera is pointed in the correct location
public Mat drawAlign (Mat mat)
		{
        Rect rc = new Rect ();
        rc.x = 164;
        rc.y = 120;
        rc.height = 120;
        rc.width = 1;
        Scalar  color = new Scalar(0, 255, 0); //Color of selection
	    Imgproc.rectangle(mat, rc.tl(), rc.br(), color, 1);
        return mat;
		}

//Draw a drift ellipse on captured image
public Mat drawDrift (Mat mat)
		{
        RotatedRect rc = new RotatedRect ();
        rc.center = new Point(260, 90);
        rc.size   = new Size ( 25, 15);
        rc.angle  = 0;
        Imgproc.ellipse(mat, rc, color, 2);
        return mat;
		}

//Draw a drift curve on captured image
public Mat drawLine (Mat mat)
		{
        Point pt1 = new Point(238,238);
        Point pt2 = new Point(270,90);
        Imgproc.line (mat, pt1,pt2,color, 2);
        return mat;
		}

/*
private void rgbThreshold(Mat input, double[] red, double[] green, double[] blue,
		Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2RGB);
		Core.inRange(out, new Scalar(red[0], green[0], blue[0]),
			new Scalar(red[1], green[1], blue[1]), out);
	}
*/

private void hslThreshold(Mat input, double[] h, double[] s, double[] l,
		Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HLS);
		Core.inRange(out, new Scalar(h[0], l[0], s[0]),
			new Scalar(h[1], l[1], s[1]), out);
	}


private void findContours(Mat input, boolean externalOnly,
		List<MatOfPoint> contours) {
		Mat hierarchy = new Mat();
		contours.clear();
		int mode;
		if (externalOnly) {
			mode = Imgproc.RETR_EXTERNAL;
		}
		else {
			mode = Imgproc.RETR_LIST;
		}
		int method = Imgproc.CHAIN_APPROX_SIMPLE;
		Imgproc.findContours(input, contours, hierarchy, mode, method);
	}



}
