#include "opencv2/opencv.hpp"
#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>
#include "multitracker.h"
#include <iostream>
#include <vector>
#include <time.h>
#include "string.h"
#include <omp.h>
#include <Windows.h>
using namespace cv;
using namespace std;
//stringstream ss;
FILE * pFile;         ////get coordinates for graph
FILE * pFile2;
FILE * pFile3;
FILE * pFile4;

extern "C"{
	_declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;}


Mat fgMaskMOG2;
Ptr<BackgroundSubtractor> pMOG2; 
Scalar Colors[]={Scalar(255,0,0),Scalar(0,255,0),Scalar(0,0,255),Scalar(255,255,0),Scalar(0,255,255),Scalar(255,0,255),Scalar(255,127,255),Scalar(127,0,255),Scalar(127,0,127)};
int ran[]={255,0,127};

time_t start_time;/////////////////////
//float sampling = 0.2;
float acceleration =0.2;
int max_distance= 60;
int max_misses= 60;
int max_trace= 1;
float dt = 0.2;
vector<kalman_track*> tracks;///////////
int picno;
int kalman_track::NextID=0;


/*void timer(void){
	int time=0;
	while(42){
		calc++;
		if(calc == 20){
			time=1;
			printf("done");
			break;
		}
	}
}*/

/*class timerclass{
public:
	time_t start_T;
	//timer* TF;
	//int end_T;
	//int value;

	int gettime(void)
      {
		 time_t now;
		 struct tm newyear;
		 double seconds;
      }
};*/
			



kalman_track::kalman_track(Point2f pt, float dt, float acceleration)
{
	track_id=NextID;
	//track_no=track_id;//////////////////////
	//time_t start;///////////////////////////////////
	
	NextID++;

	KF = new TKalmanFilter(pt,dt,acceleration);
	///time (&start);/////////
	///printf("%.2lf",start);///////////////////////////

	begin_time = clock();/////
	suspicious = 0;///////////////
	//std::cout << float(begin_time ) ;////


	



	//timerclass TF =  timerclass(time(NULL)); //


	// Here stored points coordinates, used for next position prediction.
	prediction=pt;
	misses=0;

}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
kalman_track::~kalman_track()
{
	// Free resources.
	delete KF;
	//delete TF; //
}


/*track_manager::track_manager(float _dt, float _acceleration, double _max_distance, int _max_misses,int _max_trace)
{
dt=_dt;
acceleration=_acceleration;
max_distance=_max_distance;
max_misses=_max_misses;
max_trace=_max_trace;
}*/
/*track_manager::track_manager(float sampling)
{
	dt = sampling;

}*/

// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------

//void track_manager::Update(vector<Point2f>& detections)
void Update(vector<Point2f>& detections)
{
	// --------------------------------
	// If there is no tracks yet, then every point begins its own track.
	// -----------------------------------
	if(tracks.size()==0)
	{
		// If no tracks yet
		for(int i=0;i<detections.size();i++)
		{
			kalman_track* tr=new kalman_track(detections[i],dt,acceleration);
			tracks.push_back(tr);
		}	
	}


	int N=tracks.size();		
	int M=detections.size();	


	vector< vector<double> > Cost(N,vector<double>(M));
	vector<int> assignment;

	double dist;
	for(int i=0;i<tracks.size();i++)
	{	
		// Point2d prediction=tracks[i]->prediction;
		// cout << prediction << endl;
		for(int j=0;j<detections.size();j++)
		{
			Point2d diff=(tracks[i]->prediction-detections[j]);
			dist=sqrtf(diff.x*diff.x+diff.y*diff.y);
			Cost[i][j]=dist;
		}
	}
	// -----------------------------------
	// Solving assignment problem (tracks and predictions of Kalman filter)
	// -----------------------------------
	AssignmentProblemSolver APS;
	APS.Solve(Cost,assignment,AssignmentProblemSolver::optimal);

	// -----------------------------------
	// clean assignment from pairs with large distance
	// -----------------------------------
	// Not assigned tracks
	vector<int> not_assigned_tracks;

	for(int i=0;i<assignment.size();i++)
	{
		if(assignment[i]!=-1)
		{
			if(Cost[i][assignment[i]]>max_distance)
			{
				assignment[i]=-1;
				// Mark unassigned tracks, and increment skipped frames counter,
				// when skipped frames counter will be larger than threshold, track will be deleted.
				not_assigned_tracks.push_back(i);
			}
		}
		else
		{			
			// If track have no assigned detect, then increment skipped frames counter.
			tracks[i]->misses++;
		}

	}

	// -----------------------------------
	// If track didn't get detects long time, remove it.
	// -----------------------------------
	for(int i=0;i<tracks.size();i++)
	{
		if(tracks[i]->misses>max_misses)
		{
			delete tracks[i];
			tracks.erase(tracks.begin()+i);
			assignment.erase(assignment.begin()+i);
			i--;
		}
	}
	// -----------------------------------
	// Search for unassigned detects
	// -----------------------------------
	vector<int> not_assigned_detections;
	vector<int>::iterator it;
	for(int i=0;i<detections.size();i++)
	{
		it=find(assignment.begin(), assignment.end(), i);
		if(it==assignment.end())
		{
			not_assigned_detections.push_back(i);
		}
	}

	// -----------------------------------
	// and start new tracks for them.
	// -----------------------------------
	if(not_assigned_detections.size()!=0)
	{
		for(int i=0;i<not_assigned_detections.size();i++)
		{
			kalman_track* tr=new kalman_track(detections[not_assigned_detections[i]],dt,acceleration);
			tracks.push_back(tr);
		}	
	}

	// Update Kalman Filters state

	for(int i=0;i<assignment.size();i++)
	{
		// If track updated less than one time, than filter state is not correct.

		tracks[i]->KF->GetPrediction();

		if(assignment[i]!=-1) // If we have assigned detect, then update using its coordinates,
		{
			tracks[i]->misses=0;
			tracks[i]->prediction=tracks[i]->KF->Update(detections[assignment[i]],1);
		}else				  // if not continue using predictions
		{
			tracks[i]->prediction=tracks[i]->KF->Update(Point2f(0,0),0);	
		}
		
		if(tracks[i]->trace.size()>max_trace)
		{
			tracks[i]->trace.erase(tracks[i]->trace.begin(),tracks[i]->trace.end()-max_trace);
		}

		tracks[i]->trace.push_back(tracks[i]->prediction);
		tracks[i]->KF->LastResult=tracks[i]->prediction;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////

	/*for( int x=0; x<track_no;x++)
	{
		tracker.tracks[x].begintime*/


}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
/*track_manager::~track_manager(void)
{
	for(int i=0;i<tracks.size();i++)
	{
	delete tracks[i];
	}
	tracks.clear();
}*/

int main(int ac, char** av)
{

   Mat frame,thresh_frame;
  vector<Mat> channels;

 
  vector<Vec4i> hierarchy;
  vector<vector<Point> > contours;
  

  vector<Point2f> centers;
  

/* pFile = fopen ("cordinatex.txt","w");
  pFile2 = fopen("cordinatey.txt","w");
  pFile3 = fopen("kalmanx.txt","w");
  pFile4 = fopen("kalmany.txt","w");*/

    cv::Mat fore;

		

    cv::BackgroundSubtractorMOG2 bg;
	//cv::BackgroundSubtractorGMG bg;

   // bg.nmixtures = 3;
   // bg.bShadowDetection = false;
    //int incr=0;

    //int track=0;

	VideoCapture cap;

 //cap.open("F://input.mp4");/////////////////
cap.open("F://4943.mp4");
// cap.open(0);
		//cap.open("/home/speedblazer/Desktop/kalmansingle/video/iphonecase.mp4");
	//cap.open("G:\Users\ROMI\Desktop\Multiple Object Tracking\iphonecase.mov");

  //cap.set(CV_CAP_PROP_FPS, 25);
  //cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
  //cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);

  if(!cap.isOpened())
  {
    cerr << "Problem opening video source" << endl;
  }



   

  while((char)waitKey(60) != 'q' && cap.grab())
    {

   //Point s, p;
  bool bSuccess =cap.retrieve(frame);


  /*stringstream ss;
        rectangle(frame, cv::Point(10, 2), cv::Point(100,20),
            cv::Scalar(255,255,255), -1);
        ss << cap.get(CV_CAP_PROP_POS_FRAMES);
		
        string frameNumberString = ss.str();
		
        putText(frame, frameNumberString.c_str(), cv::Point(15, 15),
            FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(0,0,0));*/


		centers.clear();

			
        if (!bSuccess) 
       {
             cout << "ERROR: Cannot read a frame from video file" << endl;
             break;
        }
        bg.operator ()(frame,fore);
		
		//threshold(fore,fore,127,255,CV_THRESH_BINARY);
		medianBlur(fore,fore,3);
		
        erode(fore,fore,Mat());
        erode(fore,fore,Mat());
        dilate(fore,fore,Mat());
        dilate(fore,fore,Mat());
        dilate(fore,fore,Mat());
        dilate(fore,fore,Mat());
        dilate(fore,fore,Mat());
        dilate(fore,fore,Mat());
        dilate(fore,fore,Mat());
		dilate(fore,fore,Mat());
        dilate(fore,fore,Mat());
        dilate(fore,fore,Mat());
        dilate(fore,fore,Mat());
		dilate(fore,fore,Mat());///
		dilate(fore,fore,Mat());///



       cv::normalize(fore, fore, 0, 1., cv::NORM_MINMAX);
       cv::threshold(fore, fore, .5, 1., CV_THRESH_BINARY);

	    split(frame, channels);
      add(channels[0], channels[1], channels[1]);
      subtract(channels[2], channels[1], channels[2]);
      threshold(channels[2], thresh_frame, 50, 255, CV_THRESH_BINARY);
      medianBlur(thresh_frame, thresh_frame, 5);

	  
	  
      findContours(fore, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	 
      vector<vector<Point> > contours_poly( contours.size() );
      vector<Rect> boundRect( contours.size() );

      Mat drawing = Mat::zeros(thresh_frame.size(), CV_8UC1);

	 #pragma omp parallel section
      for(size_t i = 0; i < contours.size(); i++)
        {
//          cout << contourArea(contours[i]) << endl;
          if(contourArea(contours[i]) > 12000)                                                  //contour area
            drawContours(drawing, contours, i, Scalar::all(255), CV_FILLED, 8, vector<Vec4i>(), 0, Point());
        }
	
      //thresh_frame = drawing;
#pragma omp parallel section
       for( size_t i = 0; i < contours.size(); i++ )
       {
		  approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
         boundRect[i] = boundingRect( Mat(contours_poly[i]) );

     }
#pragma omp parallel section
      for( size_t i = 0; i < contours.size(); i++ )
       {
           if(contourArea(contours[i]) > 12000){                                                            //// area define
        // rectangle( frame, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 255, 0), 2, 8, 0 );
		 // Rect r = boundRect[i];
        //  frame(r).copyTo(images_array);  
		// img_array.push_back(images_array);
        Point center = Point(boundRect[i].x + (boundRect[i].width /2), boundRect[i].y + (boundRect[i].height/2));
        //cv::circle(frame,center, 8, Scalar(0, 0, 255), -1, 1,0);
		centers.push_back(center);

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//fprintf (pFile, "%d \n",center.x);   
		//fprintf (pFile2, "%d \n",center.y);  
		//detect++;
		//kal++;
		
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////
		   }
	  }


	/*if(centers.size()>0)
	{
		tracker.Update(centers);
		
		//cout << tracker.tracks.size()  << endl;

		for(int i=0;i<tracker.tracks.size();i++)
		{
			if(tracker.tracks[i]->trace.size()>1)
			{
				for(int j=0;j<tracker.tracks[i]->trace.size()-1;j++)
				{
					line(frame,tracker.tracks[i]->trace[j],tracker.tracks[i]->trace[j+1],Colors[tracker.tracks[i]->track_id%9],2,CV_AA);
					//drawCross(frame, tracker.tracks[i]->trace[j], Scalar(255, 255, 255), 5);
				}

				////////////////////////////////////////////////////////////////////////////////////////////

		for(int x=0;x<tracker.tracks.size();x++)
		{
			start_time = tracker.tracks[x]->begin_time;
			//std::cout << float(start_time ) ;////
			if((clock() - start_time) > 5000 )
			{
				if(tracker.tracks[x]->suspicious != 1)
				{
					printf("person %d is suspicious\n",tracker.tracks[x]->track_id+1);
					tracker.tracks[x]->suspicious = 1;
				}
			}

		}

			}
		}




	}*/
#pragma omp parallel section
	  if(centers.size()>0)
	{
		Update(centers);
		
		//cout << tracker.tracks.size()  << endl;

		for(int i=0;i<tracks.size();i++)
		{
			if(tracks[i]->trace.size()>1)
			{
				for(int j=0;j<tracks[i]->trace.size()-1;j++)
				{
					//line(frame,tracks[i]->trace[j],tracks[i]->trace[j+1],Colors[tracks[i]->track_id%9],2,CV_AA);
					//drawCross(frame, tracker.tracks[i]->trace[j], Scalar(255, 255, 255), 5);
					////////////////////////////////////////////////////////////////////////////
					stringstream ss;
					ss << tracks[i]->track_id+1;
					string label = ss.str();
					putText(frame, label.c_str(), cv::Point(tracks[i]->prediction.x,tracks[i]->prediction.y),FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,255),3);


					//////////////////////////////////////////////////////////////////////////////////////////////////////////
					

					//fprintf (pFile3, "%d \n",cv::Point(tracks[i]->prediction.x));   ////graph function
					//fprintf (pFile4, "%d \n",cv::Point(tracks[i]->prediction.y));



					///////////////////////////////////////////////////////////////////////////////////////////////////////////

					/////substitute for rec contours
					rectangle(frame, cv::Point((tracks[i]->prediction.x-50),(tracks[i]->prediction.y-150)), cv::Point((tracks[i]->prediction.x+50),(tracks[i]->prediction.y+150)),Colors[tracks[i]->track_id%9], 2, 8, 0 );

				}

				////////////////////////////////////////////////////////////////////////////////////////////
				#pragma omp parallel section
				for(int x=0;x<tracks.size();x++)
				{
					start_time = tracks[x]->begin_time;
					//std::cout << float(start_time ) ;////
					if((clock() - start_time) > 40000 )                              ////////////time define
					{
						if(tracks[x]->suspicious != 1)
						{
							printf("person %d is suspicious\n",tracks[x]->track_id+1);
							tracks[x]->suspicious = 1;
							// save cropped image                     /// suspicious crop image size
							/*cv::Rect person(cv::Point((tracks[x]->prediction.x-30),(tracks[x]->prediction.y-90)), cv::Point((tracks[x]->prediction.x+30),(tracks[x]->prediction.y+90)));
							cv::Mat personImage;
							personImage = frame(person).clone();
							imshow("Cropped Image",personImage);
							stringstream file;
							file << "image" << picno << ".jpg";
							picno++;
							imwrite(file.str(), personImage);*/


						}                                           ///////// suspicious highlighter size

						//rectangle(frame, cv::Point((tracks[x]->prediction.x-30),(tracks[x]->prediction.y-90)), cv::Point((tracks[x]->prediction.x+20),(tracks[x]->prediction.y+90)),cv::Scalar(0,0,255), 2, 8, 0 );
						
						putText(frame, "suspicious", cv::Point(tracks[i]->prediction.x-50,tracks[i]->prediction.y+50),FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255),2);

					}

				}

			}
		}
	  }
     
		

	
	imshow("OUTPUT Camera",frame);
	//imshow("OUTPUT Camera_B",drawing);
	//imshow("OUTPUT Camera_B",fore);

	waitKey(81);
	}

	destroyAllWindows();
	return 0;

}