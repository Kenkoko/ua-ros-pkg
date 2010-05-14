#include <iostream>
#include <fstream>
#include <map>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/RegionOfInterest.h"
#include "cv_bridge/CvBridge.h"
#include "color_based_tracking/FindByColor.h"
#include "geometry_msgs/PolygonStamped.h"

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

using namespace std;

ros::CallbackQueue callback_queue_;

IplImage* bg = NULL, *planes[3]; 
IplImage *frame, *original, *curr_image, *mono_image, *roi_color_image, *roi_bw_image, *contour_image;
CvMemStorage *storage;
CvSeq *contours, *contour, *result_contour = NULL;
bool isFirst = true;
sensor_msgs::CvBridge bridge;
ros::Publisher sub_pub, mask_pub, object_pub;
CvHistogram *result_hist, *combined_hists;
geometry_msgs::PolygonStamped foundpoints;
bool all_words_known;
std::vector<string> known;
std::vector<string> unknown;
char* word;
CvBox2D minBox;
CvPoint2D32f some_stupid_points[4];
double distance_score = INFINITY, lowest_score = INFINITY;
string result_shape;
bool undecidable = true, clicked = false;

void add_hist_to_hist(CvHistogram* original, CvHistogram* addee)
{
    for( int m=0; m<8; m++){
    for( int n=0; n<8; n++){
    for( int o=0; o<8; o++){
        *(cvGetHistValue_3D(original,m,n,o)) = cvQueryHistValue_3D(original,m,n,o) + cvQueryHistValue_3D(addee,m,n,o);
    }}}
}

void get_color_hist(IplImage* planes[3], IplImage* mono_image, CvSeq* contour, CvHistogram* result )
{
    printf("Getting color hist\n");

    CvRect boundingBox = cvBoundingRect(contour);

    for( int i=0; i<3; i++ )
        cvSetImageROI(planes[i], boundingBox);
    cvSetImageROI(mono_image, boundingBox);
            
    cvCalcHist( planes, result, 0, mono_image );
    
    for( int i=0; i<3; i++ )
        cvResetImageROI(planes[i]);
    cvResetImageROI(mono_image);
    
    printf("Finished getting color hist\n");
    
    return;
}

class Word2 {
    CvHistogram* color;
    static const int color_num_bins = 8; // bins per dimension
    
    CvHistogram* areaRatio;
    static const int aR_num_bins = 10;
    
    CvHistogram* dimRatio;
    static const int dR_num_bins = 10;

public:

    Word2()
    {
        color = cvCreateHist( 3, // Number of dimensions
                             (int[]){color_num_bins,color_num_bins,color_num_bins}, // For each dim, num of bins
                             CV_HIST_ARRAY, // Dense array
                             (float*[]) { (float[]){0,255}, (float[]){0,255}, (float[]){0,255} }, // For each dim, range
                             1 );
                             
        areaRatio = cvCreateHist( 1, 
                                  (int[]){aR_num_bins}, 
                                  CV_HIST_ARRAY, 
                                  (float*[]) {(float[]){0,1}},
                                  1 );
                                  
        dimRatio = cvCreateHist( 1,
                                 (int[]){dR_num_bins}, 
                                 CV_HIST_ARRAY, 
                                 (float*[]) {(float[]){0,1}},
                                 1 );
    };
    
    Word2( IplImage* planes[3], IplImage* mono_image, CvSeq* contour, CvHistogram* result )
    {
        Word2();

        get_color_hist( planes, mono_image, contour, color );
        
    };
    
    void update_1D_hist(CvHistogram* updatee, float value)
    {
        int size = updatee->mat.dim->size;
        float lower_limit = updatee->thresh[0][0];
        float upper_limit = updatee->thresh[0][1];
        float range = upper_limit - lower_limit;
        int bin_to_go_in = int( (value - lower_limit)*size/range );
        *(cvGetHistValue_1D(updatee, bin_to_go_in)) += 1;
        return;
    }
};

class Word {
    CvHistogram* color;
    int square;
    int circle;
    
public:
    Word( CvHistogram* color, string shape )
    {
        this->color = color;
        update_shape(shape);
    }
    
    ~Word()
    {
        std::cout << "DESTRUCTING" << std::endl;
    }
    
    void update_shape( string shape )
    {
        if (shape == "square")
            this->square++;
        else
            this->circle++;
        return;    
    }
    
    void update_color( CvHistogram* color_to_add )
    {
        add_hist_to_hist( this->color, color_to_add );
        return;
    }
    
    CvHistogram* get_color()
    {
        return color;
    }
};

map<string, Word*> known_words;

int dims[] = {8,8,8};
float range[] = {0, 255};
float* ranges_arr[] = {range, range, range};

string phrase = "";

// This function taken from http://jmpelletier.com/data/OpenCVcontours.c
void onMouse(int event, int x, int y, int flags, void *param){
    //params points to the contour sequence. Here we cast the pointer to CvContour instead of CvSeq.
    //CvContour is in fact an extension of CvSeq and is the structure used by cvFindContours, if we cast 
    //to CvSeq, we won't be able to access the fields specific to CvContour.
    CvSeq *contours = *(CvSeq **)param;
    
//    printf("In onMouse\n");
    
    if(!contours){
        printf("You didn't send me any contours\n");
        return; //If there are no contours, we don't bother doing anything.
    }
    
    //"event" tells us what occured
    switch(event){
        case CV_EVENT_LBUTTONDOWN: //single click
        case CV_EVENT_LBUTTONDBLCLK: //double click

            printf("Click: %d %d\n",x,y);  //Write out where the user clicked.
            
            contour = contours;
            for( ; contour != 0; contour = contour->h_next )
            {
                bool isInside = cvPointPolygonTest(contour, cvPoint2D32f( (double) x, (double) y ), false) > 0;
                printf("isInside? %i\n", isInside);
                if( isInside ){
                    result_contour = contour;
                    break;
                }
            }

            break;
        case CV_EVENT_LBUTTONUP: //single click up
        case 10: //double-click up? (not documented)
            break;
    }
}

void onMouse2(int event, int x, int y, int flags, void *param){
    switch(event){
        case CV_EVENT_LBUTTONDOWN: //single click
        case CV_EVENT_LBUTTONDBLCLK: //double click
        case CV_EVENT_LBUTTONUP: //single click up
        case 10: //double-click up? (not documented)
            clicked = true;
            break;
    }
}

string get_shape(CvSeq* contour)
{
//    printf("Getting shape\n");

    double area = fabs(cvContourArea(contour));
    CvBox2D minBox = cvMinAreaRect2(contour, storage);
    double minBoxArea = minBox.size.height * minBox.size.width;
    double areaRatio = area / minBoxArea;

//    printf("Finished getting shape\n");

    if (area < 250 || areaRatio < 0.6) 
        return "";
    else if (areaRatio > 0.78)
        return "square";
    else
        return "circle";
}

void update_words(CvHistogram* given_hist, string given_shape, std::vector<string> known, std::vector<string> unknown)
{

    std::cout << "=====================" << unknown.size() << "=====================" << std::endl;
    for( uint i=0; i<unknown.size(); i++)
    {                    
        known_words[unknown[i]] = new Word( given_hist, given_shape );
    }
                
    for( uint i=0; i<known.size(); i++)
    {
        known_words[known[i]]->update_color( given_hist );
        known_words[known[i]]->update_shape( given_shape );
    }
    
    return;
}

void handleImage(const sensor_msgs::ImageConstPtr& msg_ptr) 
{

    printf("Test 2\n");

    if( phrase == "" )
    {
        return;
    }
    else
    {
        // We now know that the last iteration provided correct results
        //   so update the words (the values should still be in these globals)
        if ( phrase != "no" and not isFirst)
            update_words(result_hist, result_shape, known, unknown);

        printf("Test 3\n");
        
        try
        {
            original = bridge.imgMsgToCv(msg_ptr, "bgr8");
            foundpoints.header.stamp = ros::Time();
            foundpoints.header.frame_id = "something";
        }
        catch (sensor_msgs::CvBridgeException error)
        {
            ROS_ERROR("error");
        }



            
        if (isFirst) {
            curr_image = cvCreateImage(cvGetSize(original), 8, 3);
            mono_image = cvCreateImage(cvGetSize(original), 8, 1);
            contour_image = cvCreateImage(cvGetSize(original), 8, 1);
            storage  = cvCreateMemStorage(0);
            contours = NULL;
                
            result_hist = cvCreateHist( 3, dims, CV_HIST_ARRAY, ranges_arr, 1 );
            combined_hists = cvCreateHist( 3, dims, CV_HIST_ARRAY, ranges_arr, 1 );
            planes[0] = cvCreateImage(cvGetSize(original), 8, 1);
            planes[1] = cvCreateImage(cvGetSize(original), 8, 1);
            planes[2] = cvCreateImage(cvGetSize(original), 8, 1);
                
            isFirst = false;
        }



        
        cvSub(bg, original, curr_image);
        // TODO: See if diff will work better than sub with some tweaks
        //cvAbsDiff(bg, original, curr_image);
        
        cvCvtColor(curr_image, mono_image, CV_RGB2GRAY);
        
        cvThreshold(mono_image, mono_image, 50, 255, CV_THRESH_BINARY);
        
        //cvCopy(original, curr_image, mono_image);
        
        cvCopy(mono_image, contour_image);
        
        cvFindContours(contour_image, storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        
        printf("Test 3.1\n");



        
        // Sort out bad contours
        while ( contours != 0 and get_shape(contours) == "" )
        {
            contours = contours->h_next;
        }
        contour = contours->h_next;
        for( ; contour != 0; contour = contour->h_next )
        {
            if ( get_shape(contour) == "" )
            {
                contour->h_prev->h_next = contour->h_next;
            }
        }



        cvSplit( original, planes[0], planes[1], planes[2], 0 );
        
        printf("Test 3.2\n");
        
        known.clear();
        unknown.clear();
        all_words_known = true;
        word = strtok ( strdup(phrase.c_str()), " ");
        for( ; word != 0; word = strtok (NULL, " ") )
        {
            if (known_words.count(word) > 0)
                known.push_back(string(word));
            else {
                unknown.push_back(string(word));
                all_words_known = false;
            }
        }




        printf("Test 3.3\n");

        if (all_words_known and phrase != "no")
        {
            undecidable = false;
            // Still need to account for shape in comparison here
        
            cvClearHist(combined_hists);
            for( uint i=1; i<known.size(); i++)
                add_hist_to_hist(combined_hists, known_words[known[i]]->get_color());
                
            distance_score = INFINITY;
            lowest_score = INFINITY;
            
            for( contour = contours, result_contour = NULL; contour != 0; contour = contour->h_next )
            {
                printf("What the what\n");
                get_color_hist( planes, mono_image, contour, result_hist );
                distance_score = cvCompareHist( combined_hists, result_hist, CV_COMP_CORREL );
                if (distance_score < lowest_score)
                {
                    result_contour = contour;
                    lowest_score = distance_score;
                }
            }
//            if something undecidable = true;
        }
        
        
        if (!all_words_known or phrase == "no" or undecidable)
        {
            CvScalar color = cvScalarAll( 0 );
            for( contour = contours; contour != 0; contour = contour->h_next )
            {
                cvDrawContours( original, contour, color, color, -1, 2, 8 );
            }
        
            cvNamedWindow( "Window", 1 );
            cvSetMouseCallback("Window", onMouse, &contours);
            for( result_contour = NULL; result_contour == NULL;    )
            {            
                cvShowImage( "Window", original);
                cvWaitKey(3);
            }
            cvSetMouseCallback("Window", 0);
            cvDestroyWindow( "Window" );
            cvDestroyWindow( "Window" );
            
        }

            CvScalar color = cvScalarAll( 255 );
            cvDrawContours( original, result_contour, color, color, -1, 2, 8 );
 
            cvNamedWindow( "Window", 1 );
            cvSetMouseCallback("Window", onMouse2, 0);
            for( clicked = false; clicked == false;    )
            {            
                cvShowImage( "Window", original);
                cvWaitKey(3);
            }
            cvSetMouseCallback("Window", 0);
            cvDestroyWindow( "Window" );
            cvDestroyWindow( "Window" );

        
        printf("Test 3.5\n");
        
        // Store results for update later
        get_color_hist( planes, mono_image, result_contour, result_hist );
        result_shape = get_shape( result_contour );
        
        
        
        printf("Test 3.5\n");
        
        minBox = cvMinAreaRect2(result_contour, storage);
        cvBoxPoints( minBox, some_stupid_points );
        
        printf("Test 3.6\n");

        foundpoints.header = msg_ptr->header;

        printf("Test 3.7\n");

        foundpoints.polygon.set_points_size(4);
            
        for( int i=0; i<4; i++ ) {
            printf("Test 3.7%i\n", i);
            foundpoints.polygon.points[i].x = some_stupid_points[i].x;
            foundpoints.polygon.points[i].y = some_stupid_points[i].y;
            printf("%g %g\n",foundpoints.polygon.points[i].x,foundpoints.polygon.points[i].y);
        }

        printf("Test 4\n");
            
        phrase = "";
        
        printf("Test 4.1\n");


    }//end if (phrase != "") else

    return;
    
}

bool serviceCallback(color_based_tracking::FindByColor::Request &req, color_based_tracking::FindByColor::Response &res)
{
    if (phrase == "yes")
    {
            update_words(result_hist, result_shape, known, unknown);
    }
    else
    {
        printf("Test 1\n");
        phrase = req.color;
    
        ros::Rate rate(10);
        while( phrase != "" )
        {
            printf("Testing\n");
            callback_queue_.callOne(ros::WallDuration());
            rate.sleep();
        }
    
        printf("Test 5\n");

        for( int i=0; i<4; i++ ) {
            printf("%g %g\n",foundpoints.polygon.points[i].x,foundpoints.polygon.points[i].y);
        }
        
        printf("Test 6\n");

        res.bounding_box = foundpoints;
        foundpoints.header.frame_id = "";
    
        for( int i=0; i<4; i++ ) {
            printf("%g %g\n",res.bounding_box.polygon.points[i].x,res.bounding_box.polygon.points[i].y);
        }
    }
        
    return true;
}

int main (int argc, char **argv) 
{
    bg = cvLoadImage(argv[1]);
    
    ros::init(argc, argv, "wubble_world");
    ros::NodeHandle n;
    n.setCallbackQueue(&callback_queue_);
    
    foundpoints.header.frame_id = "";
                cvNamedWindow( "Window", 1 );
    ros::ServiceServer service = n.advertiseService("find_by_color", serviceCallback);

    ros::Subscriber image_sub = n.subscribe(argv[2], 20, handleImage);
    
    ros::MultiThreadedSpinner spinner(0);
    spinner.spin(&callback_queue_);
    
    return 0;
}
