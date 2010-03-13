//#ifdef _CH_
//#pragma package <opencv>
//#endif

//#define CV_NO_BACKWARD_COMPATIBILITY

//#ifndef _EiC
//#include "cv.h"
//#include "highgui.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <math.h>
#include <ctype.h>
//#endif

IplImage *image, *frame, *mask, *backproject, *sum, *r, *g, *b, *planes[3];
CvHistogram *bg_hist, *fg_hist, *lglikrat;

int select_object;
int track_object;
int show_hist;
CvPoint origin;
CvRect selection, box;
CvRect track_window;
CvBox2D track_box;
CvConnectedComp track_comp;
int dims[3];
float range_arr[2], *ranges[3], max_val, value, maxlik, epsilon;
int thepoint[2];
double *sumdata;
int c, h, w, step, boxhstep;

char* object;
FILE *histogram;

//Method for selecting the object to track
void on_mouse( int event, int x, int y, int flags, void* param )
{
    if( !image )
        return;

    if( image->origin )
        y = image->height - y;

    if( select_object )
    {
        selection.x = MIN(x,origin.x);
        selection.y = MIN(y,origin.y);
        selection.width = selection.x + CV_IABS(x - origin.x);
        selection.height = selection.y + CV_IABS(y - origin.y);

        selection.x = MAX( selection.x, 0 );
        selection.y = MAX( selection.y, 0 );
        selection.width = MIN( selection.width, image->width );
        selection.height = MIN( selection.height, image->height );
        selection.width -= selection.x;
        selection.height -= selection.y;
    }

    switch( event )
    {
    case CV_EVENT_LBUTTONDOWN:
        origin = cvPoint(x,y);
        selection = cvRect(x,y,0,0);
        select_object = 1;
        break;
    case CV_EVENT_LBUTTONUP:
        select_object = 0;
        if( selection.width > 0 && selection.height > 0 )
            track_object = -1;
        break;
    }
}

//Takes 8-bit image, 64-bit float image, and a histogram
void myBackProjectAndIntegrate( IplImage** img, IplImage* dst, const CvHistogram* hist )
{
	if( !CV_IS_HIST(hist))
		CV_Error( CV_StsBadArg, "Bad histogram pointer" );

	if( !img )
		CV_Error( CV_StsNullPtr, "Null double array pointer" );

	int size[CV_MAX_DIM];
	int i, dims = cvGetDims( hist->bins, size );
	
	bool uniform = CV_IS_UNIFORM_HIST(hist);
	const float* uranges[CV_MAX_DIM] = {0};
	const float** ranges = 0;
	unsigned char* imgdata[3];
	double* dstdata;
	double linetotal;
	
	if( hist->type & CV_HIST_RANGES_FLAG )
	{
		ranges = (const float**)hist->thresh2;
		if( uniform )
		{
			for( i = 0; i < dims; i++ )
				uranges[i] = &hist->thresh[i][0];
			ranges = uranges;
		}
	}

	//Get pointers to image data	
	for( int i=0; i<3; i++)
		imgdata[i]= (uchar *)(img[i]->imageData);
	dstdata= (double *)(dst->imageData);	

	//Iterate through image, backproject pixel then add value
	//   of all pixels above/to-left of it
	int w = img[0]->width;
	int h = img[0]->height;
	int step = img[0]->widthStep/sizeof(uchar); //for moving up a whole line
	for (int i=0; i<h; i++) {
		linetotal = 0.0; //running total of current line
		for (int j=0; j<w; j++) {
			linetotal+=(double)cvQueryHistValue_3D(hist,int( (imgdata[0][j])/32),int( (imgdata[1][j])/32),int( (imgdata[2][j])/32));
			if( i<1 ) // first line
				(dstdata)[j]=linetotal;
			else
				//value of integrated pixel above you plus linetotal
				(dstdata)[j]=(double)(dstdata)[j-step]+linetotal;
		}
		imgdata[0]+=step;
		imgdata[1]+=step;
		imgdata[2]+=step;
		dstdata+=step;
	}
}

int main( int argc, char** argv )
{
	select_object = 0; //0 until you draw select box
	track_object = 0; //same
	dims = {8,8,8}; //dimensions of histogram 8x8x8
	range_arr = {0,255}; //RGB values can range over this
	ranges = {range_arr,range_arr,range_arr};
	epsilon = 0.1; //avoid divide by zero in calculating lglikrat values
	fg_hist = cvCreateHist( 3, dims, CV_HIST_ARRAY, ranges, 1 );

/**/	cvNamedWindow( "Extra", 1 );
	cvNamedWindow( "Demo", 1 );
	cvSetMouseCallback( "Demo", on_mouse, 0 );

	object = 0;
	if( argc > 2 ){ //if object histogram passed in on command line
		object = argv[2]; //string of histogram file location
		selection = cvRect(0,0,1023,1023); //since we don't know where to start, select whole image to search
		track_object = -1; //flag to initialize tracking

		histogram = fopen(object,"rb");
		fread(cvGetHistValue_3D(fg_hist,0,0,0),sizeof(float),dims[0]*dims[1]*dims[2],histogram);
		fclose(histogram);
	}
	
    CvCapture* capture = 0;

    //capture from cam or clip
    if( argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
        capture = cvCaptureFromCAM( argc == 2 ? argv[1][0] - '0' : 0 );
    else if( argc == 2 )
        capture = cvCaptureFromAVI( argv[1] );

    if( !capture )
    {
        fprintf(stderr,"Could not initialize capturing...\n");
        return -1;
    }

    printf( "Hot keys: \n"
        "\tESC - quit the program\n"
        "To initialize tracking, select the object with mouse\n" );

    for(;;)
    {
	image = cvQueryFrame( capture );
        if( !image ){
            printf("Shit!\n");
            break;
        }

	if( !mask )
	{
		mask = cvCreateImage( cvGetSize(image), IPL_DEPTH_8U, 1 );
		backproject = cvCreateImage( cvGetSize(image), IPL_DEPTH_8U, 1 );
		sum = cvCreateImage( cvGetSize(image), IPL_DEPTH_64F, 1 );
		bg_hist = cvCreateHist( 3, dims, CV_HIST_ARRAY, ranges, 1 );
		lglikrat = cvCreateHist( 3, dims, CV_HIST_ARRAY, ranges, 1 );
		r = cvCreateImage( cvGetSize(image), 8, 1 );
		g = cvCreateImage( cvGetSize(image), 8, 1 );
		b = cvCreateImage( cvGetSize(image), 8, 1 );
	}

	if( track_object )
	{
		cvSplit( image, r, g, b, 0 ); //have to pass color planes separately
		planes = {r,g,b};

		if( track_object < 0 )
		{
			cvZero(mask);
			max_val = 0.f;
			if( !object ){
				cvRectangle(mask,cvPoint(selection.x,selection.y),cvPoint(selection.x+selection.width,selection.y+selection.height),cvScalarAll(255),CV_FILLED,8,0);
				cvCalcHist( planes, fg_hist, 0, mask);
			}
			cvNot(mask,mask);
			cvCalcHist( planes, bg_hist, 0, mask );
			cvNot(mask,mask);
			//make fg_hist and bg_hist have same max value
			cvGetMinMaxHistValue( fg_hist, 0, &max_val, 0, 0 );
			cvNormalizeHist(bg_hist, max_val);
			//Make new Hist basically log(fg_hist/bg_hist)
			for( int m=0; m<8; m++){
			for( int n=0; n<8; n++){
			for( int o=0; o<8; o++){
				*(cvGetHistValue_3D(lglikrat,m,n,o)) = log((cvQueryHistValue_3D(fg_hist,m,n,o)+epsilon)/(cvQueryHistValue_3D(bg_hist,m,n,o)+epsilon));
			}}}
/**/			track_window = selection;
			track_object = 1;
		}

/*		cvCalcBackProject( planes, backproject, lglikrat );
		
//		cvAnd( backproject, mask, backproject, 0 );
		cvCamShift( backproject, track_window,
			cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),
			&track_comp, &track_box );
		track_window = track_comp.rect;

		if( backproject_mode )
			cvCvtColor( backproject, image, CV_GRAY2BGR );
		if( !image->origin )
			track_box.angle = -track_box.angle;
		cvEllipseBox( image, track_box, CV_RGB(255,0,0), 3, CV_AA, 0 );
/**/

		myBackProjectAndIntegrate( planes, sum, lglikrat );
		
		box = selection;
		maxlik = -INFINITY; //max probability of any box so far
		thepoint = {0,0}; //top left corner of max box
		h = sum->height;
		w = sum->width;
		step = sum->widthStep/sizeof(double);
		boxhstep = box.height*step;
		sumdata= (double *)(sum->imageData);
		// Run box over whole image, lookup corner values, keep max box
		for (int i=0; i<(h-box.height); i++) {
			for (int j=0; j<(w-box.width); j++) {
				value =  sumdata[j]+sumdata[j+box.width+boxhstep]-sumdata[j+boxhstep]-sumdata[j+box.width];
				if (value > maxlik){
					maxlik = value;
					thepoint = {j,i};
				}
			}
			sumdata+=step;
		}

/**/
		cvRectangle(image,cvPoint(box.x,box.y),cvPoint(box.x+box.width,box.y+box.height),cvScalar(0,0,255),3,8,0);
		cvRectangle(image,cvPoint(thepoint[0],thepoint[1]),cvPoint(thepoint[0]+box.width,thepoint[1]+box.height),cvScalarAll(255),1,8,0);	
}

	//Highlight during selection
	if( select_object && selection.width > 0 && selection.height > 0 )
	{
		cvSetImageROI( image, selection );
		cvXorS( image, cvScalarAll(255), image, 0 );
		cvResetImageROI( image );
	}
/**/

	cvShowImage( "Demo", image );
	cvShowImage( "Extra", backproject);

        c = cvWaitKey(10);
        if( (char) c == 27 )
            break;
    }

    cvReleaseCapture( &capture );
    cvDestroyWindow("Extra");
    cvDestroyWindow("Demo");

    return 0;
}

//#ifdef _EiC
//main(1,"camshiftdemo.c");
//#endif
