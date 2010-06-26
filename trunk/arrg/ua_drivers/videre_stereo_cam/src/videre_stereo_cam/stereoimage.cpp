/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <sstream>
#include <iostream>

#include <videre_stereo_cam/stereolib.h>
#include <videre_stereo_cam/stereoimage.h>

#define PRINTF(a...) printf(a)

using namespace cam;

// stereo class fns
StereoData::StereoData()
{
    imLeft = new ImageData();
    imRight = new ImageData();

    // disparity buffer
    imDisp = NULL;
    imDispSize = 0;
    buf = NULL;
    flim = NULL;
    frim = NULL;
    wbuf = NULL;
    rbuf = NULL;
    lbuf = NULL;
    maxyim = maxxim = maxdlen = maxcorr = 0;

    // nominal values
    imWidth = 640;
    imHeight = 480;
    corrSize = 15;
    filterSize = 11;
    horOffset = 0;
    setDispOffsets();
    dpp = 16;
    numDisp = 64;
    offx = 0;

    textureThresh = 10;
    uniqueThresh = 12;
    speckleDiff = 8;
    speckleRegionSize = 100;

    hasRectification = false;

    // point array/vector
    numPts = 0;
    imPts = NULL;
    imCoords = NULL;
    imPtsColor = NULL;
    isPtArray = false;
    imPtsSize = 0;
}

StereoData::~StereoData()
{
    releaseBuffers();
    free(imLeft);
    free(imRight);

    // should free all buffers
    MEMFREE(buf);
    MEMFREE(flim);
    MEMFREE(frim);
}

bool StereoData::setHoropter(int val)
{
    if (val < 0) { val = 0; }
    if (val > 63) { val = 63; }
    offx = val;

    return true;
}

bool StereoData::setTextureThresh(int val)
{
    if (val < 0) { val = 0; }
    if (val > 100) { val = 10; }
    textureThresh = val;

    return true;
}

bool StereoData::setUniqueThresh(int val)
{
    if (val < 0) { val = 0; }
    if (val > 100) { val = 10; }
    uniqueThresh = val;

    return true;
}

void StereoData::setDispOffsets()
{
    /*
    * disparity image size
    * ====================
    * dleft  : (logs + corrs - 2)/2 - 1 + offx
    * dwidth : w - (logs + corrs + offx - 2)
    * dtop   : (logs + corrs - 2)/2
    * dheight: h - (logs + corrs)
    *
    */

    imDtop = (filterSize + corrSize - 2) / 2;
    imDleft = (filterSize + corrSize - 2) / 2 - 1 + offx;
    imDwidth = imWidth - (filterSize + corrSize + offx - 2);
    imDheight = imHeight - (filterSize + corrSize);
}

void StereoData::releaseBuffers()
{
    MEMFREE(imDisp);
    imDisp = NULL;
    imDispSize = 0;
    hasDisparity = false;
    MEMFREE(imPts);
    MEMFREE(imCoords);
    imPtsSize = 0;
    numPts = 0;
    imLeft->releaseBuffers();
    imRight->releaseBuffers();
}

// image size needs to deal with buffers
void StereoData::setSize(int width, int height)
{
    imWidth = width;
    imHeight = height;

    imLeft->imWidth = width;
    imLeft->imHeight = height;
    imLeft->cam_info.width = width;
    imLeft->cam_info.height = height;

    imRight->imWidth = width;
    imRight->imHeight = height;
    imRight->cam_info.width = width;
    imRight->cam_info.height = height;
}

bool StereoData::doRectify()
{
    bool res = imLeft->doRectify();
    res = imRight->doRectify() && res;

    return res;
}

void StereoData::doBayerColorRGB()
{
    imLeft->doBayerColorRGB();
    imRight->doBayerColorRGB();
}

void StereoData::doBayerMono()
{
    imLeft->doBayerMono();
    imRight->doBayerMono();
}

bool StereoData::doDisparity()
{
    // first do any rectification necessary
    doRectify();

    // check if disparity is already present
    if (hasDisparity) { return true; }

    return false;
}

//
// apply speckle filter
// useful for STOC processing, where it's not done on-camera
//
bool StereoData::doSpeckle()
{
    if (!hasDisparity) { return false; }

    // speckle filter
    if (speckleRegionSize > 0)
    {
        int xim = imWidth;
        int yim = imHeight;

        // local storage for the algorithm
        if (!rbuf) { rbuf = (uint8_t*) malloc(xim * yim); }

        // local storage for the algorithm
        if (!lbuf) { lbuf = (uint32_t*) malloc(xim * yim * sizeof(uint32_t)); }

        // local storage for the algorithm
        if (!wbuf) { wbuf = (uint32_t*) malloc(xim * yim * sizeof(uint32_t)); }

        do_speckle(imDisp, 0, xim, yim, speckleDiff, speckleRegionSize, lbuf, wbuf, rbuf);
    }

    return true;
}

bool StereoData::setSpeckleRegionSize(int val)
{
    speckleRegionSize = val;
    return true;
}

bool StereoData::setSpeckleDiff(int val)
{
    speckleDiff = val;
    return true;
}

//
// param sting parsing routines
//

#include <iostream>
using namespace std;

template <class T>
void extract(std::string& data, std::string section, std::string param, T& t)
{
  size_t found = data.find(section);
  if (found != string::npos)
    {
      found = data.find(param,found);
      if (found != string::npos)
	{
	  std::istringstream iss(data.substr(found+param.length()));
	  iss >> t;
	}
    }
}

void extract(std::string& data, std::string section,
		  std::string param, double *m, int n)
{
  size_t found = data.find(section);
  if (found != string::npos)
    {
      found = data.find(param,found);
      if (found != string::npos)
	{
	  std::istringstream iss(data.substr(found+param.length()));
	  double v;
	  for (int i=0; i<n; i++)
	    {
	      iss >> v;
	      m[i] = v;
	    }
	}
    }
}


//
// Conversion to 3D points
// Convert to vector or image array of pts, depending on isArray arg
// Should we do disparity automatically here?
//
bool StereoData::doCalcPts(bool isArray)
{
  numPts = 0;
  doDisparity();
  if (!hasDisparity)
    return false;

  int ix = imDleft;
  int iy = imDtop;
  int ih = imDheight;
  int iw = imDwidth;
  int w = imWidth;
  int h = imHeight;
  int dmax = 0;
  int dmin = 0x7fff;

  if (isArray)			// don't need these for arrays
    {
      isPtArray = true;
      MEMFREE(imCoords);
      MEMFREE(imPtsColor);
    }
  else
    isPtArray = false;

  if (imPtsSize < 4*w*h*sizeof(float))
    {
      MEMFREE(imPts);
      imPtsSize = 4*w*h*sizeof(float);
      imPts = (float *)MEMALIGN(imPtsSize);
      MEMFREE(imCoords);
      MEMFREE(imPtsColor);
      if (!isArray)
	{
	  imPtsColor = (uint8_t *)MEMALIGN(3*w*h);
	  imCoords = (int *)MEMALIGN(2*w*h*sizeof(int));
	}
    }

  float *pt = imPts;
  pt_xyza_t *ppt = (pt_xyza_t *)imPts;
  int *pcoord;
  int y = iy;
  float cx = (float)RP[3];
  float cy = (float)RP[7];
  float f  = (float)RP[11];
  float itx = (float)RP[14];
  itx *= 1.0 / (float)dpp; // adjust for subpixel interpolation
  pcoord = imCoords;

  if (isArray)			// make an array of pts
    {
      numPts = w*h;
      for (int j=0; j<h; j++)
	{
	  int16_t *p = imDisp + j*w;

	  for (int i=0; i<w; i++, p++, ppt++)
	    {
	      if (*p > dmax && *p < dmin)
		{
		  float ax = (float)i + cx;
		  float ay = (float)j + cy;
		  float aw = 1.0 / (itx * (float)*p);
		  ppt->X = ax*aw; // X
		  ppt->Y = ay*aw; // Y
		  ppt->Z = f*aw; // Z
		  ppt->A = 0;
		}
	      else
		{
		  ppt->X = 0.0; // X
		  ppt->Y = 0.0; // Y
		  ppt->Z = 0.0; // Z
		  ppt->A = -1;	// invalid point
		}
	    }
	}
    }
  else				// make a vector of pts
    {
      for (int j=0; j<ih; j++, y++)
	{
	  int x = ix;
	  int16_t *p = imDisp + x + y*w;

	  for (int i=0; i<iw; i++, x++, p++)
	    {
	      if (*p > dmax && *p < dmin)
		{
		  float ax = (float)x + cx;
		  float ay = (float)y + cy;
		  float aw = 1.0 / (itx * (float)*p);
		  *pt++ = ax*aw;	// X
		  *pt++ = ay*aw;	// Y
		  *pt++ = f*aw;	// Z
		  // store point image coordinates
		  *pcoord++ = x;
		  *pcoord++ = y;
		  numPts++;
		}
	    }
	}
    }


  if (isArray) return true;

  if (imLeft->imRectColorType != COLOR_CODING_NONE) // ok, have color
    {
      y = iy;
      uint8_t *pcout = imPtsColor;
      for (int j=0; j<ih; j++, y++)
	{
	  int x = ix;
	  int16_t *p = imDisp + x + y*w;
	  uint8_t *pc = imLeft->imRectColor + (x + y*w)*3;

	  for (int i=0; i<iw; i++, x++, p++, pc+=3)
	    {
	      if (*p > dmax && *p < dmin)
		{
		  *pcout++ = *pc;
		  *pcout++ = *(pc+1);
		  *pcout++ = *(pc+2);
		}
	    }
	}
    }
  else if (imLeft->imRectType != COLOR_CODING_NONE) // ok, have mono
    {
      y = iy;
      uint8_t *pcout = imPtsColor;
      for (int j=0; j<ih; j++, y++)
	{
	  int x = ix;
	  int16_t *p = imDisp + x + y*w;
	  uint8_t *pc = imLeft->imRect + (x + y*w);

	  for (int i=0; i<iw; i++, p++, pc++)
	    {
	      if (*p > dmax && *p < dmin)
		{
		  *pcout++ = *pc;
		  *pcout++ = *pc;
		  *pcout++ = *pc;
		}
	    }
	}
    }


  //  printf("[Calc Pts] Number of pts: %d\n", numPts);
  return true;
}

bool StereoData::parseCalibrationSVS(string params, stereo_side_t stereo_side)
{
    string side;
    ImageData* im_data;

    switch (stereo_side)
    {
        case SIDE_LEFT:
            side = "left";
            im_data = imLeft;
            break;
        case SIDE_RIGHT:
            side = "right";
            im_data = imRight;
            break;
        default:
            ROS_ERROR("Unknown stereo side specified");
            return false;
    }

    // K - original camera matrix
    extract(params, "[" + side + " camera]", "f ", im_data->cam_info.K[0]);
    extract(params, "[" + side + " camera]", "fy", im_data->cam_info.K[4]);
    extract(params, "[" + side + " camera]", "Cx", im_data->cam_info.K[2]);
    extract(params, "[" + side + " camera]", "Cy", im_data->cam_info.K[5]);

    im_data->cam_info.K[8] = 1.0;

    // D - distortion params
    extract(params, "[" + side + " camera]", "kappa1", im_data->cam_info.D[0]);
    extract(params, "[" + side + " camera]", "kappa2", im_data->cam_info.D[1]);
    extract(params, "[" + side + " camera]", "tau1", im_data->cam_info.D[2]);
    extract(params, "[" + side + " camera]", "tau2", im_data->cam_info.D[3]);
    extract(params, "[" + side + " camera]", "kappa3", im_data->cam_info.D[4]);

    // R - rectification matrix
    extract(params, "[" + side + " camera]", "rect",  im_data->cam_info.R.c_array(), 9);

    // P - projection matrix
    extract(params, "[" + side + " camera]", "proj",  im_data->cam_info.P.c_array(), 12);
    im_data->cam_info.P[3] *= .001;  // convert from mm to m

    return true;
}

bool StereoData::parseCalibrationOST(string params, stereo_side_t stereo_side)
{
    string side;
    ImageData* im_data;

    switch (stereo_side)
    {
        case SIDE_LEFT:
            side = "left";
            im_data = imLeft;
            break;
        case SIDE_RIGHT:
            side = "right";
            im_data = imRight;
            break;
        default:
            ROS_ERROR("Unknown stereo side specified");
            return false;
    }

    // K - original camera matrix
    extract(params, "[" + side + " camera]", "camera matrix", im_data->cam_info.K.c_array(), 9);

    // D - distortion params
    extract(params, "[" + side + " camera]", "distortion", im_data->cam_info.D.c_array(), 5);

    // R - rectification matrix
    extract(params, "[" + side + " camera]", "rectification",  im_data->cam_info.R.c_array(), 9);

    // P - projection matrix
    extract(params, "[" + side + " camera]", "projection",  im_data->cam_info.P.c_array(), 12);

    return true;
}

void StereoData::printCameraInfo(stereo_side_t stereo_side)
{
    string side;
    ImageData* im_data;

    switch (stereo_side)
    {
        case SIDE_LEFT:
            side = "left";
            im_data = imLeft;
            break;
        case SIDE_RIGHT:
            side = "right";
            im_data = imRight;
            break;
        default:
            ROS_ERROR("Unknown stereo side specified");
            return;
    }

    PRINTF("[dcam] %s camera matrix (K)\n", side.c_str());

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            PRINTF(" %.4f", im_data->cam_info.K[i * 3 + j]);
        }

        PRINTF("\n");
    }

    PRINTF("\n[dcam] %s distortion vector (D)\n", side.c_str());

    for (int i = 0; i < 5; ++i)
    {
        PRINTF(" %.4f", im_data->cam_info.D[i]);
    }

    PRINTF("\n\n[dcam] %s rectification matrix (R)\n", side.c_str());

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            PRINTF(" %.4f", im_data->cam_info.R[i * 3 + j]);
        }

        PRINTF("\n");
    }

    PRINTF("\n[dcam] %s projection matrix (P)\n", side.c_str());

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            PRINTF(" %.4f", im_data->cam_info.P[i * 4 + j]);
        }

        PRINTF("\n");
    }

    PRINTF("\n");
}

void StereoData::printCalibration()
{
    PRINTF("[dcam] Disparity resolution: 1/%d pixel\n", dpp);
    PRINTF("[dcam] Correlation window: %d\n", corrSize);
    PRINTF("[dcam] Prefilter window: %d\n", filterSize);
    PRINTF("[dcam] Number of disparities: %d\n", numDisp);

    printCameraInfo(SIDE_LEFT);
    printCameraInfo(SIDE_RIGHT);

    if (hasRectification) { PRINTF("[dcam] Has rectification\n\n"); }
    else { PRINTF("[dcam] No rectification\n\n"); }

    PRINTF("[dcam] Reprojection matrix\n");

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            PRINTF(" %.4f", RP[i * 4 + j]);
        }

        PRINTF("\n");
    }

    PRINTF("\n[dcam] External translation vector\n");

    for (int i = 0; i < 3; ++i)
    {
        PRINTF(" %.4f", T[i]);
    }

    PRINTF("\n\n[dcam] External rotation vector\n");

    for (int i = 0; i < 3; ++i)
    {
        PRINTF(" %.4f", Om[i]);
    }

    PRINTF("\n\n");
}

//
// gets params from a string
// "SVS"-type parameter strings use mm for the projection matrices, convert to m
// "OST"-type parameter strings use m for projection matrices
//
void StereoData::extractParams(char *ps, bool store)
{
    std::string params;
    params = ps;

    if (store && ps != NULL)
    {
        if (imLeft->params) { delete [] imLeft->params; }
        char *bb = new char[strlen(ps)];
        strcpy(bb, ps);
        imLeft->params = bb;
    }

    PRINTF("\n\n[extractParams] Parameters:\n\n");

    // Initialize Translation parameters
    for (int i = 0; i < 3; ++i)
    {
        T[i] = 0.0;
    }

    // Initialize Rotation parameters
    for (int i = 0; i < 3; ++i)
    {
        Om[i] = 0.0;
    }

    if (strncmp(ps,"# SVS",5)==0) // SVS-type parameters
    {
        PRINTF("[dcam] SVS-type parameters\n");

        // Left camera calibration parameters
        parseCalibrationSVS(params, SIDE_LEFT);

        // Right camera calibration parameters
        parseCalibrationSVS(params, SIDE_RIGHT);

        // external params of undistorted cameras
        extract(params, "[external]", "Tx", T[0]);
        extract(params, "[external]", "Ty", T[1]);
        extract(params, "[external]", "Tz", T[2]);
        extract(params, "[external]", "Rx", Om[0]);
        extract(params, "[external]", "Ry", Om[1]);
        extract(params, "[external]", "Rz", Om[2]);

        T[0] *= .001;
        T[1] *= .001;
        T[2] *= .001;
    }
    else // OST-type parameters
    {
      PRINTF("[dcam] OST-type parameters\n");

      // Left camera calibration parameters
      parseCalibrationOST(params, SIDE_LEFT);

      // Right camera calibration parameters
      parseCalibrationOST(params, SIDE_RIGHT);

      // external params of undistorted cameras
      extract(params, "[externals]", "translation", T, 3);
      extract(params, "[externals]", "rotation", Om, 3);
    }

    // disparity resolution
    extract(params, "[stereo]", "dpp", dpp);
    extract(params, "[stereo]", "corrxsize", corrSize);
    extract(params, "[stereo]", "convx", filterSize);
    extract(params, "[stereo]", "ndisp", numDisp);

    // check for left camera matrix
    if (imLeft->cam_info.K[0] == 0.0)
    {
        hasRectification = false;
    }
    else
    {
        hasRectification = true;
        imLeft->hasRectification = true;
        imLeft->initRect = false;   // haven't initialized arrays, wait for image size
    }

    // check for right camera matrix
    if (imRight->cam_info.K[0] == 0.0)
    {
        hasRectification = false;
    }
    else
    {
        imRight->hasRectification = true;
        imRight->initRect = false;  // haven't initialized arrays, wait for image size
    }

    // reprojection matrix
    double Tx = imRight->cam_info.P[0] / imRight->cam_info.P[3];

    // first column
    RP[0] = 1.0;
    RP[4] = RP[8] = RP[12] = 0.0;

    // second column
    RP[5] = 1.0;
    RP[1] = RP[9] = RP[13] = 0.0;

    // third column
    RP[2] = RP[6] = RP[10] = 0.0;
    RP[14] = -Tx;

    // fourth column
    RP[3] = -imLeft->cam_info.P[2];  // cx
    RP[7] = -imLeft->cam_info.P[6];  // cy
    RP[11] = imLeft->cam_info.P[0];  // fx, fy
    RP[15] = (imLeft->cam_info.P[2] - imRight->cam_info.P[2] - (double) offx) / Tx;

    printCalibration();
}


//
// Create parameters string and save it in the imLeft->params location
//

static int
PrintMatStr(double *mat, int n, int m, char *str)
{
  int c=0;
  for (int i=0; i<n; i++)
    {
      for (int j=0; j<m; j++)
	c += sprintf(&str[c],"%8.5f ", mat[i*m+j]);
      c += sprintf(&str[c],"\n");
    }
  return c;
}

static void
PrintMat(double *mat, int n, int m)
{
  for (int i=0; i<n; i++)
    {
      for (int j=0; j<m; j++)
	printf("%8.5f ", mat[i*m+j]);
      printf("\n");
    }
}


static int
PrintStr(int val, char *str)
{
  int c=0;
  c += sprintf(&str[c],"%d ", val);
  return c;
}

char *
StereoData::createParams(bool store)
{
    char *str = new char[4096];
    int n = 0;

    // header
    n += sprintf(str,"# oST version %d.%d parameters\n\n", OST_MAJORVERSION, OST_MINORVERSION);

    // stereo params
    n += sprintf(&str[n],"\n[stereo]\n");
    n += sprintf(&str[n],"\nndisp    ");
    n += PrintStr(numDisp,&str[n]);
    n += sprintf(&str[n],"\ndpp      ");
    n += PrintStr(dpp,&str[n]);
    n += sprintf(&str[n],"\ncorrsize ");
    n += PrintStr(corrSize,&str[n]);
    n += sprintf(&str[n],"\npresize  ");
    n += PrintStr(filterSize,&str[n]);

    // externals
    n += sprintf(&str[n],"\n\n[externals]\n");

    n += sprintf(&str[n],"\ntranslation\n");
    n += PrintMatStr(T,1,3,&str[n]);

    n += sprintf(&str[n],"\nrotation\n");
    n += PrintMatStr(Om,1,3,&str[n]);

    // left camera
    n += sprintf(&str[n],"\n[left camera]\n");

    n += sprintf(&str[n],"\ncamera matrix\n");
    n += PrintMatStr(imLeft->cam_info.K.c_array(),3,3,&str[n]);

    n += sprintf(&str[n],"\ndistortion\n");
    n += PrintMatStr(imLeft->cam_info.D.c_array(),1,5,&str[n]);

    n += sprintf(&str[n],"\nrectification\n");
    n += PrintMatStr(imLeft->cam_info.R.c_array(),3,3,&str[n]);

    n += sprintf(&str[n],"\nprojection\n");
    n += PrintMatStr(imLeft->cam_info.P.c_array(),3,4,&str[n]);

    // right camera
    n += sprintf(&str[n],"\n[right camera]\n");
    n += sprintf(&str[n],"\ncamera matrix\n");
    n += PrintMatStr(imRight->cam_info.K.c_array(),3,3,&str[n]);

    n += sprintf(&str[n],"\ndistortion\n");
    n += PrintMatStr(imRight->cam_info.D.c_array(),1,5,&str[n]);

    n += sprintf(&str[n],"\nrectification\n");
    n += PrintMatStr(imRight->cam_info.R.c_array(),3,3,&str[n]);

    n += sprintf(&str[n],"\nprojection\n");
    n += PrintMatStr(imRight->cam_info.P.c_array(),3,4,&str[n]);

    str[n] = 0; // just in case

    if (store)
    {
        if (imLeft->params) { delete [] imLeft->params; }
        char *bb = new char[n];
        strcpy(bb, str);
        imLeft->params = bb;
    }

    return str;
}

//
// color processing
// two algorithms: linear interpolation, and edge-tracking interpolation
//

#define AVG(a,b) (((int)(a) + (int)(b))>>1)

// convert from Bayer to RGB (3 bytes)
void ImageData::doBayerColorRGB()
{
  if (imRawType == COLOR_CODING_NONE)
  {
    return;		// nothing to colorize
  }

  if (imColorType != COLOR_CODING_NONE)
  {
    return;		// already done
  }

  // check allocation
  size_t size = imWidth*imHeight;
  if (imSize < size)
    {
      MEMFREE(im);
      im = (uint8_t *)MEMALIGN(size);
      imSize = size;
    }
  if (imColorSize < size*3)
    {
      MEMFREE(imColor);
      imColor = (uint8_t *)MEMALIGN(size*3);
      imColorSize = size*3;
    }
  switch (imRawType) {
    case COLOR_CODING_MONO8:
      memcpy(im, imRaw, size);
      imType = COLOR_CODING_MONO8;
      return;
    case COLOR_CODING_BAYER8_GRBG:
      convertBayerGRBGColorRGB(imRaw, imColor, im, imWidth, imHeight, colorConvertType);
      break;
    case COLOR_CODING_BAYER8_BGGR:
      convertBayerBGGRColorRGB(imRaw, imColor, im, imWidth, imHeight, colorConvertType);
      break;

    default:
      PRINTF("Unsupported color coding %i", imRawType);
      return;
  }
  imType = COLOR_CODING_MONO8;
  imColorType = COLOR_CODING_RGB8;
}


void
ImageData::doBayerMono()
{

  if (imRawType == COLOR_CODING_NONE)
  {
    return;		// nothing to colorize
  }

  if (imType != COLOR_CODING_NONE)
  {
    return;		// already done
  }

  // check allocation
  size_t size = imWidth*imHeight;
  if (imSize < size)
    {
      MEMFREE(im);
      im = (uint8_t *)MEMALIGN(size);
      imSize = size;
    }
  switch (imRawType) {
    case COLOR_CODING_MONO8:
      memcpy(im, imRaw, size);
      break;
    case COLOR_CODING_BAYER8_GRBG:
      convertBayerGRBGMono(imRaw, im, imWidth, imHeight, colorConvertType);
      break;
    case COLOR_CODING_BAYER8_BGGR:
      convertBayerBGGRMono(imRaw, im, imWidth, imHeight, colorConvertType);
      break;

    default:
      PRINTF("Unsupported color coding %i", imRawType);
      return;
  }
  imType = COLOR_CODING_MONO8;
}


// real funtion to do the job
// converts to RGB

void
ImageData::convertBayerGRBGColorRGB(uint8_t *src, uint8_t *dstc, uint8_t *dstm,
				    int width, int height, color_conversion_t colorAlg)
{
  uint8_t *s;
  int i, j;
  int ll = width;
  int ll2 = width*2;

  s = src;
  int pp2 = width*3*2;          // previous 2 color lines
  int pp = width*3;             // previous color line
  uint8_t *cd = dstc;		// color
  uint8_t *md = dstm;		// monochrome

  // simple, but has "zipper" artifacts
  if (colorAlg == COLOR_CONVERSION_BILINEAR)
    {
      for (i=0; i<height; i+=2)
	{
	  // red line (GRGR...)
	  for (j=0; j<width; j+=2, cd+=6, md+=2)
	    {
	      *md = *(cd+1) = *s++;	// green pixel
	      *(cd+3+0) = *s++;	// red pixel
	      *(cd+0) = AVG(*(cd+3+0), *(cd-3+0)); // interpolated red pixel
	      if (i > 1)
		{
		  *(cd-pp+0) = AVG(*(cd-pp2+0), *(cd+0)); // interpolated red pixel
		  *(cd-pp+3+0) = AVG(*(cd-pp2+3+0), *(cd+3+0)); // interpolated red pixel
		  *(md-ll) = *(cd-pp+1) = ((int)*(cd+1) + (int)*(cd-pp-3+1) + (int)*(cd-pp+3+1) + (int)*(cd-pp2+1)) >> 2;
		}
	    }

	  // blue line (BGBG...)
	  *(cd+2) = *s;		// blue pixel
	  for (j=0; j<width-2; j+=2, cd+=6, md+=2)
	    {
	      s++;
	      *(md+1) = *(cd+3+1) = *s++; // green pixel
	      *(cd+6+2) = *s;
	      *(cd+3+2) = AVG(*(cd+2), *(cd+6+2)); // interpolated blue pixel
	      if (i > 1)
		{
		  *(cd-pp+2) = AVG(*(cd-pp2+2), *(cd+2)); // interpolated blue pixel
		  *(cd-pp+3+2) = AVG(*(cd-pp2+3+2), *(cd+3+2)); // interpolated blue pixel
		  *(md-ll+1) = *(cd-pp+3+1) = ((int)*(cd+3+1) + (int)*(cd-pp+1) + (int)*(cd-pp+6+1) + (int)*(cd-pp2+3+1)) >> 2;
		}
	    }
	  // last pixels
	  s++;
	  *(md+1) = *(cd+3+1) = *s++;      // green pixel
	  *(cd+3+2) = *(cd+2);	// interpolated blue pixel
	  if (i > 1)
	    {
	      *(cd-pp+2) = AVG(*(cd-pp2+2), *(cd+2)); // interpolated blue pixel
	      *(cd-pp+3+2) = AVG(*(cd-pp2+3+2), *(cd+3+2)); // interpolated blue pixel
	    }
	  cd +=6;
	  md +=2;
	}
    }

  // EDGE color algorithm, better but slower
  else
    {
      int a,b,c,d;
      int dc, dv, dh;
      int ww;

      // do first two lines
      cd += pp2;
      s += ll2;

      for (i=0; i<height-4; i+=2)
	{
	  // GR line
	  // do first two pixels
	  cd += 6;
	  md += 2;
	  s += 2;

	  // do most of line
	  for (j=0; j<width-4; j+=2, cd+=6, md+=2)
	    {
	      // green pixels
	      *md = *(cd+1) = *s++;
	      dc = 2*(int)*(s);
	      dh = dc - (int)*(s-2) - (int)*(s+2);
	      if (dh < 0) dh = -dh;
	      dv = dc - (int)*(s-ll2) - (int)*(s+ll2);
	      if (dv < 0) dv = -dv;
	      if (dv > dh) // vert is stronger, use horz
		*(md+1) = *(cd+3+1) = ((int)*(s-1) + (int)*(s+1))>>1;
	      else	// horz is stronger, use vert
		*(md+1) = *(cd+3+1) = ((int)*(s-ll) + (int)*(s+ll))>>1;

	      // color pixels
	      *(cd+3+0) = *s;	// red pixel

	      a = (int)*(s) - (int)*(cd+3+1);
	      b = (int)*(s-2) - (int)*(cd-3+1);
	      c = (int)*(s-ll2) - (int)*(cd-pp2+3+1);
	      d = (int)*(s-ll2-2) - (int)*(cd-pp2-3+1);

	      ww = 2*(int)*(cd+1) + (a + b);
	      if (ww < 0) ww = 0;
	      *(cd+0) = ww>>1;	// interpolated red pixel

	      ww = 2*(int)*(cd-pp+3+1) + (a + c);
	      if (ww < 0) ww = 0;
	      *(cd-pp+3+0) = ww>>1; // interpolated red pixel

	      ww = 4*(int)*(cd-pp+1) + (a + b + c + d);
	      if (ww < 0) ww = 0;
	      *(cd-pp+0) = ww>>2; // interpolated red pixel

	      s++;
	    }
	  // last two pixels
	  cd += 6;
	  md += 2;
	  s += 2;

	  // BG line
	  // do first two pixels
	  cd += 6;
	  md += 2;
	  s += 2;

	  // do most of line
	  for (j=0; j<width-4; j+=2, cd+=6, md+=2)
	    {
	      dc = 2*(int)*s;
	      dh = dc - (int)*(s-2) - (int)*(s+2);
	      if (dh < 0) dh = -dh;
	      dv = dc - (int)*(s-ll2) - (int)*(s+ll2);
	      if (dv < 0) dv = -dv;
	      if (dh < dv) // vert is stronger, use horz
		*md = *(cd+1) = ((int)*(s-1) + (int)*(s+1))>>1;
	      else	// horz is stronger, use vert
		*md = *(cd+1) = ((int)*(s-ll) + (int)*(s+ll))>>1;

	      *(md+1) = *(cd+3+1) = *(s+1); // green pixel

	      // color pixels
	      *(cd+3+2) = *s;	// blue pixel

	      a = (int)*(s) - (int)*(cd+3+1);
	      b = (int)*(s-2) - (int)*(cd-3+1);
	      c = (int)*(s-ll2) - (int)*(cd-pp2+3+1);
	      d = (int)*(s-ll2-2) - (int)*(cd-pp2-3+1);

	      ww = 2*(int)*(cd+1) + (a + b);
	      if (ww < 0) ww = 0;
	      *(cd+2) = ww>>1;	// interpolated blue pixel

	      ww = 2*(int)*(cd-pp+3+1) + (a + c);
	      if (ww < 0) ww = 0;
	      *(cd-pp+3+2) = ww>>1; // interpolated blue pixel

	      ww = 4*(int)*(cd-pp+1) + (a + b + c + d);
	      if (ww < 0) ww = 0;
	      *(cd-pp+2) = ww>>2; // interpolated blue pixel

	      s+=2;
	    }
	  // last two pixels
	  cd += 6;
	  md += 2;
	  s += 2;
	}

      // last two lines
      for (j=0; j<width; j+=2)
	{
	}
    }

}

void
ImageData::convertBayerBGGRColorRGB(uint8_t *src, uint8_t *dstc, uint8_t *dstm,
				    int width, int height, color_conversion_t colorAlg)
{
  uint8_t *s;
  int i, j;
  int ll = width;
  int ll2 = width*2;

  s = src;
  int pp2 = width*3*2;          // previous 2 color lines
  int pp = width*3;             // previous color line
  uint8_t *cd = dstc;		// color
  uint8_t *md = dstm;		// monochrome

  // simple, but has "zipper" artifacts
  if (colorAlg == COLOR_CONVERSION_BILINEAR)
    {
      for (i=0; i<height; i+=2)
	{
          // blue line (BGBG...)
	  *(cd+2) = *s;		// blue pixel
	  for (j=0; j<width-2; j+=2, cd+=6, md+=2)
	    {
	      s++;
	      *(md+1) = *(cd+3+1) = *s++; // green pixel
	      *(cd+6+2) = *s;
	      *(cd+3+2) = AVG(*(cd+2), *(cd+6+2)); // interpolated blue pixel
	      if (i > 1)
		{
		  *(cd-pp+2) = AVG(*(cd-pp2+2), *(cd+2)); // interpolated blue pixel
		  *(cd-pp+3+2) = AVG(*(cd-pp2+3+2), *(cd+3+2)); // interpolated blue pixel
		  *(md-ll+1) = *(cd-pp+3+1) = ((int)*(cd+3+1) + (int)*(cd-pp+1) + (int)*(cd-pp+6+1) + (int)*(cd-pp2+3+1)) >> 2;
		}
	    }
	  // last pixels
	  s++;
	  *(md+1) = *(cd+3+1) = *s++;      // green pixel
	  *(cd+3+2) = *(cd+2);	// interpolated blue pixel
	  if (i > 1)
	    {
	      *(cd-pp+2) = AVG(*(cd-pp2+2), *(cd+2)); // interpolated blue pixel
	      *(cd-pp+3+2) = AVG(*(cd-pp2+3+2), *(cd+3+2)); // interpolated blue pixel
	    }
	  cd +=6;
	  md +=2;

	  // red line (GRGR...)
	  for (j=0; j<width; j+=2, cd+=6, md+=2)
	    {
	      *md = *(cd+1) = *s++;	// green pixel
	      *(cd+3+0) = *s++;	// red pixel
	      *(cd+0) = AVG(*(cd+3+0), *(cd-3+0)); // interpolated red pixel
	      if (i > 1)
		{
		  *(cd-pp+0) = AVG(*(cd-pp2+0), *(cd+0)); // interpolated red pixel
		  *(cd-pp+3+0) = AVG(*(cd-pp2+3+0), *(cd+3+0)); // interpolated red pixel
		  *(md-ll) = *(cd-pp+1) = ((int)*(cd+1) + (int)*(cd-pp-3+1) + (int)*(cd-pp+3+1) + (int)*(cd-pp2+1)) >> 2;
		}
	    }
	}
    }

  // EDGE color algorithm, better but slower
  else
    {
      int a,b,c,d;
      int dc, dv, dh;
      int ww;

      // do first two lines
      cd += pp2;
      s += ll2;

      for (i=0; i<height-4; i+=2)
	{
          // BG line
	  // do first two pixels
	  cd += 6;
	  md += 2;
	  s += 2;

	  // do most of line
	  for (j=0; j<width-4; j+=2, cd+=6, md+=2)
	    {
	      dc = 2*(int)*s;
	      dh = dc - (int)*(s-2) - (int)*(s+2);
	      if (dh < 0) dh = -dh;
	      dv = dc - (int)*(s-ll2) - (int)*(s+ll2);
	      if (dv < 0) dv = -dv;
	      if (dh < dv) // vert is stronger, use horz
		*md = *(cd+1) = ((int)*(s-1) + (int)*(s+1))>>1;
	      else	// horz is stronger, use vert
		*md = *(cd+1) = ((int)*(s-ll) + (int)*(s+ll))>>1;

	      *(md+1) = *(cd+3+1) = *(s+1); // green pixel

	      // color pixels
	      *(cd+3+2) = *s;	// blue pixel

	      a = (int)*(s) - (int)*(cd+3+1);
	      b = (int)*(s-2) - (int)*(cd-3+1);
	      c = (int)*(s-ll2) - (int)*(cd-pp2+3+1);
	      d = (int)*(s-ll2-2) - (int)*(cd-pp2-3+1);

	      ww = 2*(int)*(cd+1) + (a + b);
	      if (ww < 0) ww = 0;
	      *(cd+2) = ww>>1;	// interpolated blue pixel

	      ww = 2*(int)*(cd-pp+3+1) + (a + c);
	      if (ww < 0) ww = 0;
	      *(cd-pp+3+2) = ww>>1; // interpolated blue pixel

	      ww = 4*(int)*(cd-pp+1) + (a + b + c + d);
	      if (ww < 0) ww = 0;
	      *(cd-pp+2) = ww>>2; // interpolated blue pixel

	      s+=2;
	    }
	  // last two pixels
	  cd += 6;
	  md += 2;
	  s += 2;

	  // GR line
	  // do first two pixels
	  cd += 6;
	  md += 2;
	  s += 2;

	  // do most of line
	  for (j=0; j<width-4; j+=2, cd+=6, md+=2)
	    {
	      // green pixels
	      *md = *(cd+1) = *s++;
	      dc = 2*(int)*(s);
	      dh = dc - (int)*(s-2) - (int)*(s+2);
	      if (dh < 0) dh = -dh;
	      dv = dc - (int)*(s-ll2) - (int)*(s+ll2);
	      if (dv < 0) dv = -dv;
	      if (dv > dh) // vert is stronger, use horz
		*(md+1) = *(cd+3+1) = ((int)*(s-1) + (int)*(s+1))>>1;
	      else	// horz is stronger, use vert
		*(md+1) = *(cd+3+1) = ((int)*(s-ll) + (int)*(s+ll))>>1;

	      // color pixels
	      *(cd+3+0) = *s;	// red pixel

	      a = (int)*(s) - (int)*(cd+3+1);
	      b = (int)*(s-2) - (int)*(cd-3+1);
	      c = (int)*(s-ll2) - (int)*(cd-pp2+3+1);
	      d = (int)*(s-ll2-2) - (int)*(cd-pp2-3+1);

	      ww = 2*(int)*(cd+1) + (a + b);
	      if (ww < 0) ww = 0;
	      *(cd+0) = ww>>1;	// interpolated red pixel

	      ww = 2*(int)*(cd-pp+3+1) + (a + c);
	      if (ww < 0) ww = 0;
	      *(cd-pp+3+0) = ww>>1; // interpolated red pixel

	      ww = 4*(int)*(cd-pp+1) + (a + b + c + d);
	      if (ww < 0) ww = 0;
	      *(cd-pp+0) = ww>>2; // interpolated red pixel

	      s++;
	    }
	  // last two pixels
	  cd += 6;
	  md += 2;
	  s += 2;
	}

      // last two lines
      for (j=0; j<width; j+=2)
	{
	}
    }

}


// real function to do the job
// converts to monochrome

void
ImageData::convertBayerGRBGMono(uint8_t *src, uint8_t *dstm,
				int width, int height, color_conversion_t colorAlg)
{
  uint8_t *s;
  int i, j;
  int ll = width;
  int ll2 = width*2;

  s = src;
  uint8_t *md = dstm;		// monochrome

  // simple, but has "zipper" artifacts
  if (colorAlg == COLOR_CONVERSION_BILINEAR)
    {
      for (i=0; i<height; i+=2)
	{
	  // red line (GRGR...)
	  for (j=0; j<width; j+=2, md+=2)
	    {
	      *md = *s++;	// green pixel
	      s++;	// red pixel
	      if (i > 1)
		*(md-ll) = ((int)*(md) + (int)*(md-ll-1) + (int)*(md-ll+1) + (int)*(md-ll2)) >> 2;
	    }

	  // blue line (BGBG...)
	  for (j=0; j<width-2; j+=2, md+=2)
	    {
	      s++;
	      *(md+1) = *s++; // green pixel
	      if (i > 1)
		*(md-ll+1) = ((int)*(md+1) + (int)*(md-ll) + (int)*(md-ll+2) + (int)*(md-ll2+1)) >> 2;
	    }
	  // last pixels
	  s++;
	  *(md+1) = *s++;	// green pixel
	  md +=2;
	}
    }

  // EDGE color algorithm, better but slower
  else
    {
      int dc, dv, dh;

      // do first two lines
      s += ll2;

      for (i=0; i<height-4; i+=2)
	{
	  // GR line
	  // do first two pixels
	  md += 2;
	  s += 2;

	  // do most of line
	  for (j=0; j<width-4; j+=2, md+=2)
	    {
	      // green pixels
	      *md = *s++;
	      dc = 2*(int)*(s);
	      dh = dc - (int)*(s-2) - (int)*(s+2);
	      if (dh < 0) dh = -dh;
	      dv = dc - (int)*(s-ll2) - (int)*(s+ll2);
	      if (dv < 0) dv = -dv;
	      if (dv > dh) // vert is stronger, use horz
		*(md+1) = ((int)*(s-1) + (int)*(s+1))>>1;
	      else	// horz is stronger, use vert
		*(md+1) = ((int)*(s-ll) + (int)*(s+ll))>>1;

	      s++;
	    }
	  // last two pixels
	  md += 2;
	  s += 2;

	  // BG line
	  // do first two pixels
	  md += 2;
	  s += 2;

	  // do most of line
	  for (j=0; j<width-4; j+=2, md+=2)
	    {
	      dc = 2*(int)*s;
	      dh = dc - (int)*(s-2) - (int)*(s+2);
	      if (dh < 0) dh = -dh;
	      dv = dc - (int)*(s-ll2) - (int)*(s+ll2);
	      if (dv < 0) dv = -dv;
	      if (dh < dv) // vert is stronger, use horz
		*md = ((int)*(s-1) + (int)*(s+1))>>1;
	      else	// horz is stronger, use vert
		*md = ((int)*(s-ll) + (int)*(s+ll))>>1;

	      *(md+1) = *(s+1); // green pixel

	      s+=2;
	    }
	  // last two pixels
	  md += 2;
	  s += 2;
	}

      // last two lines
      for (j=0; j<width; j+=2)
	{
	}
    }

}

void
ImageData::convertBayerBGGRMono(uint8_t *src, uint8_t *dstm,
				int width, int height, color_conversion_t colorAlg)
{
  uint8_t *s;
  int i, j;
  int ll = width;
  int ll2 = width*2;

  s = src;
  uint8_t *md = dstm;		// monochrome

  // simple, but has "zipper" artifacts
  if (colorAlg == COLOR_CONVERSION_BILINEAR)
    {
      for (i=0; i<height; i+=2)
	{
          // blue line (BGBG...)
	  for (j=0; j<width-2; j+=2, md+=2)
	    {
	      s++;
	      *(md+1) = *s++; // green pixel
	      if (i > 1)
		*(md-ll+1) = ((int)*(md+1) + (int)*(md-ll) + (int)*(md-ll+2) + (int)*(md-ll2+1)) >> 2;
	    }
	  // last pixels
	  s++;
	  *(md+1) = *s++;	// green pixel
	  md +=2;

	  // red line (GRGR...)
	  for (j=0; j<width; j+=2, md+=2)
	    {
	      *md = *s++;	// green pixel
	      s++;	// red pixel
	      if (i > 1)
		*(md-ll) = ((int)*(md) + (int)*(md-ll-1) + (int)*(md-ll+1) + (int)*(md-ll2)) >> 2;
	    }
	}
    }

  // EDGE color algorithm, better but slower
  else
    {
      int dc, dv, dh;

      // do first two lines
      s += ll2;

      for (i=0; i<height-4; i+=2)
	{
          // BG line
	  // do first two pixels
	  md += 2;
	  s += 2;

	  // do most of line
	  for (j=0; j<width-4; j+=2, md+=2)
	    {
	      dc = 2*(int)*s;
	      dh = dc - (int)*(s-2) - (int)*(s+2);
	      if (dh < 0) dh = -dh;
	      dv = dc - (int)*(s-ll2) - (int)*(s+ll2);
	      if (dv < 0) dv = -dv;
	      if (dh < dv) // vert is stronger, use horz
		*md = ((int)*(s-1) + (int)*(s+1))>>1;
	      else	// horz is stronger, use vert
		*md = ((int)*(s-ll) + (int)*(s+ll))>>1;

	      *(md+1) = *(s+1); // green pixel

	      s+=2;
	    }
	  // last two pixels
	  md += 2;
	  s += 2;

	  // GR line
	  // do first two pixels
	  md += 2;
	  s += 2;

	  // do most of line
	  for (j=0; j<width-4; j+=2, md+=2)
	    {
	      // green pixels
	      *md = *s++;
	      dc = 2*(int)*(s);
	      dh = dc - (int)*(s-2) - (int)*(s+2);
	      if (dh < 0) dh = -dh;
	      dv = dc - (int)*(s-ll2) - (int)*(s+ll2);
	      if (dv < 0) dv = -dv;
	      if (dv > dh) // vert is stronger, use horz
		*(md+1) = ((int)*(s-1) + (int)*(s+1))>>1;
	      else	// horz is stronger, use vert
		*(md+1) = ((int)*(s-ll) + (int)*(s+ll))>>1;

	      s++;
	    }
	  // last two pixels
	  md += 2;
	  s += 2;
	}

      // last two lines
      for (j=0; j<width; j+=2)
	{
	}
    }

}
