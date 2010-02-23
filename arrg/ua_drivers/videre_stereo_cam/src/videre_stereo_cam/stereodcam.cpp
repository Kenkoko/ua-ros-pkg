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

//
// Stereo driver and processor
// Main class is StereoCam for processing
// Subclasses:
//   StereoDcam - gets stereo images from DCAM (IEEE1394) devices
//   

#include <stereo_image_proc/stereolib.h>
#include "videre_stereo_cam/stereodcam.h"

#define PRINTF(a...) printf(a)


// StereoCam class
// Performs rectification and disparity calculation
//

using namespace dcam;


// StereoDcam class
// Conjoins image grabbing and stereo
//

StereoDcam::StereoDcam(uint64_t guid, size_t buffersize)
  : Dcam(guid,buffersize)
{
  // set up stereo image data
  stIm = new StereoData();
  
  // set up a Videre stereo cam
  if (isVidereStereo)
    {
      char *params;
      params = getParameters(); // ok, these are the params

      if (params == NULL)
	{
	  PRINTF("Whoa nellie - no params!\n");
	}
      else			// parse out the stereo params
	stIm->extractParams(params);
    }
}


StereoDcam::~StereoDcam()
{
  delete stIm;
}
 

// format of image

void 
StereoDcam::setFormat(dc1394video_mode_t video, dc1394framerate_t fps,
		      dc1394speed_t speed)
{
  stIm->releaseBuffers();	// release all buffers in the stereo data structure
  Dcam::setFormat(video, fps, speed);
}



// starting and stopping

void
StereoDcam::start()
{
  Dcam::start();
  Dcam::setCompanding(true);	// must be set after starting camera
  setUniqueThresh(stIm->uniqueThresh);
  setTextureThresh(stIm->textureThresh);
}

void
StereoDcam::stop()
{
  Dcam::stop();
}


// other stuff

bool
StereoDcam::getImage(int ms)	// gets the next image, with timeout
{
  bool ret = Dcam::getImage(ms);
  if (!ret)			// did we get a good image?
    return false;

  // image size
  stIm->setSize(camIm->imWidth,camIm->imHeight);

  // zero out image data indicators
  stIm->imLeft->imType = COLOR_CODING_NONE;
  stIm->imLeft->imColorType = COLOR_CODING_NONE;
  stIm->imLeft->imRectType = COLOR_CODING_NONE;
  stIm->imLeft->imRectColorType = COLOR_CODING_NONE;
  stIm->imRight->imType = COLOR_CODING_NONE;
  stIm->imRight->imColorType = COLOR_CODING_NONE;
  stIm->imRight->imRectType = COLOR_CODING_NONE;
  stIm->imRight->imRectColorType = COLOR_CODING_NONE;
  stIm->hasDisparity = false;

  // check for single-device stereo, and process
  if (isVidereStereo)
    {
      switch (rawType)
	{
	case VIDERE_STOC_RECT_RECT:
	  stereoDeinterlace(camIm->imRaw, &stIm->imLeft->imRect, &stIm->imLeft->imRectSize,
			    &stIm->imRight->imRect, &stIm->imRight->imRectSize);
	  stIm->imLeft->imRectType = COLOR_CODING_MONO8;
	  stIm->imRight->imRectType = COLOR_CODING_MONO8;
	  break;

	case VIDERE_STOC_RAW_RAW_MONO:
	case VIDERE_STEREO_MONO:
	  stereoDeinterlace(camIm->imRaw, &stIm->imLeft->im, &stIm->imLeft->imSize, 
			    &stIm->imRight->im, &stIm->imRight->imSize);
	  stIm->imLeft->imType = COLOR_CODING_MONO8;
	  stIm->imRight->imType = COLOR_CODING_MONO8;
	  break;

	case VIDERE_STOC_RAW_RAW_GRBG:
	case VIDERE_STEREO_RGGB:
	case VIDERE_STEREO_BGGR:
	case VIDERE_STEREO_GRBG:
	  stereoDeinterlace(camIm->imRaw, &stIm->imLeft->imRaw, &stIm->imLeft->imRawSize,
			    &stIm->imRight->imRaw, &stIm->imRight->imRawSize);
	  stIm->imLeft->imRawType = COLOR_CODING_BAYER8_GRBG;
	  stIm->imRight->imRawType = COLOR_CODING_BAYER8_GRBG;
	  stIm->imLeft->doBayerColorRGB();
	  stIm->imRight->doBayerColorRGB();
	  break;


	case VIDERE_STOC_RECT_DISP:
	  stereoDeinterlace2(camIm->imRaw, &stIm->imLeft->imRect, &stIm->imLeft->imRectSize, 
			    &stIm->imDisp, &stIm->imDispSize);
	  stIm->imLeft->imRectType = COLOR_CODING_MONO8;
	  stIm->hasDisparity = true;
	  stIm->doSpeckle();	// apply speckle filter
	  break;

	case VIDERE_STOC_RAW_DISP_MONO:
	  stereoDeinterlace2(camIm->imRaw, &stIm->imLeft->im, &stIm->imLeft->imSize, 
			    &stIm->imDisp, &stIm->imDispSize);
	  stIm->imLeft->imType = COLOR_CODING_MONO8;
	  //          printf("Setting stIm->imLeft->imType to %d\n", stIm->imLeft->imType);
          //	  stIm->imLeft->doBayerMono();
	  stIm->hasDisparity = true;
	  stIm->doSpeckle();	// apply speckle filter
	  break;

	case VIDERE_STOC_RAW_DISP_GRBG:
	  stereoDeinterlace2(camIm->imRaw, &stIm->imLeft->imRaw, &stIm->imLeft->imRawSize, 
			    &stIm->imDisp, &stIm->imDispSize);
	  stIm->imLeft->imRawType = COLOR_CODING_BAYER8_GRBG;
	  stIm->imLeft->doBayerColorRGB();
	  stIm->hasDisparity = true;
	  stIm->doSpeckle();	// apply speckle filter
	  break;

	default:
	  break;
	}
    }

  return ret;
}

void
StereoDcam::setCapturePolicy(dc1394capture_policy_t policy)
{
  Dcam::setCapturePolicy(policy);
}

void
StereoDcam::setFeature(dc1394feature_t feature, uint32_t value, uint32_t value2)
{
  Dcam::setFeature(feature, value, value2);
}


void 
StereoDcam::setFeatureAbsolute(dc1394feature_t feature, float value)
{
  Dcam::setFeatureAbsolute(feature, value);
}


void 
StereoDcam::setFeatureMode(dc1394feature_t feature, dc1394feature_mode_t mode)
{
  Dcam::setFeatureMode(feature, mode);
}


void 
StereoDcam::setRegister(uint64_t offset, uint32_t value)
{
  Dcam::setRegister(offset, value);
}


bool
StereoDcam::setTextureThresh(int thresh)
{
  stIm->setTextureThresh(thresh);
  if (isSTOC)
    {
      thresh = thresh/3;
      usleep(50000);
      if (thresh < 0)
	thresh = 0;
      if (thresh > 63)
	thresh = 63;
      uint32_t t_thresh = 0x08000000 | (0x40 << 16) | ( thresh << 16);
      setRegister(0xFF000, t_thresh);
    }
  return true;
}

bool
StereoDcam::setUniqueThresh(int thresh)
{
  stIm->setUniqueThresh(thresh);
  if (isSTOC)
    {
      thresh = thresh/3;
      usleep(50000);
      if (thresh < 0)
	thresh = 0;
      if (thresh > 63)
	thresh = 63;
      uint32_t u_thresh = 0x08000000 | (0x00 << 16) | ( thresh << 16);
      setRegister(0xFF000, u_thresh);
    }
  return true;
}

bool
StereoDcam::setSmoothnessThresh(int thresh)
{
  stIm->setSmoothnessThresh(thresh);
  return true;
}


bool
StereoDcam::setHoropter(int val)
{
  stIm->setHoropter(val);
  stIm->setDispOffsets();	// reset offsets

  // set it on STOC
  if (isSTOC)
    {
      usleep(50000);
      if (val < 0)
	val = 0;
      if (val > 63)
	val = 63;
      uint32_t u_val = 0x08000000 | (0xC0 << 16) | ( val << 16);
      setRegister(0xFF000, u_val);
    }
  return true;
}

bool
StereoDcam::setNumDisp(int val)
{
  stIm->setNumDisp(val);	
  return false;			// can't set number of disparities in STOC
}

bool
StereoDcam::setSpeckleSize(int val)
{
  stIm->speckleRegionSize = val;
  return true;
}

bool
StereoDcam::setSpeckleDiff(int val)
{
  stIm->speckleDiff = val;
  return true;
}

bool
StereoDcam::setCorrsize(int val)
{
  stIm->corrSize = val;
  stIm->setDispOffsets();	// reset offsets
  return true;
}

bool
StereoDcam::setRangeMax(double val)
{
  stIm->setRangeMax(val);	// set max range in pt cloud
  return true;
}

bool
StereoDcam::setRangeMin(double val)
{
  stIm->setRangeMin(val);	// set min range in pt cloud
  return true;
}

bool
StereoDcam::setUniqueCheck(bool unique_check)
{
  stIm->setUniqueCheck(unique_check);
  return true;
}

//
// utility functions
//


// de-interlace stereo data, reserving storage if necessary

void
StereoDcam::stereoDeinterlace(uint8_t *src, uint8_t **d1, size_t *s1, 
			     uint8_t **d2, size_t *s2)
{
  size_t size = stIm->imWidth*stIm->imHeight;
  if (*s1 < size)		// need to check alignment here...
    {
      MEMFREE(*d1);
      *d1 = (uint8_t *)MEMALIGN(size);
      *s1 = size;
    }
  if (*s2 < size)
    {
      MEMFREE(*d2);
      *d2 = (uint8_t *)MEMALIGN(size);
      *s2 = size;
    }
  
  uint8_t *dd1 = *d1;
  uint8_t *dd2 = *d2;
  for (int i=0; i<(int)size; i++)
    {
      *dd2++ = *src++;
      *dd1++ = *src++;
    }
}


// de-interlace stereo data, reserving storage if necessary
// second buffer is 16-bit disparity data

void
StereoDcam::stereoDeinterlace2(uint8_t *src, uint8_t **d1, size_t *s1, 
			      int16_t **d2, size_t *s2)
{
  int w = stIm->imWidth;
  int h = stIm->imHeight;
  size_t size = w*h;

  if (*s1 < size)		// need to check alignment here...
    {
      MEMFREE(*d1);
      *d1 = (uint8_t *)MEMALIGN(size);
      *s1 = size;
    }
  if (*s2 < size*2)
    {
      MEMFREE(*d2);
      *d2 = (int16_t *)MEMALIGN(size*2);
      *s2 = size*2;
    }
  
  uint8_t *dd1 = *d1;
  int16_t *dd2 = *d2;

  int dt = stIm->imDtop;
  int dl = stIm->imDleft;
  int dw = stIm->imDwidth;
  int dh = stIm->imDheight;

  /*
   * source rectangle
   * ================
   * (w-dwidth, h-dheight) => upper left
   * (dwidth, dheight)     => size
   *
   * dest rectangle
   * ==============
   * (dleft-6,dtop)        => upper left
   * (dwidth,dheight)      => size
   */
  
  dd2 += (dt*w + dl - 6) - ((h-dh)*w + w - dw);

  size = (h-dh)*w;
  for (int i=0; i<(int)size; i++)
    {
      dd2++;
      src++;
      *dd1++ = *src++;
    }

  size = dh*w;
  for (int i=0; i<(int)size; i++)
    {
      *dd2++ = ((uint16_t)*src++)<<2;
      *dd1++ = *src++;
    }
}



// visible calls
void StereoDcam::doBayerColorRGB() { stIm->doBayerColorRGB(); }
void StereoDcam::doBayerMono() { stIm->doBayerColorRGB(); }

bool StereoDcam::doDisparity(stereo_algorithm_t alg) { return stIm->doDisparity(alg); }
bool StereoDcam::doRectify() { return stIm->doRectify(); }
bool StereoDcam::doCalcPts() { return stIm->doCalcPts(); }

