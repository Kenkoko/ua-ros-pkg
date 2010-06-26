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
// image.cpp
// classes for monocular and stereo image
//

#include <sstream>
#include <iostream>

#include <dcam1394/image_proc.h>

#define PRINTF(a...) printf(a)

using namespace cam;

// image class fns

ImageData::ImageData()
{
  imWidth = imHeight = 0;

  // buffers
  im = NULL;
  imColor = NULL;
  imRect = NULL;
  imRectColor = NULL;
  imRaw = NULL;
  imRawType = COLOR_CODING_NONE;
  imRawSize = 0;
  imType = COLOR_CODING_NONE;
  imSize = 0;
  imColorType = COLOR_CODING_NONE;
  imColorSize = 0;
  imRectType = COLOR_CODING_NONE;
  imRectSize = 0;
  imRectColorType = COLOR_CODING_NONE;
  imRectColorSize = 0;
  params = NULL;

  // color conversion
  colorConvertType = COLOR_CONVERSION_BILINEAR;
  //  colorConvertType = COLOR_CONVERSION_EDGE;

  // rectification mapping
  hasRectification = false;
  initRect = false;
  rMapxy = NULL;
  rMapa = NULL;

  // calibration matrices
  rD = cvCreateMat(5,1,CV_64F);
  rK = cvCreateMat(3,3,CV_64F);
  rR = cvCreateMat(3,3,CV_64F);
  rKp = cvCreateMat(3,3,CV_64F);

  // temp images, these will be changed when used
  srcIm = cvCreateImageHeader(cvSize(640,480), IPL_DEPTH_8U, 1);
  dstIm = cvCreateImageHeader(cvSize(640,480), IPL_DEPTH_8U, 1);
}

ImageData::~ImageData()
{
  releaseBuffers();
  cvReleaseImageHeader(&srcIm);
  cvReleaseImageHeader(&dstIm);
}

// storage

void
ImageData::releaseBuffers()
{
  // should we release im_raw???
  MEMFREE(im);
  MEMFREE(imColor);
  MEMFREE(imRect);
  MEMFREE(imRectColor);
  im = NULL;
  imColor = NULL;
  imRect = NULL;
  imRectColor = NULL;
  imRaw = NULL;

  imRawType = COLOR_CODING_NONE;
  imRawSize = 0;
  imType = COLOR_CODING_NONE;
  imSize = 0;
  imColorType = COLOR_CODING_NONE;
  imColorSize = 0;
  imRectType = COLOR_CODING_NONE;
  imRectSize = 0;
  imRectColorType = COLOR_CODING_NONE;
  imRectColorSize = 0;

  initRect = false;
  if (rMapxy)
    cvReleaseMat(&rMapxy);
  if (rMapa)
    cvReleaseMat(&rMapa);
  rMapxy = NULL;
  rMapa = NULL;
}



// rectification

bool
ImageData::initRectify(bool force)
{
  if (force)
    hasRectification = true;

  if (!hasRectification || imWidth == 0 || imHeight == 0)
    return false;

  if (initRect && !force && rMapxy != NULL && rMapa != NULL)
    return true;		// already done

  // set values of cal matrices
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      CV_MAT_ELEM(*rK, double, i, j) = cam_info.K[i*3+j];

  // rectified K matrix, from projection matrix
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      CV_MAT_ELEM(*rKp, double, i, j) = cam_info.P[i*4+j];

  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      CV_MAT_ELEM(*rR, double, i, j) = cam_info.R[i*3+j];

  for (int i=0; i<5; i++)
    CV_MAT_ELEM(*rD, double, i, 0) = cam_info.D[i];

  // Set up rectification mapping
  rMapxy = cvCreateMat(imHeight, imWidth, CV_16SC2);
  rMapa  = cvCreateMat(imHeight, imWidth, CV_16UC1);//CV_16SC1);
  mx = cvCreateMat(imHeight, imWidth, CV_32FC1);
  my = cvCreateMat(imHeight, imWidth, CV_32FC1);
  cvInitUndistortRectifyMap(rK,rD,rR,rKp,mx,my);
  cvConvertMaps(mx,my,rMapxy,rMapa);

  initRect = true;
  return true;
}


bool
ImageData::doRectify()
{
  if (!hasRectification)
  {
    return false;		// has no rectification
  }

  if (imWidth == 0 || imHeight == 0)
  {
    return false;
  }

  doBayerMono();

  if (imType == COLOR_CODING_NONE && imColorType == COLOR_CODING_NONE)
  {
    return false;		// nothing to rectify
  }

  if (!((imType != COLOR_CODING_NONE && imRectType == COLOR_CODING_NONE) ||
	(imColorType != COLOR_CODING_NONE && imRectColorType == COLOR_CODING_NONE)))
  {
    return true;		// already done
  }

  initRectify();		// ok to call multiple times

  CvSize size = cvSize(imWidth,imHeight);

  // rectify grayscale image
  if (imType != COLOR_CODING_NONE)
    {
      // set up rectified data buffer
      if (imRectSize < imSize)
	{
	  MEMFREE(imRect);
	  imRectSize = imWidth*imHeight;
	  imRect = (uint8_t *)MEMALIGN(imSize);
	}

      // set up images
      imRectType = imType;
      cvInitImageHeader(srcIm, size, IPL_DEPTH_8U, 1);
      cvInitImageHeader(dstIm, size, IPL_DEPTH_8U, 1);
      cvSetData(srcIm, im, imWidth);
      cvSetData(dstIm, imRect, imWidth);

      cvRemap(srcIm,dstIm,rMapxy,rMapa);
      //cvRemap(srcIm,dstIm,mx,my);
    }

  // rectify color image
  // assumes RGB
  if (imColorType != COLOR_CODING_NONE)
    {
      // set up rectified data buffer
      if (imRectColorSize < imColorSize)
	{
	  MEMFREE(imRectColor);
	  imRectColorSize = imWidth*imHeight*3;
	  imRectColor = (uint8_t *)MEMALIGN(imRectColorSize);
	}

      // set up images
      imRectColorType = imColorType;
      cvInitImageHeader(srcIm, size, IPL_DEPTH_8U, 3);
      cvInitImageHeader(dstIm, size, IPL_DEPTH_8U, 3);
      cvSetData(srcIm, imColor, imWidth*3);
      cvSetData(dstIm, imRectColor, imWidth*3);

      cvRemap(srcIm,dstIm,rMapxy,rMapa);
      //cvRemap(srcIm,dstIm,mx,my);
    }
  return true;
}


//
// color processing
// two algorithms: linear interpolation, and edge-tracking interpolation
//

// convert from Bayer to RGB (3 bytes)

#define AVG(a,b) (((int)(a) + (int)(b))>>1)

void
ImageData::doBayerColorRGB()
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
      ROS_WARN("Unsupported color coding %i", imRawType);
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
      ROS_WARN("Unsupported color coding %i", imRawType);
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
