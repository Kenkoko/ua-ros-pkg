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

#include <stdint.h>
#include <cstring>

#include <videre_stereo_cam/stereolib.h>

//
// speckle removal algorithm
//
// basic idea: find coherent regions of similar disparities
// use 4-neighbors for checking disparity diffs
// reject regions less than a certain size
//
// algorithm:
//   read through pixels in scan-line order
//   for each pixel
//     - if not a disparity, ignore
//     - if labeled, check label status (good/bad) and assign disparity
//     - if not labeled, assign next label and propagate
//          * put on wavefront list
//          * pop first wavefront member
//              - if labeled, ignore
//              - if no disparity, ignore
//              - if not within diff, ignore
//              - label, increment count, and push 4 neighbors
//              - repeat until wavefront empty
//
//
//
// args:
//   disp:  disparity image (16b)
//   badval: value of bad disparity, usually 0
//   width, height: size of image
//   rdiff: max diff between pixels in a region
//   rcount: min count for non-speckle region, in pixels
//
// buffers:
//   labs:  holds pixel labels (32b), size is image
//   wbuf:  holds propagating wavefront (32b), size is image
//   rtype: holds region types (1=bad, 0=good)
//

#define push(x,y) { if ((disp[x] != badval) && (!labels[x]) &&           \
                        (disp[x] - y < rdiff) && (disp[x] - y > -rdiff)) \
                    { labels[x] = cur; *ws++ = x; }}

void do_speckle(int16_t *disp, int16_t badval, int width, int height,
                int rdiff, int rcount,
                uint32_t *labels, uint32_t *wbuf, uint8_t *rtype)
{
    int i, j, k, cnt;
    uint32_t *ws, *ls;
    uint32_t p, cur;
    int16_t dp, *ds;

    // clear out label assignments
    memset(labels, 0x0, width * height * sizeof(uint32_t));

    // loop over rows, omit borders (TDB: use dtop etc here)
    const uint32_t MIN_P = 4 * width + 4;
    const uint32_t MAX_P = (height - 4) * width - 4;
    cur = 0; // current label

    for (i = 4; i < height - 4; i++)
    {
        k = i * width + 4;  // index of pixel
        ds = disp + k;      // buffer ptrs
        ls = labels + k;

        for (j = 4; j < width - 4; j++, k++, ls++, ds++)
        {
            // not a bad disparity
            if (*ds != badval)
            {
                // has a label, check for bad label
                if (*ls)
                {
                    // small region, zero out disparity
                    if (rtype[*ls]) { *ds = badval; }
                }
                // no label, assign and propagate
                else
                {
                    ws = wbuf;  // initialize wavefront
                    p = k;      // current pixel
                    cur++;      // next label
                    cnt = 0;    // current region count
                    labels[p] = cur;

                    // wavefront propagation
                    while (ws >= wbuf) // wavefront not empty
                    {
                        cnt++;
                        // put neighbors onto wavefront
                        dp = disp[p];
                        /// @todo These checks shouldn't really be necessary if the border is properly marked invalid
                        if (p + 1 < MAX_P)      { push(p + 1, dp); }
                        if (p - 1 >= MIN_P)     { push(p - 1, dp); }
                        if (p - width >= MIN_P) { push(p - width, dp); }
                        if (p + width < MAX_P)  { push(p + width, dp); }

                        // pop most recent and propagate
                        // NB: could try least recent, maybe better convergence
                        p = *--ws;
                    }

                    // assign label type
                    if (cnt < rcount)   // speckle region
                    {
                        // small region label
                        rtype[*ls] = 1;
                        *ds = badval;
                    }
                    else
                    {
                        // large region label
                        rtype[*ls] = 0;
                    }
                }
            }
        }
    }
}
