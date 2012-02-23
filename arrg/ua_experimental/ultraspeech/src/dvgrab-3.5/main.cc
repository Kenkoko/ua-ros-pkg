/*
* main.cc -- A DV/1394 capture utility
* Copyright (C) 2000-2002 Arne Schirmacher <arne@schirmacher.de>
* Copyright (C) 2003-2008 Dan Dennedy <dan@dennedy.org>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software Foundation,
* Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
*/

/** the dvgrab main program
 
    contains the main logic
*/

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

// C++ includes

#include <string>
#include <iostream>
using std::cout;
using std::endl;

// C includes

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <string.h>
#include <sys/time.h> 
#include <sys/resource.h>
#include <sys/mman.h>

// local includes

#include "io.h"
#include "dvgrab.h"
#include "error.h"

// ros
#include <ros/ros.h>
#include "ultraspeech/Control.h"
#include "std_msgs/UInt64.h"

bool g_done = false;

class FakeMain
{
private:
u_int8_t run_status_;
std::string subdir_;
ros::Subscriber sub_;
// ros::Publisher framenum_pub_;

public:
DVgrab dvgrab; 
  
FakeMain( const ros::NodeHandle& nh, int argc, char **argv )
	:run_status_(0), dvgrab(argc, argv)
{
	int ret = 0;
	//dvgrab = DVgrab( argc, argv );
	
	ros::NodeHandle local_nh("~");
	sub_ = local_nh.subscribe("/control", 1, &FakeMain::control_cb, this);
//         framenum_pub_ = local_nh.advertise<std_msgs::UInt64>("framenum", 1);

	
	fcntl( fileno( stderr ), F_SETFL, O_NONBLOCK );
	try
	{
		if ( rt_raisepri( 1 ) != 0 )
			setpriority( PRIO_PROCESS, 0, -20 );

#if _POSIX_MEMLOCK > 0
		mlockall( MCL_CURRENT | MCL_FUTURE );
#endif

	}
	catch ( std::string s )
	{
		fprintf( stderr, "Error: %s\n", s.c_str() );
		fflush( stderr );
		ret = 1;
	}
	catch ( ... )
	{
		fprintf( stderr, "Error: unknown\n" );
		fflush( stderr );
		ret = 1;
	}

	fprintf( stderr, "\n" );
		
	//return ret;
}


void control_cb(const ultraspeech::ControlConstPtr& msg)
{
	using std::cout;
	using std::endl;
    
	std::string save_string(msg->ultrasound_filename);
	
	run_status_ = msg->run;
	if (run_status_ > 0){
	    // Tell it where to save the file
	    dvgrab.set_dst_filename(save_string);
            
            
//             std_msgs::UInt64 msg;
//             msg.data = (uint64_t)(-1);
//             framenum_pub_.publish(msg);

	    //  Now go ahead and start capturing 
	    dvgrab.startCapture();
	}
	else{
	    std::cout << "StoppingCapture" << std::endl;
	    dvgrab.stopCapture();
	    
	}
}


int rt_raisepri (int pri)
{
#ifdef _SC_PRIORITY_SCHEDULING
	struct sched_param scp;

	/*
	 * Verify that scheduling is available
	 */
	if ( sysconf( _SC_PRIORITY_SCHEDULING ) == -1)
	{
// 		sendEvent( "Warning: RR-scheduler not available, disabling." );
		return -1;
	}
	else 
	{
		memset( &scp, '\0', sizeof( scp ) );
		scp.sched_priority = sched_get_priority_max( SCHED_RR ) - pri;
		if ( sched_setscheduler( 0, SCHED_RR, &scp ) < 0 )
		{
// 			sendEvent( "Warning: Cannot set RR-scheduler" );
			return -1;
		}
	}
	return 0;

#else
	return -1;
#endif
}


};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ros_dvgrab");
	ros::NodeHandle nh;

	FakeMain fm(nh, argc, argv);

	ros::spin();

	return 0;
}
