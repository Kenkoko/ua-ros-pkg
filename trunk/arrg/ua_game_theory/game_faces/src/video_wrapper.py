import sys
import time
import socket
import threading

import shlex, subprocess

 
class videowrapper:
    def __init__(self, player_id):
        self.player_id = player_id
        self.starttime = time.time()
        
        # Using rosrun spawns two processes, and the second does not terminate correctly 
        #run_video_command = 'rosrun game_faces game_video_capture ' + str(self.player.player_id)
        run_video_command = '../bin/game_video_capture ' + str(self.player_id)
        
        self.vcommand = shlex.split(run_video_command)
        
    def start_video(self):
        self.video = subprocess.Popen(self.vcommand, stdin=subprocess.PIPE)
        
    def send_msg(self, play_number, amount, msgtime = -1):
        timenow = time.time()
        #self.video.communicate(input)  # This is the recommended way to write to stdin, but it waits for the process to terminate
        self.video.stdin.write("m\n") # Send m to let the video know there's a message
        self.video.stdin.flush()
        self.video.stdin.write("%d %d %d\n" % (play_number, amount, timenow) )   # This is not the recommended way to write to stdin
        self.video.stdin.flush()
        
        #print "Times: ", msgtime, ' ', timenow
        
    def end_video(self):
        self.video.terminate()
        