#!/usr/bin/env python


import sys
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
 
if __name__ == '__main__':
    
     
   rospy.init_node('play_sound_test', anonymous = True)
     
   soundclient = SoundClient()
  
   rospy.sleep(1)
   
   # Built-in sounds
   # BACKINGUP = 1
   # NEEDS_UNPLUGGING = 2
   # NEEDS_PLUGGING = 3
   # NEEDS_UNPLUGGING_BADLY = 4
   # NEEDS_PLUGGING_BADLY = 5

   
   # Horn
   soundclient.play(4)
   rospy.sleep(1)




