#!/usr/bin/env python3                                                                                  

import sensor_msgs.msg
import actionlib                                                                                           
import rospy

TOPIC = '/joint_states'
                                                                                                       
class JointStateReader(object):                                                                        
    """Listens to /joint_states and provides the latest joint angles.                                  
                                                                                                       
    Usage:                                                                                             
        joint_reader = JointStateReader()                                                              
        rospy.sleep(0.1)                                                                               
        joint_reader.get_joint('shoulder_pan_joint')                                                   
        joint_reader.get_joints(['shoulder_pan_joint', 'shoulder_lift_joint'])                         
    """        

    _joint_dict: dict

    def __init__(self):          
        self._joint_dict = {}        
        rospy.Subscriber(TOPIC, sensor_msgs.msg.JointState, self._callback)                                     

    def _callback(self, data):
        for i, joint_name in enumerate(data.name):
            self._joint_dict[joint_name] = data.position[i]

    def get_joint(self, name):                                                                         
        """Gets the latest joint value.                                                                
                                                                                                       
        Args:                                                                                          
            name: string, the name of the joint whose value we want to read.                           
                                                                                                       
        Returns: the joint value, or None if we do not have a value yet.                               
        """                                                                                            
        return self._joint_dict.get(name, None)                                                                                    
                                                                                                       
    def get_joints(self, names):                                                                       
        """Gets the latest values for a list of joint names.                    
                                                                                
        Args:                                                                   
            name: list of strings, the names of the joints whose values we want 
                to read.                                                        
                                                                                
        Returns: A list of the joint values. Values may be None if we do not    
            have a value for that joint yet.                                    
        """                                                                     
        return [self.get_joint(x) for x in names]