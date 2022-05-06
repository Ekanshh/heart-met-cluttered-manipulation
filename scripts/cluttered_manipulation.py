#!/usr/bin/env python

import rospy
from smach import State, StateMachine # State machine library
import smach_ros
import mir_states.common.action_states as gas
import mir_states.common.basic_states as mir_gbs
import mir_states.common.manipulation_states as gms
from metrics_refbox_msgs.msg import Command as CommandMsg


class ClutteredManipulation(object):
    def __init__(self, ):
        self.nh = rospy.init_node('cluttered_manipulation_node', anonymous=True)
        self.task =1
        self.command = 1
        
class RefBoxSubscriber(ClutteredManipulation):

    def __init__(self, ):
        super(RefBoxSubscriber, self).__init__()
        self.sub = rospy.Subscriber('/metrics_refbox_client/command', CommandMsg, self.get_task_callback)

    def get_task_callback(self, CommandMsg):
        # self.task = CommandMsg.task
        # self.command = CommandMsg.command
        rospy.loginfo('[RefBoxSubsriber] Received RefBox Command Message: Task: %s Command: %s' % (task, command))

class ClutteredManipulationStateMachine(RefBoxSubscriber):

    def __init__(self, ):

        super(ClutteredManipulationStateMachine, self).__init__()

        # Create a SMACH state machine container
        sm = StateMachine(outcomes=['OVERALL_SUCCESS','OVERALL_FAILED'])
        
        with sm:
            rospy.loginfo('[ClutteredManipulationStateMachine] Executing MOveARM SM...')
            StateMachine.add(
                            "MOVE_ARM_TO_MIDDLE_POSE",
                            gms.move_arm("ppt_cavity_middle"),
                            transitions={
                                "succeeded": "Perceive",
                                "failed": "MOVE_ARM_TO_MIDDLE_POSE",
                            },
                        )
            StateMachine.add(label= 'Perceive',
                            state= gas.perceive_location(),
                            transitions={'success': 'PICK_OBJECTS',
                                        'failed': 'OVERALL_FAILED',})
            rospy.loginfo('[ClutteredManipulationStateMachine] Executing PICK OBJECT SM..')
            StateMachine.add(label= 'PICK_OBJECTS',
                            state= gas.pick_object(),
                            transitions={'success': 'WAIT_FOR_ARM_TO_STABILIZE',
                                        'failed': 'OVERALL_FAILED',})
            StateMachine.add("WAIT_FOR_ARM_TO_STABILIZE",
                             mir_gbs.wait_for(0.5),
                                transitions={
                                    "succeeded": "OVERALL_SUCCESS",
                                             },
                            )
            # rospy.loginfo('[ClutteredManipulationStateMachine] Executing PLACE OBJECT SM..')
            # StateMachine.add(label= 'PLACE_OBJECTS',
            #                 state= gas.place_object(),
            #                 transitions={'success': 'OVERALL_SUCCESS',
            #                             'failed': 'OVERALL_FAILED',})
        
        try:
            if self.task == 1 and self.command == 2:
                # View our state transitions using ROS by creating and starting the instrospection server
                sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
                sis.start()
                
                # Execute the state machine 
                outcome = sm.execute()
            
                rospy.spin()
            else: 
                pass 
        except Exception as e:
            print('Error: %s', (e))
        except KeyboardInterrupt:
            sis.stop()
            print('KeyboardInterrupt Error')
    

if __name__ == '__main__':
    cluttered_manipulation = ClutteredManipulationStateMachine()