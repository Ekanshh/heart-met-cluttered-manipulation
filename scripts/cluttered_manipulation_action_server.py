#!/usr/bin/env python

import threading
from time import sleep

from sympy import true, use

# import of generic states
import mir_states.common.action_states as gas
import mir_states.common.manipulation_states as gms
import mcr_states.common.basic_states as gbs


import rospy
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from metrics_refbox_msgs.msg import Command as CommandMsg
from metrics_refbox_msgs.msg import ClutteredPickResult as cpr

from mas_perception_msgs.msg import ObjectList
import smach

OBJECT_POSE = PoseStamped()
OBJECT_POSE.header.frame_id = "base_link"
OBJECT_POSE.pose.position.x = 0.495981603861
OBJECT_POSE.pose.position.y = 0.0566313937306
OBJECT_POSE.pose.position.z = 0.0907953720766
OBJECT_POSE.pose.orientation.x = 0.0
OBJECT_POSE.pose.orientation.y = 0.0
OBJECT_POSE.pose.orientation.z = -0.00901521310841
        

class LoopThroughPerceivedObjectList(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["pose_set", "tried_all"],
            input_keys=["object_list", "object_index"],
            output_keys=["object_index", "object_pose"],
        )

    def execute(self, userdata):
        rospy.loginfo("Loop through Object List")
        rospy.loginfo(userdata.object_list)
        if userdata.object_index >= len(userdata.object_list):
            return "tried_all"
        # set arm pose to next pose in list
        userdata.object_pose = userdata.object_list[userdata.object_index]
        userdata.object_index += 1 
        rospy.loginfo("Select Object Pose")
        return "pose_set"

class PublishObjectPose(smach.State):
    def __init__(self, ):
        smach.State.__init__(self, outcomes=["success", "failed"],
                                    input_keys=["object_pose", "crate_pose"],
                                    output_keys=["object_name"])

        self.obj_pose_pub = rospy.Publisher(
            "/mcr_perception/object_selector/output/object_pose",
            PoseStamped,
            queue_size=10
        )

    def execute(self, userdata):
        rospy.loginfo("Publish pose")
        if userdata.object_pose:
            pose = userdata.object_pose.pose
            userdata.object_name = userdata.object_pose.name
        elif userdata.crate_pose:
            pose = userdata.crate_pose
        rospy.loginfo(pose)
        self.obj_pose_pub.publish(pose)
        return "success"

class GetObjectList(smach.State):
    def __init__(self, ):
        smach.State.__init__(self, outcomes=["success", "failed"],
                                    output_keys=['object_list'])

        _obj_list_sub = rospy.Subscriber(
            "/mcr_perception/object_list_merger/output_object_list",
            ObjectList,
            self._obj_list_cb,
        )
        self._obj_list = []

    def _obj_list_cb(self, msg):
        
        object_detected = msg
        for i in range(0,10):
            if object_detected.objects[i]:
                self._obj_list.append(object_detected.objects[i])
        
    def execute(self, userdata):
        rospy.loginfo("Get perceived object list")
        userdata.object_list = self._obj_list
        
        return "success"


class GoToPreGraspPose(smach.State):
    def __init__(self, ):
        smach.State.__init__(self, outcomes=["success", "failed"])

        self.waypoint_pub = rospy.Publisher(
            "/waypoint_trajectory_generation/event_in",
            String,
            queue_size=10)
        self.pregrasp_pub = rospy.Publisher(
            "/pregrasp_planner_node/event_in",
            String,
            queue_size=10
        )

    def execute(self, userdata):
        rospy.loginfo("Publishing 'e_start' for pregrasp planner")
        self.pregrasp_pub.publish("e_start")
        rospy.loginfo("Publishing 'e_start' for waypoint trajectory generation")
        self.waypoint_pub.publish("e_start")
        return "success"

class PublishRelativeCratePose(smach.State):
    def __init__(self, ):
        smach.State.__init__(self, outcomes=["success", "failed"],
                                    output_keys=["crate_pose"])

        self.crate_pose_pub = rospy.Publisher(
            "/mcr_navigation/direct_base_controller/input_pose",
            PoseStamped,
            queue_size=10
        )

    def execute(self, userdata):
        rospy.loginfo("Publish relative crate pose to move base")
        crate_pose = PoseStamped()
        crate_pose.header.frame_id="base_link_static"
        crate_pose.header.stamp = rospy.Time.now()
        crate_pose.pose.position.y = -0.75     # Hard-coded: Put relative location of the crate here
        crate_pose.pose.orientation.w = 1
        userdata.crate_pose = crate_pose
        rospy.loginfo(crate_pose)
        self.crate_pose_pub.publish(crate_pose)
        return "success"

class PublishRelativeCratePose2(smach.State):
    def __init__(self, ):
        smach.State.__init__(self, outcomes=["success", "failed"],
                                    output_keys=["crate_pose"])

        self.crate_pose_pub = rospy.Publisher(
            "/mcr_navigation/direct_base_controller/input_pose",
            PoseStamped,
            queue_size=10
        )

    def execute(self, userdata):
        rospy.loginfo("Publish relative crate pose to move base")
        crate_pose = PoseStamped()
        crate_pose.header.frame_id="base_link_static"
        crate_pose.header.stamp = rospy.Time.now()
        crate_pose.pose.position.y = 0     # Hard-coded: Put relative location of the crate here
        crate_pose.pose.orientation.w = 1
        userdata.crate_pose = crate_pose
        rospy.loginfo(crate_pose)
        self.crate_pose_pub.publish(crate_pose)
        return "success"


class GetCrateCenterPose(smach.State):
    def __init__(self, ):
        smach.State.__init__(self, outcomes=["success", "failed"],
                                    output_keys=['object_pose', 'crate_pose'])

        _obj_list_sub = rospy.Subscriber(
            "/mcr_perception/crate_detector/crate_pose",
            PoseStamped,
            self._crate_pose,
        )

    def _crate_pose(self, msg):
        self.crate_pose = msg
        
    def execute(self, userdata):
        rospy.loginfo("Get crate center pose")
        cp= rospy.wait_for_message("/mcr_perception/crate_detector/crate_pose", PoseStamped)
        rospy.loginfo(cp)
        userdata.object_pose = None
        userdata.crate_pose = self.crate_pose
        return "success"

class PublishRefBoxFeedback():

    def __init__(self, bool_picked):
        smach.State.__init__(self, outcomes=["success", "failed"],
                                    input_keys=["object_name", "num_object_picked"],
                                    output_keys=["num_object_picked"])

        self.crate_pose_pub = rospy.Publisher(
            "/metrics_refbox_client/cluttered_pick_result",
            cpr,
            queue_size=10
        )
        self.obj_picked = bool_picked

    def execute(self, userdata):
        rospy.loginfo("Publish feedback to Refree Box Client")
        f_msg = cpr()
        f_msg.message_type = 1
        f_msg.object_name= userdata.object_name
        if self.obj_picked == True:
            f_msg.action_completed = 1
            userdata.num_object_picker += 1
        else:
            f_msg.action_completed = 2
        return "success"


def main():

    rospy.init_node("cluttered_manipulation_state_machine")
    
    rospy.Subscriber('/metrics_refbox_client/command', CommandMsg, ref_box_listener_cb)

    # Create a SMACH state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_FAILURE", "OVERALL_PREEMPT", "OVERALL_SUCCESS"]
    )

    sm.userdata.object_list = []
    sm.userdata.object_index = 0
    sm.userdata.num_object_picked = 0

    # Open the container
    with sm:

        smach.StateMachine.add(
            "START_PERCEIVE",
            gbs.send_event(
                [("/cluttered_picking/event_in", "e_start")]
            ),
            transitions={"success": "GET_PERCEIVED_OBJECT_LIST"}
        )

        smach.StateMachine.add(
            "GET_PERCEIVED_OBJECT_LIST",
            GetObjectList(),
            transitions={"success": "GET_OBJECT_POSE", "failed": "OVERALL_FAILURE"}

        )

        smach.StateMachine.add(
            "STOP_PERCEIVE",
            gbs.send_event(
                [("/cluttered_picking/event_in", "e_stop")]
            ),
            transitions={"success": "GET_OBJECT_POSE"}
        )


        smach.StateMachine.add(
            "GET_OBJECT_POSE",
            LoopThroughPerceivedObjectList(),
            transitions={"tried_all": "PERCEIVE", "pose_set": "PUBLISH_OBJECT_POSE"}

        )

        smach.StateMachine.add(
            "PUBLISH_OBJECT_POSE",
            PublishObjectPose(),
            transitions={"success": "CHECK_PRE_GRASP_POSE", "failed": "OVERALL_FAILURE"}

        )

        smach.StateMachine.add(
            "CHECK_PRE_GRASP_POSE",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    ("/pregrasp_planner_node/event_in", "e_start")
                ],
                event_out_list=[
                    (
                        "/pregrasp_planner_node/event_out",
                        "e_success",
                        True,
                    )
                ],
                timeout_duration=20,
            ),
            transitions={
                "success": "GO_TO_PRE_GRASP_POSE",
                "timeout": "OVERALL_FAILURE",
                "failure": "GET_OBJECT_POSE",
            },
        )

        smach.StateMachine.add(
            "GO_TO_PRE_GRASP_POSE",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    ("/waypoint_trajectory_generation/event_in", "e_start")
                ],
                event_out_list=[
                    (
                        "/waypoint_trajectory_generation/event_out",
                        "e_success",
                        True,
                    )
                ],
                timeout_duration=20,
            ),
            transitions={
                "success": "CLOSE_GRIPPER",
                "timeout": "OVERALL_FAILURE",
                "failure": "OVERALL_FAILURE",
            },
        )

        smach.StateMachine.add(
            "CLOSE_GRIPPER",
            gms.control_gripper("close"),
            transitions={"succeeded": "MOVE_ARM_STAGE_INTERMEDIATE"}
        )

        smach.StateMachine.add(
            "MOVE_ARM_STAGE_INTERMEDIATE",
            gms.move_arm("stage_intermediate"),
            transitions={"succeeded": "MOVE_ARM_PLATFORM_INTERMEDIATE_PRE", "failed": "OVERALL_FAILURE"}
        )

        smach.StateMachine.add(
            "MOVE_ARM_PLATFORM_INTERMEDIATE_PRE",
            gms.move_arm("platform_middle_pre"),
            transitions={"succeeded": "MOVE_ARM_PLATFORM_INTERMEDIATE", "failed": "OVERALL_FAILURE"}
        )

        smach.StateMachine.add(
            "MOVE_ARM_PLATFORM_INTERMEDIATE",
            gms.move_arm("platform_middle"),
            transitions={"succeeded": "OPEN_GRIPPER", "failed": "OVERALL_FAILURE"}
        )

        smach.StateMachine.add(
            "OPEN_GRIPPER",
            gms.control_gripper("open"),
            transitions={"succeeded": "MOVE_ARM_DEFAULT"}
        )

        smach.StateMachine.add(
            "MOVE_ARM_DEFAULT",
            gms.move_arm("look_at_workspace_from_near"),
            transitions={"succeeded": "STOP_WAYPOINT", "failed": "OVERALL_FAILURE"}
        )

        smach.StateMachine.add(
            "STOP_WAYPOINT",
            gbs.send_event(
                [("/waypoint_trajectory_generation/event_in", "e_stop")]
            ),
            transitions={"success": "STOP_PRE_GRASP_PLANNAR"},
        )

        smach.StateMachine.add(
            "STOP_PRE_GRASP_PLANNAR",
            gbs.send_event(
                [("/pregrasp_planner_node/event_in", "e_stop")]
            ),
            transitions={"success": "PUBLISH_REF_BOX_FEEDBACK"},
        )

        smach.StateMachine.add(
            "PUBLISH_REF_BOX_FEEDBACK",
            PublishRefBoxFeedback(bool_picked=True),
            transitions={"success": "PUBLISH_RELATIVE_CRATE_LOCATION", "failed": "OVERALL_FAILURE"}
        )


        smach.StateMachine.add(
            "PUBLISH_RELATIVE_CRATE_LOCATION",
            PublishRelativeCratePose(),
            transitions={"success": "MOVE_BASE_TO_CRATE", "failed": "OVERALL_FAILURE"}
            
        )

        smach.StateMachine.add(
            "MOVE_BASE_TO_CRATE",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    ("/mcr_navigation/direct_base_controller/coordinator/event_in", "e_start")
                ],
                event_out_list=[
                    (
                        "/mcr_navigation/direct_base_controller/coordinator/event_out",
                        "e_success",
                        True,
                    )
                ],
                timeout_duration=20,
            ),
            transitions={
                "success": "DETECT_CRATE",
                "timeout": "OVERALL_FAILURE",
                "failure": "OVERALL_FAILURE",
            },
        )

        smach.StateMachine.add(
            "DETECT_CRATE",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    ("/mir_perception/crate_segmentation/event_in", "e_start")
                ],
                event_out_list=[
                    (
                        "/mir_perception/crate_segmentation/event_out",
                        "e_started",
                        True,
                    )
                ],
                timeout_duration=20,
            ),
            transitions={
                "success": "GET_CRATE_CENTER_POSE",
                "timeout": "DETECT_CRATE",
                "failure": "OVERALL_FAILURE",
            },
        )

        smach.StateMachine.add(
            "GET_CRATE_CENTER_POSE",
            GetCrateCenterPose(),
            transitions={"success": "PUBLISH_CRATE_POSE", "failed": "OVERALL_FAILURE"}
        )

        smach.StateMachine.add(
            "PUBLISH_CRATE_POSE",
            PublishObjectPose(),
            transitions={"success": "STOP_CRATE_DETECTION", "failed": "OVERALL_FAILURE"}

        )

        smach.StateMachine.add(
            "STOP_CRATE_DETECTION",
            gbs.send_event(
                [("/mir_perception/crate_segmentation/event_in", "e_stop")]
            ),
            transitions={"success": "MOVE_ARM_STAGE_INTERMEDIATE_PICKUP"},
        )
    

        smach.StateMachine.add(
            "MOVE_ARM_STAGE_INTERMEDIATE_PICKUP",
            gms.move_arm("stage_intermediate"),
            transitions={"succeeded": "MOVE_ARM_PLATFORM_INTERMEDIATE_PRE_PICKUP", "failed": "OVERALL_FAILURE"}
        )

        smach.StateMachine.add(
            "MOVE_ARM_PLATFORM_INTERMEDIATE_PRE_PICKUP",
            gms.move_arm("platform_middle_pre"),
            transitions={"succeeded": "MOVE_ARM_PLATFORM_INTERMEDIATE_PICKUP", "failed": "OVERALL_FAILURE"}
        )

        smach.StateMachine.add(
            "MOVE_ARM_PLATFORM_INTERMEDIATE_PICKUP",
            gms.move_arm("platform_middle"),
            transitions={"succeeded": "CLOSE_GRIPPER_PICKUP", "failed": "OVERALL_FAILURE"}
        )

        smach.StateMachine.add(
            "CLOSE_GRIPPER_PICKUP",
            gms.control_gripper("close"),
            transitions={"succeeded": "MOVE_ARM_STAGE_INTERMEDIATE_PICKUP_2"}
        )

        smach.StateMachine.add(
            "MOVE_ARM_STAGE_INTERMEDIATE_PICKUP_2",
            gms.move_arm("stage_intermediate"),
            transitions={"succeeded": "CHECK_PRE_CRATE_POSE", "failed": "OVERALL_FAILURE"}
        )

        smach.StateMachine.add(
            "CHECK_PRE_CRATE_POSE",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    ("/pregrasp_planner_node/event_in", "e_start")
                ],
                event_out_list=[
                    (
                        "/pregrasp_planner_node/event_out",
                        "e_success",
                        True,
                    )
                ],
                timeout_duration=20,
            ),
            transitions={
                "success": "GO_TO_PRE_CRATE_POSE",
                "timeout": "OVERALL_FAILURE",
                "failure": "OVERALL_FAILURE",
            },
        )

        smach.StateMachine.add(
            "GO_TO_PRE_CRATE_POSE",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    ("/waypoint_trajectory_generation/event_in", "e_start")
                ],
                event_out_list=[
                    (
                        "/waypoint_trajectory_generation/event_out",
                        "e_success",
                        True,
                    )
                ],
                timeout_duration=20,
            ),
            transitions={
                "success": "OPEN_GRIPPER_PICKUP",
                "timeout": "OVERALL_FAILURE",
                "failure": "OVERALL_FAILURE",
            },
        )

        smach.StateMachine.add(
            "OPEN_GRIPPER_PICKUP",
            gms.control_gripper("open"),
            transitions={"succeeded": "MOVE_ARM_DEFAULT_PICKUP"}
        )

        smach.StateMachine.add(
            "MOVE_ARM_DEFAULT_PICKUP",
            gms.move_arm("folded"),
            transitions={"succeeded": "PUBLISH_REF_BOX_FEEDBACK2", "failed": "OVERALL_FAILURE"}
        )

        smach.StateMachine.add(
            "PUBLISH_REF_BOX_FEEDBACK2",
            PublishRefBoxFeedback(bool_picked=False),
            transitions={"success": "PUBLISH_RELATIVE_CRATE_LOCATION2", "failed": "OVERALL_FAILURE"}
        )

        smach.StateMachine.add(
            "PUBLISH_RELATIVE_CRATE_LOCATION2",
            PublishRelativeCratePose2(),
            transitions={"success": "MOVE_BASE_TO_CRATE2", "failed": "OVERALL_FAILURE"}
            
        )

        smach.StateMachine.add(
            "MOVE_BASE_TO_CRATE2",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    ("/mcr_navigation/direct_base_controller/coordinator/event_in", "e_start")
                ],
                event_out_list=[
                    (
                        "/mcr_navigation/direct_base_controller/coordinator/event_out",
                        "e_success",
                        True,
                    )
                ],
                timeout_duration=20,
            ),
            transitions={
                "success": "PERCEIVE_WS02",
                "timeout": "OVERALL_FAILURE",
                "failure": "OVERALL_FAILURE",
            },
        )


    # Create a thread to execute the smach container

    def ref_box_listener_cb(msg):
    
        while msg.task == 5:

            smach_thread = threading.Thread(target=sm.execute)
            smach_thread.start()

            while not rospy.is_shutdown() and smach_thread.isAlive():
                rospy.sleep(1.0)

            if rospy.is_shutdown() or msg.command == 2:
                rospy.logwarn("preempting smach execution")
                # Request the container to preempt
                sm.request_preempt()
            smach_thread.join()

if __name__ == "__main__":
    main()
