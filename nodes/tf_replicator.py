#!/usr/bin/env python

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


sub = None
timestamp = None
starttime = None

transforms = []

for cam in ["camera_yellow", "camera_purple"]:

    for stream in ["rgb", "ir", "depth"]:
        t = TransformStamped()
        t.header.frame_id = "/" + cam + "_link"
        t.child_frame_id = "/" + cam + "_" + stream + "_frame"
        t.transform.translation.x = 0.0 if stream == 'rgb' else 0.00382254
        t.transform.translation.y = 0.0 if stream == 'rgb' else -0.0247
        t.transform.translation.z = 0.0 if stream == 'rgb' else 0.000733
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        transforms.append(t)

        t = TransformStamped()
        t.header.frame_id = "/" + cam + "_" + stream + "_frame"
        t.child_frame_id = "/" + cam + "_" + stream + "_optical_frame"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = -0.5
        t.transform.rotation.y = 0.5
        t.transform.rotation.z = -0.5
        t.transform.rotation.w = 0.5
        transforms.append(t)

def callback(msg):
    global timestamp, starttime

    sub.unregister()
    timestamp = msg.transforms[0].header.stamp
    starttime = rospy.get_rostime()
    rospy.loginfo("Starting to publish cameras transforms. Initial timestamp: %s" % str(timestamp))

if __name__ == "__main__":

    rospy.init_node("tfreplicator")
    sub = rospy.Subscriber("/tf", TFMessage, callback)
    tfpub = rospy.Publisher("/tf", TFMessage, queue_size=1)

    r = rospy.Rate(10)

    while not rospy.is_shutdown():

        if timestamp is not None:

            now = rospy.get_rostime()

            stamp = timestamp + ((now - starttime) / 1)

            tfmsg = TFMessage()

            for t in transforms:
                t.header.stamp  = stamp
                tfmsg.transforms.append(t)

            tfpub.publish(tfmsg)

        r.sleep()

    rospy.loginfo("Bye")
