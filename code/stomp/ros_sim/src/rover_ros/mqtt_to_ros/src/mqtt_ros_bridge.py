#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



#!/usr/bin/env python2.7
import rospy

from communication_manager import CommunicationManager

from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

class MqttToRosBridge():

  # Node initialization
  def __init__(self):
    # Create the communication manager
    self.comm_link = CommunicationManager()
    # Create the publishers 
    self.reset_sub = rospy.Subscriber("/reset_simulation", String, self._reset_callback, queue_size=1)

    # Used to save the reset request
    self.reset_requested = False
    self.reset_tag = ""

    # Create a dictionary of publishers
    self.target_publishers = {}

    # Call the mainloop of our class
    self.mainloop()

  def _reset_callback(self, msg):
    self.reset_tag = msg.data
    self.reset_requested = True

  def create_target_publisher(self, target_name):

    # If this is already in the dictionary, do not create
    if target_name in self.target_publishers:
      return

    # Otherwise create all the publishers we need
    self.target_publishers[target_name] = {}
    self.target_publishers[target_name]["type"]                = rospy.Publisher("/{}/{}".format(target_name, "type"), String, queue_size=10)
    self.target_publishers[target_name]["discovered"]          = rospy.Publisher("/{}/{}".format(target_name, "discovered"), Bool, queue_size=10)
    self.target_publishers[target_name]["position"]            = rospy.Publisher("/{}/{}".format(target_name, "position"), NavSatFix, queue_size=10)
    self.target_publishers[target_name]["captured"]            = rospy.Publisher("/{}/{}".format(target_name, "captured"), Bool, queue_size=10)
    self.target_publishers[target_name]["required_payloads"]   = rospy.Publisher("/{}/{}".format(target_name, "required_payloads"), Int16, queue_size=10)
    self.target_publishers[target_name]["ready_for_capture"]   = rospy.Publisher("/{}/{}".format(target_name, "ready_for_capture"), Bool, queue_size=10)
    self.target_publishers[target_name]["networks"]            = rospy.Publisher("/{}/{}".format(target_name, "networks"), String, queue_size=10)
    self.target_publishers[target_name]["networks_captured"]   = rospy.Publisher("/{}/{}".format(target_name, "networks_captured"), Bool, queue_size=10)

  def publish_target_data(self, target_name, target_info):

    # Publish the target type
    msg = String()
    msg.data = target_info["type"]
    self.target_publishers[target_name]["type"].publish(msg)

    # Publish if it has been discovered
    msg = Bool()
    msg.data = target_info["discovered"]
    self.target_publishers[target_name]["discovered"].publish(msg)

    # Publish its positions
    msg = NavSatFix()
    msg.latitude = target_info["latitude"]
    msg.longitude = target_info["longitude"]
    msg.altitude = target_info["altitude"]
    self.target_publishers[target_name]["position"].publish(msg)

    # Publish if it has been captured
    msg = Bool()
    msg.data = target_info["captured"]
    self.target_publishers[target_name]["captured"].publish(msg)
    
    # Publish the required number of payloads for capture
    msg = Int16()
    msg.data = target_info["required_payloads"]
    self.target_publishers[target_name]["required_payloads"].publish(msg)

    # Publish if it is ready for capture
    msg = Bool()
    msg.data = target_info["ready_for_capture"]
    self.target_publishers[target_name]["ready_for_capture"].publish(msg)

    # Publish if it is networks
    msg = String()
    msg.data = target_info["networks"]
    self.target_publishers[target_name]["networks"].publish(msg)

    # Publish if it is the networks captured
    msg = Bool()
    msg.data = target_info["networks_captured"]
    self.target_publishers[target_name]["networks_captured"].publish(msg)


  # The main loop of the function
  def mainloop(self):
    # Set the rate of this loop
    rate = rospy.Rate(0.5)

    # While ROS is still running
    while not rospy.is_shutdown():

      # Get the latest data on targets and payloads
      target_info = self.comm_link.get_target_information()
      payload_info = self.comm_link.get_payload_information()

      # For each target
      for target in target_info:

        # Create any publishers which have yet to be created
        if target not in self.target_publishers:
          self.create_target_publisher(target)

        # Publish the latest data
        self.publish_target_data(target, target_info[target])

      # Check if the reset has been requested
      if self.reset_requested:
        self.comm_link.reset_simulation(self.reset_tag)

        self.reset_requested = False
        self.reset_tag = ""

      # Sleep for the remainder of the loop
      rate.sleep()

if __name__ == '__main__':
  rospy.init_node('MQTT_to_ROS_bridge_node')
  try:
    bridge = MqttToRosBridge()
  except rospy.ROSInterruptException:
    pass
