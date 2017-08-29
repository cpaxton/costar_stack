from costar_component import CostarComponent
from os.path import join
import rospy
from std_srvs.srv import Empty

# The CoSTAR Gripper component contains a lot less special logic than the
# CoSTAR Arm does. It mostly just has a few tools.
class CostarGripper(CostarComponent):
  def __init__(self,
      name, #name of the gripper
      input_topic, # topic on which we receive messages from the gripper
      output_topic, # topic on which we send messages to the gripper
      InputMsgType, # type of input message
      OutputMsgType, # type of output message
      GripperPredicatorType, # construct a predicator node to send status info
      ns, # operating namespace
      verbose, # verbose or not
      *args, **kwargs):

    self.verbose = verbose
    self.predicator = GripperPredicatorType(
        start_subscriber=False,
        publish_predicates=True,
        gripper_name=name)

    if GripperPredicatorType is not None:
      self.predicator = GripperPredicatorType(
          start_subscriber=False,
          publish_predicates=True,
          gripper_name=name)
    else:
      self.predicator = None

    if InputMsgType is not None:
      self.sub = rospy.Subscriber(input_topic, InputMsgType, self.status_cb)
    if OutputMsgType is not None:
      self.pub = rospy.Publisher(output_topic, OutputMsgType, queue_size = 100)
    self.open = rospy.Service(join(ns,"open"), Empty, self.open_gripper)
    self.close = rospy.Service(join(ns,"close"), Empty, self.close_gripper)
    self.wide_mode_srv = rospy.Service(join(ns,"wide_mode"), Empty, self.wide_mode)
    self.pinch_mode_srv = rospy.Service(join(ns,"pinch_mode"), Empty, self.pinch_mode)
    self.basic_mode_srv = rospy.Service(join(ns,"basic_mode"), Empty, self.basic_mode)
    self.scissor_mode_srv = rospy.Service(join(ns,"scissor_mode"), Empty, self.scissor_mode)
    self.reactivate_srv = rospy.Service(join(ns,"activate"), Empty, self.activate)
    self.reset_srv = rospy.Service(join(ns,"reset"), Empty, self.reset)
    self.command = self.getDefaultMsg()

    self.activated = True;

    self.activate()

  def getDefaultMsg(self):
    raise NotImplementedError('not implemented in CostarGripper base class!')

  def activate(self,msg=None):
    raise NotImplementedError('not implemented in CostarGripper base class!')

  def reset(self, msg=None):
    raise NotImplementedError('not implemented in CostarGripper base class!')

  def open_gripper(self,msg=None):
    raise NotImplementedError('not implemented in CostarGripper base class!')

  def close_gripper(self,msg=None):
    raise NotImplementedError('not implemented in CostarGripper base class!')

  def wide_mode(self,msg=None):
    raise NotImplementedError('not implemented in CostarGripper base class!')

  def pinch_mode(self,msg=None):
    raise NotImplementedError('not implemented in CostarGripper base class!')

  def basic_mode(self,msg=None):
    raise NotImplementedError('not implemented in CostarGripper base class!')

  def scissor_mode(self,msg=None):
    raise NotImplementedError('not implemented in CostarGripper base class!')

  def statusInterpreter(self,status):
    raise NotImplementedError('not implemented in CostarGripper base class!')

  def status_cb(self,msg):
      if self.verbose:
        rospy.loginfo(self.statusInterpreter(msg))
      if self.predicator is not None:
        self.predicator.handle(msg)


