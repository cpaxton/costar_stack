
import os
import rospy

class CostarComponent(object):

  def make_service(self, name, srv_t, callback, *args, **kwargs):
    service_name = os.path.join(self.namespace, name)
    return rospy.Service(service_name, srv_t, callback, *args, **kwargs)

  '''
  Publishers are globally visible -- they go into the top-level CoSTAR namespace.
  '''
  def make_pub(self, name, msg_t, *args, **kwargs):
    pub_name = os.path.join(self.namespace, name)
    return rospy.Publisher(pub_name, msg_t, *args, **kwargs)

  def make_service_proxy(self, name, srv_t, use_namespace=True):
    if use_namespace:
      service_name = os.path.join(self.namespace, name)
    else:
      service_name = name
      rospy.loginfo("Connecting to service with name: %s"%service_name)
      rospy.wait_for_service(service_name)
      rospy.loginfo("Connected to service successfully.")
      return rospy.ServiceProxy(service_name,srv_t)

  def __init__(self, name, namespace):
    self.name = name
    self.namespace = namespace
