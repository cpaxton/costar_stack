
# -------Waypoint Variable

class NodeVariableWaypoint(Node):
    def __init__(self,name,label):
        L = 'Variable\\n['+label+']'
        L_alt = '{VARIABLE | ' + label.lower()+'}'
        color = '#BC83DE'
        super(NodeVariableWaypoint,self).__init__(name,L,color,alt_label=L_alt,attach=False)
        self.waypoint=label
    def get_node_type(self):
        return 'VARIABLE'
    def get_node_name(self):
        return 'Variable'
    def execute(self):
        pass

class NodeVariableWaypointGUI(NodeGUI):
    def __init__(self):
        super(NodeVariableWaypointGUI,self).__init__()

        rospack = rospkg.RosPack()
        ui_path = rospack.get_path('instructor_plugins') + '/ui/variable_waypoint.ui'

        self.title.setText('WAYPOINT VARIABLE')
        self.title.setStyleSheet('background-color:#6CC90E;color:#ffffff')

        self.waypoint_ui = QWidget()
        uic.loadUi(ui_path, self.waypoint_ui)
        self.scrollArea = QScrollArea()
        self.scrollArea.setWidget(self.waypoint_ui)
        self.layout_.addWidget(self.scrollArea)

        self.new_waypoint_name = None
        self.waypoint_selected = False
        self.command_waypoint_name = None
        self.listener_ = tf.TransformListener()

        self.waypoint_ui.waypoint_name_field.textChanged.connect(self.waypoint_name_entered)
        self.waypoint_ui.waypoint_set_btn.clicked.connect(self.add_waypoint)

    def add_waypoint(self):
        if self.new_waypoint_name:
            try:
                F_waypoint = tf_c.fromTf(self.listener_.lookupTransform('/world','/endpoint',rospy.Time(0)))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr('Could not find the tf frame for the robot endpoint')
                return
            try:
                rospy.wait_for_service('/instructor_core/AddWaypoint',2)
            except rospy.ROSException as e:
                rospy.logerr(e)
                return
            try:
                add_waypoint_proxy = rospy.ServiceProxy('/instructor_core/AddWaypoint',AddWaypoint)
                msg = AddWaypointRequest()
                msg.name = '/' + self.new_waypoint_name
                msg.world_pose = tf_c.toMsg(F_waypoint)
                rospy.loginfo(add_waypoint_proxy(msg))
                # self.update_waypoints()
                # Set command waypoint if call to waypoint manager succeeds
                self.set_command_waypoint(self.new_waypoint_name)
            except rospy.ServiceException, e:
                rospy.logwarn(e)
        else:
            rospy.logerr('You need to input a name for the waypoint')

    def waypoint_name_entered(self,t):
        self.new_waypoint_name = str(t)

    def set_command_waypoint(self,waypoint_name):
        rospy.logwarn('Setting Waypoint')
        self.waypoint_ui.waypoint_label.setText(waypoint_name)
        self.waypoint_ui.waypoint_label.setStyleSheet('color:#ffffff;background-color:#AFEB1A')
        self.waypoint_selected = True
        self.command_waypoint_name = waypoint_name

    def save_data(self,data):
        data['waypoint_name'] = {'value':self.command_waypoint_name}
        return data

    def load_data(self,data):
        waypoint_name = data['waypoint_name']['value']
        if waypoint_name:
            self.set_command_waypoint(waypoint_name)

    def generate(self):
        if all([self.name.full(),self.label.full(), self.command_waypoint_name]):
            return NodeVariableWaypoint(self.get_name(),self.command_waypoint_name)
        else:
            rospy.logwarn('NODE NOT PROPERLY DEFINED')
            return 'ERROR: node not properly defined'

