#include <predicator.h>

/**
  predicator_planning
  Can generate the set of predicates that would be true for a given location.
  Also used to generate predicates based on current state.
 **/
namespace predicator_planning {

  void PredicateContext::initialize(bool publish) {
    ros::NodeHandle nh_tilde("~");

    nh_tilde.param("verbosity", verbosity, 0);
    nh_tilde.param("padding", padding, 0.01);
    nh_tilde.param("world_frame", world_frame, std::string("/world"));

    if(publish == true) {
      pub = nh.advertise<predicator_msgs::PredicateList>("/predicator/input", 1000);
      vpub = nh.advertise<predicator_msgs::ValidPredicates>("/predicator/valid_input", 1000);
    }

    if(nh_tilde.hasParam("description_list")) {
      nh_tilde.param("description_list", descriptions, descriptions);
    } else {
      ROS_ERROR("No list of robot description parameters!");
      exit(-1);
    }

    if(nh_tilde.hasParam("joint_state_topic_list")) {
      nh_tilde.param("joint_state_topic_list", topics, topics);
    } else {
      ROS_ERROR("No list of joint state topics!");
      exit(-1);
    }

    bool load_floating = false;
    if(nh_tilde.hasParam("floating_root_list")) {
      nh_tilde.param("floating_root_list", floating, floating);
      load_floating = true;
    } else {
      ROS_INFO("No list of robots with floating root joints given.");
    }

    if(descriptions.size() != topics.size()) {
      ROS_WARN("An unequal number of joint state and robot topics was provided!");
    }

    // define valid predicates topic
    predicator_msgs::ValidPredicates pval;
    pval.pheader.source = ros::this_node::getName();
    pval.predicates.push_back("touching");
    pval.value_predicates.push_back("mesh_distance");

    // read in topics and descriptions
    for(unsigned int i = 0; i < descriptions.size(); ++i) {
      std::string desc;
      std::string topic;

      if(descriptions[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
        desc = static_cast<std::string>(descriptions[i]);
        if(verbosity > 0) {
          std::cout << "Robot Description parameter name: " << desc << std::endl;
        }
      } else {
        ROS_WARN("Description %u was not of type \"string\"!", i);
        continue;
      }

      // create a robot model with state desc
      robot_model_loader::RobotModelLoader robot_model_loader(desc);
      robot_model::RobotModelPtr model = robot_model_loader.getModel();
      PlanningScene *scene = new PlanningScene(model);
      scene->getCollisionRobotNonConst()->setPadding(padding);
      scene->propogateRobotPadding();

      // get all link names as possible assignments
      for(typename std::vector<std::string>::const_iterator it = model->getLinkModelNames().begin();
          it != model->getLinkModelNames().end();
          ++it)
      {
        pval.assignments.push_back(*it);
      }

      robots.push_back(model);
      scenes.push_back(scene);

      RobotState *state = new RobotState(model);
      states.push_back(state);

      if(i < topics.size() && topics[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
        topic = static_cast<std::string>(topics[i]);
        if(verbosity > 0) {
          std::cout << "JointState topic name: " << topic << std::endl;
        }

        // create the subscriber
        subs.push_back(nh.subscribe<sensor_msgs::JointState>
                       (topic, 1000,
                        boost::bind(joint_state_callback, _1, state)));
      } else if (verbosity > 0) {
        ROS_WARN("no topic corresponding to description %s!", desc.c_str());
      }
    }

    ROS_INFO("about to parse floating");
    if(load_floating) {
      // read in root TF frames
      for(unsigned int i = 0; i < floating.size(); ++i) {
        std::string id = floating[i]["id"];
        std::string frame = floating[i]["frame"];

        floating_frames[id] = frame;
      }
    }
    ROS_INFO("parsed floating");

    // define spin rate
    ros::Rate rate(30);

    // print out information on all the different joints
    if(verbosity > 0) {
      unsigned int i = 0;
      for(typename std::vector<PlanningScene *>::iterator it1 = scenes.begin();
          it1 != scenes.end();
          ++it1, ++i)
      {
        collision_detection::CollisionRobotConstPtr robot1 = (*it1)->getCollisionRobot();
        // -----------------------------------------------------------
        std::cout << std::endl;
        std::cout << "PRINTING STATE INFO:";
        std::cout << robot1->getRobotModel()->getName() << std::endl;
        std::cout << robots[i]->getRootJointName() << std::endl;
        states[i]->update(true);
        states[i]->printStateInfo(std::cout);
        // -----------------------------------------------------------
      }
    }


  }

  /**
   * cleanup()
   * Delete memory allocated for robot states and subscribers
   */
  void PredicateContext::cleanup() {
    for (typename std::vector<RobotState *>::iterator it = states.begin();
         it != states.end();
         ++it)
    {
      delete *it;
    }

    for (typename std::vector<PlanningScene *>::iterator it = scenes.begin();
         it != scenes.end();
         ++it)
    {
      delete *it;
    }
  }

}
