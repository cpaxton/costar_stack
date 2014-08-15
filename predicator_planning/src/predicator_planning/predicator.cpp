#include "predicator.h"

/**
  predicator_planning
  Can generate the set of predicates that would be true for a given location.
  Also used to generate predicates based on current state.
 **/
namespace predicator_planning {

  /*
   * createStatement()
   * Simple helper function to create predicates with
   */
  PredicateStatement createStatement(std::string predicate, double value, std::string param1, std::string param2, std::string param3 = "") {
    PredicateStatement ps;
    ps.predicate = predicate;
    ps.params[0] = param1;
    ps.params[1] = param2;
    ps.value = value;

    int num_params = 2;

    if (param3.size() > 0) {
      ++num_params;
      ps.params[2] = param3;
    }
    ps.num_params = num_params;

    return ps;
  }

  /*
   * joint_state_callback()
   * Update the robot state variable values
   */
  void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg, RobotState *state) {
    state->setVariableValues(*msg);
  }

  PredicateContext::PredicateContext(bool publish) {
    ros::NodeHandle nh_tilde("~");
    ros::NodeHandle nh;

    XmlRpc::XmlRpcValue frames_list;
    XmlRpc::XmlRpcValue descriptions;
    XmlRpc::XmlRpcValue topics;
    XmlRpc::XmlRpcValue floating; // set of floating root joints that need to be updated


    nh_tilde.param("verbosity", verbosity, 0);
    nh_tilde.param("padding", padding, 0.01);
    nh_tilde.param("world_frame", world_frame, std::string("/world"));

    // should we publish predicate messages?
    // or what?
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

    if(nh_tilde.hasParam("frames")) {
      nh_tilde.param("frames", frames_list, frames_list);
    } else {
      ROS_ERROR("No list of frames for geometryic predicates!");
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
    pval.predicates.push_back("touching");
    pval.value_predicates.push_back("mesh_distance");
    pval.pheader.source = ros::this_node::getName();

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

    // read in frames of interest
    for(unsigned int i = 0; i < frames_list.size(); ++i) {
      std::string frame;

      if(frames_list[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
        frame = static_cast<std::string>(frames_list[i]);
        if(verbosity > 0) {
          std::cout << "Including frame: " << frame << std::endl;
        }
        frames.push_back(frame);
      } else {
        ROS_WARN("Frame list entry %u was not of type \"string\"!", i);
        continue;
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


  /**
   * updatRobotStates()
   * make sure base frames are up to date
   * some objects, such as free-floating robots (aka the ring) need to be updated by TF
   * not sure why this doesn't work naturally
   */
  void PredicateContext::updateRobotStates() {
    unsigned int i = 0;


    for(typename std::vector<PlanningScene *>::iterator it1 = scenes.begin();
        it1 != scenes.end();
        ++it1, ++i)
    {

      collision_detection::CollisionRobotConstPtr robot1 = (*it1)->getCollisionRobot();
      std::string name = robot1->getRobotModel()->getName();

      if(floating_frames.find(name) != floating_frames.end()) {
        std::string base_frame = floating_frames[name];

        tf::StampedTransform transform;
        Eigen::Affine3d t;

        try{
          listener.lookupTransform(world_frame, base_frame,
                                   ros::Time(0), transform);
          tf::transformTFToEigen(transform,t);
          states[i]->setJointPositions(robot1->getRobotModel()->getRootJointName(), t);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
        }

        if(verbosity > 1) {
          std::cout << "----------------------------" << std::endl;
          std::cout << "PRINTING STATE INFO:";
          std::cout << robot1->getRobotModel()->getName() << std::endl;
          std::cout << robots[i]->getRootJointName() << std::endl;
          states[i]->update(true);
          states[i]->printStateInfo(std::cout);
        }

      } else {
        continue;
      }
    }
  }

  /**
   * addCollisionPredicates()
   * main collision checking loop
   * checks for all pairs of objects, determines collisions and distances
   * publishes the relationships between all of these objects
   */
  void PredicateContext::addCollisionPredicates(PredicateList &output, std::vector<double> &heuristics, const std::vector<RobotState *> &states) {

    unsigned i = 0;
    for(typename std::vector<PlanningScene *>::iterator it1 = scenes.begin();
        it1 != scenes.end();
        ++it1, ++i)
    {

      collision_detection::CollisionRobotConstPtr robot1 = (*it1)->getCollisionRobot();


      typename std::vector<PlanningScene *>::iterator it2 = it1;
      unsigned int j = i+1;
      for(++it2; it2 != scenes.end(); ++it2, ++j) {

        //if (i == j) continue;

        collision_detection::CollisionRobotConstPtr robot2 = (*it2)->getCollisionRobot();

        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        req.contacts = true;
        req.max_contacts = 1000;

        // force an update
        // source: https://groups.google.com/forum/#!topic/moveit-users/O9CEef6sxbE
        states[i]->update(true);
        states[j]->update(true);
        robot1->checkOtherCollision(req, res, *states[i], *robot2, *states[j]);
        double dist = robot1->distanceOther(*states[i], *robot2, *states[j]);


        // write distance predicate
        predicator_msgs::PredicateStatement ps_dist;
        ps_dist.predicate = "mesh_distance";
        ps_dist.value = dist;
        ps_dist.num_params = 2;
        ps_dist.params[0] = robot1->getRobotModel()->getName();
        ps_dist.params[1] = robot2->getRobotModel()->getName();
        output.statements.push_back(ps_dist);

        // the reverse is also true, so write it as well
        predicator_msgs::PredicateStatement ps_dist2;
        ps_dist2.predicate = "mesh_distance";
        ps_dist2.value = dist;
        ps_dist2.num_params = 2;
        ps_dist2.params[0] = robot1->getRobotModel()->getName();
        ps_dist2.params[1] = robot2->getRobotModel()->getName();
        output.statements.push_back(ps_dist2);

        // iterate over all collisions
        for(collision_detection::CollisionResult::ContactMap::const_iterator cit = res.contacts.begin(); 
            cit != res.contacts.end(); 
            ++cit)
        {
          // write the correct predicate
          predicator_msgs::PredicateStatement ps;
          ps.predicate = "touching";
          ps.value = 1.0;
          ps.num_params = 2;
          ps.params[0] = cit->first.first;
          ps.params[1] = cit->first.second;
          output.statements.push_back(ps);

          // the reverse is also true, so update it
          predicator_msgs::PredicateStatement ps2;
          ps2.predicate = "touching";
          ps2.value = 1.0;
          ps2.num_params = 2;
          ps2.params[0] = cit->first.second;
          ps2.params[1] = cit->first.first;
          output.statements.push_back(ps2);
        }

        if (verbosity > 1) {
          std::cout << "(" << robot1->getRobotModel()->getName()
            << ", " << robot2->getRobotModel()->getName()
            << ") : Distance to collision: " << dist << std::endl;
        }
      }
    }

  }

  /**
   * tick()
   * Run one iteration of the predicator computations 
   */
  void PredicateContext::tick() {
    predicator_msgs::PredicateList output;
    output.pheader.source = ros::this_node::getName();

    std::vector<double> heuristics;

    updateRobotStates();
    addCollisionPredicates(output, heuristics, states);
    addGeometryPredicates(output, heuristics, states);
    addReachabilityPredicates(output, heuristics, states);

    pub.publish(output);
    vpub.publish(pval);
  }

  /**
   * addReachabilityPredicates()
   * compute whether or not we can reach certain points or waypoints
   */
  void PredicateContext::addReachabilityPredicates(PredicateList &list, std::vector<double> &heuristics, const std::vector<RobotState *> &states) {

  }

  /**
   * getLinkTransform
   * Check to see if this is in the list of floating transfoms
   * If so, compose with TF frame
   * NOTE: actually, it looks like we don't need this at all
   */
  Eigen::Affine3d PredicateContext::getLinkTransform(const RobotState *state, const std::string &linkName) const {

      std::string name = state->getRobotModel()->getName();
      Eigen::Affine3d tf1 = state->getGlobalLinkTransform(linkName);

      //std::cout << tf1.translation()[0] << "," << tf1.translation()[1] << "," << tf1.translation()[2] << std::endl;
      /*if(floating_frames.find(name) != floating_frames.end()) {
        std::string base_frame = floating_frames.at(name);

        tf::StampedTransform transform;
        Eigen::Affine3d t;

        try{
          listener.lookupTransform(world_frame, base_frame,
                                   ros::Time(0), transform);
          tf::transformTFToEigen(transform,t);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
        }
        return t * tf1;
      } else {
        return tf1;
      }*/

      return tf1;
  }

  /**
   * addGeometryPredicates()
   * compute the set of geometry predicates
   *
   * Links for the different world objects from the peg demo:
   world wam/base_link wam/shoulder_yaw_link wam/shoulder_pitch_link wam/upper_arm_link wam/forearm_link wam/wrist_yaw_link wam/wrist_pitch_link wam/wrist_palm_link wam/hand/bhand_palm_link wam/hand/bhand_grasp_link wam/hand/bhand_palm_surface_link wam/hand/finger_1/prox_link wam/hand/finger_1/med_link wam/hand/finger_1/dist_link wam/hand/finger_2/prox_link wam/hand/finger_2/med_link wam/hand/finger_2/dist_link wam/hand/finger_3/med_link wam/hand/finger_3/dist_link wam/wrist_palm_stump_link 
   world wam2/base_link wam2/shoulder_yaw_link wam2/shoulder_pitch_link wam2/upper_arm_link wam2/forearm_link wam2/wrist_yaw_link wam2/wrist_pitch_link wam2/wrist_palm_link wam2/hand/bhand_palm_link wam2/hand/bhand_grasp_link wam2/hand/bhand_palm_surface_link wam2/hand/finger_1/prox_link wam2/hand/finger_1/med_link wam2/hand/finger_1/dist_link wam2/hand/finger_2/prox_link wam2/hand/finger_2/med_link wam2/hand/finger_2/dist_link wam2/hand/finger_3/med_link wam2/hand/finger_3/dist_link wam2/wrist_palm_stump_link 
   world peg1/base_link peg1/peg_link peg1/peg_top_link 
   world peg2/base_link peg2/peg_link peg2/peg_top_link 
   ring1/ring_link 
   world stage_link 
   */
  void PredicateContext::addGeometryPredicates(PredicateList &list, std::vector<double> &heuristic, const std::vector<RobotState *> &states) {

    unsigned int i = 0;
    for(typename std::vector<RobotState *>::const_iterator it = states.begin();
        it != states.end();
        ++it, ++i)
    {

      // get the list of joints for the robot state
      for (typename std::vector<std::string>::const_iterator link1 = (*it)->getRobotModel()->getLinkModelNames().begin();
           link1 != (*it)->getRobotModel()->getLinkModelNames().end();
           ++link1)
      {
        if (link1->compare(std::string("world")) == 0) {
          continue;
        }

        // access world coordinates
        // NOTE: does not work for the ring yet!
        Eigen::Affine3d tf1 = getLinkTransform(*it, *link1);
        std::cout << tf1.translation()[0] << "," << tf1.translation()[1] << "," << tf1.translation()[2] << std::endl;

        // loop over the other objects in the world

        unsigned int j = 0;
        for(typename std::vector<RobotState *>::const_iterator it2 = states.begin();
            it2 != states.end();
            ++it2, ++j)
        {
          if (i == j) {
            continue;
          }

          // loop over the non-world links of this object
          // get the list of joints for the robot state
          for (typename std::vector<std::string>::const_iterator link2 = (*it2)->getRobotModel()->getLinkModelNames().begin();
               link2 != (*it2)->getRobotModel()->getLinkModelNames().end();
               ++link2)
          {
            if (link2->compare(std::string("world")) == 0) {
              continue;
            }

            if (verbosity > 0) {
              std::cout << *link1 << ", " << *link2 << std::endl;
            }
          }
        }
      }
    }
  }
}
