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
  PredicateStatement createStatement(std::string predicate, double value, std::string param1, std::string param2, std::string param3) {
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

    nh_tilde.param("rel_x_threshold", rel_x_threshold, 0.1);
    nh_tilde.param("rel_y_threshold", rel_y_threshold, 0.1);
    nh_tilde.param("rel_z_threshold", rel_z_threshold, 0.1);
    nh_tilde.param("near_2D_threshold", near_2d_threshold, 0.2);
    nh_tilde.param("near_3D_threshold", near_3d_threshold, 0.2);

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

    pval.predicates.push_back("near_xy");
    pval.predicates.push_back("near");
    pval.predicates.push_back("left_of");
    pval.predicates.push_back("right_of");
    pval.predicates.push_back("in_front_of");
    pval.predicates.push_back("behind");
    pval.predicates.push_back("above");
    pval.predicates.push_back("below");

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

      pval.assignments.push_back(model->getName());

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

    if (load_floating) {
      if (verbosity > 0) {
        ROS_INFO("about to parse floating");
      }
      // read in root TF frames
      for(unsigned int i = 0; i < floating.size(); ++i) {
        std::string id = floating[i]["id"];
        std::string frame = floating[i]["frame"];

        floating_frames[id] = frame;
      }
    }

    // print out information on all the different joints
    unsigned int i = 0;
    for (typename std::vector<PlanningScene *>::iterator it1 = scenes.begin();
        it1 != scenes.end();
        ++it1, ++i)
    {
      collision_detection::CollisionRobotConstPtr robot1 = (*it1)->getCollisionRobot();
      // -----------------------------------------------------------
      if (verbosity > 0) {
        std::cout << std::endl;
        std::cout << "PRINTING STATE INFO:";
        std::cout << robot1->getRobotModel()->getName() << std::endl;
        std::cout << robots[i]->getRootJointName() << std::endl;
      }
      states[i]->update(true);
      if (verbosity > 0) {
        states[i]->printStateInfo(std::cout);
      }
      // -----------------------------------------------------------
    }

    if (verbosity > 0) {
      ROS_INFO("creating list of heuristic indices for possible values");
    }
    updateIndices();
  }

  /**
   * checkAndUpdate
   * helper function
   */
  static inline void checkAndUpdate(const PredicateStatement &pred,
                                    heuristic_map_t &indices,
                                    unsigned int &next_idx)
  {
    if (indices.find(pred) == indices.end()) {
      indices[pred] = next_idx++;
      //std::cout << next_idx << " ";
    }
  }

  /**
   * updateIndices()
   * Records where the values we can use as heuristics are going to be stored.
   * May also look at things like waypoints, etc.
   */
  void PredicateContext::updateIndices() {
    unsigned int idx = 0;
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

        //std::cout << *link1 << std::endl;

        // access world coordinates
        // NOTE: does not work for the ring yet!
        Eigen::Affine3d tf1 = getLinkTransform(*it, *link1);

        // loop over the other objects in the world
        // this does NOT include waypoints or anything like that -- we need a separate loop
        // the second loop can handle abstract entities like these
        unsigned int j = 0;
        for(typename std::vector<RobotState *>::const_iterator it2 = states.begin();
            it2 != states.end();
            ++it2, ++j)
        {

          PredicateStatement near_mesh = createStatement("near_mesh",0,(*it)->getRobotModel()->getName(),(*it2)->getRobotModel()->getName());
          PredicateStatement touching_robot = createStatement("touching",0,(*it)->getRobotModel()->getName(),(*it2)->getRobotModel()->getName());

          //std::cout << heuristic_indices.size() << ", idx = " << idx << std::endl;

          checkAndUpdate(near_mesh, heuristic_indices, idx);
          checkAndUpdate(touching_robot, heuristic_indices, idx);

          //std::cout << (*it)->getRobotModel()->getName() << ", " << (*it2)->getRobotModel()->getName() << std::endl;

          // loop over the non-world links of this object
          // get the list of joints for the robot state
          for (typename std::vector<std::string>::const_iterator link2 = (*it2)->getRobotModel()->getLinkModelNames().begin();
               link2 != (*it2)->getRobotModel()->getLinkModelNames().end();
               ++link2)
          {
            if (link2->compare(std::string("world")) == 0) {
              continue;
            }

            // create the predicates
            PredicateStatement left = createStatement("left_of",0,*link1,*link2,"world");
            PredicateStatement right = createStatement("right_of",0,*link1,*link2,"world");
            PredicateStatement front = createStatement("in_front_of",0,*link1,*link2,"world");
            PredicateStatement back = createStatement("behind",0,*link1,*link2,"world");
            PredicateStatement up = createStatement("above",0,*link1,*link2,"world");
            PredicateStatement down = createStatement("below",0,*link1,*link2,"world");
            PredicateStatement touching = createStatement("touching",0,*link1,*link2);
            PredicateStatement near = createStatement("near",0,*link1,*link2);
            PredicateStatement near_xy = createStatement("near_xy",0,*link1,*link2);

            checkAndUpdate(left, heuristic_indices, idx);
            checkAndUpdate(right, heuristic_indices, idx);
            checkAndUpdate(front, heuristic_indices, idx);
            checkAndUpdate(back, heuristic_indices, idx);
            checkAndUpdate(up, heuristic_indices, idx);
            checkAndUpdate(down, heuristic_indices, idx);
            checkAndUpdate(touching, heuristic_indices, idx);
            checkAndUpdate(near, heuristic_indices, idx);
            checkAndUpdate(near_xy, heuristic_indices, idx);

            //std::cout << *link1 << ", " << *link2 << std::endl;
          }
        }
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
   * updateHeuristics
   * helper function to store heuristic values
   */
  static inline void updateHeuristics(const PredicateStatement &pred, const heuristic_map_t &indices, std::vector<double> &heuristics) {
    if (indices.find(pred) == indices.end()) {
      ROS_ERROR("(UPDATE) Failed to look up predicate \"%s\" with arguments (%s, %s, %s)", pred.predicate.c_str(),
                pred.params[0].c_str(),
                pred.params[1].c_str(),
                pred.params[2].c_str());
    } else if (indices.at(pred) > heuristics.size()) {
      ROS_ERROR("(UPDATE) Indexing error from predicate \"%s\" with arguments (%s, %s, %s)", pred.predicate.c_str(),
                pred.params[0].c_str(),
                pred.params[1].c_str(),
                pred.params[2].c_str());
      ROS_ERROR("index = %u, length=%lu", indices.at(pred), heuristics.size());
    }
    heuristics[indices.at(pred)] = pred.value;
  }

  /**
   * addCollisionPredicates()
   * main collision checking loop
   * checks for all pairs of objects, determines collisions and distances
   * publishes the relationships between all of these objects
   */
  void PredicateContext::addCollisionPredicates(PredicateList &output, std::vector<double> &heuristics, const std::vector<RobotState *> &states, unsigned int idx) {

    unsigned i = 0;
    for(typename std::vector<PlanningScene *>::iterator it1 = scenes.begin();
        it1 != scenes.end();
        ++it1, ++i)
    {

      collision_detection::CollisionRobotConstPtr robot1 = (*it1)->getCollisionRobot();

      typename std::vector<PlanningScene *>::iterator it2 = it1;
      unsigned int j = i+1;

      // skip if we are only computing predicates for a single planning scene
      if (idx < scenes.size() && i != idx) {
        continue;
      } else if (idx < scenes.size() && i == idx){
        j = 0;
        it2 = scenes.begin();
      }

      for(++it2; it2 != scenes.end(); ++it2, ++j) {

        if (i == j) continue;

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

        PredicateStatement ps = createStatement("touching", -1.0 * dist,
                                                robot1->getRobotModel()->getName(),
                                                robot2->getRobotModel()->getName());
        PredicateStatement ps2 = createStatement("touching", -1.0 * dist,
                                                 robot2->getRobotModel()->getName(),
                                                 robot1->getRobotModel()->getName());

        updateHeuristics(ps, heuristic_indices, heuristics);
        updateHeuristics(ps2, heuristic_indices, heuristics);

        if (dist <= 0) {
          output.statements.push_back(ps);
          output.statements.push_back(ps2);
        }

        if (verbosity > 4) {
          std::cout << res.contacts.size() << " contacts found" << std::endl;
        }

        // iterate over all collisions
        for(collision_detection::CollisionResult::ContactMap::const_iterator cit = res.contacts.begin(); 
            cit != res.contacts.end(); 
            ++cit)
        {
          // write the correct predicate
          predicator_msgs::PredicateStatement ps;
          ps.predicate = "touching";
          ps.value = -1.0 * dist;
          ps.num_params = 2;
          ps.params[0] = cit->first.first;
          ps.params[1] = cit->first.second;
          output.statements.push_back(ps);

          // the reverse is also true, so update it
          predicator_msgs::PredicateStatement ps2;
          ps2.predicate = "touching";
          ps2.value = -1.0 * dist;
          ps2.num_params = 2;
          ps2.params[0] = cit->first.second;
          ps2.params[1] = cit->first.first;
          output.statements.push_back(ps2);

          updateHeuristics(ps, heuristic_indices, heuristics);
          updateHeuristics(ps2, heuristic_indices, heuristics);
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
   * numHeuristics()
   */
  size_t PredicateContext::numHeuristics() const {
    return heuristic_indices.size();
  }

  /**
   * getHeuristic
   * Looks up a score from a vector of possible values
   */
  double PredicateContext::getHeuristic(const PredicateStatement &pred, const std::vector<double> &heuristics) const {
    if (heuristic_indices.find(pred) == heuristic_indices.end()) {
      ROS_ERROR("(GET) Failed to lookup predicate \"%s\" with arguments (%s, %s, %s)", pred.predicate.c_str(),
                pred.params[0].c_str(),
                pred.params[1].c_str(),
                pred.params[2].c_str());
      return 0;
    } else if (heuristic_indices.at(pred) > heuristics.size()) {
      ROS_ERROR("(GET) Indexing error from predicate \"%s\" with arguments (%s, %s, %s)", pred.predicate.c_str(),
                pred.params[0].c_str(),
                pred.params[1].c_str(),
                pred.params[2].c_str());
      ROS_ERROR("index = %u, length=%lu", heuristic_indices.at(pred), heuristics.size());
      return 0;
    }
    return heuristics.at(heuristic_indices.at(pred));
  }

  /**
   * tick()
   * Run one iteration of the predicator computations 
   */
  void PredicateContext::tick() {
    predicator_msgs::PredicateList output;
    output.pheader.source = ros::this_node::getName();

    std::vector<double> heuristics;
    heuristics.resize(heuristic_indices.size());

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
    // update list of reachable waypoints
    // use a service call to predicator to get the relevant waypoints

    // compute whether or not that point can be reached
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
  void PredicateContext::addGeometryPredicates(PredicateList &list, std::vector<double> &heuristics, const std::vector<RobotState *> &states) {

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

        // loop over the other objects in the world
        // this does NOT include waypoints or anything like that -- we need a separate loop
        // the second loop can handle abstract entities like these
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

            Eigen::Affine3d tf2 = getLinkTransform(*it2, *link2);

            if (verbosity > 2) {
              std::cout << *link1 << ", " << *link2 << std::endl;
            }

            if (verbosity > 3) {
              std::cout << tf1.translation()[0] << "," << tf1.translation()[1] << "," << tf1.translation()[2] << " --> ";
              std::cout << tf2.translation()[0] << "," << tf2.translation()[1] << "," << tf2.translation()[2] << std::endl;
            }

            double xdiff = tf1.translation()[1] - tf2.translation()[1]; // x = red = front/back from stage
            double ydiff = tf1.translation()[0] - tf2.translation()[0]; // y = green = left/right?
            double zdiff = tf1.translation()[2] - tf2.translation()[2]; // z = blue = up/down
            double dist_xy = sqrt((xdiff*xdiff) + (ydiff*ydiff)); // compute xy distance only
            double dist = sqrt((xdiff*xdiff) + (ydiff*ydiff) + (zdiff*zdiff)); // compute xyz distance

            PredicateStatement left = createStatement("left_of",xdiff - rel_x_threshold,*link1,*link2,"world");
            PredicateStatement right = createStatement("right_of",-1.0 * xdiff - rel_x_threshold,*link1,*link2,"world");
            PredicateStatement front = createStatement("in_front_of",ydiff - rel_y_threshold,*link1,*link2,"world");
            PredicateStatement back = createStatement("behind",-1.0 * ydiff - rel_y_threshold,*link1,*link2,"world");
            PredicateStatement up = createStatement("above",zdiff - rel_z_threshold,*link1,*link2,"world");
            PredicateStatement down = createStatement("below",-1.0 * zdiff - rel_z_threshold,*link1,*link2,"world");
            PredicateStatement near = createStatement("near",-1.0 * dist + near_3d_threshold,*link1,*link2);
            PredicateStatement near_xy = createStatement("near_xy",-1.0 * dist_xy + near_2d_threshold,*link1,*link2);

            updateHeuristics(left, heuristic_indices, heuristics);
            updateHeuristics(right, heuristic_indices, heuristics);
            updateHeuristics(front, heuristic_indices, heuristics);
            updateHeuristics(back, heuristic_indices, heuristics);
            updateHeuristics(up, heuristic_indices, heuristics);
            updateHeuristics(down, heuristic_indices, heuristics);
            updateHeuristics(near, heuristic_indices, heuristics);
            updateHeuristics(near_xy, heuristic_indices, heuristics);

            // x is left/right
            if (xdiff < -1.0 * rel_x_threshold){
              list.statements.push_back(right);
            } else if (xdiff > rel_x_threshold) {
              list.statements.push_back(left);
            }

            // y is front/back
            if (ydiff < -1.0 * rel_y_threshold) {
              list.statements.push_back(back);
            } else if (ydiff > rel_y_threshold) {
              list.statements.push_back(front);
            }

            // z is front/back
            if (zdiff < -1.0 * rel_z_threshold) {
              list.statements.push_back(down);
            } else if (zdiff > rel_z_threshold) {
              list.statements.push_back(up);
            }

            if (dist < near_3d_threshold) {
              list.statements.push_back(near);
            }

            if (dist_xy < near_2d_threshold) {
              list.statements.push_back(near_xy);
            }

            // somehow we need to do this from other points of view as well... but maybe not for now
          }
        }
      }
    }
  }
}
