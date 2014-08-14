// ROS
#include <ros/ros.h>
#include <XmlRpcValue.h>

// for debugging
#include <iostream>

// predicator stuff
#include <predicator.h>



/*
 * joint_state_callback()
 * Update the robot state variable values
 */
void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg, RobotState *state) {
  state->setVariableValues(*msg);
}

/**
 * cleanup()
 * Delete memory allocated for robot states and subscribers
 */
void cleanup() {
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

int main(int argc, char **argv) {

  double padding; // how much padding do we give robot links?
  int verbosity; // how much should get printed for debugging purposes
  std::map<std::string, std::string> floating_frames;
  std::string world_frame;

  ros::init(argc, argv, "predicator_robot_planning_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_tilde("~");

  XmlRpc::XmlRpcValue descriptions;
  XmlRpc::XmlRpcValue topics;
  XmlRpc::XmlRpcValue floating; // set of floating root joints that need to be updated

  nh_tilde.param("verbosity", verbosity, 0);
  nh_tilde.param("padding", padding, 0.01);
  nh_tilde.param("world_frame", world_frame, std::string("/world"));

  ros::Publisher pub = nh.advertise<predicator_msgs::PredicateList>("/predicator/input", 1000);
  ros::Publisher vpub = nh.advertise<predicator_msgs::ValidPredicates>("/predicator/valid_input", 1000);

  // start main loop
  while(ros::ok()) {
    ros::spinOnce();

    predicator_msgs::PredicateList output;
    output.pheader.source = ros::this_node::getName();

    unsigned int i = 0;

    // make sure base frames are up to date
    // some objects, such as free-floating robots (aka the ring) need to be updated by TF
    // not sure why this doesn't work naturally
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

    // main collision checking loop
    // checks for all pairs of objects, determines collisions and distances
    // publishes the relationships between all of these objects
    i = 0;
    for(typename std::vector<PlanningScene *>::iterator it1 = scenes.begin();
        it1 != scenes.end();
        ++it1, ++i)
    {

      collision_detection::CollisionRobotConstPtr robot1 = (*it1)->getCollisionRobot();


      typename std::vector<PlanningScene *>::iterator it2 = it1;
      unsigned int j = i+1;
      for(++it2; it2 != scenes.end(); ++it2, ++j) {
      //unsigned int j = 0;
      //for(typename std::vector<PlanningScene *>::iterator it2 = scenes.begin();
      //    it2 != scenes.end();
      //    ++it2, ++j)
      //{

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
            << ": Distance to collision: " << dist << std::endl;
        }
      }
    }

    pub.publish(output);
    vpub.publish(pval);

    rate.sleep();
  }

  cleanup();
}
