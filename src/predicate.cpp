#include "predicate.h"

namespace predicator {
  
      /**
       */
      void GeometryParser::planningSceneCallback(const moveit_msgs::PlanningScene::ConstPtr &msg) {

      }

      /**
       */
      GeometryParser::GeometryParser(const std::string &planning_topic,
                                     const std::string &predicate_topic)
        : nh_()
      {
        // advertize a topic with predicate information
        pub_ = nh_.advertise<predicator_msgs::PredicateList>(predicate_topic, 1000);

        // call something to initialize the planning scene
        
        // subscribe to planning messages
        sub_ = nh_.subscribe(planning_topic, 1000, &GeometryParser::planningSceneCallback, this);
      }


}
