#ifndef _PP_UTILITY
#define _PP_UTILITY

#include <unordered_map>
#include <predicator_msgs/PredicateStatement.h>
#include <predicator_msgs/PredicateSet.h>
#include <predicator_msgs/PredicateList.h>
#include <predicator_msgs/PredicateAssignment.h>
#include <string>
#include <functional>

using std::string;
using std::unordered_map;
using namespace predicator_msgs;

namespace predicator_planning {

  struct Hash {

    std::hash<std::string> hash_str;

    Hash() : hash_str() {}

    size_t operator()(const PredicateStatement &msg) const {

      size_t res = hash_str(msg.predicate);

      for (unsigned int i = 0; i < msg.params.size(); ++i) {
        res += hash_str(msg.params[i]) << (3*i);
      }

      return res;
    }
  };

  struct Equals {
    bool operator()(const PredicateStatement &msg1,
                    const PredicateStatement &msg2) const
    {
      return msg1.predicate == msg2.predicate &&
        //msg1.num_params == msg2.num_params &&
        msg1.params[0] == msg2.params[0] &&
        msg1.params[1] == msg2.params[1] &&
        msg1.params[2] == msg2.params[2];
    }
  };
}

#endif
