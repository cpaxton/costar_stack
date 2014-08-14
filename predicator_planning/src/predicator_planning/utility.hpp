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

namespace std {

  struct Hash {

    std::hash<std::string> hash_str;

    Hash() : hash_str() {}

    size_t operator()(const PredicateStatement &msg) {

      size_t res = hash_str(msg.predicate);

      for (unsigned int i = 0; i < msg.num_params; ++i) {
        res += hash_str(msg.params[i]) << (3*i);
      }

      return res;
    }
  };

  struct Equals {
    bool operator()(const PredicateStatement &msg1,
                    const PredicateStatement &msg2)
    {
      return msg1.predicate == msg2.predicate &&
        msg1.num_params == msg2.num_params &&
        msg1.params[0] == msg2.params[0] &&
        msg1.params[1] == msg2.params[1] &&
        msg1.params[2] == msg2.params[2] &&
        msg1.params[3] == msg2.params[3];
    }
  };
}

