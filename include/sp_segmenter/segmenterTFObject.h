#ifndef SP_TF_MAP
#define SP_TF_MAP

class segmentedObjectTF
{
private:
    tf::Transform transform;
public:
    std::string TFnames;
    segmentedObjectTF(const poseT &input,const unsigned int &tmpIndex);
    segmentedObjectTF(const poseT &input, const std::string &tfName);
    segmentedObjectTF();
    tf::StampedTransform generateStampedTransform(const std::string &parent) const;
};

segmentedObjectTF::segmentedObjectTF()
{
    this->transform.setIdentity();
    this->TFnames = "dummyTF";
}

segmentedObjectTF::segmentedObjectTF(const poseT &input, const std::string &tfName)
{
  this->transform.setOrigin(tf::Vector3( input.shift.x(), input.shift.y(), input.shift.z() ));
  this->transform.setRotation(tf::Quaternion(
                                             input.rotation.x(),input.rotation.y(),input.rotation.z(),input.rotation.w()));
  this->TFnames = tfName;
}

segmentedObjectTF::segmentedObjectTF(const poseT &input, const unsigned int &tmpIndex)
{
    this->transform.setOrigin(tf::Vector3( input.shift.x(), input.shift.y(), input.shift.z() ));
    this->transform.setRotation(tf::Quaternion(
                                               input.rotation.x(),input.rotation.y(),input.rotation.z(),input.rotation.w()));
    
    std::stringstream child;
    // Does not have tracking yet, can not keep the label on object.
    child << "Obj::" << input.model_name << "::" << tmpIndex;
    this->TFnames = child.str();
}

tf::StampedTransform segmentedObjectTF::generateStampedTransform(const std::string &parent) const
{
    return tf::StampedTransform(this->transform,ros::Time::now(),parent, this->TFnames);
}

#endif
