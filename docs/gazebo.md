# Gazebo

In general, you should be able to install Gazebo from binary for your ROS distro. These are some rough notes for you if that does not work.

## 14.04

Dependencies:

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup keys and update:

```
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
```

(From OSRF) Now install prerequisites. A clean Ubuntu system will need the following:

```
wget https://bitbucket.org/osrf/release-tools/raw/default/jenkins-scripts/lib/dependencies_archive.sh -O /tmp/dependencies.sh
ROS_DISTRO=indigo GAZEBO_VERSION=7 . /tmp/dependencies.sh
sudo apt-get install $(sed 's:\\ ::g' <<< $BASE_DEPENDENCIES) $(sed 's:\\ ::g' <<< $GAZEBO_BASE_DEPENDENCIES)
```

We need 7 because that is the last one that supports 14.04.

Install these other dependencies:

```
# Only needed on Trusty. Ubuntu packages since Utopic.
sudo apt-add-repository ppa:libccd-debs
sudo apt-add-repository ppa:fcl-debs

# Main repository
sudo apt-add-repository ppa:dartsim
sudo apt-get update
sudo apt-get install libdart-core5-dev libccd-dev 
```

Alright, that should at least let you compile Gazebo.

## 16.04 

Not yet supported. Hope the binaries work.

## References

  - [Installing Gazebo from source](http://gazebosim.org/tutorials?tut=install_from_source)
