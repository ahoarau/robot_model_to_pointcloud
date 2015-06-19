// Author : Antoine Hoarau <hoarau.robotics@gmail.com>
#include <pluginlib/class_loader.h>
#include <stdio.h>
// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
// MoveIt!

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
// Eigen
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigen>


#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

int main(int argc, char** argv)
{
    ros::Time::init();
    ros::init(argc, argv, "robot_model_to_pointcloud");
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh("~");
    
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("robot_cloud2", 1);
    
    ROS_INFO("Loading robot from the parameter server");

    std::string robot_description;
    if(! nh.getParam("/robot_description",robot_description))
    {
        ROS_ERROR("/robot_description is not broadcasted");
        return 0;
    }

    std::string joint_states_topic("/joint_states");
    if(! nh.getParam("/joint_states",joint_states_topic))
    {
        ROS_WARN("joint_states_topic will be set to default : %s",joint_states_topic.c_str());
    }

    planning_scene_monitor::PlanningSceneMonitor psm("/robot_description");
    psm.startStateMonitor(joint_states_topic);
    
    const std::vector<const moveit::core::LinkModel*> link_models = psm.getStateMonitor()->getCurrentState()->getRobotModel()->getLinkModels();
    std::vector<std::string> mesh_filenames(link_models.size());
    
    
    std::map<std::string,shapes::Mesh* > meshes;
    
    
    for (unsigned i=0;i<mesh_filenames.size() ; ++i)
    {
      mesh_filenames[i] = link_models[i]->getVisualMeshFilename();
      ROS_INFO_STREAM("- "<<mesh_filenames[i]);
      if(!mesh_filenames[i].empty())
	meshes[link_models[i]->getName()] =  shapes::createMeshFromResource(mesh_filenames[i]);
    }
    
    
    sensor_msgs::PointCloud cloud;
    sensor_msgs::PointCloud2 cloud2;
    geometry_msgs::Point32 pt;
    bool initialized = false;
    
    ROS_INFO("Starting");
    while(ros::ok()){
        unsigned j = 0;
        ros::Time start = ros::Time::now();
	//if(psm.getStateMonitor()->waitForCurrentState(1.0)){
        cloud.header.frame_id=psm.getStateMonitor()->getRobotModel()->getRootLinkName();
        ROS_DEBUG_STREAM("Frame id : "<<cloud.header.frame_id);
	
        for (std::map<std::string,shapes::Mesh* >::iterator it=meshes.begin(); it!=meshes.end() ; ++it)
        {
            std::string name = it->first;
            ROS_DEBUG_STREAM(""<<name);
	    
            // Get the link transform (base to link_i )
            const Eigen::Affine3d& Transform = psm.getStateMonitor()->getCurrentState()->getGlobalLinkTransform(name);
	    ROS_DEBUG_STREAM("Transform : \n"<<Transform.matrix());
	    
	    for(std::size_t i=0;i<3*it->second->vertex_count;i=i+3)
	    {
		const Eigen::Vector3d vertice(it->second->vertices[i],it->second->vertices[i+1],it->second->vertices[i+2]);
		// Get the point location with respect to the base
		const Eigen::Vector3d vertice_transformed = Transform*vertice;

		pt.x = vertice_transformed[0];
		pt.y = vertice_transformed[1];
		pt.z = vertice_transformed[2];
		if(!initialized)
		  cloud.points.push_back(pt);
		else
		  cloud.points[j] = pt;
		j++;
	    }
        }
        cloud.header.stamp = ros::Time::now();
        // PointCloud to PointCloud2
        sensor_msgs::convertPointCloudToPointCloud2(cloud,cloud2);
        cloud_pub.publish(cloud2);
	
	if(!initialized)
	  initialized = true;
	  
        ROS_INFO_STREAM("Computation time : "<<(ros::Time::now() - start).toSec());
        //ros::Duration(1./50.).sleep();
        //}
	
    }
    return 0;
}
