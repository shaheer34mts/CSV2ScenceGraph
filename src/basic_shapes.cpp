#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>


visualization_msgs::MarkerArray create_marker_array(const ros::Time& stamp, std::vector<double> walls_x_coord, std::vector<double> wall_y_coord, std::vector<double> room_x_coord, std::vector<double> room_y_coord)  {
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker mesh_marker;
    mesh_marker.header.frame_id = "map";
    mesh_marker.header.stamp = stamp;
    mesh_marker.ns = "mesh";
    mesh_marker.id = markers.markers.size();
    mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh_marker.mesh_resource = "package://using_markers/meshes/1st_build_EX_114551_2nd_floor.dae";

    mesh_marker.pose.position.x = 0;
    mesh_marker.pose.position.y = 0;
    mesh_marker.pose.position.z = 0;
    mesh_marker.pose.orientation.x = 0.0;
    mesh_marker.pose.orientation.y = 0.0;
    mesh_marker.pose.orientation.z = 0.0;
    mesh_marker.pose.orientation.w = 1.0;
    mesh_marker.scale.x = 1.0;
    mesh_marker.scale.y = 1.0;
    mesh_marker.scale.z = 1.0;

    mesh_marker.color.r = 0.6f;
    mesh_marker.color.g = 0.6f;
    mesh_marker.color.b = 0.7f;
    mesh_marker.color.a = 1.0;

    markers.markers.push_back(mesh_marker);

    // room 1
    visualization_msgs::Marker Room_1;
    Room_1.header.frame_id = "map";
    Room_1.header.stamp = stamp;
    Room_1.ns = "room_1";
    Room_1.id = markers.markers.size();
    Room_1.type = visualization_msgs::Marker::CUBE;

    Room_1.pose.position.x = room_x_coord[0];
    Room_1.pose.position.y = room_y_coord[0];
    Room_1.pose.position.z = 7.0;
    Room_1.pose.orientation.x = 0.0;
    Room_1.pose.orientation.y = 0.0;
    Room_1.pose.orientation.z = 0.0;
    Room_1.pose.orientation.w = 1.0;
    Room_1.scale.x = 0.4;
    Room_1.scale.y = 0.4;
    Room_1.scale.z = 0.4;

    Room_1.color.r = 1.0f;
    Room_1.color.g = 0.0f;
    Room_1.color.b = 0.0f;
    Room_1.color.a = 1.0;
    markers.markers.push_back(Room_1);

    // room 1 name marker
    visualization_msgs::Marker room_1_name_marker;
    room_1_name_marker.header.frame_id = "map";
    room_1_name_marker.header.stamp = stamp;
    room_1_name_marker.ns = "room_1_name";
    room_1_name_marker.id = markers.markers.size()+1;
    room_1_name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    room_1_name_marker.pose.position.x = room_x_coord[0];
    room_1_name_marker.pose.position.y = room_y_coord[0];
    room_1_name_marker.pose.position.z = 7.4;
    room_1_name_marker.pose.orientation.x = 0.0;
    room_1_name_marker.pose.orientation.y = 0.0;
    room_1_name_marker.pose.orientation.z = 0.0;
    room_1_name_marker.pose.orientation.w = 1.0;
    room_1_name_marker.text = "Room 284";
    room_1_name_marker.scale.x = 0.4;
    room_1_name_marker.scale.y = 0.4;
    room_1_name_marker.scale.z = 0.4;

    room_1_name_marker.color.r = 1.0f;
    room_1_name_marker.color.g = 1.0f;
    room_1_name_marker.color.b = 1.0f;
    room_1_name_marker.color.a = 1.0;
    markers.markers.push_back(room_1_name_marker);


// // room 2
    visualization_msgs::Marker Room_2;
    Room_2.header.frame_id = "map";
    Room_2.header.stamp = stamp;
    Room_2.ns = "room_2";
    Room_2.id = markers.markers.size();
    Room_2.type = visualization_msgs::Marker::CUBE;

    Room_2.pose.position.x = room_x_coord[1];
    Room_2.pose.position.y = room_y_coord[1];
    Room_2.pose.position.z = 7.0;
    Room_2.pose.orientation.x = 0.0;
    Room_2.pose.orientation.y = 0.0;
    Room_2.pose.orientation.z = 0.0;
    Room_2.pose.orientation.w = 1.0;
    Room_2.scale.x = 0.4;//0.1;
    Room_2.scale.y = 0.4;//0.7;
    Room_2.scale.z = 0.4;//0.2;

    Room_2.color.r = 1.0f;
    Room_2.color.g = 0.0f;
    Room_2.color.b = 0.0f;
    Room_2.color.a = 1.0;
    markers.markers.push_back(Room_2);

    // room 2 name marker
    visualization_msgs::Marker room_2_name_marker;
    room_2_name_marker.header.frame_id = "map";
    room_2_name_marker.header.stamp = stamp;
    room_2_name_marker.ns = "room_2_name";
    room_2_name_marker.id = markers.markers.size()+1;
    room_2_name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    room_2_name_marker.pose.position.x = room_x_coord[1];
    room_2_name_marker.pose.position.y = room_y_coord[1];
    room_2_name_marker.pose.position.z = 7.4;
    room_2_name_marker.pose.orientation.x = 0.0;
    room_2_name_marker.pose.orientation.y = 0.0;
    room_2_name_marker.pose.orientation.z = 0.0;
    room_2_name_marker.pose.orientation.w = 1.0;
    room_2_name_marker.text = "Room 299";
    room_2_name_marker.scale.x = 0.4;
    room_2_name_marker.scale.y = 0.4;
    room_2_name_marker.scale.z = 0.4;

    room_2_name_marker.color.r = 1.0f;
    room_2_name_marker.color.g = 1.0f;
    room_2_name_marker.color.b = 1.0f;
    room_2_name_marker.color.a = 1.0;
    markers.markers.push_back(room_2_name_marker);



//       // room 3
    visualization_msgs::Marker Room_3;
    Room_3.header.frame_id = "map";
    Room_3.header.stamp = stamp;
    Room_3.ns = "room_3";
    Room_3.id = markers.markers.size();
    Room_3.type = visualization_msgs::Marker::CUBE;

    Room_3.pose.position.x = room_x_coord[2];
    Room_3.pose.position.y = room_y_coord[2];
    Room_3.pose.position.z = 7.0;
    Room_3.pose.orientation.x = 0.0;
    Room_3.pose.orientation.y = 0.0;
    Room_3.pose.orientation.z = 0.0;
    Room_3.pose.orientation.w = 1.0;
    Room_3.scale.x = 0.4;
    Room_3.scale.y = 0.4;
    Room_3.scale.z = 0.4;

    Room_3.color.r = 1.0f;
    Room_3.color.g = 0.0f;
    Room_3.color.b = 0.0f;
    Room_3.color.a = 1.0;
    markers.markers.push_back(Room_3);

    // room 3 name marker
    visualization_msgs::Marker room_3_name_marker;
    room_3_name_marker.header.frame_id = "map";
    room_3_name_marker.header.stamp = stamp;
    room_3_name_marker.ns = "room_3_name";
    room_3_name_marker.id = markers.markers.size()+1;
    room_3_name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    room_3_name_marker.pose.position.x = room_x_coord[2];
    room_3_name_marker.pose.position.y = room_y_coord[2];
    room_3_name_marker.pose.position.z = 7.4;
    room_3_name_marker.pose.orientation.x = 0.0;
    room_3_name_marker.pose.orientation.y = 0.0;
    room_3_name_marker.pose.orientation.z = 0.0;
    room_3_name_marker.pose.orientation.w = 1.0;
    room_3_name_marker.text = "Room 302";
    room_3_name_marker.scale.x = 0.4;
    room_3_name_marker.scale.y = 0.4;
    room_3_name_marker.scale.z = 0.4;

    room_3_name_marker.color.r = 1.0f;
    room_3_name_marker.color.g = 1.0f;
    room_3_name_marker.color.b = 1.0f;
    room_3_name_marker.color.a = 1.0;
    markers.markers.push_back(room_3_name_marker);


//     // room 4
    visualization_msgs::Marker Room_4;
    Room_4.header.frame_id = "map";
    Room_4.header.stamp = stamp;
    Room_4.ns = "room_4";
    Room_4.id = markers.markers.size();
    Room_4.type = visualization_msgs::Marker::CUBE;

    Room_4.pose.position.x = room_x_coord[3];
    Room_4.pose.position.y = room_y_coord[3];
    Room_4.pose.position.z = 7.0;
    Room_4.pose.orientation.x = 0.0;
    Room_4.pose.orientation.y = 0.0;
    Room_4.pose.orientation.z = 0.0;
    Room_4.pose.orientation.w = 1.0;
    Room_4.scale.x = 0.4;
    Room_4.scale.y = 0.4;
    Room_4.scale.z = 0.4;

    Room_4.color.r = 1.0f;
    Room_4.color.g = 0.0f;
    Room_4.color.b = 0.0f;
    Room_4.color.a = 1.0;
    markers.markers.push_back(Room_4);

    // room 1 name marker
    visualization_msgs::Marker room_4_name_marker;
    room_4_name_marker.header.frame_id = "map";
    room_4_name_marker.header.stamp = stamp;
    room_4_name_marker.ns = "room_4_name";
    room_4_name_marker.id = markers.markers.size()+1;
    room_4_name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    room_4_name_marker.pose.position.x = room_x_coord[3];
    room_4_name_marker.pose.position.y = room_y_coord[3];
    room_4_name_marker.pose.position.z = 7.4;
    room_4_name_marker.pose.orientation.x = 0.0;
    room_4_name_marker.pose.orientation.y = 0.0;
    room_4_name_marker.pose.orientation.z = 0.0;
    room_4_name_marker.pose.orientation.w = 1.0;
    room_4_name_marker.text = "Room 322";
    room_4_name_marker.scale.x = 0.4;
    room_4_name_marker.scale.y = 0.4;
    room_4_name_marker.scale.z = 0.4;

    room_4_name_marker.color.r = 1.0f;
    room_4_name_marker.color.g = 1.0f;
    room_4_name_marker.color.b = 1.0f;
    room_4_name_marker.color.a = 1.0;
    markers.markers.push_back(room_4_name_marker);


//     // room 5
    visualization_msgs::Marker Room_5;
    Room_5.header.frame_id = "map";
    Room_5.header.stamp = stamp;
    Room_5.ns = "room_5";
    Room_5.id = markers.markers.size();
    Room_5.type = visualization_msgs::Marker::CUBE;

    Room_5.pose.position.x = room_x_coord[4];
    Room_5.pose.position.y = room_y_coord[4];
    Room_5.pose.position.z = 7.0;
    Room_5.pose.orientation.x = 0.0;
    Room_5.pose.orientation.y = 0.0;
    Room_5.pose.orientation.z = 0.0;
    Room_5.pose.orientation.w = 1.0;
    Room_5.scale.x = 0.4;
    Room_5.scale.y = 0.4;
    Room_5.scale.z = 0.4;

    Room_5.color.r = 1.0f;
    Room_5.color.g = 0.0f;
    Room_5.color.b = 0.0f;
    Room_5.color.a = 1.0;
    markers.markers.push_back(Room_5);

    // room 5 name marker
    visualization_msgs::Marker room_5_name_marker;
    room_5_name_marker.header.frame_id = "map";
    room_5_name_marker.header.stamp = stamp;
    room_5_name_marker.ns = "room_5_name";
    room_5_name_marker.id = markers.markers.size()+1;
    room_5_name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    room_5_name_marker.pose.position.x = room_x_coord[4];
    room_5_name_marker.pose.position.y = room_y_coord[4];
    room_5_name_marker.pose.position.z = 7.4;
    room_5_name_marker.pose.orientation.x = 0.0;
    room_5_name_marker.pose.orientation.y = 0.0;
    room_5_name_marker.pose.orientation.z = 0.0;
    room_5_name_marker.pose.orientation.w = 1.0;
    room_5_name_marker.text = "Room 324";
    room_5_name_marker.scale.x = 0.4;
    room_5_name_marker.scale.y = 0.4;
    room_5_name_marker.scale.z = 0.4;

    room_5_name_marker.color.r = 1.0f;
    room_5_name_marker.color.g = 1.0f;
    room_5_name_marker.color.b = 1.0f;
    room_5_name_marker.color.a = 1.0;
    markers.markers.push_back(room_5_name_marker);


//         // room 6
    visualization_msgs::Marker Room_6;
    Room_6.header.frame_id = "map";
    Room_6.header.stamp = stamp;
    Room_6.ns = "room_6";
    Room_6.id = markers.markers.size();
    Room_6.type = visualization_msgs::Marker::CUBE;

    Room_6.pose.position.x = room_x_coord[5];
    Room_6.pose.position.y = room_y_coord[5];
    Room_6.pose.position.z = 7.0;
    Room_6.pose.orientation.x = 0.0;
    Room_6.pose.orientation.y = 0.0;
    Room_6.pose.orientation.z = 0.0;
    Room_6.pose.orientation.w = 1.0;
    Room_6.scale.x = 0.4;
    Room_6.scale.y = 0.4;
    Room_6.scale.z = 0.4;

    Room_6.color.r = 1.0f;
    Room_6.color.g = 0.0f;
    Room_6.color.b = 0.0f;
    Room_6.color.a = 1.0;
    markers.markers.push_back(Room_6);

    // room 6 name marker
    visualization_msgs::Marker room_6_name_marker;
    room_6_name_marker.header.frame_id = "map";
    room_6_name_marker.header.stamp = stamp;
    room_6_name_marker.ns = "room_6_name";
    room_6_name_marker.id = markers.markers.size()+1;
    room_6_name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    room_6_name_marker.pose.position.x = room_x_coord[5];
    room_6_name_marker.pose.position.y = room_y_coord[5];
    room_6_name_marker.pose.position.z = 7.4;
    room_6_name_marker.pose.orientation.x = 0.0;
    room_6_name_marker.pose.orientation.y = 0.0;
    room_6_name_marker.pose.orientation.z = 0.0;
    room_6_name_marker.pose.orientation.w = 1.0;
    room_6_name_marker.text = "Room 325";
    room_6_name_marker.scale.x = 0.4;
    room_6_name_marker.scale.y = 0.4;
    room_6_name_marker.scale.z = 0.4;
    room_6_name_marker.color.r = 1.0f;
    room_6_name_marker.color.g = 1.0f;
    room_6_name_marker.color.b = 1.0f;
    room_6_name_marker.color.a = 1.0;
    markers.markers.push_back(room_6_name_marker);

    
//             // room 7
    visualization_msgs::Marker Room_7;
    Room_7.header.frame_id = "map";
    Room_7.header.stamp = stamp;
    Room_7.ns = "room_7";
    Room_7.id = markers.markers.size();
    Room_7.type = visualization_msgs::Marker::CUBE;

    Room_7.pose.position.x = room_x_coord[6];
    Room_7.pose.position.y = room_y_coord[6];
    Room_7.pose.position.z = 7.0;
    Room_7.pose.orientation.x = 0.0;
    Room_7.pose.orientation.y = 0.0;
    Room_7.pose.orientation.z = 0.0;
    Room_7.pose.orientation.w = 1.0;
    Room_7.scale.x = 0.4;
    Room_7.scale.y = 0.4;
    Room_7.scale.z = 0.4;

    Room_7.color.r = 1.0f;
    Room_7.color.g = 0.0f;
    Room_7.color.b = 0.0f;
    Room_7.color.a = 1.0;
    markers.markers.push_back(Room_7);

    // room 7 name marker
    visualization_msgs::Marker room_7_name_marker;
    room_7_name_marker.header.frame_id = "map";
    room_7_name_marker.header.stamp = stamp;
    room_7_name_marker.ns = "room_7_name";
    room_7_name_marker.id = markers.markers.size()+1;
    room_7_name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    room_7_name_marker.pose.position.x = room_x_coord[6];
    room_7_name_marker.pose.position.y = room_y_coord[6];
    room_7_name_marker.pose.position.z = 7.4;
    room_7_name_marker.pose.orientation.x = 0.0;
    room_7_name_marker.pose.orientation.y = 0.0;
    room_7_name_marker.pose.orientation.z = 0.0;
    room_7_name_marker.pose.orientation.w = 1.0;
    room_7_name_marker.text = "Room 326";
    room_7_name_marker.scale.x = 0.4;
    room_7_name_marker.scale.y = 0.4;
    room_7_name_marker.scale.z = 0.4;
    room_7_name_marker.color.r = 1.0f;
    room_7_name_marker.color.g = 1.0f;
    room_7_name_marker.color.b = 1.0f;
    room_7_name_marker.color.a = 1.0;
    markers.markers.push_back(room_7_name_marker);

    //             // room 8
    visualization_msgs::Marker Room_8;
    Room_8.header.frame_id = "map";
    Room_8.header.stamp = stamp;
    Room_8.ns = "room_8";
    Room_8.id = markers.markers.size();
    Room_8.type = visualization_msgs::Marker::CUBE;

    Room_8.pose.position.x = room_x_coord[7];
    Room_8.pose.position.y = room_y_coord[7];
    Room_8.pose.position.z = 7.0;
    Room_8.pose.orientation.x = 0.0;
    Room_8.pose.orientation.y = 0.0;
    Room_8.pose.orientation.z = 0.0;
    Room_8.pose.orientation.w = 1.0;
    Room_8.scale.x = 0.4;
    Room_8.scale.y = 0.4;
    Room_8.scale.z = 0.4;

    Room_8.color.r = 1.0f;
    Room_8.color.g = 0.0f;
    Room_8.color.b = 0.0f;
    Room_8.color.a = 1.0;
    markers.markers.push_back(Room_8);

    // room 8 name marker
    visualization_msgs::Marker room_8_name_marker;
    room_8_name_marker.header.frame_id = "map";
    room_8_name_marker.header.stamp = stamp;
    room_8_name_marker.ns = "room_8_name";
    room_8_name_marker.id = markers.markers.size()+1;
    room_8_name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    room_8_name_marker.pose.position.x = room_x_coord[7];
    room_8_name_marker.pose.position.y = room_y_coord[7];
    room_8_name_marker.pose.position.z = 7.4;
    room_8_name_marker.pose.orientation.x = 0.0;
    room_8_name_marker.pose.orientation.y = 0.0;
    room_8_name_marker.pose.orientation.z = 0.0;
    room_8_name_marker.pose.orientation.w = 1.0;
    room_8_name_marker.text = "Room 327";
    room_8_name_marker.scale.x = 0.4;
    room_8_name_marker.scale.y = 0.4;
    room_8_name_marker.scale.z = 0.4;
    room_8_name_marker.color.r = 1.0f;
    room_8_name_marker.color.g = 1.0f;
    room_8_name_marker.color.b = 1.0f;
    room_8_name_marker.color.a = 1.0;
    markers.markers.push_back(room_8_name_marker);







    visualization_msgs::Marker wall_1;
    wall_1.header.frame_id = "map";
    wall_1.header.stamp = stamp;
    wall_1.ns = "wall_1";
    wall_1.id = markers.markers.size();
    wall_1.type = visualization_msgs::Marker::SPHERE;

    wall_1.pose.position.x = walls_x_coord[3];
    wall_1.pose.position.y = wall_y_coord[3];
    wall_1.pose.position.z = 5.0;
    wall_1.pose.orientation.x = 0.0;
    wall_1.pose.orientation.y = 0.0;
    wall_1.pose.orientation.z = 0.0;
    wall_1.pose.orientation.w = 1.0;
    wall_1.scale.x = 0.4;
    wall_1.scale.y = 0.4;
    wall_1.scale.z = 0.4;

    wall_1.color.r = 0.0f;
    wall_1.color.g = 1.0f;
    wall_1.color.b = 0.0f;
    wall_1.color.a = 1.0;
    markers.markers.push_back(wall_1);

    visualization_msgs::Marker wall_2;
    wall_2.header.frame_id = "map";
    wall_2.header.stamp = stamp;
    wall_2.ns = "wall_2";
    wall_2.id = markers.markers.size();
    wall_2.type = visualization_msgs::Marker::SPHERE;

    wall_2.pose.position.x = walls_x_coord[4];
    wall_2.pose.position.y = wall_y_coord[4];
    wall_2.pose.position.z = 5.0;
    wall_2.pose.orientation.x = 0.0;
    wall_2.pose.orientation.y = 0.0;
    wall_2.pose.orientation.z = 0.0;
    wall_2.pose.orientation.w = 1.0;
    wall_2.scale.x = 0.4;
    wall_2.scale.y = 0.4;
    wall_2.scale.z = 0.4;

    wall_2.color.r = 0.0f;
    wall_2.color.g = 1.0f;
    wall_2.color.b = 0.0f;
    wall_2.color.a = 1.0;
    markers.markers.push_back(wall_2);

    visualization_msgs::Marker wall_3;
    wall_3.header.frame_id = "map";
    wall_3.header.stamp = stamp;
    wall_3.ns = "wall_3";
    wall_3.id = markers.markers.size();
    wall_3.type = visualization_msgs::Marker::SPHERE;

    wall_3.pose.position.x = walls_x_coord[5];
    wall_3.pose.position.y = wall_y_coord[5];
    wall_3.pose.position.z = 5.0;
    wall_3.pose.orientation.x = 0.0;
    wall_3.pose.orientation.y = 0.0;
    wall_3.pose.orientation.z = 0.0;
    wall_3.pose.orientation.w = 1.0;
    wall_3.scale.x = 0.4;
    wall_3.scale.y = 0.4;
    wall_3.scale.z = 0.4;

    wall_3.color.r = 0.0f;
    wall_3.color.g = 1.0f;
    wall_3.color.b = 0.0f;
    wall_3.color.a = 1.0;
    markers.markers.push_back(wall_3);

    visualization_msgs::Marker wall_4;
    wall_4.header.frame_id = "map";
    wall_4.header.stamp = stamp;
    wall_4.ns = "wall_4";
    wall_4.id = markers.markers.size();
    wall_4.type = visualization_msgs::Marker::SPHERE;

    wall_4.pose.position.x = walls_x_coord[6];
    wall_4.pose.position.y = wall_y_coord[6];
    wall_4.pose.position.z = 5.0;
    wall_4.pose.orientation.x = 0.0;
    wall_4.pose.orientation.y = 0.0;
    wall_4.pose.orientation.z = 0.0;
    wall_4.pose.orientation.w = 1.0;
    wall_4.scale.x = 0.4;
    wall_4.scale.y = 0.4;
    wall_4.scale.z = 0.4;

    wall_4.color.r = 0.0f;
    wall_4.color.g = 1.0f;
    wall_4.color.b = 0.0f;
    wall_4.color.a = 1.0;
    markers.markers.push_back(wall_4);

    visualization_msgs::Marker wall_5;
    wall_5.header.frame_id = "map";
    wall_5.header.stamp = stamp;
    wall_5.ns = "wall_5";
    wall_5.id = markers.markers.size();
    wall_5.type = visualization_msgs::Marker::SPHERE;

    wall_5.pose.position.x = walls_x_coord[7];
    wall_5.pose.position.y = wall_y_coord[7];
    wall_5.pose.position.z = 5.0;
    wall_5.pose.orientation.x = 0.0;
    wall_5.pose.orientation.y = 0.0;
    wall_5.pose.orientation.z = 0.0;
    wall_5.pose.orientation.w = 1.0;
    wall_5.scale.x = 0.4;
    wall_5.scale.y = 0.4;
    wall_5.scale.z = 0.4;

    wall_5.color.r = 0.0f;
    wall_5.color.g = 1.0f;
    wall_5.color.b = 0.0f;
    wall_5.color.a = 1.0;
    markers.markers.push_back(wall_5);

    visualization_msgs::Marker wall_6;
    wall_6.header.frame_id = "map";
    wall_6.header.stamp = stamp;
    wall_6.ns = "wall_6";
    wall_6.id = markers.markers.size();
    wall_6.type = visualization_msgs::Marker::SPHERE;

    wall_6.pose.position.x = walls_x_coord[8];
    wall_6.pose.position.y = wall_y_coord[8];
    wall_6.pose.position.z = 5.0;
    wall_6.pose.orientation.x = 0.0;
    wall_6.pose.orientation.y = 0.0;
    wall_6.pose.orientation.z = 0.0;
    wall_6.pose.orientation.w = 1.0;
    wall_6.scale.x = 0.4;
    wall_6.scale.y = 0.4;
    wall_6.scale.z = 0.4;

    wall_6.color.r = 0.0f;
    wall_6.color.g = 1.0f;
    wall_6.color.b = 0.0f;
    wall_6.color.a = 1.0;
    markers.markers.push_back(wall_6);
    
    
    visualization_msgs::Marker wall_7;
    wall_7.header.frame_id = "map";
    wall_7.header.stamp = stamp;
    wall_7.ns = "wall_7";
    wall_7.id = markers.markers.size();
    wall_7.type = visualization_msgs::Marker::SPHERE;

    wall_7.pose.position.x = walls_x_coord[9];
    wall_7.pose.position.y = wall_y_coord[9];
    wall_7.pose.position.z = 5.0;
    wall_7.pose.orientation.x = 0.0;
    wall_7.pose.orientation.y = 0.0;
    wall_7.pose.orientation.z = 0.0;
    wall_7.pose.orientation.w = 1.0;
    wall_7.scale.x = 0.4;
    wall_7.scale.y = 0.4;
    wall_7.scale.z = 0.4;

    wall_7.color.r = 0.0f;
    wall_7.color.g = 1.0f;
    wall_7.color.b = 0.0f;
    wall_7.color.a = 1.0;
    markers.markers.push_back(wall_7);

    visualization_msgs::Marker wall_8;
    wall_8.header.frame_id = "map";
    wall_8.header.stamp = stamp;
    wall_8.ns = "wall_8";
    wall_8.id = markers.markers.size();
    wall_8.type = visualization_msgs::Marker::SPHERE;

    wall_8.pose.position.x = walls_x_coord[10];
    wall_8.pose.position.y = wall_y_coord[10];
    wall_8.pose.position.z = 5.0;
    wall_8.pose.orientation.x = 0.0;
    wall_8.pose.orientation.y = 0.0;
    wall_8.pose.orientation.z = 0.0;
    wall_8.pose.orientation.w = 1.0;
    wall_8.scale.x = 0.4;
    wall_8.scale.y = 0.4;
    wall_8.scale.z = 0.4;

    wall_8.color.r = 0.0f;
    wall_8.color.g = 1.0f;
    wall_8.color.b = 0.0f;
    wall_8.color.a = 1.0;
    markers.markers.push_back(wall_8);

    visualization_msgs::Marker wall_9;
    wall_9.header.frame_id = "map";
    wall_9.header.stamp = stamp;
    wall_9.ns = "wall_9";
    wall_9.id = markers.markers.size();
    wall_9.type = visualization_msgs::Marker::SPHERE;

    wall_9.pose.position.x = walls_x_coord[11];
    wall_9.pose.position.y = wall_y_coord[11]-0.2;  // merges with another wall
    wall_9.pose.position.z = 5.0;
    wall_9.pose.orientation.x = 0.0;
    wall_9.pose.orientation.y = 0.0;
    wall_9.pose.orientation.z = 0.0;
    wall_9.pose.orientation.w = 1.0;
    wall_9.scale.x = 0.4;
    wall_9.scale.y = 0.4;
    wall_9.scale.z = 0.4;

    wall_9.color.r = 0.0f;
    wall_9.color.g = 1.0f;
    wall_9.color.b = 0.0f;
    wall_9.color.a = 1.0;
    markers.markers.push_back(wall_9);

    visualization_msgs::Marker wall_10;
    wall_10.header.frame_id = "map";
    wall_10.header.stamp = stamp;
    wall_10.ns = "wall_10";
    wall_10.id = markers.markers.size();
    wall_10.type = visualization_msgs::Marker::SPHERE;

    wall_10.pose.position.x = walls_x_coord[12];
    wall_10.pose.position.y = wall_y_coord[12];
    wall_10.pose.position.z = 5.0;
    wall_10.pose.orientation.x = 0.0;
    wall_10.pose.orientation.y = 0.0;
    wall_10.pose.orientation.z = 0.0;
    wall_10.pose.orientation.w = 1.0;
    wall_10.scale.x = 0.4;
    wall_10.scale.y = 0.4;
    wall_10.scale.z = 0.4;

    wall_10.color.r = 0.0f;
    wall_10.color.g = 1.0f;
    wall_10.color.b = 0.0f;
    wall_10.color.a = 1.0;
    markers.markers.push_back(wall_10);

    visualization_msgs::Marker wall_11;
    wall_11.header.frame_id = "map";
    wall_11.header.stamp = stamp;
    wall_11.ns = "wall_11";
    wall_11.id = markers.markers.size();
    wall_11.type = visualization_msgs::Marker::SPHERE;

    wall_11.pose.position.x = walls_x_coord[13];
    wall_11.pose.position.y = wall_y_coord[13];
    wall_11.pose.position.z = 5.0;
    wall_11.pose.orientation.x = 0.0;
    wall_11.pose.orientation.y = 0.0;
    wall_11.pose.orientation.z = 0.0;
    wall_11.pose.orientation.w = 1.0;
    wall_11.scale.x = 0.4;
    wall_11.scale.y = 0.4;
    wall_11.scale.z = 0.4;

    wall_11.color.r = 0.0f;
    wall_11.color.g = 1.0f;
    wall_11.color.b = 0.0f;
    wall_11.color.a = 1.0;
    markers.markers.push_back(wall_11);

    visualization_msgs::Marker wall_12;
    wall_12.header.frame_id = "map";
    wall_12.header.stamp = stamp;
    wall_12.ns = "wall_12";
    wall_12.id = markers.markers.size();
    wall_12.type = visualization_msgs::Marker::SPHERE;

    wall_12.pose.position.x = walls_x_coord[14];
    wall_12.pose.position.y = wall_y_coord[14];
    wall_12.pose.position.z = 5.0;
    wall_12.pose.orientation.x = 0.0;
    wall_12.pose.orientation.y = 0.0;
    wall_12.pose.orientation.z = 0.0;
    wall_12.pose.orientation.w = 1.0;
    wall_12.scale.x = 0.4;
    wall_12.scale.y = 0.4;
    wall_12.scale.z = 0.4;

    wall_12.color.r = 0.0f;
    wall_12.color.g = 1.0f;
    wall_12.color.b = 0.0f;
    wall_12.color.a = 1.0;
    markers.markers.push_back(wall_12);

    visualization_msgs::Marker wall_13;
    wall_13.header.frame_id = "map";
    wall_13.header.stamp = stamp;
    wall_13.ns = "wall_13";
    wall_13.id = markers.markers.size();
    wall_13.type = visualization_msgs::Marker::SPHERE;

    wall_13.pose.position.x = walls_x_coord[15];
    wall_13.pose.position.y = wall_y_coord[15];
    wall_13.pose.position.z = 5.0;
    wall_13.pose.orientation.x = 0.0;
    wall_13.pose.orientation.y = 0.0;
    wall_13.pose.orientation.z = 0.0;
    wall_13.pose.orientation.w = 1.0;
    wall_13.scale.x = 0.4;
    wall_13.scale.y = 0.4;
    wall_13.scale.z = 0.4;

    wall_13.color.r = 0.0f;
    wall_13.color.g = 1.0f;
    wall_13.color.b = 0.0f;
    wall_13.color.a = 1.0;
    markers.markers.push_back(wall_13);


    visualization_msgs::Marker wall_14;
    wall_14.header.frame_id = "map";
    wall_14.header.stamp = stamp;
    wall_14.ns = "wall_14";
    wall_14.id = markers.markers.size();
    wall_14.type = visualization_msgs::Marker::SPHERE;

    wall_14.pose.position.x = walls_x_coord[16];
    wall_14.pose.position.y = wall_y_coord[16];
    wall_14.pose.position.z = 5.0;
    wall_14.pose.orientation.x = 0.0;
    wall_14.pose.orientation.y = 0.0;
    wall_14.pose.orientation.z = 0.0;
    wall_14.pose.orientation.w = 1.0;
    wall_14.scale.x = 0.4;
    wall_14.scale.y = 0.4;
    wall_14.scale.z = 0.4;

    wall_14.color.r = 0.0f;
    wall_14.color.g = 1.0f;
    wall_14.color.b = 0.0f;
    wall_14.color.a = 1.0;
    markers.markers.push_back(wall_14);

    visualization_msgs::Marker wall_15;
    wall_15.header.frame_id = "map";
    wall_15.header.stamp = stamp;
    wall_15.ns = "wall_15";
    wall_15.id = markers.markers.size();
    wall_15.type = visualization_msgs::Marker::SPHERE;

    wall_15.pose.position.x = walls_x_coord[17];
    wall_15.pose.position.y = wall_y_coord[17];
    wall_15.pose.position.z = 5.0;
    wall_15.pose.orientation.x = 0.0;
    wall_15.pose.orientation.y = 0.0;
    wall_15.pose.orientation.z = 0.0;
    wall_15.pose.orientation.w = 1.0;
    wall_15.scale.x = 0.4;
    wall_15.scale.y = 0.4;
    wall_15.scale.z = 0.4;

    wall_15.color.r = 0.0f;
    wall_15.color.g = 1.0f;
    wall_15.color.b = 0.0f;
    wall_15.color.a = 1.0;
    markers.markers.push_back(wall_15);

    visualization_msgs::Marker wall_16;
    wall_16.header.frame_id = "map";
    wall_16.header.stamp = stamp;
    wall_16.ns = "wall_16";
    wall_16.id = markers.markers.size();
    wall_16.type = visualization_msgs::Marker::SPHERE;

    wall_16.pose.position.x = walls_x_coord[18];
    wall_16.pose.position.y = wall_y_coord[18];
    wall_16.pose.position.z = 5.0;
    wall_16.pose.orientation.x = 0.0;
    wall_16.pose.orientation.y = 0.0;
    wall_16.pose.orientation.z = 0.0;
    wall_16.pose.orientation.w = 1.0;
    wall_16.scale.x = 0.4;
    wall_16.scale.y = 0.4;
    wall_16.scale.z = 0.4;

    wall_16.color.r = 0.0f;
    wall_16.color.g = 1.0f;
    wall_16.color.b = 0.0f;
    wall_16.color.a = 1.0;
    markers.markers.push_back(wall_16);

    visualization_msgs::Marker wall_17;
    wall_17.header.frame_id = "map";
    wall_17.header.stamp = stamp;
    wall_17.ns = "wall_17";
    wall_17.id = markers.markers.size();
    wall_17.type = visualization_msgs::Marker::SPHERE;

    wall_17.pose.position.x = walls_x_coord[19];
    wall_17.pose.position.y = wall_y_coord[19];
    wall_17.pose.position.z = 5.0;
    wall_17.pose.orientation.x = 0.0;
    wall_17.pose.orientation.y = 0.0;
    wall_17.pose.orientation.z = 0.0;
    wall_17.pose.orientation.w = 1.0;
    wall_17.scale.x = 0.4;
    wall_17.scale.y = 0.4;
    wall_17.scale.z = 0.4;

    wall_17.color.r = 0.0f;
    wall_17.color.g = 1.0f;
    wall_17.color.b = 0.0f;
    wall_17.color.a = 1.0;
    markers.markers.push_back(wall_17);

    visualization_msgs::Marker wall_18;
    wall_18.header.frame_id = "map";
    wall_18.header.stamp = stamp;
    wall_18.ns = "wall_18";
    wall_18.id = markers.markers.size();
    wall_18.type = visualization_msgs::Marker::SPHERE;

    wall_18.pose.position.x = walls_x_coord[20];
    wall_18.pose.position.y = wall_y_coord[20];
    wall_18.pose.position.z = 5.0;
    wall_18.pose.orientation.x = 0.0;
    wall_18.pose.orientation.y = 0.0;
    wall_18.pose.orientation.z = 0.0;
    wall_18.pose.orientation.w = 1.0;
    wall_18.scale.x = 0.4;
    wall_18.scale.y = 0.4;
    wall_18.scale.z = 0.4;

    wall_18.color.r = 0.0f;
    wall_18.color.g = 1.0f;
    wall_18.color.b = 0.0f;
    wall_18.color.a = 1.0;
    markers.markers.push_back(wall_18);

    visualization_msgs::Marker wall_19;
    wall_19.header.frame_id = "map";
    wall_19.header.stamp = stamp;
    wall_19.ns = "wall_18";
    wall_19.id = markers.markers.size();
    wall_19.type = visualization_msgs::Marker::SPHERE;

    wall_19.pose.position.x = walls_x_coord[21];
    wall_19.pose.position.y = wall_y_coord[21];
    wall_19.pose.position.z = 5.0;
    wall_19.pose.orientation.x = 0.0;
    wall_19.pose.orientation.y = 0.0;
    wall_19.pose.orientation.z = 0.0;
    wall_19.pose.orientation.w = 1.0;
    wall_19.scale.x = 0.4;
    wall_19.scale.y = 0.4;
    wall_19.scale.z = 0.4;

    wall_19.color.r = 0.0f;
    wall_19.color.g = 1.0f;
    wall_19.color.b = 0.0f;
    wall_19.color.a = 1.0;
    markers.markers.push_back(wall_19);


    visualization_msgs::Marker wall_20;
    wall_20.header.frame_id = "map";
    wall_20.header.stamp = stamp;
    wall_20.ns = "wall_20";
    wall_20.id = markers.markers.size();
    wall_20.type = visualization_msgs::Marker::SPHERE;

    wall_20.pose.position.x = walls_x_coord[22];
    wall_20.pose.position.y = wall_y_coord[22];
    wall_20.pose.position.z = 5.0;
    wall_20.pose.orientation.x = 0.0;
    wall_20.pose.orientation.y = 0.0;
    wall_20.pose.orientation.z = 0.0;
    wall_20.pose.orientation.w = 1.0;
    wall_20.scale.x = 0.4;
    wall_20.scale.y = 0.4;
    wall_20.scale.z = 0.4;

    wall_20.color.r = 0.0f;
    wall_20.color.g = 1.0f;
    wall_20.color.b = 0.0f;
    wall_20.color.a = 1.0;
    markers.markers.push_back(wall_20);

    visualization_msgs::Marker wall_21;
    wall_21.header.frame_id = "map";
    wall_21.header.stamp = stamp;
    wall_21.ns = "wall_21";
    wall_21.id = markers.markers.size();
    wall_21.type = visualization_msgs::Marker::SPHERE;

    wall_21.pose.position.x = walls_x_coord[23];
    wall_21.pose.position.y = wall_y_coord[23];
    wall_21.pose.position.z = 5.0;
    wall_21.pose.orientation.x = 0.0;
    wall_21.pose.orientation.y = 0.0;
    wall_21.pose.orientation.z = 0.0;
    wall_21.pose.orientation.w = 1.0;
    wall_21.scale.x = 0.4;
    wall_21.scale.y = 0.4;
    wall_21.scale.z = 0.4;

    wall_21.color.r = 0.0f;
    wall_21.color.g = 1.0f;
    wall_21.color.b = 0.0f;
    wall_21.color.a = 1.0;
    markers.markers.push_back(wall_21);

    visualization_msgs::Marker wall_22;
    wall_22.header.frame_id = "map";
    wall_22.header.stamp = stamp;
    wall_22.ns = "wall_22";
    wall_22.id = markers.markers.size();
    wall_22.type = visualization_msgs::Marker::SPHERE;

    wall_22.pose.position.x = walls_x_coord[24];
    wall_22.pose.position.y = wall_y_coord[24]+0.2;
    wall_22.pose.position.z = 5.0;
    wall_22.pose.orientation.x = 0.0;
    wall_22.pose.orientation.y = 0.0;
    wall_22.pose.orientation.z = 0.0;
    wall_22.pose.orientation.w = 1.0;
    wall_22.scale.x = 0.4;
    wall_22.scale.y = 0.4;
    wall_22.scale.z = 0.4;

    wall_22.color.r = 0.0f;
    wall_22.color.g = 1.0f;
    wall_22.color.b = 0.0f;
    wall_22.color.a = 1.0;
    markers.markers.push_back(wall_22);

    visualization_msgs::Marker wall_23;
    wall_23.header.frame_id = "map";
    wall_23.header.stamp = stamp;
    wall_23.ns = "wall_23";
    wall_23.id = markers.markers.size();
    wall_23.type = visualization_msgs::Marker::SPHERE;

    wall_23.pose.position.x = walls_x_coord[25];
    wall_23.pose.position.y = wall_y_coord[25];
    wall_23.pose.position.z = 5.0;
    wall_23.pose.orientation.x = 0.0;
    wall_23.pose.orientation.y = 0.0;
    wall_23.pose.orientation.z = 0.0;
    wall_23.pose.orientation.w = 1.0;
    wall_23.scale.x = 0.4;
    wall_23.scale.y = 0.4;
    wall_23.scale.z = 0.4;

    wall_23.color.r = 0.0f;
    wall_23.color.g = 1.0f;
    wall_23.color.b = 0.0f;
    wall_23.color.a = 1.0;
    markers.markers.push_back(wall_23);

    visualization_msgs::Marker wall_24;
    wall_24.header.frame_id = "map";
    wall_24.header.stamp = stamp;
    wall_24.ns = "wall_24";
    wall_24.id = markers.markers.size();
    wall_24.type = visualization_msgs::Marker::SPHERE;

    wall_24.pose.position.x = walls_x_coord[26];
    wall_24.pose.position.y = wall_y_coord[26];
    wall_24.pose.position.z = 5.0;
    wall_24.pose.orientation.x = 0.0;
    wall_24.pose.orientation.y = 0.0;
    wall_24.pose.orientation.z = 0.0;
    wall_24.pose.orientation.w = 1.0;
    wall_24.scale.x = 0.4;
    wall_24.scale.y = 0.4;
    wall_24.scale.z = 0.4;

    wall_24.color.r = 0.0f;
    wall_24.color.g = 1.0f;
    wall_24.color.b = 0.0f;
    wall_24.color.a = 1.0;
    markers.markers.push_back(wall_24);

    visualization_msgs::Marker wall_25;
    wall_25.header.frame_id = "map";
    wall_25.header.stamp = stamp;
    wall_25.ns = "wall_25";
    wall_25.id = markers.markers.size();
    wall_25.type = visualization_msgs::Marker::SPHERE;

    wall_25.pose.position.x = walls_x_coord[27];
    wall_25.pose.position.y = wall_y_coord[27];
    wall_25.pose.position.z = 5.0;
    wall_25.pose.orientation.x = 0.0;
    wall_25.pose.orientation.y = 0.0;
    wall_25.pose.orientation.z = 0.0;
    wall_25.pose.orientation.w = 1.0;
    wall_25.scale.x = 0.4;
    wall_25.scale.y = 0.4;
    wall_25.scale.z = 0.4;

    wall_25.color.r = 0.0f;
    wall_25.color.g = 1.0f;
    wall_25.color.b = 0.0f;
    wall_25.color.a = 1.0;
    markers.markers.push_back(wall_25);

    visualization_msgs::Marker wall_26;
    wall_26.header.frame_id = "map";
    wall_26.header.stamp = stamp;
    wall_26.ns = "wall_26";
    wall_26.id = markers.markers.size();
    wall_26.type = visualization_msgs::Marker::SPHERE;

    wall_26.pose.position.x = walls_x_coord[28];
    wall_26.pose.position.y = wall_y_coord[28];
    wall_26.pose.position.z = 5.0;
    wall_26.pose.orientation.x = 0.0;
    wall_26.pose.orientation.y = 0.0;
    wall_26.pose.orientation.z = 0.0;
    wall_26.pose.orientation.w = 1.0;
    wall_26.scale.x = 0.4;
    wall_26.scale.y = 0.4;
    wall_26.scale.z = 0.4;

    wall_26.color.r = 0.0f;
    wall_26.color.g = 1.0f;
    wall_26.color.b = 0.0f;
    wall_26.color.a = 1.0;
    markers.markers.push_back(wall_26);

    visualization_msgs::Marker wall_27;
    wall_27.header.frame_id = "map";
    wall_27.header.stamp = stamp;
    wall_27.ns = "wall_27";
    wall_27.id = markers.markers.size();
    wall_27.type = visualization_msgs::Marker::SPHERE;

    wall_27.pose.position.x = walls_x_coord[29];
    wall_27.pose.position.y = wall_y_coord[29];
    wall_27.pose.position.z = 5.0;
    wall_27.pose.orientation.x = 0.0;
    wall_27.pose.orientation.y = 0.0;
    wall_27.pose.orientation.z = 0.0;
    wall_27.pose.orientation.w = 1.0;
    wall_27.scale.x = 0.4;
    wall_27.scale.y = 0.4;
    wall_27.scale.z = 0.4;

    wall_27.color.r = 0.0f;
    wall_27.color.g = 1.0f;
    wall_27.color.b = 0.0f;
    wall_27.color.a = 1.0;
    markers.markers.push_back(wall_27);

    visualization_msgs::Marker wall_28;
    wall_28.header.frame_id = "map";
    wall_28.header.stamp = stamp;
    wall_28.ns = "wall_28";
    wall_28.id = markers.markers.size();
    wall_28.type = visualization_msgs::Marker::SPHERE;

    wall_28.pose.position.x = walls_x_coord[30];
    wall_28.pose.position.y = wall_y_coord[30]+0.2;
    wall_28.pose.position.z = 5.0;
    wall_28.pose.orientation.x = 0.0;
    wall_28.pose.orientation.y = 0.0;
    wall_28.pose.orientation.z = 0.0;
    wall_28.pose.orientation.w = 1.0;
    wall_28.scale.x = 0.4;
    wall_28.scale.y = 0.4;
    wall_28.scale.z = 0.4;

    wall_28.color.r = 0.0f;
    wall_28.color.g = 1.0f;
    wall_28.color.b = 0.0f;
    wall_28.color.a = 1.0;
    markers.markers.push_back(wall_28);

    visualization_msgs::Marker wall_29;
    wall_29.header.frame_id = "map";
    wall_29.header.stamp = stamp;
    wall_29.ns = "wall_29";
    wall_29.id = markers.markers.size();
    wall_29.type = visualization_msgs::Marker::SPHERE;

    wall_29.pose.position.x = walls_x_coord[31];
    wall_29.pose.position.y = wall_y_coord[31];
    wall_29.pose.position.z = 5.0;
    wall_29.pose.orientation.x = 0.0;
    wall_29.pose.orientation.y = 0.0;
    wall_29.pose.orientation.z = 0.0;
    wall_29.pose.orientation.w = 1.0;
    wall_29.scale.x = 0.4;
    wall_29.scale.y = 0.4;
    wall_29.scale.z = 0.4;

    wall_29.color.r = 0.0f;
    wall_29.color.g = 1.0f;
    wall_29.color.b = 0.0f;
    wall_29.color.a = 1.0;
    markers.markers.push_back(wall_29);

    visualization_msgs::Marker wall_30;
    wall_30.header.frame_id = "map";
    wall_30.header.stamp = stamp;
    wall_30.ns = "wall_30";
    wall_30.id = markers.markers.size();
    wall_30.type = visualization_msgs::Marker::SPHERE;

    wall_30.pose.position.x = walls_x_coord[32];
    wall_30.pose.position.y = wall_y_coord[32];
    wall_30.pose.position.z = 5.0;
    wall_30.pose.orientation.x = 0.0;
    wall_30.pose.orientation.y = 0.0;
    wall_30.pose.orientation.z = 0.0;
    wall_30.pose.orientation.w = 1.0;
    wall_30.scale.x = 0.4;
    wall_30.scale.y = 0.4;
    wall_30.scale.z = 0.4;

    wall_30.color.r = 0.0f;
    wall_30.color.g = 1.0f;
    wall_30.color.b = 0.0f;
    wall_30.color.a = 1.0;
    markers.markers.push_back(wall_30);

    visualization_msgs::Marker room_284_conn_1;
    room_284_conn_1.header.frame_id = "map";
    room_284_conn_1.header.stamp = stamp;
    room_284_conn_1.ns = "room_284_conn_1";
    room_284_conn_1.id = markers.markers.size();
    room_284_conn_1.type = visualization_msgs::Marker::LINE_LIST;
    room_284_conn_1.color.r=1.0f;
    room_284_conn_1.color.g=1.0f;
    room_284_conn_1.color.b=1.0f;
    room_284_conn_1.color.a=1.0f;
    room_284_conn_1.scale.x = room_284_conn_1.scale.y = room_284_conn_1.scale.z = 0.01;
    geometry_msgs::Point point_1;
    point_1.x = room_x_coord[0];
    point_1.y = room_y_coord[0];
    point_1.z = 7;
    room_284_conn_1.points.push_back(point_1);
    geometry_msgs::Point point_2;
    point_2.x = walls_x_coord[7];
    point_2.y = wall_y_coord[7];
    point_2.z = 5;
    room_284_conn_1.points.push_back(point_2);
    markers.markers.push_back(room_284_conn_1);

    visualization_msgs::Marker room_284_conn_2;
    room_284_conn_2.header.frame_id = "map";
    room_284_conn_2.header.stamp = stamp;
    room_284_conn_2.ns = "room_284_conn_2";
    room_284_conn_2.id = markers.markers.size();
    room_284_conn_2.type = visualization_msgs::Marker::LINE_LIST;
    room_284_conn_2.color.r=1.0f;
    room_284_conn_2.color.g=1.0f;
    room_284_conn_2.color.b=1.0f;
    room_284_conn_2.color.a=1.0f;
    room_284_conn_2.scale.x = room_284_conn_2.scale.y = room_284_conn_2.scale.z = 0.01;
    point_1.x = room_x_coord[0];
    point_1.y = room_y_coord[0];
    point_1.z = 7;
    room_284_conn_2.points.push_back(point_1);
    point_2.x = walls_x_coord[16];
    point_2.y = wall_y_coord[16];
    point_2.z = 5;
    room_284_conn_2.points.push_back(point_2);
    markers.markers.push_back(room_284_conn_2);

     visualization_msgs::Marker room_284_conn_3;
    room_284_conn_3.header.frame_id = "map";
    room_284_conn_3.header.stamp = stamp;
    room_284_conn_3.ns = "room_284_conn_3";
    room_284_conn_3.id = markers.markers.size();
    room_284_conn_3.type = visualization_msgs::Marker::LINE_LIST;
    room_284_conn_3.color.r=1.0f;
    room_284_conn_3.color.g=1.0f;
    room_284_conn_3.color.b=1.0f;
    room_284_conn_3.color.a=1.0f;
    room_284_conn_3.scale.x = room_284_conn_3.scale.y = room_284_conn_3.scale.z = 0.01;
    point_1.x = room_x_coord[0];
    point_1.y = room_y_coord[0];
    point_1.z = 7;
    room_284_conn_3.points.push_back(point_1);
    point_2.x = walls_x_coord[17];
    point_2.y = wall_y_coord[17];
    point_2.z = 5;
    room_284_conn_3.points.push_back(point_2);
    markers.markers.push_back(room_284_conn_3);

    visualization_msgs::Marker room_284_conn_4;
    room_284_conn_4.header.frame_id = "map";
    room_284_conn_4.header.stamp = stamp;
    room_284_conn_4.ns = "room_284_conn_4";
    room_284_conn_4.id = markers.markers.size();
    room_284_conn_4.type = visualization_msgs::Marker::LINE_LIST;
    room_284_conn_4.color.r=1.0f;
    room_284_conn_4.color.g=1.0f;
    room_284_conn_4.color.b=1.0f;
    room_284_conn_4.color.a=1.0f;
    room_284_conn_4.scale.x = room_284_conn_4.scale.y = room_284_conn_4.scale.z = 0.01;
    point_1.x = room_x_coord[0];
    point_1.y = room_y_coord[0];
    point_1.z = 7;
    room_284_conn_4.points.push_back(point_1);
    point_2.x = walls_x_coord[31];
    point_2.y = wall_y_coord[31];
    point_2.z = 5;
    room_284_conn_4.points.push_back(point_2);
    markers.markers.push_back(room_284_conn_4);

    visualization_msgs::Marker room_299_conn_1;
    room_299_conn_1.header.frame_id = "map";
    room_299_conn_1.header.stamp = stamp;
    room_299_conn_1.ns = "room_299_conn_1";
    room_299_conn_1.id = markers.markers.size();
    room_299_conn_1.type = visualization_msgs::Marker::LINE_LIST;
    room_299_conn_1.color.r=1.0f;
    room_299_conn_1.color.g=1.0f;
    room_299_conn_1.color.b=1.0f;
    room_299_conn_1.color.a=1.0f;
    room_299_conn_1.scale.x = room_299_conn_1.scale.y = room_299_conn_1.scale.z = 0.01;
    point_1.x = room_x_coord[1];
    point_1.y = room_y_coord[1];
    point_1.z = 7;
    room_299_conn_1.points.push_back(point_1);
    point_2.x = walls_x_coord[9];
    point_2.y = wall_y_coord[9];
    point_2.z = 5;
    room_299_conn_1.points.push_back(point_2);
    markers.markers.push_back(room_299_conn_1);


    visualization_msgs::Marker room_299_conn_2;
    room_299_conn_2.header.frame_id = "map";
    room_299_conn_2.header.stamp = stamp;
    room_299_conn_2.ns = "room_299_conn_2";
    room_299_conn_2.id = markers.markers.size();
    room_299_conn_2.type = visualization_msgs::Marker::LINE_LIST;
    room_299_conn_2.color.r=1.0f;
    room_299_conn_2.color.g=1.0f;
    room_299_conn_2.color.b=1.0f;
    room_299_conn_2.color.a=1.0f;
    room_299_conn_2.scale.x = room_299_conn_2.scale.y = room_299_conn_2.scale.z = 0.01;
    point_1.x = room_x_coord[1];
    point_1.y = room_y_coord[1];
    point_1.z = 7;
    room_299_conn_2.points.push_back(point_1);
    point_2.x = walls_x_coord[3];
    point_2.y = wall_y_coord[3];
    point_2.z = 5;
    room_299_conn_2.points.push_back(point_2);
    markers.markers.push_back(room_299_conn_2);

    visualization_msgs::Marker room_299_conn_3;
    room_299_conn_3.header.frame_id = "map";
    room_299_conn_3.header.stamp = stamp;
    room_299_conn_3.ns = "room_299_conn_3";
    room_299_conn_3.id = markers.markers.size();
    room_299_conn_3.type = visualization_msgs::Marker::LINE_LIST;
    room_299_conn_3.color.r=1.0f;
    room_299_conn_3.color.g=1.0f;
    room_299_conn_3.color.b=1.0f;
    room_299_conn_3.color.a=1.0f;
    room_299_conn_3.scale.x = room_299_conn_3.scale.y = room_299_conn_3.scale.z = 0.01;
    point_1.x = room_x_coord[1];
    point_1.y = room_y_coord[1];
    point_1.z = 7;
    room_299_conn_3.points.push_back(point_1);
    point_2.x = walls_x_coord[15];
    point_2.y = wall_y_coord[15];
    point_2.z = 5;
    room_299_conn_3.points.push_back(point_2);
    markers.markers.push_back(room_299_conn_3);

    visualization_msgs::Marker room_299_conn_4;
    room_299_conn_4.header.frame_id = "map";
    room_299_conn_4.header.stamp = stamp;
    room_299_conn_4.ns = "room_299_conn_4";
    room_299_conn_4.id = markers.markers.size();
    room_299_conn_4.type = visualization_msgs::Marker::LINE_LIST;
    room_299_conn_4.color.r=1.0f;
    room_299_conn_4.color.g=1.0f;
    room_299_conn_4.color.b=1.0f;
    room_299_conn_4.color.a=1.0f;
    room_299_conn_4.scale.x = room_299_conn_4.scale.y = room_299_conn_4.scale.z = 0.01;
    point_1.x = room_x_coord[1];
    point_1.y = room_y_coord[1];
    point_1.z = 7;
    room_299_conn_4.points.push_back(point_1);
    point_2.x = walls_x_coord[8];
    point_2.y = wall_y_coord[8];
    point_2.z = 5;
    room_299_conn_4.points.push_back(point_2);
    markers.markers.push_back(room_299_conn_4);

    visualization_msgs::Marker room_299_conn_5;
    room_299_conn_5.header.frame_id = "map";
    room_299_conn_5.header.stamp = stamp;
    room_299_conn_5.ns = "room_299_conn_5";
    room_299_conn_5.id = markers.markers.size();
    room_299_conn_5.type = visualization_msgs::Marker::LINE_LIST;
    room_299_conn_5.color.r=1.0f;
    room_299_conn_5.color.g=1.0f;
    room_299_conn_5.color.b=1.0f;
    room_299_conn_5.color.a=1.0f;
    room_299_conn_5.scale.x = room_299_conn_5.scale.y = room_299_conn_5.scale.z = 0.01;
    point_1.x = room_x_coord[1];
    point_1.y = room_y_coord[1];
    point_1.z = 7;
    room_299_conn_5.points.push_back(point_1);
    point_2.x = walls_x_coord[32];
    point_2.y = wall_y_coord[32];
    point_2.z = 5;
    room_299_conn_5.points.push_back(point_2);
    markers.markers.push_back(room_299_conn_5);

    visualization_msgs::Marker room_299_conn_6;
    room_299_conn_6.header.frame_id = "map";
    room_299_conn_6.header.stamp = stamp;
    room_299_conn_6.ns = "room_299_conn_5";
    room_299_conn_6.id = markers.markers.size();
    room_299_conn_6.type = visualization_msgs::Marker::LINE_LIST;
    room_299_conn_6.color.r=1.0f;
    room_299_conn_6.color.g=1.0f;
    room_299_conn_6.color.b=1.0f;
    room_299_conn_6.color.a=1.0f;
    room_299_conn_6.scale.x = room_299_conn_6.scale.y = room_299_conn_6.scale.z = 0.01;
    point_1.x = room_x_coord[1];
    point_1.y = room_y_coord[1];
    point_1.z = 7;
    room_299_conn_6.points.push_back(point_1);
    point_2.x = walls_x_coord[16];
    point_2.y = wall_y_coord[16];
    point_2.z = 5;
    room_299_conn_6.points.push_back(point_2);
    markers.markers.push_back(room_299_conn_6);

    visualization_msgs::Marker room_302_conn_1;
    room_302_conn_1.header.frame_id = "map";
    room_302_conn_1.header.stamp = stamp;
    room_302_conn_1.ns = "room_302_conn_1";
    room_302_conn_1.id = markers.markers.size();
    room_302_conn_1.type = visualization_msgs::Marker::LINE_LIST;
    room_302_conn_1.color.r=1.0f;
    room_302_conn_1.color.g=1.0f;
    room_302_conn_1.color.b=1.0f;
    room_302_conn_1.color.a=1.0f;
    room_302_conn_1.scale.x = room_302_conn_1.scale.y = room_302_conn_1.scale.z = 0.01;
    point_1.x = room_x_coord[2];
    point_1.y = room_y_coord[2];
    point_1.z = 7;
    room_302_conn_1.points.push_back(point_1);
    point_2.x = walls_x_coord[11];
    point_2.y = wall_y_coord[11];
    point_2.z = 5;
    room_302_conn_1.points.push_back(point_2);
    markers.markers.push_back(room_302_conn_1);

    visualization_msgs::Marker room_302_conn_2;
    room_302_conn_2.header.frame_id = "map";
    room_302_conn_2.header.stamp = stamp;
    room_302_conn_2.ns = "room_302_conn_2";
    room_302_conn_2.id = markers.markers.size();
    room_302_conn_2.type = visualization_msgs::Marker::LINE_LIST;
    room_302_conn_2.color.r=1.0f;
    room_302_conn_2.color.g=1.0f;
    room_302_conn_2.color.b=1.0f;
    room_302_conn_2.color.a=1.0f;
    room_302_conn_2.scale.x = room_302_conn_2.scale.y = room_302_conn_2.scale.z = 0.01;
    point_1.x = room_x_coord[2];
    point_1.y = room_y_coord[2];
    point_1.z = 7;
    room_302_conn_2.points.push_back(point_1);
    point_2.x = walls_x_coord[16];
    point_2.y = wall_y_coord[16];
    point_2.z = 5;
    room_302_conn_2.points.push_back(point_2);
    markers.markers.push_back(room_302_conn_2);

    visualization_msgs::Marker room_302_conn_3;
    room_302_conn_3.header.frame_id = "map";
    room_302_conn_3.header.stamp = stamp;
    room_302_conn_3.ns = "room_302_conn_3";
    room_302_conn_3.id = markers.markers.size();
    room_302_conn_3.type = visualization_msgs::Marker::LINE_LIST;
    room_302_conn_3.color.r=1.0f;
    room_302_conn_3.color.g=1.0f;
    room_302_conn_3.color.b=1.0f;
    room_302_conn_3.color.a=1.0f;
    room_302_conn_3.scale.x = room_302_conn_3.scale.y = room_302_conn_3.scale.z = 0.01;
    point_1.x = room_x_coord[2];
    point_1.y = room_y_coord[2];
    point_1.z = 7;
    room_302_conn_3.points.push_back(point_1);
    point_2.x = walls_x_coord[5];
    point_2.y = wall_y_coord[5];
    point_2.z = 5;
    room_302_conn_3.points.push_back(point_2);
    markers.markers.push_back(room_302_conn_3);

    visualization_msgs::Marker room_302_conn_4;
    room_302_conn_4.header.frame_id = "map";
    room_302_conn_4.header.stamp = stamp;
    room_302_conn_4.ns = "room_302_conn_4";
    room_302_conn_4.id = markers.markers.size();
    room_302_conn_4.type = visualization_msgs::Marker::LINE_LIST;
    room_302_conn_4.color.r=1.0f;
    room_302_conn_4.color.g=1.0f;
    room_302_conn_4.color.b=1.0f;
    room_302_conn_4.color.a=1.0f;
    room_302_conn_4.scale.x = room_302_conn_4.scale.y = room_302_conn_4.scale.z = 0.01;
    point_1.x = room_x_coord[2];
    point_1.y = room_y_coord[2];
    point_1.z = 7;
    room_302_conn_4.points.push_back(point_1);
    point_2.x = walls_x_coord[6];
    point_2.y = wall_y_coord[6];
    point_2.z = 5;
    room_302_conn_4.points.push_back(point_2);
    markers.markers.push_back(room_302_conn_4);

    visualization_msgs::Marker room_302_conn_5;
    room_302_conn_5.header.frame_id = "map";
    room_302_conn_5.header.stamp = stamp;
    room_302_conn_5.ns = "room_302_conn_5";
    room_302_conn_5.id = markers.markers.size();
    room_302_conn_5.type = visualization_msgs::Marker::LINE_LIST;
    room_302_conn_5.color.r=1.0f;
    room_302_conn_5.color.g=1.0f;
    room_302_conn_5.color.b=1.0f;
    room_302_conn_5.color.a=1.0f;
    room_302_conn_5.scale.x = room_302_conn_5.scale.y = room_302_conn_5.scale.z = 0.01;
    point_1.x = room_x_coord[2];
    point_1.y = room_y_coord[2];
    point_1.z = 7;
    room_302_conn_5.points.push_back(point_1);
    point_2.x = walls_x_coord[14];
    point_2.y = wall_y_coord[14];
    point_2.z = 5;
    room_302_conn_5.points.push_back(point_2);
    markers.markers.push_back(room_302_conn_5);

    visualization_msgs::Marker room_302_conn_6;
    room_302_conn_6.header.frame_id = "map";
    room_302_conn_6.header.stamp = stamp;
    room_302_conn_6.ns = "room_302_conn_6";
    room_302_conn_6.id = markers.markers.size();
    room_302_conn_6.type = visualization_msgs::Marker::LINE_LIST;
    room_302_conn_6.color.r=1.0f;
    room_302_conn_6.color.g=1.0f;
    room_302_conn_6.color.b=1.0f;
    room_302_conn_6.color.a=1.0f;
    room_302_conn_6.scale.x = room_302_conn_6.scale.y = room_302_conn_6.scale.z = 0.01;
    point_1.x = room_x_coord[2];
    point_1.y = room_y_coord[2];
    point_1.z = 7;
    room_302_conn_6.points.push_back(point_1);
    point_2.x = walls_x_coord[13];
    point_2.y = wall_y_coord[13];
    point_2.z = 5;
    room_302_conn_6.points.push_back(point_2);
    markers.markers.push_back(room_302_conn_6);

    visualization_msgs::Marker room_322_conn_1;
    room_322_conn_1.header.frame_id = "map";
    room_322_conn_1.header.stamp = stamp;
    room_322_conn_1.ns = "room_322_conn_1";
    room_322_conn_1.id = markers.markers.size();
    room_322_conn_1.type = visualization_msgs::Marker::LINE_LIST;
    room_322_conn_1.color.r=1.0f;
    room_322_conn_1.color.g=1.0f;
    room_322_conn_1.color.b=1.0f;
    room_322_conn_1.color.a=1.0f;
    room_322_conn_1.scale.x = room_322_conn_1.scale.y = room_322_conn_1.scale.z = 0.01;
    point_1.x = room_x_coord[3];
    point_1.y = room_y_coord[3];
    point_1.z = 7;
    room_322_conn_1.points.push_back(point_1);
    point_2.x = walls_x_coord[4];
    point_2.y = wall_y_coord[4];
    point_2.z = 5;
    room_322_conn_1.points.push_back(point_2);
    markers.markers.push_back(room_322_conn_1);

    visualization_msgs::Marker room_322_conn_2;
    room_322_conn_2.header.frame_id = "map";
    room_322_conn_2.header.stamp = stamp;
    room_322_conn_2.ns = "room_322_conn_2";
    room_322_conn_2.id = markers.markers.size();
    room_322_conn_2.type = visualization_msgs::Marker::LINE_LIST;
    room_322_conn_2.color.r=1.0f;
    room_322_conn_2.color.g=1.0f;
    room_322_conn_2.color.b=1.0f;
    room_322_conn_2.color.a=1.0f;
    room_322_conn_2.scale.x = room_322_conn_2.scale.y = room_322_conn_2.scale.z = 0.01;
    point_1.x = room_x_coord[3];
    point_1.y = room_y_coord[3];
    point_1.z = 7;
    room_322_conn_2.points.push_back(point_1);
    point_2.x = walls_x_coord[26];
    point_2.y = wall_y_coord[26];
    point_2.z = 5;
    room_322_conn_2.points.push_back(point_2);
    markers.markers.push_back(room_322_conn_2);

    visualization_msgs::Marker room_322_conn_3;
    room_322_conn_3.header.frame_id = "map";
    room_322_conn_3.header.stamp = stamp;
    room_322_conn_3.ns = "room_322_conn_3";
    room_322_conn_3.id = markers.markers.size();
    room_322_conn_3.type = visualization_msgs::Marker::LINE_LIST;
    room_322_conn_3.color.r=1.0f;
    room_322_conn_3.color.g=1.0f;
    room_322_conn_3.color.b=1.0f;
    room_322_conn_3.color.a=1.0f;
    room_322_conn_3.scale.x = room_322_conn_3.scale.y = room_322_conn_3.scale.z = 0.01;
    point_1.x = room_x_coord[3];
    point_1.y = room_y_coord[3];
    point_1.z = 7;
    room_322_conn_3.points.push_back(point_1);
    point_2.x = walls_x_coord[32];
    point_2.y = wall_y_coord[32];
    point_2.z = 5;
    room_322_conn_3.points.push_back(point_2);
    markers.markers.push_back(room_322_conn_3);

    visualization_msgs::Marker room_322_conn_4;
    room_322_conn_4.header.frame_id = "map";
    room_322_conn_4.header.stamp = stamp;
    room_322_conn_4.ns = "room_322_conn_4";
    room_322_conn_4.id = markers.markers.size();
    room_322_conn_4.type = visualization_msgs::Marker::LINE_LIST;
    room_322_conn_4.color.r=1.0f;
    room_322_conn_4.color.g=1.0f;
    room_322_conn_4.color.b=1.0f;
    room_322_conn_4.color.a=1.0f;
    room_322_conn_4.scale.x = room_322_conn_4.scale.y = room_322_conn_4.scale.z = 0.01;
    point_1.x = room_x_coord[3];
    point_1.y = room_y_coord[3];
    point_1.z = 7;
    room_322_conn_4.points.push_back(point_1);
    point_2.x = walls_x_coord[5];
    point_2.y = wall_y_coord[5];
    point_2.z = 5;
    room_322_conn_4.points.push_back(point_2);
    markers.markers.push_back(room_322_conn_4);

    visualization_msgs::Marker room_324_conn_1;
    room_324_conn_1.header.frame_id = "map";
    room_324_conn_1.header.stamp = stamp;
    room_324_conn_1.ns = "room_324_conn_1";
    room_324_conn_1.id = markers.markers.size();
    room_324_conn_1.type = visualization_msgs::Marker::LINE_LIST;
    room_324_conn_1.color.r=1.0f;
    room_324_conn_1.color.g=1.0f;
    room_324_conn_1.color.b=1.0f;
    room_324_conn_1.color.a=1.0f;
    room_324_conn_1.scale.x = room_324_conn_1.scale.y = room_324_conn_1.scale.z = 0.01;
    point_1.x = room_x_coord[4];
    point_1.y = room_y_coord[4];
    point_1.z = 7;
    room_324_conn_1.points.push_back(point_1);
    point_2.x = walls_x_coord[26];
    point_2.y = wall_y_coord[26];
    point_2.z = 5;
    room_324_conn_1.points.push_back(point_2);
    markers.markers.push_back(room_324_conn_1);

    visualization_msgs::Marker room_324_conn_2;
    room_324_conn_2.header.frame_id = "map";
    room_324_conn_2.header.stamp = stamp;
    room_324_conn_2.ns = "room_324_conn_2";
    room_324_conn_2.id = markers.markers.size();
    room_324_conn_2.type = visualization_msgs::Marker::LINE_LIST;
    room_324_conn_2.color.r=1.0f;
    room_324_conn_2.color.g=1.0f;
    room_324_conn_2.color.b=1.0f;
    room_324_conn_2.color.a=1.0f;
    room_324_conn_2.scale.x = room_324_conn_2.scale.y = room_324_conn_2.scale.z = 0.01;
    point_1.x = room_x_coord[4];
    point_1.y = room_y_coord[4];
    point_1.z = 7;
    room_324_conn_2.points.push_back(point_1);
    point_2.x = walls_x_coord[2];
    point_2.y = wall_y_coord[2];
    point_2.z = 5;
    room_324_conn_2.points.push_back(point_2);
    markers.markers.push_back(room_324_conn_2);

    visualization_msgs::Marker room_324_conn_3;
    room_324_conn_3.header.frame_id = "map";
    room_324_conn_3.header.stamp = stamp;
    room_324_conn_3.ns = "room_324_conn_3";
    room_324_conn_3.id = markers.markers.size();
    room_324_conn_3.type = visualization_msgs::Marker::LINE_LIST;
    room_324_conn_3.color.r=1.0f;
    room_324_conn_3.color.g=1.0f;
    room_324_conn_3.color.b=1.0f;
    room_324_conn_3.color.a=1.0f;
    room_324_conn_3.scale.x = room_324_conn_3.scale.y = room_324_conn_3.scale.z = 0.01;
    point_1.x = room_x_coord[4];
    point_1.y = room_y_coord[4];
    point_1.z = 7;
    room_324_conn_3.points.push_back(point_1);
    point_2.x = walls_x_coord[15];
    point_2.y = wall_y_coord[15];
    point_2.z = 5;
    room_324_conn_3.points.push_back(point_2);
    markers.markers.push_back(room_324_conn_3);

    visualization_msgs::Marker room_324_conn_4;
    room_324_conn_4.header.frame_id = "map";
    room_324_conn_4.header.stamp = stamp;
    room_324_conn_4.ns = "room_324_conn_4";
    room_324_conn_4.id = markers.markers.size();
    room_324_conn_4.type = visualization_msgs::Marker::LINE_LIST;
    room_324_conn_4.color.r=1.0f;
    room_324_conn_4.color.g=1.0f;
    room_324_conn_4.color.b=1.0f;
    room_324_conn_4.color.a=1.0f;
    room_324_conn_4.scale.x = room_324_conn_4.scale.y = room_324_conn_4.scale.z = 0.01;
    point_1.x = room_x_coord[4];
    point_1.y = room_y_coord[4];
    point_1.z = 7;
    room_324_conn_4.points.push_back(point_1);
    point_2.x = walls_x_coord[32];
    point_2.y = wall_y_coord[32];
    point_2.z = 5;
    room_324_conn_4.points.push_back(point_2);
    markers.markers.push_back(room_324_conn_4);


    visualization_msgs::Marker room_325_conn_1;
    room_325_conn_1.header.frame_id = "map";
    room_325_conn_1.header.stamp = stamp;
    room_325_conn_1.ns = "room_325_conn_1";
    room_325_conn_1.id = markers.markers.size();
    room_325_conn_1.type = visualization_msgs::Marker::LINE_LIST;
    room_325_conn_1.color.r=1.0f;
    room_325_conn_1.color.g=1.0f;
    room_325_conn_1.color.b=1.0f;
    room_325_conn_1.color.a=1.0f;
    room_325_conn_1.scale.x = room_325_conn_1.scale.y = room_325_conn_1.scale.z = 0.01;
    point_1.x = room_x_coord[5];
    point_1.y = room_y_coord[5];
    point_1.z = 7;
    room_325_conn_1.points.push_back(point_1);
    point_2.x = walls_x_coord[11];
    point_2.y = wall_y_coord[11];
    point_2.z = 5;
    room_325_conn_1.points.push_back(point_2);
    markers.markers.push_back(room_325_conn_1);

    visualization_msgs::Marker room_325_conn_2;
    room_325_conn_2.header.frame_id = "map";
    room_325_conn_2.header.stamp = stamp;
    room_325_conn_2.ns = "room_325_conn_2";
    room_325_conn_2.id = markers.markers.size();
    room_325_conn_2.type = visualization_msgs::Marker::LINE_LIST;
    room_325_conn_2.color.r=1.0f;
    room_325_conn_2.color.g=1.0f;
    room_325_conn_2.color.b=1.0f;
    room_325_conn_2.color.a=1.0f;
    room_325_conn_2.scale.x = room_325_conn_2.scale.y = room_325_conn_2.scale.z = 0.01;
    point_1.x = room_x_coord[5];
    point_1.y = room_y_coord[5];
    point_1.z = 7;
    room_325_conn_2.points.push_back(point_1);
    point_2.x = walls_x_coord[16];
    point_2.y = wall_y_coord[16];
    point_2.z = 5;
    room_325_conn_2.points.push_back(point_2);
    markers.markers.push_back(room_325_conn_2);

    visualization_msgs::Marker room_325_conn_3;
    room_325_conn_3.header.frame_id = "map";
    room_325_conn_3.header.stamp = stamp;
    room_325_conn_3.ns = "room_325_conn_3";
    room_325_conn_3.id = markers.markers.size();
    room_325_conn_3.type = visualization_msgs::Marker::LINE_LIST;
    room_325_conn_3.color.r=1.0f;
    room_325_conn_3.color.g=1.0f;
    room_325_conn_3.color.b=1.0f;
    room_325_conn_3.color.a=1.0f;
    room_325_conn_3.scale.x = room_325_conn_3.scale.y = room_325_conn_3.scale.z = 0.01;
    point_1.x = room_x_coord[5];
    point_1.y = room_y_coord[5];
    point_1.z = 7;
    room_325_conn_3.points.push_back(point_1);
    point_2.x = walls_x_coord[17];
    point_2.y = wall_y_coord[17];
    point_2.z = 5;
    room_325_conn_3.points.push_back(point_2);
    markers.markers.push_back(room_325_conn_3);

    visualization_msgs::Marker room_325_conn_4;
    room_325_conn_4.header.frame_id = "map";
    room_325_conn_4.header.stamp = stamp;
    room_325_conn_4.ns = "room_325_conn_4";
    room_325_conn_4.id = markers.markers.size();
    room_325_conn_4.type = visualization_msgs::Marker::LINE_LIST;
    room_325_conn_4.color.r=1.0f;
    room_325_conn_4.color.g=1.0f;
    room_325_conn_4.color.b=1.0f;
    room_325_conn_4.color.a=1.0f;
    room_325_conn_4.scale.x = room_325_conn_4.scale.y = room_325_conn_4.scale.z = 0.01;
    point_1.x = room_x_coord[5];
    point_1.y = room_y_coord[5];
    point_1.z = 7;
    room_325_conn_4.points.push_back(point_1);
    point_2.x = walls_x_coord[7];
    point_2.y = wall_y_coord[7];
    point_2.z = 5;
    room_325_conn_4.points.push_back(point_2);
    markers.markers.push_back(room_325_conn_4);


   visualization_msgs::Marker room_326_conn_1;
    room_326_conn_1.header.frame_id = "map";
    room_326_conn_1.header.stamp = stamp;
    room_326_conn_1.ns = "room_326_conn_1";
    room_326_conn_1.id = markers.markers.size();
    room_326_conn_1.type = visualization_msgs::Marker::LINE_LIST;
    room_326_conn_1.color.r=1.0f;
    room_326_conn_1.color.g=1.0f;
    room_326_conn_1.color.b=1.0f;
    room_326_conn_1.color.a=1.0f;
    room_326_conn_1.scale.x = room_326_conn_1.scale.y = room_326_conn_1.scale.z = 0.01;
    point_1.x = room_x_coord[6];
    point_1.y = room_y_coord[6];
    point_1.z = 7;
    room_326_conn_1.points.push_back(point_1);
    point_2.x = walls_x_coord[10];
    point_2.y = wall_y_coord[10];
    point_2.z = 5;
    room_326_conn_1.points.push_back(point_2);
    markers.markers.push_back(room_326_conn_1);

    visualization_msgs::Marker room_326_conn_2;
    room_326_conn_2.header.frame_id = "map";
    room_326_conn_2.header.stamp = stamp;
    room_326_conn_2.ns = "room_326_conn_2";
    room_326_conn_2.id = markers.markers.size();
    room_326_conn_2.type = visualization_msgs::Marker::LINE_LIST;
    room_326_conn_2.color.r=1.0f;
    room_326_conn_2.color.g=1.0f;
    room_326_conn_2.color.b=1.0f;
    room_326_conn_2.color.a=1.0f;
    room_326_conn_2.scale.x = room_326_conn_2.scale.y = room_326_conn_2.scale.z = 0.01;
    point_1.x = room_x_coord[6];
    point_1.y = room_y_coord[6];
    point_1.z = 7;
    room_326_conn_2.points.push_back(point_1);
    point_2.x = walls_x_coord[31];
    point_2.y = wall_y_coord[31];
    point_2.z = 5;
    room_326_conn_2.points.push_back(point_2);
    markers.markers.push_back(room_326_conn_2);

    visualization_msgs::Marker room_326_conn_3;
    room_326_conn_3.header.frame_id = "map";
    room_326_conn_3.header.stamp = stamp;
    room_326_conn_3.ns = "room_326_conn_3";
    room_326_conn_3.id = markers.markers.size();
    room_326_conn_3.type = visualization_msgs::Marker::LINE_LIST;
    room_326_conn_3.color.r=1.0f;
    room_326_conn_3.color.g=1.0f;
    room_326_conn_3.color.b=1.0f;
    room_326_conn_3.color.a=1.0f;
    room_326_conn_3.scale.x = room_326_conn_3.scale.y = room_326_conn_3.scale.z = 0.01;
    point_1.x = room_x_coord[6];
    point_1.y = room_y_coord[6];
    point_1.z = 7;
    room_326_conn_3.points.push_back(point_1);
    point_2.x = walls_x_coord[4];
    point_2.y = wall_y_coord[4];
    point_2.z = 5;
    room_326_conn_3.points.push_back(point_2);
    markers.markers.push_back(room_326_conn_3);

    visualization_msgs::Marker room_326_conn_4;
    room_326_conn_4.header.frame_id = "map";
    room_326_conn_4.header.stamp = stamp;
    room_326_conn_4.ns = "room_326_conn_4";
    room_326_conn_4.id = markers.markers.size();
    room_326_conn_4.type = visualization_msgs::Marker::LINE_LIST;
    room_326_conn_4.color.r=1.0f;
    room_326_conn_4.color.g=1.0f;
    room_326_conn_4.color.b=1.0f;
    room_326_conn_4.color.a=1.0f;
    room_326_conn_4.scale.x = room_326_conn_4.scale.y = room_326_conn_4.scale.z = 0.01;
    point_1.x = room_x_coord[6];
    point_1.y = room_y_coord[6];
    point_1.z = 7;
    room_326_conn_4.points.push_back(point_1);
    point_2.x = walls_x_coord[16];
    point_2.y = wall_y_coord[16];
    point_2.z = 5;
    room_326_conn_4.points.push_back(point_2);
    markers.markers.push_back(room_326_conn_4);


    visualization_msgs::Marker room_327_conn_1;
    room_327_conn_1.header.frame_id = "map";
    room_327_conn_1.header.stamp = stamp;
    room_327_conn_1.ns = "room_327_conn_1";
    room_327_conn_1.id = markers.markers.size();
    room_327_conn_1.type = visualization_msgs::Marker::LINE_LIST;
    room_327_conn_1.color.r=1.0f;
    room_327_conn_1.color.g=1.0f;
    room_327_conn_1.color.b=1.0f;
    room_327_conn_1.color.a=1.0f;
    room_327_conn_1.scale.x = room_327_conn_1.scale.y = room_327_conn_1.scale.z = 0.01;
    point_1.x = room_x_coord[7];
    point_1.y = room_y_coord[7];
    point_1.z = 7;
    room_327_conn_1.points.push_back(point_1);
    point_2.x = walls_x_coord[12];
    point_2.y = wall_y_coord[12];
    point_2.z = 5;
    room_327_conn_1.points.push_back(point_2);
    markers.markers.push_back(room_327_conn_1);

    visualization_msgs::Marker room_327_conn_2;
    room_327_conn_2.header.frame_id = "map";
    room_327_conn_2.header.stamp = stamp;
    room_327_conn_2.ns = "room_327_conn_2";
    room_327_conn_2.id = markers.markers.size();
    room_327_conn_2.type = visualization_msgs::Marker::LINE_LIST;
    room_327_conn_2.color.r=1.0f;
    room_327_conn_2.color.g=1.0f;
    room_327_conn_2.color.b=1.0f;
    room_327_conn_2.color.a=1.0f;
    room_327_conn_2.scale.x = room_327_conn_2.scale.y = room_327_conn_2.scale.z = 0.01;
    point_1.x = room_x_coord[7];
    point_1.y = room_y_coord[7];
    point_1.z = 7;
    room_327_conn_2.points.push_back(point_1);
    point_2.x = walls_x_coord[13];
    point_2.y = wall_y_coord[13];
    point_2.z = 5;
    room_327_conn_2.points.push_back(point_2);
    markers.markers.push_back(room_327_conn_2);

    visualization_msgs::Marker room_327_conn_3;
    room_327_conn_3.header.frame_id = "map";
    room_327_conn_3.header.stamp = stamp;
    room_327_conn_3.ns = "room_327_conn_3";
    room_327_conn_3.id = markers.markers.size();
    room_327_conn_3.type = visualization_msgs::Marker::LINE_LIST;
    room_327_conn_3.color.r=1.0f;
    room_327_conn_3.color.g=1.0f;
    room_327_conn_3.color.b=1.0f;
    room_327_conn_3.color.a=1.0f;
    room_327_conn_3.scale.x = room_327_conn_3.scale.y = room_327_conn_3.scale.z = 0.01;
    point_1.x = room_x_coord[7];
    point_1.y = room_y_coord[7];
    point_1.z = 7;
    room_327_conn_3.points.push_back(point_1);
    point_2.x = walls_x_coord[5];
    point_2.y = wall_y_coord[5];
    point_2.z = 5;
    room_327_conn_3.points.push_back(point_2);
    markers.markers.push_back(room_327_conn_3);

    visualization_msgs::Marker room_327_conn_4;
    room_327_conn_4.header.frame_id = "map";
    room_327_conn_4.header.stamp = stamp;
    room_327_conn_4.ns = "room_327_conn_4";
    room_327_conn_4.id = markers.markers.size();
    room_327_conn_4.type = visualization_msgs::Marker::LINE_LIST;
    room_327_conn_4.color.r=1.0f;
    room_327_conn_4.color.g=1.0f;
    room_327_conn_4.color.b=1.0f;
    room_327_conn_4.color.a=1.0f;
    room_327_conn_4.scale.x = room_327_conn_4.scale.y = room_327_conn_4.scale.z = 0.01;
    point_1.x = room_x_coord[7];
    point_1.y = room_y_coord[7];
    point_1.z = 7;
    room_327_conn_4.points.push_back(point_1);
    point_2.x = walls_x_coord[4];
    point_2.y = wall_y_coord[4];
    point_2.z = 5;
    room_327_conn_4.points.push_back(point_2);
    markers.markers.push_back(room_327_conn_4);

    






  return markers;
}

  std::vector<std::pair<std::string, std::vector<double>>> read_csv(std::string filename){
    // Reads a CSV file into a vector of <string, vector<int>> pairs where
    // each pair represents <column name, column values>

    // Create a vector of <string, int vector> pairs to store the result
    std::vector<std::pair<std::string, std::vector<double>>> result;

    // Create an input filestream
    std::ifstream myFile(filename);

    // Make sure the file is open
    if(!myFile.is_open()) throw std::runtime_error("Could not open file");

    // Helper vars
    std::string line, colname;
    double val;

    // Read the column names
    if(myFile.good())
    {
        // Extract the first line in the file
        std::getline(myFile, line);

        // Create a stringstream from line
        std::stringstream ss(line);

        // Extract each column name
        while(std::getline(ss, colname, ',')){
            
            // Initialize and add <colname, int vector> pairs to result
            result.push_back({colname, std::vector<double> {}});
        }
    }
     while(std::getline(myFile, line))
    {
        // Create a stringstream of the current line
        std::stringstream ss(line);
        
        // Keep track of the current column index
        int colIdx = 0;
        
        // Extract each integer
        while(ss >> val){
            
            // Add the current integer to the 'colIdx' column's values vector
            result.at(colIdx).second.push_back(val);
            //std::cout<<val<<std::endl;
            
            // If the next token is a comma, ignore it and move on
            if(ss.peek() == ',') ss.ignore();
            
            // Increment the column index
            colIdx++;
        }
    }

    // Close file
    myFile.close();

    return result;
}



int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher markers_pub = n.advertise<visualization_msgs::MarkerArray>("/Stugalux/scene_graph", 16);

  // Read csv file containing coordinates of walls
  std::string filename = "/home/msh/test_algorithms/src/using_markers/csv/edited_2ND_FLOOR.csv";
  std::vector<std::pair<std::string, std::vector<double>>> column_values = read_csv(filename);
  for (auto it=column_values.begin();it != column_values.end(); it++){
    auto pClass1 = it->first;
    auto pClass2 = it->second;
    std::cout<<pClass1<<std::endl;
  }
  auto wall_x_col = column_values[1];
  std::cout<<wall_x_col.first<<std::endl;
  std::vector<double> walls_x_coord = wall_x_col.second;
  auto wall_y_col = column_values[2];
  std::vector<double> wall_y_coord = wall_y_col.second;

  auto room_x_col = column_values[4];
  std::cout<<room_x_col.first<<std::endl;
  std::vector<double> room_x_coord = room_x_col.second;
  auto room_y_col = column_values[5];
  std::vector<double> room_y_coord = room_y_col.second;



  
  


auto markers = create_marker_array(ros::Time::now(), walls_x_coord, wall_y_coord, room_x_coord, room_y_coord);
  while (ros::ok())
  {
    while (markers_pub.getNumSubscribers() < 1)
      {
        if (!ros::ok())
         {
           return 0;
         }
         ROS_WARN_ONCE("Please create a subscriber to the marker");
         sleep(1);
       }
    markers_pub.publish(markers);
    r.sleep();
  }

}


