#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>


visualization_msgs::MarkerArray create_marker_array(const ros::Time& stamp)  {
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker mesh_marker;
    mesh_marker.header.frame_id = "map";
    mesh_marker.header.stamp = stamp;
    mesh_marker.ns = "mesh";
    mesh_marker.id = markers.markers.size();
    mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh_marker.mesh_resource = "package://using_markers/meshes/stugalux_house_floor_1.dae";

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

    Room_1.pose.position.x = -1.7;
    Room_1.pose.position.y = -3.0;
    Room_1.pose.position.z = 5.0;
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

    room_1_name_marker.pose.position.x = -1.7;
    room_1_name_marker.pose.position.y = -3.0;
    room_1_name_marker.pose.position.z = 5.4;
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


// room 2
    visualization_msgs::Marker Room_2;
    Room_2.header.frame_id = "map";
    Room_2.header.stamp = stamp;
    Room_2.ns = "room_2";
    Room_2.id = markers.markers.size();
    Room_2.type = visualization_msgs::Marker::CUBE;

    Room_2.pose.position.x = -0.6;
    Room_2.pose.position.y = 0.4;
    Room_2.pose.position.z = 5.0;
    Room_2.pose.orientation.x = 0.0;
    Room_2.pose.orientation.y = 0.0;
    Room_2.pose.orientation.z = 0.0;
    Room_2.pose.orientation.w = 1.0;
    Room_2.scale.x = 0.1;
    Room_2.scale.y = 0.7;
    Room_2.scale.z = 0.2;

    Room_2.color.r = 0.0f;
    Room_2.color.g = 1.0f;
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

    room_2_name_marker.pose.position.x = -0.8;
    room_2_name_marker.pose.position.y = 0.4;
    room_2_name_marker.pose.position.z = 5.4;
    room_2_name_marker.pose.orientation.x = 0.0;
    room_2_name_marker.pose.orientation.y = 0.0;
    room_2_name_marker.pose.orientation.z = 0.0;
    room_2_name_marker.pose.orientation.w = 1.0;
    room_2_name_marker.text = "Corridor";
    room_2_name_marker.scale.x = 0.4;
    room_2_name_marker.scale.y = 0.4;
    room_2_name_marker.scale.z = 0.4;

    room_2_name_marker.color.r = 1.0f;
    room_2_name_marker.color.g = 1.0f;
    room_2_name_marker.color.b = 1.0f;
    room_2_name_marker.color.a = 1.0;
    markers.markers.push_back(room_2_name_marker);



      // room 3
    visualization_msgs::Marker Room_3;
    Room_3.header.frame_id = "map";
    Room_3.header.stamp = stamp;
    Room_3.ns = "room_3";
    Room_3.id = markers.markers.size();
    Room_3.type = visualization_msgs::Marker::CUBE;

    Room_3.pose.position.x = 1.7;
    Room_3.pose.position.y = -3.0;
    Room_3.pose.position.z = 5.0;
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

    room_3_name_marker.pose.position.x = 1.7;
    room_3_name_marker.pose.position.y = -3.0;
    room_3_name_marker.pose.position.z = 5.4;
    room_3_name_marker.pose.orientation.x = 0.0;
    room_3_name_marker.pose.orientation.y = 0.0;
    room_3_name_marker.pose.orientation.z = 0.0;
    room_3_name_marker.pose.orientation.w = 1.0;
    room_3_name_marker.text = "Room 290";
    room_3_name_marker.scale.x = 0.4;
    room_3_name_marker.scale.y = 0.4;
    room_3_name_marker.scale.z = 0.4;

    room_3_name_marker.color.r = 1.0f;
    room_3_name_marker.color.g = 1.0f;
    room_3_name_marker.color.b = 1.0f;
    room_3_name_marker.color.a = 1.0;
    markers.markers.push_back(room_3_name_marker);


    // room 4
    visualization_msgs::Marker Room_4;
    Room_4.header.frame_id = "map";
    Room_4.header.stamp = stamp;
    Room_4.ns = "room_4";
    Room_4.id = markers.markers.size();
    Room_4.type = visualization_msgs::Marker::CUBE;

    Room_4.pose.position.x = 1.4;
    Room_4.pose.position.y = 2.5;
    Room_4.pose.position.z = 5.0;
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

    room_4_name_marker.pose.position.x = 1.4;
    room_4_name_marker.pose.position.y = 2.5;
    room_4_name_marker.pose.position.z = 5.4;
    room_4_name_marker.pose.orientation.x = 0.0;
    room_4_name_marker.pose.orientation.y = 0.0;
    room_4_name_marker.pose.orientation.z = 0.0;
    room_4_name_marker.pose.orientation.w = 1.0;
    room_4_name_marker.text = "Room 302";
    room_4_name_marker.scale.x = 0.4;
    room_4_name_marker.scale.y = 0.4;
    room_4_name_marker.scale.z = 0.4;

    room_4_name_marker.color.r = 1.0f;
    room_4_name_marker.color.g = 1.0f;
    room_4_name_marker.color.b = 1.0f;
    room_4_name_marker.color.a = 1.0;
    markers.markers.push_back(room_4_name_marker);


    // room 5
    visualization_msgs::Marker Room_5;
    Room_5.header.frame_id = "map";
    Room_5.header.stamp = stamp;
    Room_5.ns = "room_5";
    Room_5.id = markers.markers.size();
    Room_5.type = visualization_msgs::Marker::CUBE;

    Room_5.pose.position.x = 1.4;
    Room_5.pose.position.y = 5.5;
    Room_5.pose.position.z = 5.0;
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

    room_5_name_marker.pose.position.x = 1.4;
    room_5_name_marker.pose.position.y = 5.5;
    room_5_name_marker.pose.position.z = 5.4;
    room_5_name_marker.pose.orientation.x = 0.0;
    room_5_name_marker.pose.orientation.y = 0.0;
    room_5_name_marker.pose.orientation.z = 0.0;
    room_5_name_marker.pose.orientation.w = 1.0;
    room_5_name_marker.text = "Room 327";
    room_5_name_marker.scale.x = 0.4;
    room_5_name_marker.scale.y = 0.4;
    room_5_name_marker.scale.z = 0.4;

    room_5_name_marker.color.r = 1.0f;
    room_5_name_marker.color.g = 1.0f;
    room_5_name_marker.color.b = 1.0f;
    room_5_name_marker.color.a = 1.0;
    markers.markers.push_back(room_5_name_marker);


        // room 6
    visualization_msgs::Marker Room_6;
    Room_6.header.frame_id = "map";
    Room_6.header.stamp = stamp;
    Room_6.ns = "room_6";
    Room_6.id = markers.markers.size();
    Room_6.type = visualization_msgs::Marker::CUBE;

    Room_6.pose.position.x = 1.4;
    Room_6.pose.position.y = .5;
    Room_6.pose.position.z = 5.0;
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

    room_6_name_marker.pose.position.x = 1.4;
    room_6_name_marker.pose.position.y = 7.5;
    room_6_name_marker.pose.position.z = 5.4;
    room_6_name_marker.pose.orientation.x = 0.0;
    room_6_name_marker.pose.orientation.y = 0.0;
    room_6_name_marker.pose.orientation.z = 0.0;
    room_6_name_marker.pose.orientation.w = 1.0;
    room_6_name_marker.text = "Room 310";
    room_6_name_marker.scale.x = 0.4;
    room_6_name_marker.scale.y = 0.4;
    room_6_name_marker.scale.z = 0.4;
    room_6_name_marker.color.r = 1.0f;
    room_6_name_marker.color.g = 1.0f;
    room_6_name_marker.color.b = 1.0f;
    room_6_name_marker.color.a = 1.0;
    markers.markers.push_back(room_6_name_marker);

    
            // room 7
    visualization_msgs::Marker Room_7;
    Room_7.header.frame_id = "map";
    Room_7.header.stamp = stamp;
    Room_7.ns = "room_7";
    Room_7.id = markers.markers.size();
    Room_7.type = visualization_msgs::Marker::CUBE;

    Room_7.pose.position.x = -1.4;
    Room_7.pose.position.y = 5.5;
    Room_7.pose.position.z = 5.0;
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

    room_7_name_marker.pose.position.x = -1.4;
    room_7_name_marker.pose.position.y = 5.5;
    room_7_name_marker.pose.position.z = 5.4;
    room_7_name_marker.pose.orientation.x = 0.0;
    room_7_name_marker.pose.orientation.y = 0.0;
    room_7_name_marker.pose.orientation.z = 0.0;
    room_7_name_marker.pose.orientation.w = 1.0;
    room_7_name_marker.text = "Room 310";
    room_7_name_marker.scale.x = 0.4;
    room_7_name_marker.scale.y = 0.4;
    room_7_name_marker.scale.z = 0.4;
    room_7_name_marker.color.r = 1.0f;
    room_7_name_marker.color.g = 1.0f;
    room_7_name_marker.color.b = 1.0f;
    room_7_name_marker.color.a = 1.0;
    markers.markers.push_back(room_7_name_marker);





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
  auto x_col = column_values[1];
  std::cout<<x_col.first<<std::endl;
  std::vector<double> x_coord = x_col.second;
  auto y_col = column_values[2];
  std::vector<double> y_coord = y_col.second;

  
  


auto markers = create_marker_array(ros::Time::now());
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


