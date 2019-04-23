# duckie_shells
Programmable shells for students of RET site

Implementation of the Motion Node

------------------------------------------------------------------------
----------------------- C++ Implementation -----------------------------
------------------------------------------------------------------------

Instructions on how to run:

1. Since there will be multiple cars in the duckietown implementation
a parameter: 'car_number' must be specified upon launch to match with
the correct vicon object (used in conjunction with vicon_track_multiple)
  - Set up private parameter of 'car_number' in launch file

2. To set up the graph of segmented places where the duckiebot will travel
to in the duckietown, function -graph_initialize()- must be manually altered.

  - To define verticies in the graph, a vertex corresponds to every beginning/end
    of an intersection AND turn. For example, a simple 90 degree turn tile in 
    duckietown would have two verticies: one at the beginning of the turn and 
    one at the end. 
  
  - To specify the graph size, alter the graph_size variable
  
  - To insert a vertex into the graph, use... 
    graph_add_vertex(int vertex_index, float x_val, float y_val)
    to add a vertex with (x_val, y_val) coordinates, corresponding to an
    index of vertex_index in the adjacency matrix.
    
  - To connect these vertices, essentially to simulate the roads, use the function:
    graph_add_edge(int vi1, int vi2)
    Where vi1 and vi2 correspond to vertex indicies in the adjacency matrix

3. The parameters for controlling the velocities also might need to be adjusted.
This must be done in the function localizationHelper(int parent[], int u).

- The following are parameters that may need to be tweaked to gain optimal results
    /// control what area around the desired point where the duckiebot should stop
	  float diff_threshold = 0.1;
	  /// how powerful the linear velocity should react
	  float v_gain = 1.0;
	  /// how powerful the angular velocity should react
	  float omega_gain = 1.0;
	  /// parameters based on the distance between the points on a turn/intersection tile
	  /// These need to actually be measured based on the duckietown turn/intersection tiles
	  float x_turn_threshold = 0.1;
    float y_turn_threshold = 0.1;

4. To make this work properly, current_pose from vicon tracking must yield correct information.
The trajectory geometry_msgs::Point messages from the QT interface must also be correct in 
correspondance with the graph initialization.

5. To suppress the printing of each step during the algorithm, 
edit the function: localization(geometry_msgs::Point desired_point).
Commment the code between the PRINTING FOR DEBUGGING sections.

6. Possible Errors:
  - While loop in localizationHelper(...) has potential of getting stuck in infinite loop
    if not the diff_threshold parameter is not initialized properly.
  - Have not tested the algorithm on the actual duckiebot
  - Have not tested in conjunction with the lane_detection node
  - Very Sensitive to input geometry_msgs::Point from QTInterface


****** Example Of Use ******* 

...
<!-- In Launch File -->
<node name="motion_node_car1" pkg="duckie_shells" type="motion_node" output="screen">
		<param name="car_number" type="int" value="1"/> 
</node>
...
// An example of a graph_intialization implementation:
void VelocityConverter::graph_initialize() {
	 graph_size = 3;
	 /// clear graph first
	 for(int i = 0; i < graph_size; i++) {
	    for(int j = 0; j < graph_size; j++) {
			    graph[i][j] = 0;
		  }
	 }
	 graph_add_vertex(0, 0.0, 0.0);
	 graph_add_vertex(1, 4.6, 3.4);
	 graph_add_vertex(2, 0.4, 2.3);
	
	 graph_add_edge(0, 1);
	 graph_add_edge(1, 2);
	 graph_add_edge(0, 2);
}
...
In localizationHelper(...)
  float diff_threshold = how close you want the duckiebot to get to the vertices;
	float v_gain = how fast you want the duckiebot to go;
	float omega_gain = how sharp you want the duckiebot to turn;
	float x_turn_threshold = differences of vertex x's on tile/intersection tile;
  float y_turn_threshold = differences of vertex y's on tile/intersection tile;
...
TODO: Develop QT interface to publish to 
 - /trajectory/car1
...
roslaunch duckie_shells *your launch file name here*

------------------------------------------------------------------------
------------------------------------------------------------------------
------------------------------------------------------------------------
