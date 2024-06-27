In this project i used three nodes.

turtle_spawner : This node is responsible for spawning turtles in turtlesim at regular intervals.
  I defined interface turtle_interfaces::msg::TurtleArray and uses it to provide the turtle_controller with information about the existing turtles.

turtle_controller : This is the node that sets which turtle1 should go to catch the turtles created in turtlesim and how it should go to catch them.
  It catches turtles based on the turtle information provided to turtle_spawner. If the parameter catch_closet_turtle_first is set to true, it catches the closest turtle first.
  For turtle1 to move, i continue to publish the /turtle1/cmd_vel topic to turtlesim, and turtlesim publishes /turtle1/pose.

turtlesim : A node that already exists in ros2 and shows how turtle behaves.

This picture is rqt_graph.
![1](https://github.com/ppangTae/turtlesim_catch_them_all/assets/92343537/83b400aa-7529-4afd-b0e0-3b00775de149)

This is a picture of the program running.
![2](https://github.com/ppangTae/turtlesim_catch_them_all/assets/92343537/099f820b-0208-4184-8f67-1b437dafa2fb)
