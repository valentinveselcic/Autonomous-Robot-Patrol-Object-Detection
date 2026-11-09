ros2 launch amr_lv3 all_systems.launch.py - za pokretanje rviza i gazeboa i spawnanje zutih objekata
ros2 launch turtlebot4_navigation localization.launch.py map:=/home/len/Len/ros2_lvs/amr_lv3/src/maps/maps.yaml - lokalizacija
ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true - nav2 

<!-- ros2 run amr_lv3 click_logger.py - pohranjivanje 5 tocaka odabranih u rvizu -->
ros2 run amr_lv3 yellow_object_detector - pokretanje node-a za detektiranje centra objekta i projekciju u 3D prostor
ros2 run amr_lv3 waypoint_follower - pokretanje navigacije kroz prethodno zadane tocke uz zaustavljanje pri detekciji zutih objektata