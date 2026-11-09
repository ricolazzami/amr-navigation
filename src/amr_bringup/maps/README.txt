maze.pgm / maze.yaml are generated from worlds/maze.sdf (do not hand-edit):

    python3 scripts/make_map.py

The map is authored in the world frame (origin -5.5, -5.5, 11 m at 0.05 m), so
world (4, 4) is map (4, 4) and the goal the README quotes needs no offset.
demo.launch.py loads it with nav2 map_server + AMCL when slam:=false (default).

To capture a map from live SLAM instead:

    ros2 launch amr_bringup demo.launch.py slam:=true
    # drive around, then
    ros2 run nav2_map_server map_saver_cli -f maps/slam_maze
