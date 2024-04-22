## Create KNN model
Save KNN model

## Detect walls
Use Lidar to detect the distance between wall and robots

## Navigation
- Wall in range: if detect wall front, then turn to a feasible range with wall and find the sign
- Follow the sign: classify the sign and get right action of robot


## Steps
1.1 Test which points are best for identify signs
1.2 Use Lidar to predict the distance of wall in front of robots

## TODO:
1. SLAM and save the map
2. Find the best points that can identify signs
3. Writing down pose logic sequence, optionally using dist2wall to double check the identify process
4. Debug, find the point that may fail to work

## Workflow
1. launch robot_camera.launch
2. run detect sign
3. Open rviz and map
4. Choose start point in scripts (face front) -> face to robots
5. run nav2goal
