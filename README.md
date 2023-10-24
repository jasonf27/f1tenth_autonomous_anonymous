# f1tenth_autonomous_anonymous
Lab submissions for Team 4 of F1Tenth, the Spring 2023 iteration of Rahul Mangharam's ESE-6150 autonomous racing course at the University of Pennsylvania. I was lucky enough to work with three fellow UPenn ROBO grad students: Irene Grace-Kolsen, Yu-Chia Shen, and Zhuolon (Alan) Zhao, and together we formed the team "Autonomous Anonymous". We spent the semester writing and testing software for a small (~1 foot long) autonomous car, and ultimately competed in three timed races.

For more information, check out [my personal portfolio website entry](https://www.jason-friedman.me/projects/f1tenth) where I elaborate on our various racing algorithms. Our initial objective was mastering ROS2, running it within an Ubuntu/Linux Virtual Machine, and managing/using it through direct terminal bash commands. Then we implemented Automatic Emergency Braking (AEB) as a safety protocol moving forward. Next for path planning/following and controls, we implemented RRT*, pure pursuit, MPC, Follow the Gap, and Wall Following, and used different approaches across the three races. On the perception side, we localized the car using a particle filter, and mapped out both required racetracks using the slam_toolbox package; these map-based methods covered both Hokuyo LiDAR and RealSense camera sensory input, and strengthened our performance in the latter two races.

