# Lab 1: Intro to ROS 2

## Written Questions

### Q1: During this assignment, you've probably ran these two following commands at some point: ```source /opt/ros/foxy/setup.bash``` and ```source install/local_setup.bash```. Functionally what is the difference between the two?

Answer: The first one, source /opt/ros/foxy/setup.bash, sources standard ros functionality, such as basic ros commands (like ros node info) that wouldn’t otherwise be recognizable to a Unix terminal; This is generally the lowest level underlay The latter, source install/local_setup.bash, sources your custom workspace which you previously built, probably using colcon. This is the overlay which superceded the underlay. Note that the overlay overrides the underlay in the event of conflicts, and the underlay must provide all dependencies demanded from the overlay

### Q2: What does the ```queue_size``` argument control when creating a subscriber or a publisher? How does different ```queue_size``` affect how messages are handled?

Answer: Queue_size controls the number of queued messages that can pile up in the event that a publisher is not sending them fast enough or a subscriber is not receiving them fast enough, and this is a QoS (quality of service) setting. The existence of a queue gives the system some time to buffer as data streams in. A longer queue_size is less likely to miss critical information since processing can run its course while large amounts of data patiently wait in the queue; however this is computationally less efficient due to high storage requirements. It takes up less space to limit your queze_size and simply remove the oldest items in the queue, and reduce buffer.

### Q3: Do you have to call ```colcon build``` again after you've changed a launch file in your package? (Hint: consider two cases: calling ```ros2 launch``` in the directory where the launch file is, and calling it when the launch file is installed with the package.)

Answer: In the first case (i.e. if you’re in the directory with the launch file itself) then you do not need to call colcon build again, since ros2 launch can simply find it and execute it directly. However in the second case (i.e. if you’re calling the launch file after having initially installed it with the package), then you must call colcon build again to translate the launch into an executable and ensure it’s located in the “install” directory.
