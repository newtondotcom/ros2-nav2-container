**Next step :**

- Write a Twist Multiplexer: Priority-based input selection ? handled by PX4 ?

- Define logic for docker emergenc takeover etc

- Add a cloud control using [Nvidia Isaac Ros Cloud Controler](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cloud_control/tree/c7234a18b022c4c6bac73725a0225e8734535127) which is a [VDA5050](https://www.vda.de/en/topics/automotive-industry/vda-5050)-compatible mission controller 

- External web link to give orders that will be executed by the dock running ROS

- Wifi stack en rust -> MavLink over UDP :
    * https://github.com/svpcom/wfb-ng-osd
    * https://github.com/svpcom/wfb-ng
    * https://github.com/mavlink/rust-mavlink
    * https://github.com/mavlink/mavlink2rest
    * https://github.com/mavlink/mavlink-camera-manager