#####    
#####               ===================================================
#####             '    ..                                         ..    `
#####            |    (  )         ZED Camera Ros wrapper        (  )    |
#####             .                                                     ,
#####               =================================================== 
#####    
#####    
#####    
#####   This sample is a wrapper for the ZED library in order to use the ZED Camera with ROS.
#####   You can publish Left+Depth or Left+Right images and camera info on the topics of your choice.
#####    
#####   A set of parameters can be specified in the launch file. Two launch files are provided in the
#####   launch directory:
#####       - zed_depth.launch: publish left and depth images with their camera info.
#####       - zed_stereo.launch: publish left and right images with their camera info.
#####    
#####   The zed_depth_stereo_wrapper is a catkin package made to run on ROS Indigo, and depends
#####   on the following ROS packages:
#####       -roscpp
#####       -rosconsole
#####       -sensor_msgs
#####       -opencv2
#####       -cv_bridge
#####       -image_transport
#####    
#####   =====================
#####   * Build the program *
#####   =====================
#####    
#####       Place the package folder "zed_wrapper" in your catkin workspace source folder "~/catkin_ws/src"
#####    
#####       Open a terminal :
#####           $ cd ~/catkin_ws
#####           $ catkin_make
#####           $ source ./devel/setup.bash
#####    
#####    
#####   ===================
#####   * Run the program *
#####   ===================
#####    
#####       Open a terminal : 
#####           $ roslaunch zed_wrapper zed_depth.launch
#####    
#####    
#####   ==========================
#####   * Launch file parameters *
#####   ==========================
#####    
#####    ____________________________________________________________________________________________
#####   |                       |                                 |                                  |
#####   |       Parameter       |           Description           |              Value               |
#####   |=======================|=================================|==================================|
#####   | computeDepth          | Toggle depth computation.       | '0': depth not computed.         |
#####   |                       |                                 |      Left+Right images published |
#####   |                       |                                 | '1': depth computed.             |
#####   |                       |                                 |      Left+Depth images published |
#####   |_______________________|_________________________________|__________________________________|
#####   | svo_file              | SVO filename                    | path to an SVO file              |
#####   |_______________________|_________________________________|__________________________________|
#####   | resolution            | ZED Camera resolution           | '0': HD2K                        |
#####   |                       |                                 | '1': HD1080                      |
#####   |                       |                                 | '2': HD720                       |
#####   |                       |                                 | '3': VGA                         |
#####   |_______________________|_________________________________|__________________________________|
#####   | quality               | Disparity Map quality           | '1': PERFORMANCE                 |
#####   |                       |                                 | '2': QUALITY                     |
#####   |_______________________|_________________________________|__________________________________|
#####   | sensing_mode          | Depth sensing mode              | '0': FULL                        |
#####   |                       |                                 | '1': RAW                         |
#####   |_______________________|_________________________________|__________________________________|
#####   | frame_rate            | Rate at which images are        | int                              |
#####   |                       | published                       |                                  |
#####   |_______________________|_________________________________|__________________________________|
#####   | left_topic            | Topic to which left images      | string                           |
#####   |                       | are published                   |                                  |
#####   |_______________________|_________________________________|__________________________________|
#####   | second_topic          | Topic to which depth or right   | string                           |
#####   |                       | images are published            |                                  |
#####   |_______________________|_________________________________|__________________________________|
#####   | left_cam_info_topic   | Topic to which left camera info | string                           |
#####   |                       | are published                   |                                  |
#####   |_______________________|_________________________________|__________________________________|
#####   | second_cam_info_topic | Topic to which right or depth   | string                           |
#####   |                       | camera info are published       |                                  |
#####   |_______________________|_________________________________|__________________________________|
#####   | left_frame_id         | ID specified in the left image  | string                           |
#####   |                       | message header                  |                                  |
#####   |_______________________|_________________________________|__________________________________|
#####   | second_frame_id       | ID specified in the depth or    | string                           |
#####   |                       | right image message header      |                                  |
#####   |_______________________|_________________________________|__________________________________|
##### 
##### 
##### 
#####
#####                          ######### ######### ######## 
###################                   #  #         #       #            ###################
###################                #     # ######  #       #            ###################
###################             #        #         #       #            ###################
                               #########  ######## ########