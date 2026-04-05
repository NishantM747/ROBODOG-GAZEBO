# Gazebo-assingment-

plugins and extra shit added: 

in rover.urdf: 
1) differential drive shi: 
    <plugin
    filename="libignition-gazebo-diff-drive-system.so"
    name="ignition::gazebo::systems::DiffDrive">
    <left_joint>wheel_joint_4</left_joint>
    <left_joint>wheel_joint_5</left_joint>
    <left_joint>wheel_joint_6</left_joint>
    <right_joint>wheel_joint_1</right_joint>
    <right_joint>wheel_joint_2</right_joint>
    <right_joint>wheel_joint_3</right_joint>
    <wheel_separation>1.2</wheel_separation>
    <wheel_radius>0.4</wheel_radius>
    <odom_publish_frequency>1</odom_publish_frequency>
    <topic>cmd_vel</topic>
    </plugin>

2) rgbd camera: 
<gazebo reference="camera_link">
    <sensor name="rgbd_camera" type="rgbd_camera">
      <gz_frame_id>camera_link</gz_frame_id>
      <topic>/rgbd</topic>
      <update_rate>1</update_rate>
      <always_on>1</always_on>
      <visualize>true</visualize>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>  <!-- depth cameras have shorter useful range -->
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
        <depth_camera>
          <output>depths</output>
        </depth_camera>
        <save enabled="false"/>
      </camera>
    </sensor>
  </gazebo>

3) Lidar: 
<sensor name='gpu_lidar' type='gpu_lidar'>"
    <pose relative_to='lidar_frame'>0 0 0 0 0 0</pose>
    <topic>lidar</topic>
    <update_rate>10</update_rate>
    <ray>
        <scan>
            <horizontal>
                <samples>640</samples>
                <resolution>1</resolution>
                <min_angle>-1.396263</min_angle>
                <max_angle>1.396263</max_angle>
            </horizontal>
            <vertical>
                <samples>1</samples>
                <resolution>0.01</resolution>
                <min_angle>0</min_angle>
                <max_angle>0</max_angle>
            </vertical>
        </scan>
        <range>
            <min>0.08</min>
            <max>10.0</max>
            <resolution>0.01</resolution>
        </range>
    </ray>
    <always_on>1</always_on>
    <visualize>true</visualize>
</sensor>

4) IMU: 
<sensor name="imu_sensor" type="imu">
    <always_on>1</always_on>
    <update_rate>1</update_rate>
    <visualize>true</visualize>
    <topic>imu</topic>
</sensor>

in sdf file: 
1) imu: 
    <plugin name='ignition::gazebo::systems::UserCommands' filename='ignition-gazebo-user-commands-system'/>
    <plugin filename="libignition-gazebo-imu-system.so"
        name="ignition::gazebo::systems::Imu">
    </plugin>

2) camera: 
        <gui fullscreen='0'>
      
      <camera name='user_camera'>
        <pose>3.19975 -2.0595 1.08073 0 0.275643 2.3562</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>

3) lidar: 
    #lidar
    <plugin
      filename="libignition-gazebo-sensors-system.so"
      name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
4) keyboard shi: 
    #frontkey
    <plugin filename="libignition-gazebo-triggered-publisher-system.so"
            name="ignition::gazebo::systems::TriggeredPublisher">
        <input type="ignition.msgs.Int32" topic="/keyboard/keypress">
            <match field="data">16777235</match>
        </input>
        <output type="ignition.msgs.Twist" topic="/cmd_vel">
            linear: {x: 5}, angular: {z: 5}
        </output>
    </plugin>
    #backkey 
    <plugin filename="libignition-gazebo-triggered-publisher-system.so"
            name="ignition::gazebo::systems::TriggeredPublisher">
        <input type="ignition.msgs.Int32" topic="/keyboard/keypress">
            <match field="data">16777237</match>
        </input>
        <output type="ignition.msgs.Twist" topic="/cmd_vel">
            linear: {x: -0.5}, angular: {z: 0.0}
        </output>
    </plugin>
    #leftkey?
    <plugin filename="libignition-gazebo-triggered-publisher-system.so"
            name="ignition::gazebo::systems::TriggeredPublisher">
        <input type="ignition.msgs.Int32" topic="/keyboard/keypress">
            <match field="data">16777236</match>
        </input>
        <output type="ignition.msgs.Twist" topic="/cmd_vel">
            linear: {x: 0.0}, angular: {z: -0.5}
        </output>
    </plugin>
    #rightkey?
    <plugin filename="libignition-gazebo-triggered-publisher-system.so"
            name="ignition::gazebo::systems::TriggeredPublisher">
        <input type="ignition.msgs.Int32" topic="/keyboard/keypress">
            <match field="data">16777234</match>
        </input>
        <output type="ignition.msgs.Twist" topic="/cmd_vel">
            linear: {x: 0.0}, angular: {z: 0.5}
        </output>
    </plugin>

5) pose publisher: 
            <plugin
            filename="gz-sim-pose-publisher-system"
            name="gz::sim::systems::PosePublisher">
            <publish_link_pose>true</publish_link_pose>
            <use_pose_vector_msg>true</use_pose_vector_msg>
            <static_publisher>true</static_publisher>
            <static_update_frequency>1</static_update_frequency>
            </plugin>
# ROBODOG-GAZEBO
