import rospy
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose


def sort_points(points):
    sorted_points = sorted(points, key=lambda p: (p[0], p[1]))
    return sorted_points


def spawn_wall(x, y, z, length, is_horizontal, index):
    model_name = "wall_{}".format(index)

    if is_horizontal:
        model_xml = """<sdf version="1.6">
                          <model name="{}">
                            <static>true</static>
                            <link name="link">
                              <collision name="collision">
                                <geometry>
                                  <box>
                                    <size>0.005 0.06 0.01</size>
                                  </box>
                                </geometry>
                              </collision>
                              <visual name="visual">
                                <geometry>
                                  <box>
                                    <size>0.005 0.086 0.01</size>
                                  </box>
                                </geometry>
                                <material>
                                  <shader type='pixel'>
                                    <normal_map>__default__</normal_map>
                                  </shader>
                                  <script>
                                    <name>ModelPreview_5::link::visual_MATERIAL_</name>
                                  </script>
                                  <ambient>1 0 0 0</ambient>
                                  <diffuse>1 0 0 1</diffuse>
                                  <specular>1 0 0 1</specular>
                                  <emissive>1 0 0 1</emissive> 
                                </material>
                              </visual>
                            </link>
                          </model>
                        </sdf>""".format(model_name, x, y, z, length)
    else:
        model_xml = """<sdf version="1.6">
                          <model name="{}">
                            <static>true</static>
                            <link name="link">
                              <collision name="collision">
                                <geometry>
                                  <box>
                                    <size>0.030 0.005 0.01</size>
                                  </box>
                                </geometry>i 
                              </collision>
                              <visual name="visual">
                                <geometry>
                                  <box>
                                    <size>0.030 0.005 0.01</size>
                                  </box>
                                </geometry>
                                <material>
                                  <shader type='pixel'>
                                    <normal_map>__default__</normal_map>
                                  </shader>
                                  <script>
                                    <name>ModelPreview_5::link::visual_MATERIAL_</name>
                                  </script>
                                  <ambient>1 0 0 0</ambient>
                                  <diffuse>1 0 0 1</diffuse>
                                  <specular>1 0 0 1</specular>
                                  <emissive>1 0 0 1</emissive> 
                                </material>
                              </visual>
                            </link>
                          </model>
                        </sdf>""".format(model_name, x, y, z, length)

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    try:
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        spawn_model(model_name, model_xml, " ", pose, "world")
        rospy.loginfo("Spawned wall at x={}, y={}, z={}, length={}, is_horizontal={}".format(x, y, z, length, is_horizontal))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))
        
def delete_model(model_name):
    try:
        delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        delete_model(model_name)
        rospy.loginfo("Deleted model: {}".format(model_name))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))       


def main():
    rospy.init_node("spawn_walls_node")
    global i 
    i=0
     
    file_path = "coordinate_punti.txt"
    if not os.path.exists(file_path):
        rospy.logerr("File 'coordinate_punti.txt' not found.")
        return

    with open(file_path, "r") as file:
        lines = file.readlines()

    coordinates = [list(map(float, line.split())) for line in lines]
    coordinates = sort_points(coordinates)
    
    for k in range(len(coordinates)):
        coordinates[k][2] = 1.049
      

    for j in range(0, len(coordinates)-1, 2):
        x1, y1, z1 = coordinates[j]
        x2, y2, z2 = coordinates[j + 1]

        if abs(x1-x2)<=0.5:
            spawn_wall(x1, (y1+y2)/2, z1, abs(y2-y1), True, i)
            i=i+1
        elif y1==y2:
            spawn_wall((x1+x2)/2, y1, z2, abs(x2-x1), False, i)
            i=i+1

    for j in range(0, len(coordinates)-2, 1):
        x1, y1, z1 = coordinates[j]
        x2, y2, z2 = coordinates[j+2]

        if x1==x2:
            spawn_wall(x1, (y1+y2)/2, z1, abs(y2-y1), True, i)
            i=i+1
        elif abs(y1-y2)<=0.5:
            spawn_wall((x1+x2)/2, y1, z2, abs(x2-x1), False, i)
            i=i+1
    
    model_name_to_delete= "wall_6"
    model_name_to_delete_1= "wall_7"
    
    delete_model(model_name_to_delete)
    delete_model(model_name_to_delete_1)
    
    rospy.spin()

if __name__ == "__main__":
    main()