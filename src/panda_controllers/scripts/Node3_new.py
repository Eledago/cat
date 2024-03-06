######------NODO CHE SPAWNA I MURI VIRTUALI----########

import rospy
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose

#Funzione che riordina le coordinate dei vertici 
def sort_points(points):
    sorted_points = sorted(points, key=lambda p: (p[0], p[1]))
    return sorted_points

#Creazione del file sdf per i muri virtuali (prima orizzontali e poi verticali)
def spawn_wall(x, y, z, length, is_horizontal, index):
    model_name = "wall_{}".format(index)

    if is_horizontal:
        model_xml = """<sdf version="1.6">
                          <model name="{}">
                            <static>true</static>
                            <link name="link">
                                <geometry>
                                  <box>
                                    <size>0.005 0.086 0.05</size>
                                  </box>
                                </geometry>
                              <visual name="visual">
                                <geometry>
                                  <box>
                                    <size>0.005 0.086 0.05</size>
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
                                <geometry>
                                  <box>
                                    <size>0.030 0.005 0.05</size>
                                  </box>
                                </geometry> 
                              <visual name="visual">
                                <geometry>
                                  <box>
                                    <size>0.030 0.005 0.05</size>
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

#Eliminazione dei muri che non servono (wall 6 e 7)      
def delete_model(model_name):
    try:
        delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        delete_model(model_name)
        rospy.loginfo("Deleted model: {}".format(model_name))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))       


def main():
   #Sleep per poter lanciare il nodo insieme agli altri nel file di launch
    rospy.sleep(15)
    rospy.init_node("spawn_walls_node")
    global i 
    i=0
     
    file_path = "/home/eleonora/catkin_ws/src/coordinate_punti_1.txt"
    if not os.path.exists(file_path):
        rospy.logerr("File 'coordinate_punti_1.txt' not found.")
        return

    with open(file_path, "r") as file:
        lines = file.readlines()

    coordinates = [list(map(float, line.split())) for line in lines]
    coordinates = sort_points(coordinates)
    
    #Settare le coordinate z di tutti i punti a 1.069
    for k in range(len(coordinates)):
        coordinates[k][2] = 1.069
      
    #Logica per creazione dei muri verticali, for che scorre i vettore delle coordinate dei vertici con passo 2 
    for j in range(0, len(coordinates)-1, 2):
        x1, y1, z1 = coordinates[j]
        x2, y2, z2 = coordinates[j + 1]
        
        #Se la differenza delle coordinate in x è minore della soglia spawna un muro orizzontale
        if abs(x1-x2)<=0.5:
            spawn_wall(x1, (y1+y2)/2, z1, abs(y2-y1), True, i)
            i=i+1

            
    #Logica per creazione dei muri orizzontali, for che scorre i vettore delle coordinate dei vertici con passo 1  
    for j in range(0, len(coordinates)-2, 1):
        x1, y1, z1 = coordinates[j]
        x2, y2, z2 = coordinates[j+2]

       #Se la differenza delle coordinate in y è minore della soglia spawna un muro verticale
        if abs(y1-y2)<=0.5:
            spawn_wall((x1+x2)/2, y1, z2, abs(x2-x1), False, i)
            i=i+1
    
    model_name_to_delete= "wall_6"
    model_name_to_delete_1= "wall_7"
    
    delete_model(model_name_to_delete)
    delete_model(model_name_to_delete_1)
    
    rospy.spin()

if __name__ == "__main__":
    main()