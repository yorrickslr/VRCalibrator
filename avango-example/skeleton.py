import avango
import avango.script
import avango.daemon
import avango.gua

import math

class Joint:
    ###contructor
    def __init__(self,
        PARENT_JOINT = None,
        ID = None,
        LOADER = None,
        PARENT_NODE = None
        ):

        if ID == None:
            print("ERROR: Joint is missing ID!")
            exit()
        if PARENT_NODE == None:
            print("ERROR: Joint is missing PARENT_NODE")
            quit()
        if PARENT_JOINT == None:
            print("ERROR: Joint is missing PARENT_JOINT")
            exit()
        if LOADER == None:
            LOADER = avango.gua.nodes.TriMeshLoader()

        self.PARENT_NODE = PARENT_NODE
        self.PARENT_JOINT = PARENT_JOINT
        self.ID = ID
        self.LOADER = LOADER

        ### parameters ###
        self.joint_thickness = 0.06
        self.link_thickness = 0.03 # in meter

        self.joint_color = avango.gua.Color(1.0,0.0,0.0)
        self.link_color = avango.gua.Color(1.0,1.0,1.0)


        ### resources ###

        self.sensor = avango.daemon.nodes.DeviceSensor(
            DeviceService=avango.daemon.DeviceService(),
            Station="kinect-joint-{0}".format(str(ID))
            )

        self.joint_geometry = LOADER.create_geometry_from_file(
            "joint_geometry"+str(self.ID), "data/objects/sphere.obj",
            avango.gua.LoaderFlags.DEFAULTS)
        self.joint_geometry.Transform.value = avango.gua.make_scale_mat(self.joint_thickness)
        self.joint_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(self.joint_color.r, self.joint_color.g, self.joint_color.b, 1.0))

        self.joint_transform = avango.gua.nodes.TransformNode(Name = "joint_transform"+str(ID))
        self.joint_transform.Children.value = [self.joint_geometry]
        self.joint_transform.Transform.connect_from(self.sensor.Matrix)
        self.PARENT_NODE.Children.value.append(self.joint_transform)


        self.link = LOADER.create_geometry_from_file(
            "link{0}".format(str(self.ID)), "data/objects/cylinder.obj",
            avango.gua.LoaderFlags.DEFAULTS)
        self.link.Material.value.set_uniform("Color", avango.gua.Vec4(self.link_color.r, self.link_color.g, self.link_color.b, 1.0))
        self.PARENT_NODE.Children.value.append(self.link)


        self.frame_trigger = avango.script.nodes.Update(Callback = self.frame_callback, Active = True)

    def frame_callback(self):
        _pos0 = self.joint_transform.Transform.value.get_translate()
        _pos1 = self.PARENT_JOINT.joint_transform.Transform.value.get_translate()

        _vec = _pos1 - _pos0
        _distance = _vec.length()

        _center = _pos0.lerp_to(_pos1, 0.5)

        self.link.Transform.value = \
            avango.gua.make_trans_mat(_center) * \
            self.get_rotation_between_vectors(avango.gua.Vec3(1.0,0.0,0.0), _vec) * \
            avango.gua.make_scale_mat(_distance, self.link_thickness, self.link_thickness)


    ### functions ###
    def get_rotation_between_vectors(self, VEC1, VEC2):
        if VEC1.length() == 0.0 or VEC2.length() == 0.0:
            return avango.gua.make_identity_mat()

        VEC1.normalize()
        VEC2.normalize()

        _angle = math.degrees(math.acos(min(max(VEC1.dot(VEC2), -1.0), 1.0)))
        _axis = VEC1.cross(VEC2)

        return avango.gua.make_rot_mat(_angle, _axis)



class HumanSkeleton:

    ### constructor
    def __init__(self,
        PARENT_NODE = None
        ):

        self.PARENT_NODE = PARENT_NODE


        ### parameters ###
        self.link_thickness = 0.03 # in meter

        _loader = avango.gua.nodes.TriMeshLoader()

        self.sensor_list = []
        self.links = []

        self.joints = []
        for _i in range(25):
            self.joints.append(Joint(ID=_i,
                PARENT_JOINT=_i,
                LOADER=_loader,
                PARENT_NODE=self.PARENT_NODE))
        self.joints[0].PARENT_JOINT = self.joints[0]
        self.joints[1].PARENT_JOINT = self.joints[0]
        self.joints[2].PARENT_JOINT = self.joints[20]
        self.joints[3].PARENT_JOINT = self.joints[2]
        self.joints[4].PARENT_JOINT = self.joints[20]
        self.joints[5].PARENT_JOINT = self.joints[4]
        self.joints[6].PARENT_JOINT = self.joints[5]
        self.joints[7].PARENT_JOINT = self.joints[6]
        self.joints[8].PARENT_JOINT = self.joints[20]
        self.joints[9].PARENT_JOINT = self.joints[8]
        self.joints[10].PARENT_JOINT = self.joints[9]
        self.joints[11].PARENT_JOINT = self.joints[10]
        self.joints[12].PARENT_JOINT = self.joints[0]
        self.joints[13].PARENT_JOINT = self.joints[12]
        self.joints[14].PARENT_JOINT = self.joints[13]
        self.joints[15].PARENT_JOINT = self.joints[14]
        self.joints[16].PARENT_JOINT = self.joints[0]
        self.joints[17].PARENT_JOINT = self.joints[16]
        self.joints[18].PARENT_JOINT = self.joints[17]
        self.joints[19].PARENT_JOINT = self.joints[18]
        self.joints[20].PARENT_JOINT = self.joints[1]
        self.joints[21].PARENT_JOINT = self.joints[7]
        self.joints[22].PARENT_JOINT = self.joints[6]
        self.joints[23].PARENT_JOINT = self.joints[11]
        self.joints[24].PARENT_JOINT = self.joints[10]
        #wundervoll

        ## trigger callbacks
        ## @var frame_trigger
        # Triggers framewise evaluation of respective callback method



