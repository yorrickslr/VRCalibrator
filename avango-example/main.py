import avango
import avango.script
import avango.daemon
import avango.gua
from examples_common.GuaVE import GuaVE

from skeleton import HumanSkeleton

import csv

# TODO
# spheres at origins

# 0, 0.9, 4.1: temp translation

global calibration;

def load_calibration():
	try:
	    with open('calibration.txt', 'r') as file:
	        reader = csv.reader(file, delimiter=" ", skipinitialspace=True)
	        csv_out = [[float(e) for e in r] for r in reader]
	except:
	    print("ERROR: cannot parse calibration file")

	calibration = avango.gua.Mat4()

	for i in range(4):
	    for k in range(4):
	        calibration.set_element(i,k,csv_out[i][k])

def calibrate(obj):
    try:
        with open('calibration.txt', 'r') as file:
            reader = csv.reader(file, delimiter=" ", skipinitialspace=True)
            csv_out = [[float(e) for e in r] for r in reader]
    except:
        print("ERROR: cannot parse calibration file")

    calibration = avango.gua.Mat4()

    for i in range(4):
        for k in range(4):
            calibration.set_element(i,k,csv_out[i][k])

    obj.Transform.value = calibration

def decalibrate(obj):
    obj.Transform.value = avango.gua.make_identity_mat()

side = 0
top = 1
def view(obj,mode):
    if mode==1:
        obj.Transform.value = avango.gua.make_rot_mat(90,1,0,0)
        obj.Transform.value *= avango.gua.make_trans_mat(0,-10,0)
        obj.Transform.value *= avango.gua.make_rot_mat(90,0,90,0)
    else:
        obj.Transform.value = avango.gua.make_trans_mat(0.0, 0.0, -10.0)

def start():

    # setup scenegraph
    graph = avango.gua.nodes.SceneGraph(Name="scenegraph")
    loader = avango.gua.nodes.TriMeshLoader()
    skeleton = avango.gua.nodes.TransformNode()
    human_skeleton = HumanSkeleton(Parent_Node = skeleton)

    hmd_service = avango.daemon.DeviceService()
    hmd_sensor = avango.daemon.nodes.DeviceSensor( \
        DeviceService = hmd_service)
    hmd_sensor.Station.value = "hmd-1"

    controller = loader.create_geometry_from_file(
        "controller", "data/objects/vive_controller.obj", \
        avango.gua.LoaderFlags.LOAD_MATERIALS)
    # controller.Material.value.set_uniform("Color",
    #     avango.gua.Vec4(1.0, 0.75, 0.75, 1.0))
    controller.Material.value.set_uniform("Roughness", 0.8)
    controller.Material.value.set_uniform("Metalness", 0.1)
    # controller.Transform.value *= avango.gua.make_scale_mat(0.2)

    hmd = avango.gua.nodes.TransformNode( \
        Children = [controller], Name="hmd")

    hmd.Transform.connect_from(hmd_sensor.Matrix)

    main = avango.gua.nodes.TransformNode( \
        Children= [hmd, skeleton], \
        Transform = avango.gua.make_trans_mat(0.0, 0.0, -10.0),
        Name="main"
        )

    light = avango.gua.nodes.LightNode(
        Type=avango.gua.LightType.POINT,
        Name="light",
        Color=avango.gua.Color(1.0, 1.0, 1.0),
        Brightness=100.0,
        Transform=(avango.gua.make_trans_mat(1, 1, 5) *
                   avango.gua.make_scale_mat(30, 30, 30)))
    graph.Root.value.Children.value.append(light)

    #size = avango.gua.Vec2ui(1024, 768)
    size = avango.gua.Vec2ui(5461, 4096)

    window = avango.gua.nodes.GlfwWindow(Size=size, LeftResolution=size)

    avango.gua.register_window("window", window)

    cam = avango.gua.nodes.CameraNode(
        LeftScreenPath="/screen",
        SceneGraph="scenegraph",
        Resolution=size,
        OutputWindowName="window",
        Transform=avango.gua.make_trans_mat(0.0, 0.0, 3.5))

    res_pass = avango.gua.nodes.ResolvePassDescription()
    res_pass.EnableSSAO.value = True
    res_pass.SSAOIntensity.value = 4.0
    res_pass.SSAOFalloff.value = 10.0
    res_pass.SSAORadius.value = 7.0
    res_pass.EnvironmentLightingColor.value = avango.gua.Color(0.1, 0.1, 0.1)
    res_pass.ToneMappingMode.value = avango.gua.ToneMappingMode.UNCHARTED
    res_pass.Exposure.value = 1.0
    res_pass.BackgroundColor.value = avango.gua.Color(0.4, 0.4, 0.4)

    anti_aliasing = avango.gua.nodes.SSAAPassDescription()

    pipeline_description = avango.gua.nodes.PipelineDescription(Passes=[
        avango.gua.nodes.TriMeshPassDescription(),
        avango.gua.nodes.LightVisibilityPassDescription(),
        res_pass,
        anti_aliasing,
    ])

    cam.PipelineDescription.value = pipeline_description

    screen = avango.gua.nodes.ScreenNode(Name="screen",
                                         Width=2,
                                         Height=1.5,
                                         Children=[cam])

    graph.Root.value.Children.value.append(screen)

    graph.Root.value.Children.value.append(main)


    viewer = avango.gua.nodes.Viewer()
    viewer.SceneGraphs.value = [graph]
    viewer.Windows.value = [window]

    guaVE = GuaVE()
    guaVE.start(locals(), globals())

    viewer.run()

def tree(obj):
    stack = [(obj, 0)]
    while stack:
        node, level = stack.pop()
        print("│   " * level + "├── {0} <{1}>".format(
            node.Name.value, node.__class__.__name__))
        stack.extend(
            [(child, level + 1) for child in reversed(node.Children.value)])

if __name__ == '__main__':
    start()
