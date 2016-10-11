import avango
import avango.script
import avango.daemon
import avango.gua
from examples_common.GuaVE import GuaVE

from skeleton import HumanSkeleton

import csv

def start():

    # acquire calibration
    try:
        with open('calibration.txt', 'r') as file:
            reader = csv.reader(file, delimiter=" ")
            csv_out = [[float(e) for e in r] for r in reader]
    except:
        print("ERROR: cannot parse calibration file")

    calibration = avango.gua.Mat4()

    for i in range(4):
        for k in range(4):
            calibration.set_element(i,k,csv_out[i][k])

    # setup scenegraph
    graph = avango.gua.nodes.SceneGraph(Name="scenegraph")
    loader = avango.gua.nodes.TriMeshLoader()
    transform = avango.gua.nodes.TransformNode( \
        Transform = calibration)
    skeleton = HumanSkeleton(PARENT_NODE = transform)

    hmd_service = avango.daemon.DeviceService()
    hmd = avango.daemon.nodes.DeviceSensor( \
        DeviceService = hmd_service)
    hmd.Station.value = "hmd-1"

    monkey = loader.create_geometry_from_file(
        "monkey", "data/objects/monkey.obj", \
        avango.gua.LoaderFlags.NORMALIZE_SCALE)
    monkey.Material.value.set_uniform("Color",
        avango.gua.Vec4(1.0, 0.75, 0.75, 1.0))
    monkey.Material.value.set_uniform("Roughness", 0.3)
    monkey.Material.value.set_uniform("Metalness", 1.0)

    hmd_transform = avango.gua.nodes.TransformNode( \
        Children = [monkey])

    hmd_transform.Transform.connect_from(hmd.Matrix)

    main = avango.gua.nodes.TransformNode( \
        Children= [hmd_transform, transform], \
        Transform = avango.gua.make_trans_mat(0.0, 0.0, -10.0)
        )

    '''
    # device sensor listening to the daemon values
    device_service = avango.daemon.DeviceService()

    device = avango.daemon.nodes.DeviceSensor(
        DeviceService=device_service)

    device2 = avango.daemon.nodes.DeviceSensor(
        DeviceService=device_service)

    # station name determines which device is used
    device.Station.value = "kinect-joint-0"  # 0 for first xbox controller
    device2.Station.value = "kinect-joint-1"

    # setup scenegraph
    graph = avango.gua.nodes.SceneGraph(Name="scenegraph")
    loader = avango.gua.nodes.TriMeshLoader()

    monkey = loader.create_geometry_from_file(
        "monkey", "data/objects/monkey.obj",
        avango.gua.LoaderFlags.NORMALIZE_SCALE)

    monkey.Material.value.set_uniform("Color",
                                      avango.gua.Vec4(1.0, 0.766, 0.336, 1.0))
    monkey.Material.value.set_uniform("Roughness", 0.3)
    monkey.Material.value.set_uniform("Metalness", 1.0)

    transform = avango.gua.nodes.TransformNode(Children=[monkey])
    transform.Transform.connect_from(device.Matrix)

    monkey2 = loader.create_geometry_from_file(
        "monkey2", "data/objects/monkey.obj",
        avango.gua.LoaderFlags.NORMALIZE_SCALE)

    monkey2.Material.value.set_uniform("Color",
                                      avango.gua.Vec4(1.0, 0.766, 0.336, 1.0))
    monkey2.Material.value.set_uniform("Roughness", 0.3)
    monkey2.Material.value.set_uniform("Metalness", 1.0)

    transform2 = avango.gua.nodes.TransformNode(Children=[monkey2])
    transform2.Transform.connect_from(device2.Matrix)
    '''
    light = avango.gua.nodes.LightNode(
        Type=avango.gua.LightType.POINT,
        Name="light",
        Color=avango.gua.Color(1.0, 1.0, 1.0),
        Brightness=100.0,
        Transform=(avango.gua.make_trans_mat(1, 1, 5) *
                   avango.gua.make_scale_mat(30, 30, 30)))
    graph.Root.value.Children.value.append(light)

    size = avango.gua.Vec2ui(1024, 768)

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
    res_pass.BackgroundColor.value = avango.gua.Color(0.45, 0.5, 0.6)

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


if __name__ == '__main__':
    start()
