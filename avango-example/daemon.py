import avango.daemon

def init_skeleton_tracking():
    joint_stations = {}
    for i in range(25):
        joint_stations[i] = avango.daemon.Station('kinect-joint-{0}'.format(str(i)))

    skeleton0 = avango.daemon.SkeletonTrack()
    skeleton0.port = "7700"
    skeleton0.server = "141.54.147.35"

    for i in range(25):
        skeleton0.stations[i] = joint_stations[i]

    device_list.append(skeleton0)

def init_hmd_tracking():
    hmd_stations = {}
    for i in range(7):
        hmd_stations[i] = avango.daemon.Station('hmd-{0}'.format(str(i)))

    hmd0 = avango.daemon.HMDTrack()
    hmd0.server = "141.54.147.35"
    hmd0.port = "7770"

    for i in range(7):
        hmd0.stations[i] = hmd_stations[i]

    device_list.append(hmd0)

device_list = []
init_skeleton_tracking()
init_hmd_tracking()

avango.daemon.run(device_list)
