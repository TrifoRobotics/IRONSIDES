import json, yaml, sys
#create extrinsic calib file
def CreateCalib4Extrinsic(data, output):
    p_T_t = data["children"]["child"]["children"]["child"]["children"]["child"]["parent_T_this"]
    left_T_right  = { "rows": 4,
                      "cols": 4,
                      "data": [float(i) for i in p_T_t[0]] + \
                              [float(i) for i in p_T_t[1]] + \
                              [float(i) for i in p_T_t[2]] + \
                              [float(i) for i in p_T_t[3]]}

    p_T_t = data["children"]["child"]["children"]["child"]["parent_T_this"]
    IMU_T_left    = { "rows": 4,
                      "cols": 4,
                      "data": [float(i) for i in p_T_t[0]] + \
                              [float(i) for i in p_T_t[1]] + \
                              [float(i) for i in p_T_t[2]] + \
                              [float(i) for i in p_T_t[3]]}

    p_T_t = data["children"]["child"]["parent_T_this"]
    root_T_IMU    = { "rows": 4,
                      "cols": 4,
                      "data": [float(i) for i in p_T_t[0]] + \
                              [float(i) for i in p_T_t[1]] + \
                              [float(i) for i in p_T_t[2]] + \
                              [float(i) for i in p_T_t[3]]}
    calib_extrinsic = { "left_T_right": left_T_right,
                        "imu_T_left"  : IMU_T_left,
                        "root_T_imu"  : root_T_IMU}
    with open(output, 'w') as yaml_file:
        #	ymal_file.write('# ')
        yaml.safe_dump(calib_extrinsic, yaml_file, default_flow_style=False)
        print "    ---created " + output

#create IMU calib file
def CreateCalib4IMU(data, output):
    model = data["children"]["child"]["model"]
    accel = {"bias": [float(i) for i in model["accel"]["bias"]],
             "TK"  : [float(i) for i in model["accel"]["TK"][0]] + \
                     [float(i) for i in model["accel"]["TK"][1]] + \
                     [float(i) for i in model["accel"]["TK"][2]]
             }
    gyro = {"bias": [float(i) for i in model["gyro"]["bias"]],
            "TK"  : [float(i) for i in model["gyro"]["TK"][0]] + \
                    [float(i) for i in model["gyro"]["TK"][1]] + \
                    [float(i) for i in model["gyro"]["TK"][2]]
            }
    calib_IMU = { "accel": accel,
                  "gyro" : gyro }
    with open(output, 'w') as yaml_file:
        yaml.safe_dump(calib_IMU, yaml_file, default_flow_style=False)
        print "    ---created " + output

#create left camera calib file
def CreateCalib4LeftCam(data, output):
    model = data["children"]["child"]["children"]["child"]["model"]
    camera_matrix = {"rows" : 3,
                     "cols" : 3}
    camera_matrix["data"] = [float(i) for i in model["camera_matrix"][0]] + \
                            [float(i) for i in model["camera_matrix"][1]] + \
                            [float(i) for i in model["camera_matrix"][2]]

    projection_matrix = { "rows" : 3,
                          "cols" : 4}
    projection_matrix["data"] = [float(i) for i in model["camera_matrix"][0]] + [0] + \
                                [float(i) for i in model["camera_matrix"][1]] + [0] + \
                                [float(i) for i in model["camera_matrix"][2]] + [0]
    calib_left = { "image_width"            : int(model["image_size"]["width"]),
                   "image_height"           : int(model["image_size"]["height"]),
                   "camera_name"            : "stereo_left",
                   "camera_matrix"          : camera_matrix,
                   "projection_matrix"      : projection_matrix
                   }
    with open(output, 'w') as yaml_file:
        yaml.safe_dump(calib_left, yaml_file, default_flow_style=False)
        print "    ---created " + output

#create right camera calib file
def CreateCalib4RightCam(data, output):
    model = data["children"]["child"]["children"]["child"]["children"]["child"]["model"]
    camera_matrix = {"rows" : 3,
                     "cols" : 3}
    camera_matrix["data"] = [float(i) for i in model["camera_matrix"][0]] + \
                            [float(i) for i in model["camera_matrix"][1]] + \
                            [float(i) for i in model["camera_matrix"][2]]
    projection_matrix = { "rows" : 3,
                          "cols" : 4}
    projection_matrix["data"] = [float(i) for i in model["camera_matrix"][0]] + [0] + \
                                [float(i) for i in model["camera_matrix"][1]] + [0] + \
                                [float(i) for i in model["camera_matrix"][2]] + [0]
    calib_right= { "image_width"            : int(model["image_size"]["width"]),
                   "image_height"           : int(model["image_size"]["height"]),
                   "camera_name"            : "stereo_right",
                   "camera_matrix"          : camera_matrix,
                   "projection_matrix"      : projection_matrix
                   }

    with open(output, 'w') as yaml_file:
        yaml.safe_dump(calib_right, yaml_file, default_flow_style=False)
        print "    ---created " + output
if __name__ == "__main__":
    if len(sys.argv)<3:
        print "Not enough input arguments."
        print "[calib_input JSON]" "[calib_output YAML]"
    calib_file = sys.argv[1]
    output_file = sys.argv[2]
    with open(calib_file) as ff:
        data = json.load(ff)
        print "Read from file:", calib_file
        CreateCalib4LeftCam(data,output_file[:-5]+"_left_raw.yaml")
        CreateCalib4RightCam(data,output_file[:-5]+"_right_raw.yaml")
        CreateCalib4IMU(data,output_file[:-5]+"_imu.yaml")
        CreateCalib4Extrinsic(data,output_file[:-5]+"_extrinsic.yaml")
