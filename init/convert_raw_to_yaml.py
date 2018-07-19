import json, yaml, sys, math
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
    fs = 200.0
    max_acc_noise = 0
    for i in model["accel"]["wideband_noise_var"]:
        if (max_acc_noise < float(i)):
            max_acc_noise = float(i)
    max_acc_rand_walk = 0
    for i in model["accel"]["bias_driving_noise_var"]:
        if (max_acc_rand_walk < float(i)):
            max_acc_rand_walk = float(i)
    max_gyro_noise = 0
    for i in model["gyro"]["wideband_noise_var"]:
        if (max_gyro_noise < float(i)):
            max_gyro_noise = float(i)
    max_gyro_rand_walk = 0
    for i in model["gyro"]["bias_driving_noise_var"]:
        if (max_gyro_rand_walk < float(i)):
            max_gyro_rand_walk = float(i)
    calib_IMU = { "Accelerometer": {
                       "accelerometer_noise_density" : math.sqrt(max_acc_noise / (fs/2)),
                       "accelerometer_random_walk"   : math.sqrt(max_acc_rand_walk / (fs/2))
                    },
                  "Gyroscope": {
                      "gyroscope_noise_density" : math.sqrt(max_gyro_noise / (fs/2)),
                      "gyroscope_random_walk"   : math.sqrt(max_gyro_rand_walk / (fs/2))
                  },
                  "update_rate" : fs
                }
    with open(output, 'w') as yaml_file:
        yaml.safe_dump(calib_IMU, yaml_file, default_flow_style=False)
	print "    ---created " + output

#create left camera calib file
def CreateCalib4LeftCam(data, output):
    model = data["children"]["child"]["children"]["child"]["model"]
    camera_model = model["type"]
    if camera_model == "fisheye_camera":
        camera_matrix = {"rows" : 3,
                         "cols" : 3}
        camera_matrix["data"] = [float(i) for i in model["camera_matrix"][0]] + \
                                [float(i) for i in model["camera_matrix"][1]] + \
                                [float(i) for i in model["camera_matrix"][2]]
        distortion_coefficients = {"rows" : 1,
                                   "cols" : 4}
        distortion_coefficients["data"] = [float(i) for i in model["distortion"][0]] + \
                                          [float(i) for i in model["distortion"][1]] + \
                                          [float(i) for i in model["distortion"][2]] + \
                                          [float(i) for i in model["distortion"][3]]
        rectification_matrix = { "rows" : 3,
                                 "cols" : 3,
                                 "data" : [1, 0, 0, 0, 1, 0, 0, 0, 1]}
        projection_matrix = { "rows" : 3,
                              "cols" : 4}
        projection_matrix["data"] = [float(i) for i in model["camera_matrix"][0]] + [0] +\
                                    [float(i) for i in model["camera_matrix"][1]] + [0] +\
                                    [float(i) for i in model["camera_matrix"][2]] + [0]
        calib_left = { "image_width"            : int(model["image_size"]["width"]),
                       "image_height"           : int(model["image_size"]["height"]),
                       "camera_name"            : "stereo_left",
                       "camera_matrix"          : camera_matrix,
                       "distortion_model"       :"fisheye",
                       "distortion_coefficients": distortion_coefficients,
                       "rectification_matrix"   : rectification_matrix,
                       "projection_matrix"      : projection_matrix
            }
        with open(output, 'w') as yaml_file:
            yaml.safe_dump(calib_left, yaml_file, default_flow_style=False)
        print "    ---created " + output + " distortion_model: fisheye"
    elif camera_model == "catadioptric_camera":
        camera_matrix = {"rows" : 3,
                         "cols" : 3}
        camera_matrix["data"] = [float(i) for i in model["camera_matrix"][0]] + \
                                [float(i) for i in model["camera_matrix"][1]] + \
                                [float(i) for i in model["camera_matrix"][2]]
        distortion_coefficients = {"rows" : 1,
                                   "cols" : 4}
        distortion_coefficients["data"] = [float(model["distortion"]["k1"])] + \
                                          [float(model["distortion"]["k2"])] + \
                                          [float(model["distortion"]["k3"])] + \
                                          [float(model["distortion"]["k4"])]
        rectification_matrix = { "rows" : 3,
                                 "cols" : 3,
                                 "data" : [1, 0, 0, 0, 1, 0, 0, 0, 1]}
        projection_matrix = { "rows" : 3,
                              "cols" : 4}
        projection_matrix["data"] = [float(i) for i in model["camera_matrix"][0]] + [0] + \
                                    [float(i) for i in model["camera_matrix"][1]] + [0] + \
                                    [float(i) for i in model["camera_matrix"][2]] + [0]
        unified_model_params = {
            "eta" : float(model["unified_model_params"]["eta"]),
            "xi" : float(model["unified_model_params"]["xi"])
        }
        calib_left = { "image_width"            : int(model["image_size"]["width"]),
                       "image_height"           : int(model["image_size"]["height"]),
                       "camera_name"            : "stereo_left",
                       "camera_matrix"          : camera_matrix,
                       "unified_model_params"   : unified_model_params,
                       "distortion_model"       :"catadioptric",
                       "distortion_coefficients": distortion_coefficients,
                       "rectification_matrix"   : rectification_matrix,
                       "projection_matrix"      : projection_matrix
                       }
        with open(output, 'w') as yaml_file:
            yaml.safe_dump(calib_left, yaml_file, default_flow_style=False)
        print "    ---created " + output + " distortion_model: catadioptric"

#create right camera calib file
def CreateCalib4RightCam(data, output):
    model = data["children"]["child"]["children"]["child"]["children"]["child"]["model"]
    camera_model = model["type"]
    if camera_model == "fisheye_camera":
        camera_matrix = {"rows" : 3,
                         "cols" : 3}
        camera_matrix["data"] = [float(i) for i in model["camera_matrix"][0]] + \
                                [float(i) for i in model["camera_matrix"][1]] + \
                                [float(i) for i in model["camera_matrix"][2]]
        distortion_coefficients = {"rows" : 1,
                                   "cols" : 4}
        distortion_coefficients["data"] = [float(i) for i in model["distortion"][0]] + \
                                          [float(i) for i in model["distortion"][1]] + \
                                          [float(i) for i in model["distortion"][2]] + \
                                          [float(i) for i in model["distortion"][3]]
        rectification_matrix = { "rows" : 3,
                                 "cols" : 3,
                                 "data" : [1, 0, 0, 0, 1, 0, 0, 0, 1]}
        projection_matrix = { "rows" : 3,
                              "cols" : 4}
        projection_matrix["data"] = [float(i) for i in model["camera_matrix"][0]] + [0] + \
                                    [float(i) for i in model["camera_matrix"][1]] + [0] + \
                                    [float(i) for i in model["camera_matrix"][2]] + [0]
        calib_right= { "image_width"            : int(model["image_size"]["width"]),
                       "image_height"           : int(model["image_size"]["height"]),
                       "camera_name"            : "stereo_right",
                       "camera_matrix"          : camera_matrix,
                       "distortion_model"       :"fisheye",
                       "distortion_coefficients": distortion_coefficients,
                       "rectification_matrix"   : rectification_matrix,
                       "projection_matrix"      : projection_matrix
            }

        with open(output, 'w') as yaml_file:
            yaml.safe_dump(calib_right, yaml_file, default_flow_style=False)
        print "    ---created " + output + " distortion_model: fisheye"
    elif camera_model == "catadioptric_camera":
        camera_matrix = {"rows" : 3,
                         "cols" : 3}
        camera_matrix["data"] = [float(i) for i in model["camera_matrix"][0]] + \
                                [float(i) for i in model["camera_matrix"][1]] + \
                                [float(i) for i in model["camera_matrix"][2]]
        distortion_coefficients = {"rows" : 1,
                                   "cols" : 4}
        distortion_coefficients["data"] = [float(model["distortion"]["k1"])] + \
                                          [float(model["distortion"]["k2"])] + \
                                          [float(model["distortion"]["k3"])] + \
                                          [float(model["distortion"]["k4"])]
        rectification_matrix = { "rows" : 3,
                                 "cols" : 3,
                                 "data" : [1, 0, 0, 0, 1, 0, 0, 0, 1]}
        projection_matrix = { "rows" : 3,
                              "cols" : 4}
        projection_matrix["data"] = [float(i) for i in model["camera_matrix"][0]] + [0] + \
                                    [float(i) for i in model["camera_matrix"][1]] + [0] + \
                                    [float(i) for i in model["camera_matrix"][2]] + [0]
        unified_model_params = {
            "eta" : float(model["unified_model_params"]["eta"]),
            "xi" : float(model["unified_model_params"]["xi"])
        }
        calib_left = { "image_width"            : int(model["image_size"]["width"]),
                       "image_height"           : int(model["image_size"]["height"]),
                       "camera_name"            : "stereo_left",
                       "camera_matrix"          : camera_matrix,
                       "unified_model_params"   : unified_model_params,
                       "distortion_model"       :"catadioptric",
                       "distortion_coefficients": distortion_coefficients,
                       "rectification_matrix"   : rectification_matrix,
                       "projection_matrix"      : projection_matrix
                       }
        with open(output, 'w') as yaml_file:
            yaml.safe_dump(calib_left, yaml_file, default_flow_style=False)
        print "    ---created " + output + " distortion_model: catadioptric"
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
