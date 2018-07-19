import json, yaml, sys, os, shutil
import convert_raw_to_yaml as convert_raw
import convert_rect_to_yaml as convert_rect

if __name__ == "__main__":

    if len(sys.argv)<3:
        print "Not enough input arguments."
        print "[SDK folder]" "[calib_raw JSON]" "[calib_rectified JSON]"

    sdk_folder = sys.argv[1]
    calib_raw = sys.argv[2]
    calib_rectified = sys.argv[3]
    sdk_path = '../pirvs'
    if not os.path.exists(sdk_path):
        os.makedirs(sdk_path)
        print "Creating " + sdk_path
        shutil.copytree(sdk_folder + '/lib', '../pirvs/lib')
        shutil.copytree(sdk_folder + '/include', '../pirvs/include')
        shutil.copyfile(sdk_folder + '/voc_ironsides.json', '../pirvs/voc_ironsides.json')
    else:
        print "Error: Please remove pirvs folder."
        sys.exit()
    calib_path = '../pirvs/calib'
    if not os.path.exists(calib_path):
        os.makedirs(calib_path)
    else:
        print "Error: Please remove pirvs/calib folder."
        sys.exit()

    shutil.copyfile(calib_raw, calib_path + '/calib_PerceptIn_Ironsides.json')
    with open(calib_raw) as ff:
        data = json.load(ff)
        print "Read from file:", calib_raw
        convert_raw.CreateCalib4LeftCam(data, calib_path + "/calib_PerceptIn_Ironsides_left_raw.yaml")
        convert_raw.CreateCalib4RightCam(data, calib_path +  "/calib_PerceptIn_Ironsides_right_raw.yaml")
        convert_raw.CreateCalib4IMU(data, calib_path + "/calib_PerceptIn_Ironsides_imu.yaml")
        convert_raw.CreateCalib4Extrinsic(data, calib_path + "/calib_PerceptIn_Ironsides_extrinsic.yaml")

    shutil.copyfile(calib_rectified, calib_path + '/calib_PerceptIn_Ironsides_rectified.json')
    with open(calib_rectified) as ff:
        data = json.load(ff)
        print "Read from file:", calib_rectified
        convert_rect.CreateCalib4LeftCam(data, calib_path + "/calib_PerceptIn_Ironsides_rectified_left_raw.yaml")
        convert_rect.CreateCalib4RightCam(data, calib_path +  "/calib_PerceptIn_Ironsides_rectified_right_raw.yaml")
        convert_rect.CreateCalib4IMU(data, calib_path + "/calib_PerceptIn_Ironsides_rectified_imu.yaml")
        convert_rect.CreateCalib4Extrinsic(data, calib_path + "/calib_PerceptIn_Ironsides_rectified_extrinsic.yaml")