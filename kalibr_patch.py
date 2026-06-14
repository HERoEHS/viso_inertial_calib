ts = 0.001313
apply_border = 1
path = '/catkin_ws/src/kalibr/aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/IccSensors.py'
with open(path) as f:
    code = f.read()
if apply_border:
    code = code.replace(
        'options.showExtractionVideo = showExtraction\n            options.minTagsForValidObs',
        'options.showExtractionVideo = showExtraction\n            options.blackTagBorder = 1\n            options.minTagsForValidObs')
    print('patched blackTagBorder=1')
code = code.replace(
    '        self.timeshiftCamToImuPrior = shift\n        \n',
    '        self.timeshiftCamToImuPrior = shift\n        self.timeshiftCamToImuPrior = ' + str(ts) +
    '\n        print("[PATCH] Timeshift prior: ' + str(round(ts * 1000, 2)) + 'ms")\n        \n')
print(f'patched timeshiftCamToImuPrior={ts:.6f}s ({ts*1000:.2f}ms)')
with open(path, 'w') as f:
    f.write(code)
