import os
import subprocess

if not os.path.exists("output"):
	os.makedirs("output")
os.chdir("./build")

detector_list = ["SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"]
descriptor_list = ["BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"]

cnt = 1
final_string_list = []
for detector in detector_list:
	for descriptor in descriptor_list:
		state, output_string = subprocess.getstatusoutput('./2D_feature_tracking %s %s' % (detector, descriptor))
		final_string_list.append(str(cnt) + "\n" + output_string+"\n")
		cnt += 1

final_string = "----------------\n".join(final_string_list)
with open("../output/result.txt", "w") as f:
	f.write(final_string)