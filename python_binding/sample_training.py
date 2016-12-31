from sp_segmenter.SpCompactPy import *

# location of the directory that contains object folder and background folder
training_folder_path = "data/training";

# list of object and background folders
obj_names = ["drill","hammer"];
bg_names =  ["UR5"];

# paths for dictionaries
shot_path = "data/UW_shot_dict";
sift_path = "data/UW_sift_dict";
fpfh_path = "data/UW_fpfh_dict";

# output folder for FEA and SVM result
out_fea_path = "./fea_pool"
out_svm_path = "./svm_pool"

skip_fea = False
train_bg_flag  = True
train_multi_flag = True

foregroundBackgroundCC =  0.001
multiclassCC = 0.001
bgSampleNum = 66
objSampleNum = 100

# Setting up all training parameters
training = SpCompact()

# Setting up SIFT/SHOT/FPFH/training directory locations. Return false if directory is not exist
training.setInputPathSIFT(sift_path)
training.setInputPathSHOT(shot_path)
training.setInputPathFPFH(fpfh_path)
training.setInputTrainingPath(training_folder_path);

# The class will load the pcd files from directory_path/object_name[1 .. n]/*.pcd
# and directory_path/background_name[1 .. n]/*.pcd
training.setObjectNames(obj_names);
training.setBackgroundNames(bg_names);

# Set file output directory location. Will create the output directory if it does not exist
training.setOutputDirectoryPathFEA(out_fea_path);
training.setOutputDirectoryPathSVM(out_svm_path);

training.setForegroundCC(foregroundBackgroundCC);
training.setMultiCC(multiclassCC);

training.setBackgroundSampleNumber(bgSampleNum);
training.setObjectSampleNumber(objSampleNum);

training.setCurOrderMax(3);

training.setSkipFeaExtraction(skip_fea);
training.setSkipBackgroundSVM(not train_bg_flag);
training.setSkipMultiSVM(not train_multi_flag);

# Do Feature extraction and SVM Training
training.startTrainingSVM();