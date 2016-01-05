/* 
 * File:   UWDataParser.h
 * Author: chi
 *
 * Created on August 5, 2014, 11:20 AM
 */

#ifndef UWDATAPARSER_H
#define	UWDATAPARSER_H

#pragma once
#include "../include/index.h"

/**************************** UW Data Parser ************************************/
// Load one instance at a time
void PreProcess_UW(std::string in_path, std::string out_path, pcl::visualization::PCLVisualizer::Ptr viewer);    //convert to binary, filter point cloud, compute normals, save in binary

void batch_PreProcess_UW(std::string in_path, std::string out_path, int c1 = -1, int c2 = -1);   //batching preprocess functions

void readUWInst(std::string path, ObjectSet &train_set, ObjectSet &test_set, int c1 = -1, int c2 = -1, int inter = 5);

void readUWInstWithImg(std::string path, ObjectSet &train_set, ObjectSet &test_set, int c1 = -1, int c2 = -1, int inter = 5);

void sweepNaN(std::string path, int c1, int c2);

void readUWInstAll(std::string path, CloudSet &clouds, NormalSet &normals, int c1 = -1, int c2 = -1, float prob = 1.0);

void readUWInstTwo(std::string path_img, std::string path_cloud, ObjectSet &train_img, ObjectSet &train_cloud, ObjectSet &test_img, ObjectSet &test_cloud, int c1, int c2, int inter_train, int inter_test);

/*
void ASCIIToBinary(std::string in_path, std::string out_path, std::string className);

void FilterCloud(std::string in_path, std::string out_path, std::string className);

void ImportOneClass(std::string in_path, int id, std::string trial_no, UWDataParser &dParser);

void DeChanel(std::string in_path, std::string out_path, int c1, int c2);

std::vector<MulInfoT> ReadSeq(std::string path, std::string class_name, std::string inst_id, std::string seq_id, int channel_id);

std::vector<MulInfoT> ReadInst(std::string path, std::string class_name, std::string inst_id, int channel_id);

void ReadClass(std::string path, int c1, int c2, int trial_id, int channel_id, std::vector<RGBDPOOL> &train_set, std::vector<RGBDPOOL> &test_set);
*/



#endif	/* UWDATAPARSER_H */

