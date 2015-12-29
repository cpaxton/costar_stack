/* 
 * File:   JHUDataParser.h
 * Author: chi
 *
 * Created on August 29, 2014, 10:15 AM
 */

#ifndef JHUDATAPARSER_H
#define	JHUDATAPARSER_H

#pragma once

#include "../include/index.h"

void batch_PreProcess_JHU(std::string in_path, std::string out_path, int c1 = -1, int c2 = -1);   //batching preprocess functions

void PreProcess_JHU(std::string in_path, std::string out_path);//, pcl::visualization::PCLVisualizer::Ptr viewer);    //convert to binary, filter point cloud, compute normals, save in binary

void readJHUInst(std::string path, ObjectSet &train_set, ObjectSet &test_set, int c1, int c2, bool all = false);

void readJHUInstWithImg(std::string path, ObjectSet &train_set, ObjectSet &test_set, int c1, int c2, bool all = false);

#endif	/* JHUDATAPARSER_H */

