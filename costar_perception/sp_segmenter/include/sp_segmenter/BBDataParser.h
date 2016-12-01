/* 
 * File:   BBDataParser.h
 * Author: chi
 *
 * Created on August 5, 2014, 2:22 PM
 */

#ifndef BBDATAPARSER_H
#define	BBDATAPARSER_H

#pragma once

#include "sp_segmenter/utility/utility.h"

void BBShiftClouds(std::string in_path, std::string out_path);

void BBStreamingShift(std::string path, std::string sub_in_path, std::string sub_out_path, int c1 = -1, int c2 = -1);

void BBStreamingNormal(std::string path, std::string sub_path, float radius = 0.03, int c1 = -1, int c2 = -1);

void readBBTrainALL(std::string path, std::string sub_path, CloudSet &clouds, NormalSet &normals, int c1 = -1, int c2 = -1);

void readBB(std::string path, ObjectSet &train_set, ObjectSet &test_set, int c1 = -1, int c2 = -1, bool all = false);



#endif	/* BBDATAPARSER_H */

