#pragma once
// #ifndef IO_FUNCTION_H_
// #define IO_FUNCTION_H_
#include "commonHeaders.h"
#include "simulator/sensors/imu_base.h"
#include "simulator/sensors/cns.h"
#include "simulator/sensors/virns.h"
#include "simulator/sensors/cmns.h"

using namespace std;

namespace MyFusion{

int readImuParam(string filename, ImuParam &param);

void readImuMotionData(string filename, vector<ImuMotionData> &imu_data);

void writeImuMotionData(string filename, vector<ImuMotionData> &imu_data);

void writeCnsData(string filename, vector<CnsData> &cnsData);

void writeVirnsData(string filename, vector<VirnsData> &virnsData);

void writeCmnsData(string filename, vector<CmnsData> &cmnsData);

void writePos(string filename, vector<ImuMotionData> &imu_data);

void writeAllanData(string filename, vector<ImuMotionData> &imu_data);

void readCnsData(string fileName, vector<CnsData> &cnsData);

void readVirnsData(string fileName, vector<VirnsData> &virnsData);

void readCmnsData(string fileName, vector<CmnsData> &cmnsData);

/**
 * @brief print percentage of progress
 * 
 * @param name 
 * @param per 
 */
void printPer(string name, float per);
/**
 * @brief print percentage of progress
 * 
 * @param name 
 * @param per 
 */
void printPer(string name, int per);

} // namespace MyFusion


// #endif