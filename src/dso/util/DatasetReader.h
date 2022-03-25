/**
* This file is part of DSO, written by Jakob Engel.
* It has been modified by Lukas von Stumberg for the inclusion in DM-VIO (http://vision.in.tum.de/dm-vio).
*
* Copyright 2022 Lukas von Stumberg <lukas dot stumberg at tum dot de>
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "util/globalCalib.h"

#include "util/GTData.hpp"
#include "IMU/IMUTypes.h"

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>

#include "util/Undistort.h"
#include "IOWrapper/ImageRW.h"

#if HAS_ZIPLIB
	#include "zip.h"
#endif

#include <boost/thread.hpp>

using namespace dso;



inline int getdir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL)
    {
        return -1;
    }

    while ((dirp = readdir(dp)) != NULL) {
    	std::string name = std::string(dirp->d_name);

    	if(name != "." && name != "..")
    		files.push_back(name);
    }
    closedir(dp);


    std::sort(files.begin(), files.end());

    if(dir.at( dir.length() - 1 ) != '/') dir = dir+"/";
	for(unsigned int i=0;i<files.size();i++)
	{
		if(files[i].at(0) != '/')
			files[i] = dir + files[i];
	}

    return files.size();
}


struct PrepImageItem
{
	int id;
	bool isQueud;
	ImageAndExposure* pt;

	inline PrepImageItem(int _id)
	{
		id=_id;
		isQueud = false;
		pt=0;
	}

	inline void release()
	{
		if(pt!=0) delete pt;
		pt=0;
	}
};




class ImageFolderReader
{
public:
	ImageFolderReader(std::string path, std::string calibFile, std::string gammaFile, std::string vignetteFile, bool use16BitPassed)
	{
		this->path = path;
		this->calibfile = calibFile;
		use16Bit = use16BitPassed;

#if HAS_ZIPLIB
		ziparchive=0;
		databuffer=0;
#endif

		isZipped = (path.length()>4 && path.substr(path.length()-4) == ".zip");





		if(isZipped)
		{
#if HAS_ZIPLIB
			int ziperror=0;
			ziparchive = zip_open(path.c_str(),  ZIP_RDONLY, &ziperror);
			if(ziperror!=0)
			{
				printf("ERROR %d reading archive %s!\n", ziperror, path.c_str());
				exit(1);
			}

			files.clear();
			int numEntries = zip_get_num_entries(ziparchive, 0);
			for(int k=0;k<numEntries;k++)
			{
				const char* name = zip_get_name(ziparchive, k,  ZIP_FL_ENC_STRICT);
				std::string nstr = std::string(name);
				if(nstr == "." || nstr == "..") continue;
				files.push_back(name);
			}

			printf("got %d entries and %d files!\n", numEntries, (int)files.size());
			std::sort(files.begin(), files.end());
#else
			printf("ERROR: cannot read .zip archive, as compile without ziplib!\n");
			exit(1);
#endif
		}
		else
			getdir (path, files);


		undistort = Undistort::getUndistorterForFile(calibFile, gammaFile, vignetteFile);


		widthOrg = undistort->getOriginalSize()[0];
		heightOrg = undistort->getOriginalSize()[1];
		width=undistort->getSize()[0];
		height=undistort->getSize()[1];


		// load timestamps if possible.
		loadTimestamps();
		printf("ImageFolderReader: got %d files in %s!\n", (int)files.size(), path.c_str());

	}
	~ImageFolderReader()
	{
#if HAS_ZIPLIB
		if(ziparchive!=0) zip_close(ziparchive);
		if(databuffer!=0) delete databuffer;
#endif


		delete undistort;
	};

	Eigen::VectorXf getOriginalCalib()
	{
		return undistort->getOriginalParameter().cast<float>();
	}
	Eigen::Vector2i getOriginalDimensions()
	{
		return  undistort->getOriginalSize();
	}

	void getCalibMono(Eigen::Matrix3f &K, int &w, int &h)
	{
		K = undistort->getK().cast<float>();
		w = undistort->getSize()[0];
		h = undistort->getSize()[1];
	}

	void setGlobalCalibration()
	{
		int w_out, h_out;
		Eigen::Matrix3f K;
		getCalibMono(K, w_out, h_out);
		setGlobalCalib(w_out, h_out, K);
	}

	int getNumImages()
	{
		return files.size();
	}

	double getTimestamp(int id)
	{
		if(timestamps.size()==0) return id*0.1f;
		if(id >= (int)timestamps.size()) return 0;
		if(id < 0) return 0;
		return timestamps[id];
	}

    std::string getFilename(int id)
    {
        return files[id];
    }

	void prepImage(int id, bool as8U=false)
	{

	}


	MinimalImageB* getImageRaw(int id)
	{
			return getImageRaw_internal(id,0);
	}

	ImageAndExposure* getImage(int id, bool forceLoadDirectly=false)
	{
		return getImage_internal(id, 0);
	}


	inline float* getPhotometricGamma()
	{
		if(undistort==0 || undistort->photometricUndist==0) return 0;
		return undistort->photometricUndist->getG();
	}

    dmvio::IMUData getIMUData(int i)
    {
	    // returning IMU data between frame i-1 and frame i!
	    return imuDataAllFrames[i - 1];
    }
    
    dmvio::GTData getGTData(int id, bool &foundOut)
    {
        long long idReal = ids[id];
		auto it = gtData.lower_bound(idReal);
		long long firstDist, secondDist;
		long long dist = std::abs(idReal - it->first);
		firstDist = dist;
		if(it == gtData.end())
		{
			foundOut = false;
			return dmvio::GTData();
		}

		if(it != gtData.begin())
        {
		    it--;
		    secondDist = std::abs(idReal - it->first);
		    if(secondDist >= firstDist)
            {
		        it++;
            }else
            {
		        dist = secondDist;
            }
        }
		double distSeconds = (double) dist * 1e-9;
        std::cout << "GTData distance (seconds): " << distSeconds << std::endl;
        if(distSeconds > 0.01)
        {
            return dmvio::GTData{};
        }

        foundOut = true;
        return it->second;
    }
    
    bool loadGTData(std::string gtFile)
    {
        std::string defaultFile = path.substr(0, path.find_last_of('/')) + "/../state_groundtruth_estimate0/data.csv";
        std::cout << "Loading gt data" << std::endl;
        
        if(gtFile == "")
        {
            gtFile = defaultFile;
        }

        std::ifstream tr;
        tr.open(gtFile.c_str());
        
        if(!tr.good())
        {
            return false;
        }
        while(!tr.eof() && tr.good())
        {
            std::string line;
            char buf[1000];
            tr.getline(buf, 1000);
            
            long long id;
            double p1, p2, p3, qw, qx, qy, qz, v1, v2, v3, br1, br2, br3, bp1, bp2, bp3;
            if(17 == sscanf(buf, "%lld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &id, &p1, &p2, &p3, &qw, &qx, &qy, &qz, &v1, &v2, &v3, &br1, &br2, &br3, &bp1, &bp2, &bp3))
            {
                // EuRoC format with bias GT.
                Eigen::Vector3d translation(p1, p2, p3);
                Eigen::Quaterniond quat(qw, qx, qy, qz);
                Sophus::SE3 pose(quat, translation);
                Eigen::Vector3d velocity(v1, v2, v3);
                Eigen::Vector3d biasRot(br1, br2, br3);
                Eigen::Vector3d biasPos(bp1, bp2, bp3);
                
                gtData[id] = dmvio::GTData(pose, velocity, biasRot, biasPos);

            } else if(8 == sscanf(buf, "%lld,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &id, &p1, &p2, &p3, &qw, &qx, &qy, &qz))
            {
                // TUM-VI format
                Eigen::Vector3d translation(p1, p2, p3);
                Eigen::Quaterniond quat(qw, qx, qy, qz);
                Sophus::SE3 pose(quat, translation);
                Eigen::Vector3d velocity(0.0, 0.0, 0.0);
                Eigen::Vector3d biasRot(0.0, 0.0, 0.0);
                Eigen::Vector3d biasPos(0.0, 0.0, 0.0);

                gtData[id] = dmvio::GTData(pose, velocity, biasRot, biasPos);
            }
        }
        tr.close();
        return true;
    }


    void loadIMUData(std::string imuFile = "")
    {
        // Important: This IMU loading method expects that for each image there is an IMU 'measurement' with exactly the same timestamp (the VI-sensor does this).
        // If the sensor does not output this, a fake measurement with this timestamp has to be interpolated in advance.
        // The DM-VIO Python tools have a script to do this.
        if(imuFile == "")
        {
            imuFile = path.substr(0,path.find_last_of('/')) + "/imu.txt";
        }
        std::ifstream imuStream(imuFile);
        if(imuStream.good())
        {
            std::string line;
            std::getline(imuStream, line);
            // At the moment only comments at the beginning of the file are supported.
            while(line[0] == '#')
            {
                std::cout << "Skipping comment line in IMU data.\n";
                std::getline(imuStream, line);
            }
            std::stringstream lineStream(line);
            long long imuStamp;
            double wx, wy, wz, ax, ay, az;
            lineStream >> imuStamp >> wx >> wy >> wz >> ax >> ay
                     >> az;
            std::cout << "IMU Id: " << imuStamp << std::endl;

            // Find first frame with IMU data.
            int startFrame = -1;
            for(size_t j = 0; j < getNumImages(); ++j)
            {
                long long imageTimestamp = ids[j];
                while(imuStamp < imageTimestamp)
                {
                    imuStream >> imuStamp >> wx >> wy >> wz >> ax >> ay >> az;
                }
                if(imuStamp == imageTimestamp)
                {
                    // Success
                    startFrame = j;
                    break;
                }
                if(imuStamp > imageTimestamp)
                {
                    std::cout << "IMU-data too old -> skipping frame" << std::endl;
                    imuDataAllFrames.push_back(dmvio::IMUData{});
                    continue;
                }
            }

            if(startFrame == -1)
            {
                std::cout << "Found no start frame for IMU-data!" << std::endl;
                imuStream.close();
                return;
            }

            // For each image, we will save the IMU data between it, and the next frame, so no IMU data is needed for the last frame.
            // Note that when later accessing the imu data in the method getIMUData we output the imu data between the given frame and the previous frame.
            for(size_t j = startFrame; j < getNumImages() - 1; j++)
            {
                long long imageTimestamp = ids[j];
                long long nextTimestamp = ids[j+1];

                assert(imuStamp == imageTimestamp); // Otherwise we would need to interpolate IMU data which is not implemented atm.
                dmvio::IMUData imuData;
                long long previousIMUTime = imuStamp;

                // Each frame should get the all IMU data with:
                // thisTimestamp < imuStamp <= nextTimestamp
                while(imuStamp < nextTimestamp)
                {
                    // Get next IMU-Data.
                    imuStream >> imuStamp >> wx >> wy >> wz >> ax >> ay >> az;

                    if(imuStamp > nextTimestamp)
                    {
                        // If this happens we would have to interpolate IMU data which is not implemented at the moment.
                        assert(false);
                    }

                    Eigen::Vector3d accMeas, gyrMeas;
                    accMeas << ax, ay, az;
                    gyrMeas << wx, wy, wz;
                    // For each measurement GTSAM wants the time between it, and the previous measurement.
                    // The timestamps are in nanoseconds -> convert!
                    double integrationTime = (double) (imuStamp - previousIMUTime) * 1e-9;
                    imuData.push_back(dmvio::IMUMeasurement(accMeas, gyrMeas, integrationTime));

                    previousIMUTime = imuStamp;
                }

                imuDataAllFrames.push_back(imuData);
            }
        }else
        {
            std::cout << "Found no IMU-data." << std::endl;
        }


        imuStream.close();

    }

	// undistorter. [0] always exists, [1-2] only when MT is enabled.
	Undistort* undistort;
private:


	MinimalImageB* getImageRaw_internal(int id, int unused)
	{
	    assert(!use16Bit);
		if(!isZipped)
		{
			// CHANGE FOR ZIP FILE
			return IOWrap::readImageBW_8U(files[id]);
		}
		else
		{
#if HAS_ZIPLIB
			if(databuffer==0) databuffer = new char[widthOrg*heightOrg*6+10000];
			zip_file_t* fle = zip_fopen(ziparchive, files[id].c_str(), 0);
			long readbytes = zip_fread(fle, databuffer, (long)widthOrg*heightOrg*6+10000);

			if(readbytes > (long)widthOrg*heightOrg*6)
			{
				printf("read %ld/%ld bytes for file %s. increase buffer!!\n", readbytes,(long)widthOrg*heightOrg*6+10000, files[id].c_str());
				delete[] databuffer;
				databuffer = new char[(long)widthOrg*heightOrg*30];
				fle = zip_fopen(ziparchive, files[id].c_str(), 0);
				readbytes = zip_fread(fle, databuffer, (long)widthOrg*heightOrg*30+10000);

				if(readbytes > (long)widthOrg*heightOrg*30)
				{
					printf("buffer still to small (read %ld/%ld). abort.\n", readbytes,(long)widthOrg*heightOrg*30+10000);
					exit(1);
				}
			}

			return IOWrap::readStreamBW_8U(databuffer, readbytes);
#else
			printf("ERROR: cannot read .zip archive, as compile without ziplib!\n");
			exit(1);
#endif
		}
	}


	ImageAndExposure* getImage_internal(int id, int unused)
	{
	    if(use16Bit)
        {
            MinimalImage<unsigned short>* minimg = IOWrap::readImageBW_16U(files[id]);
            assert(minimg);
            ImageAndExposure* ret2 = undistort->undistort<unsigned short>(
                    minimg,
                    (exposures.size() == 0 ? 1.0f : exposures[id]),
                    (timestamps.size() == 0 ? 0.0 : timestamps[id]),
                    1.0f / 256.0f);
            delete minimg;
            return ret2;
        }else
        {
            MinimalImageB* minimg = getImageRaw_internal(id, 0);
            ImageAndExposure* ret2 = undistort->undistort<unsigned char>(
                    minimg,
                    (exposures.size() == 0 ? 1.0f : exposures[id]),
                    (timestamps.size() == 0 ? 0.0 : timestamps[id]));
            delete minimg;
            return ret2;
        }
	}

	inline void loadTimestamps()
	{
		std::ifstream tr;
		std::string timesFile = path.substr(0,path.find_last_of('/')) + "/times.txt";
		tr.open(timesFile.c_str());
		while(!tr.eof() && tr.good())
		{
			std::string line;
			char buf[1000];
			tr.getline(buf, 1000);

			long long id;
			double stamp;
			float exposure = 0;
			if(3 == sscanf(buf, "%lld %lf %f", &id, &stamp, &exposure))
			{
                ids.push_back(id);
				timestamps.push_back(stamp);
				exposures.push_back(exposure);
			}

			else if(2 == sscanf(buf, "%lld %lf", &id, &stamp))
			{
                ids.push_back(id);
				timestamps.push_back(stamp);
				exposures.push_back(exposure);
			}
		}
		tr.close();

		// check if exposures are correct, (possibly skip)
		bool exposuresGood = ((int)exposures.size()==(int)getNumImages()) ;
		for(int i=0;i<(int)exposures.size();i++)
		{
			if(exposures[i] == 0)
			{
				// fix!
				float sum=0,num=0;
				if(i>0 && exposures[i-1] > 0) {sum += exposures[i-1]; num++;}
				if(i+1<(int)exposures.size() && exposures[i+1] > 0) {sum += exposures[i+1]; num++;}

				if(num>0)
					exposures[i] = sum/num;
			}

			if(exposures[i] == 0) exposuresGood=false;
		}


		if((int)getNumImages() != (int)timestamps.size())
		{
			printf("set timestamps and exposures to zero!\n");
			exposures.clear();
			timestamps.clear();
		}

		if((int)getNumImages() != (int)exposures.size() || !exposuresGood)
		{
			printf("set EXPOSURES to zero!\n");
			exposures.clear();
		}

		printf("got %d images and %d timestamps and %d exposures.!\n", (int)getNumImages(), (int)timestamps.size(), (int)exposures.size());
	}

    std::map<long long, dmvio::GTData> gtData;

	std::vector<ImageAndExposure*> preloadedImages;
	std::vector<std::string> files;
	std::vector<double> timestamps;
	std::vector<float> exposures;
    std::vector<long long> ids; // Saves the ids that are used by e.g. the EuRoC dataset.

    std::vector<dmvio::IMUData> imuDataAllFrames;

	int width, height;
	int widthOrg, heightOrg;

	std::string path;
	std::string calibfile;

	bool isZipped;
	bool use16Bit;

#if HAS_ZIPLIB
	zip_t* ziparchive;
	char* databuffer;
#endif
};

