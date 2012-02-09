//
//  main.cpp
//  shotFeature
//
//  Created by Mohsen Malmir on 9/29/11.
//  Copyright 2011 University of Arizona. All rights reserved.
//
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/shot.h>
#include <pcl/features/normal_3d.h>    
#include <pcl/octree/octree.h>

int main (int argc, const char * argv[])
{
    std::string listFileName, pcdFileName;
    std::string pcdFileFullName, shotFileFullName;
    std::string pcdFilePath, outputFileName, shotDir;
    std::string kernelType, kernelSize;
    fstream outputFile, listFile, shotFile;

    //argument processing
    if (argc < 5)
    {
        std::cout << "shotFeature: Extracts SHOT features from pdc file." << endl << "Usage:" << endl << "shotFeature listFile outputFile kernelSizeType kernelSize [loadPathPrefix] [saveDirectory]" << endl;
        std::cout << "listFile is the name of file that contains the list of input pointcloud files" << endl;
        std::cout << "outputFile is the name of file where the extracted features will be saved" << endl;
        std::cout << "kernelType : possible values are: metric , knn" << endl;
        std::cout << "kernelSize: size of the kernel, depending on the kernelType it might be either in cm for metric or k for knn" << endl;
        std::cout << "[loadPathPrefix]: path prefix for loading pcd files";
        std::cout << "[saveDirectory]: path prefix of destination directory for saving individual feature files separately.";
        return 0;
    }
    
    listFileName = argv[1];
    outputFileName = argv[2];
    kernelType = argv[3];
    kernelSize = argv[4];
    
    if (argc > 5) { pcdFilePath = argv[5]; }
    if (argc > 6) { shotDir = argv[6]; }
    
    transform (kernelType.begin (), kernelType.end (), kernelType.begin (), (int(*)(int)) tolower);
    
    if ( (kernelType.compare("metric") != 0) && (kernelType.compare("knn") != 0) )
    {
        cout << "Error: Invalid kernelType:" << kernelType << endl;
        return -1;
    }
    
    outputFile.open(outputFileName.c_str(), ios::out);
    listFile.open(listFileName.c_str(), ios::in);
    
    while (!listFile.eof())
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::SHOTEstimation<pcl::PointXYZ,pcl::Normal,pcl::SHOT> shotExtractor;
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree2 (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::SHOT>::Ptr cloudShot (new pcl::PointCloud<pcl::SHOT> ());
        
        listFile >> pcdFileName;
        cout << "Loading file:" << pcdFileName << endl;
        
        if (argc > 5) { pcdFileFullName = pcdFilePath + "/" + pcdFileName; }
        else { pcdFileFullName = pcdFileName; }
        
        //loading pcd file
        pcl::io::loadPCDFile (pcdFileFullName, *cloudIn);
        
        //estimating input normals
        cout << "calculating normal…" << endl;
        ne.setInputCloud (cloudIn);
        ne.setSearchMethod (tree);
        ne.setRadiusSearch (0.03);// Use all neighbors in a sphere of radius 3cm
        ne.compute (*cloud_normals);
    
        //extract shot features for the input cloud
        cout << "extracting shot features…" << endl;
        shotExtractor.setInputCloud (cloudIn);
        shotExtractor.setInputNormals(cloud_normals);
        shotExtractor.setSearchMethod(tree2);
        
        //if(kernelType.compare("metric")==0)
            shotExtractor.setRadiusSearch(atof(kernelSize.c_str()));
        //else
            //shotExtractor.setKSearch(atof(kernelSize.c_str()));
        
        shotExtractor.compute(*cloudShot);
        
        //save the result to the outputFile
        cout << "saving to file…" << endl;
        for(int i=0;i<cloudShot->size();i++)
        {
            for(int j=0;j<cloudShot->points[i].descriptor.size();j++)
            {
                outputFile << cloudShot->points[i].descriptor[j] << " ";
            }
            outputFile << endl;
        }
        
        if(argc>6)//save individual shot feature files in shotDir
        {
            shotFileFullName = shotDir + "/" + pcdFileName.substr(0,pcdFileName.size()-3) + "txt";
            cout << "saving individual shot file:" << shotFileFullName << endl;
            shotFile.open(shotFileFullName.c_str(),ios::out);
            for(int i=0;i<cloudShot->size();i++)
            {
                for(int j=0;j<cloudShot->points[i].descriptor.size();j++)
                {
                    shotFile << cloudShot->points[i].descriptor[j] << " ";
                }
                shotFile << endl;
            }
            shotFile.close();
        }
        
        cout << " done" << endl;
    }
    listFile.close();
    outputFile.close();

    return 0;
}
