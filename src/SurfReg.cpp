#include <iostream>
#include <fstream>
#include <string>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/icp.h>
#include "ThreadMutexObject.h"

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
int count = 0;

pcl::PointCloud<pcl::PointXYZI>::Ptr evaCloud (new pcl::PointCloud<pcl::PointXYZI>);

float getScore(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input)
{
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree->setInputCloud(mCloud);

    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    double totalSum = 0;

    //count.assignValue(0);

    for(size_t i = 0; i < input->size(); i++)
    {
        tree->nearestKSearch(input->at(i), 1, pointIdxNKNSearch, pointNKNSquaredDistance);
        pcl::PointXYZI tmppt;
        tmppt.x = input->at(i).x;// TODO
        tmppt.y = input->at(i).y;
        tmppt.z = input->at(i).z;
        if(sqrt(pointNKNSquaredDistance.at(0)) > 0.2){
            tmppt.intensity = -1;
            evaCloud->points.push_back(tmppt);
            continue;
        }

        totalSum += sqrt(pointNKNSquaredDistance.at(0));
        tmppt.intensity = sqrt(pointNKNSquaredDistance.at(0));
        evaCloud->points.push_back(tmppt);
        count++;
    }

    return totalSum / (double)input->size();
}

void computeAlignment()
{
    float value = getScore(rCloud);

    // if(value < 0.05)
    // {
    //     pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;

    //     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr aligned (new pcl::PointCloud <pcl::PointXYZRGBNormal>);

    //     icp.setInputSource(rCloud);
    //     icp.setInputTarget(mCloud);
    //     icp.setMaximumIterations(1);

    //     for(int i = 0; i < 10; i++)
    //     {
    //         icp.align(*aligned, icp.getFinalTransformation());
    //         iteration++;
    //     }

    //     value = std::min(getScore(aligned), value);
    // }

    std::cout << "diff per point: " << value << std::endl;

    //done.assignValue(true);
}

int main(int argc, char ** argv)
{
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    std::string reconstructionFile;
    pcl::console::parse_argument(argc, argv, "-r", reconstructionFile);

    std::string modelFile;
    pcl::console::parse_argument(argc, argv, "-m", modelFile);


    if(reconstructionFile.length() == 0 || modelFile.length() == 0)
    {
        std::cout << "Please provide input files with -r and -m" << std::endl;
        exit(1);
    }

    pcl::PCLPointCloud2 rPoints;

    pcl::toPCLPointCloud2(*rCloud, rPoints);
    std::cout << rPoints.fields.size() << std::endl;

    for(int i = 0; i < rPoints.fields.size(); i++)
    {
        if(rPoints.fields.at(i).name.compare("curvature") == 0)
        {
            rPoints.fields.at(i).name = "radius";
        }
    }

    pcl::io::loadPLYFile(reconstructionFile, rPoints);

    pcl::fromPCLPointCloud2(rPoints, *rCloud);

    pcl::PCLPointCloud2 mPoints;

    pcl::toPCLPointCloud2(*mCloud, mPoints);

    for(int i = 0; i < mPoints.fields.size(); i++)
    {
        if(mPoints.fields.at(i).name.compare("curvature") == 0)
        {
            mPoints.fields.erase(mPoints.fields.begin() + i);
            i--;
        }
    }

    pcl::io::loadPLYFile(modelFile, mPoints);

    pcl::fromPCLPointCloud2(mPoints, *mCloud);

    float radius = 10.0f;
    float theta = 0.0f;
    float phi = 0.0f;

    pcl::visualization::PCLVisualizer cloudViewer("SurfReg");
    cloudViewer.setBackgroundColor(0, 0, 0);
    cloudViewer.initCameraParameters();
    cloudViewer.setCameraPosition(radius * sin(theta) * cos(phi),
                                  radius * sin(theta) * sin(phi),
                                  radius * cos(theta),
                                  0,
                                  1,
                                  0);
    cloudViewer.setSize(1680, 1050);

    if(pcl::console::find_argument(argc, argv, "-f") != -1)
    {
        cloudViewer.setFullScreen(true);
    }

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> color(rCloud);
    cloudViewer.addPointCloud<pcl::PointXYZRGBNormal>(rCloud, color, "Cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> colorM(1, 0, 0);
    cloudViewer.addPointCloud<pcl::PointXYZRGBNormal>(mCloud, colorM, "MCloud");
    cloudViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "MCloud");


    cloudViewer.spinOnce(1, true);
    cloudViewer.removeShape("text");

    int countVal = count;
    int sizeInput = rCloud->size();

        //std::cout << "Count number: " << countVal << std::endl; 
        //std::cout << "totla number: " << sizeInput << " " << countVal/sizeInput << std::endl;
    if(countVal == rCloud->size())
    {
        std::stringstream strs;

        //strs << "Aligning... " << iteration.getValue() << "/" << 10;

        cloudViewer.addText(strs.str(), 20, 20, 50, 1, 0, 0, "text");
    }
    else
    {
        std::stringstream strs;

        strs << "Scoring... " << countVal << "/" << rCloud->size();

        cloudViewer.addText(strs.str(), 20, 20, 50, 1, 0, 0, "text");
    }

    cloudViewer.setCameraPosition(radius * sin(theta) * cos(phi),
                                  radius * sin(theta) * sin(phi),
                                  radius * cos(theta),
                                  0,
                                  1,
                                  0);
    theta += 0.005;
    phi += 0.005;

    computeAlignment();
    // computeThread->join();

    // delete computeThread;
    std::ofstream outfile;
    outfile.open("error.txt");
    for(int i=0;i < evaCloud->size(); i++){
        outfile << evaCloud->at(i).x << " " << evaCloud->at(i).y << " "
        << evaCloud->at(i).z << " " << evaCloud->at(i).intensity << "\n";
    }
    outfile.close();

}
