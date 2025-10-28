/* \author Aaron Brown, Moorissa Tjokro */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

//#include "render/render.h"
#include "highway.h"
#include "optimization_config.h"
#include <chrono>

int main(int argc, char** argv)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	// set camera position and angle
	viewer->initCameraParameters();
	float x_pos = 0;
	viewer->setCameraPosition ( x_pos-26, 0, 15.0, x_pos+25, 0, 0, 0, 0, 1);

	Highway highway(viewer);

	//initHighway(viewer);

	int frame_per_sec = 30;
	int sec_interval = 10;
	int frame_count = 0;
	int time_us = 0;

	// RMSE tracking variables
	int rmse_count = 0;
	VectorXd rmse_sum = VectorXd::Zero(4);  // Initialize sum vector for x,y,vx,vy

	double egoVelocity = 25;

	auto chrono_start = std::chrono::high_resolution_clock::now();
	while (frame_count < (frame_per_sec*sec_interval))
	{
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		//stepHighway(egoVelocity, time_us, frame_per_sec, viewer);
			highway.stepHighway(egoVelocity, time_us, frame_per_sec, viewer);

			// Compute and track RMSE statistics
			if(!highway.tools.estimations.empty() && !highway.tools.ground_truth.empty())
			{
				VectorXd rmse = highway.tools.CalculateRMSE(highway.tools.estimations, highway.tools.ground_truth);
				if(rmse.size() == 4)
				{
					rmse_count++;
					rmse_sum += rmse;  // Accumulate RMSE values
					VectorXd rmse_avg = rmse_sum / rmse_count;  // Compute running average
					std::cout << "Frame " << rmse_count
						<< "\nInstant RMSE: [" << rmse.transpose() << "]"
						<< "\nAverage RMSE: [" << rmse_avg.transpose() << "]\n" << std::endl;
				}
			}
		viewer->spinOnce(1000/frame_per_sec);
		frame_count++;
		time_us = 1000000*frame_count/frame_per_sec;

	}
	auto chrono_end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> duration = chrono_end - chrono_start;
	std::cout << "Loop runtime: " << duration.count() << "s \n";
}
