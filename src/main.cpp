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

	double compute_time_sum = 0.0;

	while (frame_count < (frame_per_sec * sec_interval))
	{
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		auto compute_start = std::chrono::high_resolution_clock::now();

		highway.stepHighway(egoVelocity, time_us, frame_per_sec, viewer);

		auto compute_end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> compute_duration = compute_end - compute_start;
		compute_time_sum += compute_duration.count();

		if(!highway.tools.estimations.empty() && !highway.tools.ground_truth.empty())
		{
			VectorXd rmse = highway.tools.CalculateRMSE(highway.tools.estimations, highway.tools.ground_truth);
			if(rmse.size() == 4)
			{
				rmse_count++;
				rmse_sum += rmse;
				VectorXd rmse_avg = rmse_sum / rmse_count;
				std::cout << "\nAverage RMSE: [" << rmse_avg.transpose() << "]\n" << std::endl;
			}
		}

		viewer->spinOnce(1000/frame_per_sec);
		frame_count++;
		time_us = 1000000 * frame_count / frame_per_sec;
	}

	std::cout << "Total highway computation time: " << compute_time_sum << "s\n";
	std::cout << "Average per frame: "
			<< compute_time_sum / frame_count << "s\n";
}
