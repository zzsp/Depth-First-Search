#pragma once

#include <iostream>
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgproc/imgproc_c.h"
#include<opencv2/opencv.hpp>
#include <math.h>

#define AIMED_DEPTH 150				
#define GRAPH_SIZE_HEIGHT 100		
#define GRAPH_SIZE_WIDTH 100
#define USED_MAP_FLAG 255
#define UNUSED_MAP_FLAG 0
#define FORBIDDEN_POINT 0

using namespace cv;
using namespace std;

namespace DFS
{
	class DepthFirstSearch
	{
	private:
		
		int px[4] = { -1, 0, 1, 0 };
		int py[4] = { 0, 1, 0, -1 };//Direction of search:left->down->right->up
		int finishedflag = 0;

	public:
		//DFS For Coordinates
		cv::Mat DFSForCoordinates(const cv::Mat& graph, cv::Mat& used, const int x, const int y, int goal[2])
		{

			for (std::size_t i = 0; i != 4; i++)
			{
				if (!finishedflag)
				{
					if (x == goal[0] && y == goal[1])
					{
						finishedflag = 1;
						std::cout << "DFS finished!"<< std::endl;
						return used;
					}
					int new_x = x + px[i], new_y = y + py[i];
					if (new_x >= 0 && new_x < graph.cols && new_y >= 0 && new_y < graph.rows && graph.at<uchar>(new_y, new_x) != FORBIDDEN_POINT)
					{
						if (used.at<uchar>(new_y, new_x) == UNUSED_MAP_FLAG)
						{
							used.at<uchar>(new_y, new_x) = USED_MAP_FLAG;
							DFSForCoordinates(graph, used, new_x, new_y, goal);
						}
					}
				}
			}
			if (!finishedflag)
				used.at<uchar>(y, x) = UNUSED_MAP_FLAG;
			return used;
		}


		//DFS For Depth
		cv::Mat DFSForDepth(const cv::Mat& graph, cv::Mat& used, const int x, const int y, const int& depth)
		{
			for (std::size_t i = 0; i != 4; i++)
			{
				if (!finishedflag)
				{
					if (graph.at<uchar>(y, x) == depth)
					{
						finishedflag = 1;
						std::cout << "DFS finished!";
						return used;
					}
					int new_x = x + px[i], new_y = y + py[i];
					if (new_x >= 0 && new_x < graph.cols && new_y >= 0 && new_y < graph.rows && graph.at<uchar>(new_y, new_x) != FORBIDDEN_POINT)
					{
						if (used.at<uchar>(new_y, new_x) == UNUSED_MAP_FLAG)
						{
							used.at<uchar>(new_y, new_x) = USED_MAP_FLAG;
							DFSForDepth(graph, used, new_x, new_y, depth);
						}
					}
				}
			}
			if (!finishedflag)
				used.at<uchar>(y, x) = UNUSED_MAP_FLAG;
			return used;
		};

	};
}
