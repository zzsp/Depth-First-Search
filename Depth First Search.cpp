#pragma once 

#include <iostream>
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgproc/imgproc_c.h"
#include<opencv2/opencv.hpp>
#include <math.h>
#include"../include/DFS.h"

void testDPS()
{
	//Set Graph and used Graph
	cv::Mat graph = cv::Mat::ones(GRAPH_SIZE_HEIGHT, GRAPH_SIZE_WIDTH, CV_8UC1);
	for (int j = 1; j < 3; j++)
	{
		for (int i = 0; i < graph.cols - 1; i++)
		{
			graph.at<uchar>(j, i) = 0;
		}
	}
	cv::Mat used = cv::Mat::zeros(GRAPH_SIZE_HEIGHT, GRAPH_SIZE_WIDTH, CV_8UC1);

	//Set target point and start point and  target depth
	int goal[2] = { 99, 99 }, startpoint[2] = { 5, 0 };
	used.at<uchar>(startpoint[1], startpoint[0]) = USED_MAP_FLAG;
	graph.at<uchar>(goal[0], goal[1]) = AIMED_DEPTH;

	//DFS algrithm
	DFS::DepthFirstSearch DFS;
	used = DFS.DFSForDepth(graph, used, startpoint[0], startpoint[1], AIMED_DEPTH);

	//show
	cv::imshow("finished", used);
	std::cout << used << std::endl;
	cv::waitKey(0);
}

int main()
{
	testDPS();
	return 0;
}
