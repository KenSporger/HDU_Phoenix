#include<iostream>
#include<vector>
#include<random>
#include<Buff.h>

using namespace std;
struct PointCloud3d
{
	double x,y,z;
};

bool RansacCircle(vector<PointCloud3d>& cloud, double* circlePara)
{
	int cloudSize = cloud.size();    
	int maxIterCnt = cloudSize / 2;  //最大迭代次数
	double maxErrorThreshold = 0.02; //最大误差阈值（其实就是点到圆周的距离阈值）
	int consensusCntThreshold = maxIterCnt;

	if (!cloudSize || cloudSize <= 3)
	{
		return false;
	}

	//在点云链表的下标取值范围内产生随机数，目的是可以随机选择一个点云
	std::default_random_engine rng;
	std::uniform_int_distribution <int> uniform(0, cloudSize - 1);
	rng.seed(777);  //seed函数可以接收任意整数作为实参
	
	vector<int> selectIndexs;            //随机选择的点云的索引
	vector<PointCloud3d> selectPoints;   //随机选择的点云对象
	vector<int> consensusIndexs;         //满足一致性条件的点云的索引

	double centerX = 0, centerY = 0, R = 0; //平面圆参数
	double modelMeanError = 0;              //平均误差
	int bestConsensusCnt = 0;               //满足一致性的点的个数
	int iter = 0;                           //迭代次数

	//开始迭代计算
	while (iter < maxIterCnt)
	{
		selectIndexs.clear();
		selectPoints.clear();
		//在原始点云中随机取出3个点，因为至少需要3个点才能确定一个平面圆
		for (int c = 0; c < 3; ++c)
		{
			selectPoints.push_back(cloud.at(uniform(rng)));
		}

		const double& x1 = selectPoints.at(0).x;
		const double& y1 = selectPoints.at(0).y;
	
		const double& x2 = selectPoints.at(1).x;
		const double& y2 = selectPoints.at(1).y;
		
		const double& x3 = selectPoints.at(2).x;
		const double& y3 = selectPoints.at(2).y;
		
		//利用三个点坐标计算平面圆的参数
		double xa = (x1 + x2) / 2.0, ya = (y1 + y2) / 2.0;
		double xb = (x1 + x3) / 2.0, yb = (y1 + y3) / 2.0;

		double ka = (x1 - x2) / (y2 - y1);
		double kb = (x1 - x3) / (y3 - y1);

		centerX = (yb - ya + ka * xa - kb * xb) / (ka - kb);
		centerY = ka * centerX - ka * xa + ya;
		R = sqrt((centerX - xa)*(centerX - xa) + (centerY - ya)*(centerY - ya));

		double meanError = 0;
		vector<int> tmpConsensusIndexs; //满足一致性条件的点索引集合，这只是临时索引集合，需要随时更新。
		for (int j = 0; j < cloudSize; ++j)
		{
			const PointCloud3d& point = cloud.at(j);
			double distance = abs(R - sqrt((point.x - centerX)*(point.x - centerX) + (point.y - centerY)*(point.y - centerY)));
			if (distance < maxErrorThreshold)
			{
				tmpConsensusIndexs.push_back(j);
			}
			meanError += distance;
		}

		if (tmpConsensusIndexs.size() >= bestConsensusCnt && tmpConsensusIndexs.size() >= consensusCntThreshold)
		{
			bestConsensusCnt = consensusIndexs.size();  // 更新一致性索引集合元素个数
			modelMeanError = meanError / cloudSize;
			consensusIndexs.clear();
			consensusIndexs = tmpConsensusIndexs;        // 更新一致性索引集合
			circlePara[0] = centerX;  //圆心X
			circlePara[1] = centerY;  //圆心Y
			circlePara[2] = R;        //半径
		}
		iter++;
	}
	return true;
}