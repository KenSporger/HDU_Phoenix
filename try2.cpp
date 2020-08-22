#include<iostream>
#include<vector>
#include<random>
#include<Buff.h>

using namespace std;

bool RansacCircle(const vector<polarData> &lines, Point2f &interPoint)
{
	int linesAmount = lines.size();    
	int maxIterCnt = linesAmount / 2;  //最大迭代次数
	double maxErrorThreshold = 0.02; //最大误差阈值, max distance by cacuDistanceOfLine
	int consensusCntThreshold = maxIterCnt;

	if (!linesAmount || linesAmount <= 3)
	{
		return false;
	}

	//在点云链表的下标取值范围内产生随机数，目的是可以随机选择一个点云
	std::default_random_engine rng;
	std::uniform_int_distribution <int> uniform(0, linesAmount - 1);
	rng.seed(777);  //seed函数可以接收任意整数作为实参
	
	vector<int> selectIndexs;            //随机选择的点云的索引
	vector<polarData> selectLines;   //随机选择的点云对象
	vector<int> consensusIndexs;         //满足一致性条件的点云的索引

	Point2f interP(0, 0);
	double modelMeanError = 0;              //平均误差
	int bestConsensusCnt = 0;               //满足一致性的点的个数
	int iter = 0;                           //迭代次数

	double scale;

	//开始迭代计算
	while (iter < maxIterCnt)
	{
		selectIndexs.clear();
		selectLines.clear();
		//在原始点云中随机取出2个
		for (int c = 0; c < 2; ++c)
		{
			selectLines.push_back(lines.at(uniform(rng)));
		}

       scale  = selectLines[0].theta_cos * selectLines[1].theta_sin - selectLines[0].theta_sin * selectLines[1].theta_cos;
	   interP.x = (-selectLines[0].theta_sin * selectLines[1].rho + selectLines[0].rho * selectLines[1].theta_sin) / scale;
	   interP.y = (-selectLines[0].rho * selectLines[1].theta_cos + selectLines[0].theta_cos * selectLines[1].rho) / scale;


		double meanError = 0;
		vector<int> tmpConsensusIndexs; //满足一致性条件的点索引集合，这只是临时索引集合，需要随时更新。
		for (int j = 0; j < linesAmount; ++j)
		{
			polarData& line1 = lines[j];
			double distance = abs(cacuDistanceOfLine(line1, interP));
			if (distance < maxErrorThreshold)
			{
				tmpConsensusIndexs.push_back(j);
			}
			meanError += distance;
		}

		if (tmpConsensusIndexs.size() >= bestConsensusCnt && tmpConsensusIndexs.size() >= consensusCntThreshold)
		{
			bestConsensusCnt = consensusIndexs.size();  // 更新一致性索引集合元素个数
			modelMeanError = meanError / linesAmount;
			consensusIndexs.clear();
			consensusIndexs = tmpConsensusIndexs;        // 更新一致性索引集合
			interPoint = interP;
		}
		iter++;
	}
	return true;
}