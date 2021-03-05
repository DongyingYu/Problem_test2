/*
 * @Author: Donguying
 * @Date: 2021-03-05 21:51:17
 * @LastEditTime: 2021-03-05 21:56:34
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /Problem_test2/Problem_two/main.cpp
 */
#include "Sync_inter.h"


int main()
{
	//std::vector<double> cam_t;
	//std::vector<double> imu_t;
	//std::vector<SyncInter::Q> imu_q;

	SyncInter a;

	std::thread t1(&SyncInter::producer, &a);
	std::thread t2(&SyncInter::consumer, &a);
	//std::thread t2(&SyncInter::consumer, a);

	t1.join();
	t2.join();


	// system("pause");
	return 0;
}
