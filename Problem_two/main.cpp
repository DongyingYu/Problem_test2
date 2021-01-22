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


	system("pause");
	return 0;
}
