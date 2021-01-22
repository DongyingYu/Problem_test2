#include "Sync_inter.h"

bool notify = false;
bool done = false;
std::condition_variable cond;
std::mutex m;

std::vector<double> cam_t;
std::vector<double> imu_t;
std::vector<Q> imu_q;

std::queue<double> result_t;
std::queue<Q> result_q;

void producer()
{
	//SyncInter a;
	for (int i = 0; i < cam_t.size(); i++)
	{
		// 主动让出cpu，不参与cpu 的本次调度，让其他线程使用,等一秒后再参与调度
		// this_thread::sleep_for(chrono::seconds(1));
		std::unique_lock<std::mutex> lk(m);
		//GenTimestampQuaternion(cam_t, imu_t, imu_q);

		double tc = (double)cam_t[i];
		// 如果仓库中有产品，等待消费者消费完再生产
		while (notify || !result_q.empty())
		{
			cond.wait(lk);
		}
		// 仓库中无产品是，装入
		//CalImuPose(tc, GetImut(), GetImuq())
		if (CalImuPose(tc, imu_t, imu_q, result_t, result_q))
		{
		}
		// 设置有产品的通知
		notify = true;
		// 通知消费者可以取产品
		cond.notify_one();
	}

	// 通知消费者不生产了
	done = true;
	cond.notify_one();
}

void consumer()
{
	while (!done)
	{
		// 上锁保护共享资源，unique_lock一次实现上锁和解锁
		std::unique_lock<std::mutex> lk(m);
		// 等待生产者通知有资源
		while (!notify)
		{
			cond.wait(lk);
		}
		// 如队列不空，读出
		while (!result_q.empty())
		{
			std::cout << "Consumer Output Message:  \n"
				<< "Time Massage: " << result_t.front()
				<< "Quaternion Message: \n"
				<< result_q.front().x << "  "
				<< result_q.front().y << "  "
				<< result_q.front().z << "  "
				<< result_q.front().w << "  "
				<< std::endl;
			result_t.pop();
			result_q.pop();
			// 通知生产者仓库容量不足，生产产品
			notify = false;
			cond.notify_one();
		}
	}
}

int main()
{
	GenTimestampQuaternion(cam_t, imu_t, imu_q);
	std::thread t1(producer);
	std::thread t2(consumer);
	//std::thread t2(&SyncInter::consumer, a);

	t1.join();
	t2.join();


	system("pause");
	return 0;
}
