#include "Sync_inter.h"

SyncInter::SyncInter()
{
}

struct SyncInter::Q SyncInter::NLerp(const Q& q1, const Q& q2, const double& t)
{
	std::cout << " System Running into NLerp Function.  " << std::endl;
	struct Q NLerp_temp;
	NLerp_temp.x = q1.x * (1 - t) + q2.x * t;
	NLerp_temp.y = q1.y * (1 - t) + q2.y * t;
	NLerp_temp.z = q1.z * (1 - t) + q2.z * t;
	NLerp_temp.w = q1.w * (1 - t) + q2.w * t;

	// Lerp������
	double Norm = 0.0;
	Norm = sqrt(NLerp_temp.x * NLerp_temp.x + NLerp_temp.y * NLerp_temp.y
		+ NLerp_temp.z * NLerp_temp.z + NLerp_temp.w * NLerp_temp.w);
	NLerp_temp.x = NLerp_temp.x / Norm;
	NLerp_temp.y = NLerp_temp.y / Norm;
	NLerp_temp.z = NLerp_temp.z / Norm;
	NLerp_temp.w = NLerp_temp.w / Norm;

	std::cout << " NLerp Done!  " << std::endl;
	return NLerp_temp;
}

void SyncInter::GenTimestampQuaternion()
{
	std::cout << " System Running into GenTimestampQuaternion Function.  " << std::endl;
	srand(time(NULL));
	int cam_num = 1;
	cam_t.push_back(T0_CAM);
	for (double cam_time = 0.0; cam_time < DURATION; )
	{
		double random = rand() % (N + 1) / (double)(N + 1);
		cam_time = T0_CAM + (double)cam_num * (1.0 / FREQ_CAM) + 0.5 * (1.0 / FREQ_CAM) * random;
		// std::cout << "cam_time Number�� " << cam_time << std::endl;
		cam_num++;
		// std::cout << " Number of cam_num: " << cam_num << std::endl;
		cam_t.push_back(cam_time);
	}

	Q Q_update = { 0,0,0,1 };
	int imu_num = 1;
	imu_t.push_back(T0_IMU);
	imu_q.push_back(Q_update);
	for (double imu_time = 0.0; imu_time < DURATION; )
	{
		double random = rand() % (N + 1) / (double)(N + 1);
		imu_time = T0_IMU + (double)imu_num * (1.0 / FREQ_IMU) + 0.5 * (1.0 / FREQ_IMU) * random;
		Q_update.x = Q_update.x + 0.1 * rand();
		Q_update.y = Q_update.y + 0.1 * rand();
		Q_update.z = Q_update.z + 0.1 * rand();
		Q_update.w = Q_update.w + 0.1 * rand();

		double Norm = 0.0;
		Norm = sqrt(Q_update.x * Q_update.x + Q_update.y * Q_update.y
			+ Q_update.z * Q_update.z + Q_update.w * Q_update.w);
		Q_update.x = Q_update.x / Norm;
		Q_update.y = Q_update.y / Norm;
		Q_update.z = Q_update.z / Norm;
		Q_update.w = Q_update.w / Norm;

		imu_num++;
		imu_t.push_back(imu_time);
		imu_q.push_back(Q_update);
	}
	std::cout << " GenTimestampQuaternion Done!  " << std::endl;
	return;
}

// �����������棬���������߶�ȡ����
bool SyncInter::CalImuPose(const double & tc, const std::vector<double>& imu_t, const std::vector<Q>& imu_q)
{
	//double Tc = tc;
	std::cout << " System Running into CalImuPose Function.  " << std::endl;
	int n = imu_t.size();
	int left = 0, right = n - 1;
	int id = n;
	while (left <= right)
	{
		int mid = ((right - left) >> 2) + left;
		if (tc <= imu_t[mid]) {
			id = mid;
			right = mid - 1;
		}
		else {
			left = mid + 1;
		}
	}
	Q qc;
	qc = NLerp(imu_q[id - 1], imu_q[id], tc);
	//result.push_back(std::pair<double, Q>(Tc, qc));
	result_t.push(tc);
	result_q.push(qc);
	std::cout << " CalImuPose Done!  " << std::endl;
	return true;
}

std::vector<double> SyncInter::GetCamt()
{
	return std::vector<double>(cam_t);
}

std::vector<double> SyncInter::GetImut()
{
	return std::vector<double>(imu_t);
}

std::vector<SyncInter::Q> SyncInter::GetImuq()
{
	return std::vector<SyncInter::Q>(imu_q);
}

void SyncInter::producer()
{
	GenTimestampQuaternion();
	//SyncInter a;
	for (int i = 0; i < GetCamt().size(); i++)
	{
		// �����ó�cpu��������cpu �ı��ε��ȣ��������߳�ʹ��,��һ����ٲ������
		// this_thread::sleep_for(chrono::seconds(1));
		std::unique_lock<std::mutex> lk(m);
		//GenTimestampQuaternion();

		double tc = (double)GetCamt()[i];
		// ����ֿ����в�Ʒ���ȴ�������������������
		while (notify || !result_q.empty())
		{
			cond.wait(lk);
		}
		// �ֿ����޲�Ʒ�ǣ�װ��
		//CalImuPose(tc, GetImut(), GetImuq())
		if (CalImuPose(tc, GetImut(), GetImuq()))
		{
		}
		// �����в�Ʒ��֪ͨ
		notify = true;
		// ֪ͨ�����߿���ȡ��Ʒ
		cond.notify_one();
	}

	// ֪ͨ�����߲�������
	done = true;
	cond.notify_one();
}

void SyncInter::consumer()
{
	while (!done)
	{
		// ��������������Դ��unique_lockһ��ʵ�������ͽ���
		std::unique_lock<std::mutex> lk(m);
		// �ȴ�������֪ͨ����Դ
		while (!notify)
		{
			cond.wait(lk);
		}
		// ����в��գ�����
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
			// ֪ͨ�����ֿ߲��������㣬������Ʒ
			notify = false;
			cond.notify_one();
		}
	}
}


