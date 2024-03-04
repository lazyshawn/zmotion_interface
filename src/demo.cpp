#include <windows.h>
#include<iostream>

#include"zmotion_interface.h"

int test();
void commandCheckHandler(const char *command, int ret) {
	// 非 0则失败
	if (ret) {
		printf("%s return code is %d\n", command, ret);
	}
}

int main() {
	ZauxRobot robot;
	if (robot.connect("127.0.0.1") > 0) {
		std::cout << "Connect error" << std::endl;
		getchar();
		return 1;
	}

	//std::string basPath = "D:\\CIMC\\CppPro\\zmotion\\config\\test\\main.bas";
	//if (robot.load_basic_pragma(basPath.c_str()) > 0) {
	//	return 1;
	//}

	// 触发示波器
	robot.trigger_scope();

	robot.moveJ({ 0, -6.8221, 14.3160, -0.0000, 55.1156, 0, 0 });
	//robot.moveL();
	robot.swingL({ -200, 0, 0, 200 }, {0,0,1});
	//robot.swingC();
	//robot.swingC({ {800, 200, 600}, {300, 500, 1200} });

	// 断开连接
	robot.disconnect();
	return 0;
}

int test() {
	std::cout << "hello world" << std::endl;
	ZauxRobot robot;

	int ret;
	// 默认控制器 IP地址 192.168.0.11, 仿真器 IP 地址 127.0.0.1
	char *ip_addr = (char *)"127.0.0.1";
	// 控制器句柄
	ZMC_HANDLE handle = NULL;
	// 通过网口连接控制器
	if (ERR_SUCCESS != ZAux_OpenEth(ip_addr, &handle))	{
		printf("Connect controller failed!\nPress <Enter> exist!\n");
		handle = NULL;
		getchar();
		return -1;
	}
	printf("控制器连接成功！\n");

	// bas 程序路径
	std::string basPath = "D:\\CIMC\\zmotion\\robot_configuration.bas";
	// 加载 bas 程序
	if (ZAux_BasDown(handle, basPath.c_str(), 0) != 0) {
		printf("Error.");
		return 0;
	}
	// 等待 bas 程序下载
	Sleep(1000);
	// 轴号
	int m_nAxis = 0;

	// 单位正弦曲线数据写入 
	std::vector<float> sinTable(100);
	for (int i = 0; i < 100; ++i) {
		sinTable[i] = sin(2 * M_PI * i / 99);
	}

	int firstAxis = 0, secondAxis = 1;
	// 把凸轮表数据写入 Table 寄存器值
	ZAux_Direct_SetTable(handle, 200, 100, sinTable.data());
	// 触发示波器
	ZAux_Trigger(handle);

	// 切换到逆解模式：每次执行工具坐标系运动时需要绑定一次，同步计算关节坐标变化
	ZAux_Direct_Connframe(handle, robot.jointAxisIdx.size(), robot.jointAxisIdx.data(), 6, 0, robot.toolAxisIdx.size(), robot.toolAxisIdx.data());

	// 参考轴 1 运动到 100 位置时，跟随轴 0启动凸轮表运动
	/**
	* @param handle 句柄
	* @param 凸轮轴轴号
	* @param 凸轮表起始索引
	* @param 凸轮表终止索引
	* @param 位置比例
	* @param 一个凸轮运动周期的距离，运动周期 = 距离 / 凸轮轴速度
	* @param 参考轴轴号
	* @param 参考轴的关联方式
	* @param 参考轴位移
	*/
	ZAux_Direct_Cambox(handle, 6, 200, 299, 10000, 100, 8, 4, 0);

	ZAux_Direct_Single_Move(handle, 8, -500);

	Sleep(500);
	ret = ZAux_Close(handle);    //关闭连接 
	printf("connection closed!\n");
	handle = NULL;
	return 0;
}
