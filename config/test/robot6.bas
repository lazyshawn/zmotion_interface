
global sub T_Robot6()
	'''''电机、机械手参数定义
	dim LargeZ '基座的垂直高度
	dim L1 '1 轴到 2 轴的 X 偏移；转盘中心到大摆臂中心的偏移。
	dim L2 '大摆臂长度
	dim L3 '3 轴中心到 4 轴中心距离
	dim L4 '4 轴到 5 轴的距离。
	dim D5 '5 转一圈，6 转动的圈数，0 表示不关联。
	dim PulesVROneCircle '虚拟姿态轴一圈脉冲数
	dim SmalLZ '末端到 5 轴的垂直距离
	dim SmalLX, SmalLY '末端到转盘中心的 XY 偏移。 0 位置的时候
	dim InitRx, InitRy, InitRz '初始的姿态，弧度单位。 0，0，0 指向 z 正向
	LargeZ=422
	L1=193.539
	L2=800.928
	L3=174.472
	l4=1000.5
	D5=0
	
	SmalLX=0 + 5.347
	SmalLY=0 -2.061
	SmalLZ=112 +669.191 '112
	InitRx=0
	InitRy=28.890/180*pi
	InitRz=0
	PulesVROneCircle=360*1000
	dim u_m1 '电机 1 一圈脉冲数
	dim u_m2 '电机 2 一圈脉冲数
	dim u_m3 '电机 3 一圈脉冲数
	dim u_m4 '电机 4 一圈脉冲数
	dim u_m5 '电机 5 一圈脉冲数
	dim u_m6 '电机 6 一圈脉冲数
	u_m1=131072
	u_m2=131072
	u_m3=131072
	u_m4=131072
	u_m5=131072
	u_m6=131072
	dim i_1 '关节 1 传动比
	dim i_2 '关节 2 传动比
	dim i_3 '关节 3 传动比
	dim i_4 '关节 4 传动比
	dim i_5 '关节 5 传动比
	dim i_6 '关节 6 传动比
	i_1=152	'j1-152
	i_2=153	'153
	i_3=153	'-153
	i_4=2652/31	'-2652/31
	i_5=1750/33	'-1750/33
	i_6=27744/800	'-27744/800
	dim u_j1 '关节 1 实际一圈脉冲数
	dim u_j2 '关节 2 实际一圈脉冲数
	dim u_j3 '关节 3 实际一圈脉冲数
	dim u_j4 '关节 4 实际一圈脉冲数
	dim u_j5 '关节 5 实际一圈脉冲数
	dim u_j6 '关节 6 实际一圈脉冲数
	u_j1=u_m1*i_1
	u_j2=u_m2*i_2
	u_j3=u_m3*i_3
	u_j4=u_m4*i_4
	u_j5=u_m5*i_5
	u_j6=u_m6*i_6


	'''''关节轴设置
	BASE(0,1,2,3,4,5)     '选择关节轴号 0、1、2、3、4、5
'	atype=1,1,1,1,1,1     '轴类型设为脉冲轴
	uNITS = u_j1/360,u_j2/360,u_j3/360,u_j4/360,u_j5/360 ,u_j6/360    '把 units 设成每°脉冲数
	'uNITS =1000,1000,1000,1000,1000,1000
	'DPOS=0,0,0,0,0,0      '设置关节轴的位置，此处要根据实际情况来修改。
	'MPOS=0,0,0,0,0,0
	FS_LIMIT = 90,90,150,360,165,360
	rS_LIMIT = -90,-90,-150,-360,-165,-360
	speed=10,10,10,10,10,10   '速度参数设置
	accel=100,100,100,100,100,100
	decel=100,100,100,100,100,100
	CLUTCH_RATE=0,0,0,0,0,0 '使用关节轴的速度和加速度限制

	'''''''''''''''''''''设置启动拐角减速'''''''''''''''''''''
	merge=on '开启连续插补
	corner_mode = 2 '启动拐角减速
	decel_angle = 15 * (PI/180) '开始减速的角度 15 度
	stop_angle = 45 * (PI/180) '降到最低速度的角度 45 度


	'''''''''''''''''''''虚拟轴设置'''''''''''''''''''''
	BASE(7,8,9,10,11,12)
	ATYPE=0,0,0,0,0,0 '设置为虚拟轴
	TABLE(0,LargeZ,L1,L2,L3,L4,D5,u_j1,u_j2,u_j3,u_j4,u_j5,u_j6,PulesVROneCircle,SmalLX,SmalLY,SmalLZ,InitRx,InitRy,InitRz) '根据手册说明填写参数
	UNITS=1000,1000,1000,1000,1000,1000 '运动精度，要提前设置，中途不能变化
	SPEED=10,10,10,10,10,10
	ACCEL=100,100,100,100,100,100
	global T_mode = 0

	'零点设置 确保机械手德units给下去了 base轴号要确保是0-5
	for i = 0 to 6
		if i = 2 OR i = 4 THEN
			mpos(i) =  (ENCODER(i) - vr(100+i)) /UNITS(i)
			DPOS(i) = mpos(i)
		ELSE
			mpos(i) =  -1*((ENCODER(i) - vr(100+i)) /UNITS(i))
			DPOS(i) = mpos(i)
		ENDIF
	NEXT

	'''''''''''''''''''''建立机械手连接'''''''''''''''''''''
'	while 1
'		if T_mode < 0 then
'			BASE(0,1,2,3,4,5) '选择关节轴号
'			CONNFRAME(6,0,7,8,9,10,11,12) '启动逆解连接。
'			WAIT LOADED '等待运动加载，此时会自动调整虚拟轴的位置。
'			?"逆解模式"
'			T_mode = 0
'		elseif T_mode > 0 then '输入 0 下降沿触发
'			BASE(7,8,9,10,11,12) '选择虚拟轴号
'			CONNREFRAME(6,0,0,1,2,3,4,5) '启动正解连接。
'			WAIT LOADED '等待运动加载。
'			?"正解模式"
'			T_mode = 0
'		endif
'	wend
end sub