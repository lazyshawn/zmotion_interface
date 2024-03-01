global sub T_Robot95()
''''''''''''''电机、机械手参数定义
dim   ProangleX1,ProangleZ1,ProangleX2,ProangleZ2,ProangleX3,ProangleZ3
ProangleX1 = 90  '（x轴）和（辅助轴1基于固定坐标系的xy平面的投影）的夹角角度 单位 度
ProangleZ1 = 0  '（辅助轴1）和（辅助轴1基于固定坐标系的xy平面的投影）的夹角角度。 单位 度 
'
'ProangleX2 = 84.17366  '（x轴）和（辅助轴2基于固定坐标系的xy平面的投影）的夹角角度 单位 度
'ProangleZ2 = 0  '（辅助轴2）和（辅助轴2基于固定坐标系的xy平面的投影）的夹角角度。 单位 度 
'
'ProangleX3 = 0  '（x轴）和（辅助轴3基于固定坐标系的xy平面的投影）的夹角角度 单位 度
'ProangleZ3 = 90  '（辅助轴3）和（辅助轴3基于固定坐标系的xy平面的投影）的夹角角度。 单位 度 

'''''''''''关节轴设置
dim u_m1 '电机 1 一圈脉冲数 
u_m1=131072 
dim i_1 '关节 1 传动比 
i_1=10	'j1-152 
dim u_j1 '关节 1 实际一圈脉冲数 
u_j1=u_m1*i_1/240

base(7,8,9,6)                                   '定义实际轴
'atype = 0,0,0,0,0,0		                        '轴类型为脉冲轴
units=units(7),units(8),units(9),u_j1		                   'units设为每°的脉冲
				                    '设置关节轴的位置，此处要根据实际情况来修改
creep = 10,10,10,10	 		                    '轴回零时低速度
speed = 10,10,10,10 	                    '速度参数设置
accel = 100,100,100,100                    '设置关节轴的加速度限制
decel = 100,100,100,100                       '设置关节轴的减速度
clutch_rate =0,0,0                             '使用关节轴的速度和加速度限制


''''''''''虚拟轴设置
base(20,21,22)                        
atype = 0,0,0   '设置为虚拟轴
TABLE(100,ProangleX1,ProangleZ1 )               '末端点到X,Y,Z偏移的距离                      
UNITS = 1000,1000,1000        						'此脉冲当量要提前设置，中途不能变化 
DPOS=0,0,0     '设置关节轴的位置，此处要根据实际情况来修改。
speed = 10,10,10,10 	                    '速度参数设置
accel = 100,100,100,100                    '设置关节轴的加速度限制
decel = 100,100,100,100                       '设置关节轴的减速度
clutch_rate =1000,1000,1000                             '使用关节轴的速度和加速度限制

merge = on	               						'打开连续插补
CORNER_MODE =2           						'启动拐角减速
DECEL_ANGLE = 15 * (PI/180)					'开始减速的角度 15度		
STOP_ANGLE = 45 * (PI/180)						'降到最低速度的角度45度
global T_mode2
T_mode2=0

'''''''''建立机械手连接
'while 1
'	if T_mode2>0 then       '输入0上升沿触发
'		BASE(7,8,9,6) '配置关节轴
'		CONNFRAME(93,100,20,21,22,6) '第6/7/8轴为虚拟的XYZW轴，启动逆解连接。
'		wait loaded  '等待运动加载，此时会自动调整虚拟轴的位置
'		?"逆解模式"	
'	elseif T_mode2<0 then   '输入0下降沿触发
'		base(20,21,22,6)           '选择虚拟轴号
'		CONNREFRAME(93,100,7,8,9,6) '第0/1/2轴为关节轴，启动正解连接
'		wait loaded
'		?"正解模式"	
'   endif
'wend


ENDSUB
