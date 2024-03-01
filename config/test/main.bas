dim i 
'T_AxisInit

BASE(0)
STEP_RATIO(-1,1)
ENCODER_RATIO(1,-1)
BASE(1)
STEP_RATIO(-1,1)
ENCODER_RATIO(1,-1)
BASE(3)
STEP_RATIO(-1,1)
ENCODER_RATIO(1,-1)
BASE(5)
STEP_RATIO(-1,1)
ENCODER_RATIO(1,-1)
BASE(6)
STEP_RATIO(-1,1)
ENCODER_RATIO(1,-1)

?"RUN T_Robot6"
runtask 1,T_Robot6

DELAY(1000)
BASE(0,1,2,3,4,5)
speed = 3,3,3,3,3,3
MOVEABS(0,0,0,0,0,0)

' 矢量长度轴
BASE(15)
ATYPE = 0
'UNITS = 1000
DPOS = 0
MPOS = 0

WAIT UNTIL IDLE
?"HOME"

DELAY(1000)
?"RUN T_Robot95"
runtask 2,T_Robot95

DELAY(1000)
T_mode=1
'DELAY(100)
'T_mode2=2	'建立叠加
'
'DELAY(1000)

?"Ready!"
' 正逆解模式切换: 0-初始, 1-正解, 2-逆解
GLOBAL kinematic = 0
WHILE 1
	if T_mode < 0 and kinematic <> 2 THEN
		BASE(7,8,9,6) ' 配置关节轴
		CONNFRAME(93, 100, 20, 21, 22, 6) ' 第6/7/8轴为虚拟的XYZW轴
		
		BASE(0,1,2,3,4,5) ' 选择关节轴号
		CONNFRAME(6,0,7,8,9,10,11,12)
		
		WAIT LOADED
		?"Frame6-93 逆解模式"
		
		T_mode = 0
		kinematic = 2
		
	elseif T_mode > 0 and kinematic <> 1 THEN
		BASE(20,21,22,6)
		CONNREFRAME(93,100,7,8,9,6)
		
		BASE(7,8,9,10,11,12)
		CONNREFRAME(6,0,0,1,2,3,4,5)
		
		WAIT LOADED
		?"Frame6-93 正解模式"
		
		T_mode=0
		kinematic = 1
	endif
wend

end

GLOBAL SUB TEST222()
	BASE(7,8,9,11)
	moveabs(2100, -100, 1000,50)
	BASE(7,8,9,11)
	MSPHERICALABS(2100,500,1000,2300,0,1000,0,-10)
	
	
END SUB

GLOBAL sub test(value,value2)
		
	BASE(7,8,9,11)
	SPEED = value,value,value
	base(20,21,22,11,6)
	SPEED = value,value,value
	moveabs(2100,-100,1000,67.473,0)
	
	WAIT UNTIL IDLE
	?"直线去到圆弧起点完成"
	
	
	BASE(7,8,9,11)
	SPEED = value2,value2,value2
	
	base(20,21,22,11,6)
	SPEED = value2,value2,value2
	MSPHERICALABS(2100,500,1000,2300,0,1000,0,67.473,100)
	
	?"圆弧ok"
end sub

GLOBAL sub test333(value,value2)
		
	BASE(7,8,9,   10,11,12,6)
	SPEED = value,value,value,value,value,value,value
	BASE(20,21,22,10,11,12,6)
	SPEED = value,value,value,value,value,value,value
	moveabs(1179.2910, -94.9300+100, 182.4590, 180, -19.9590, 180,0)
	
	WAIT UNTIL IDLE
	?"直线去到圆弧起点完成"
	
	
	BASE(7,8,9,   10,11,12,6)
	SPEED = value2,value2,value2,value2,value2,value2,value2
	
	base(20,21,22,10,11,12,6)
	SPEED = value2,value2,value2,value2,value2,value2,value2
	MSPHERICALABS(1167.0510, -361.6500+100, 182.4590,1312.8110, -257.4300+100, 182.4590,0,174.8200, -19.9590, 152.9600,100)
	
	?"圆弧ok"
end sub
