GLOBAL SUB Swing_L()

DIM dx = -200
DIM dy = 0
DIM dz = 0

DIM gx = 20
DIM gy = 21
DIM gz = 22

DIM i

' 切换到逆解模式

' 设置虚拟轴
base(23,24,25,26,27,28)
atype = 0,0,0,0,0,0
UNITS = 1000,1000,1000,1000,1000,1000

' 同步虚拟轴状态
DPOS = DPOS(20), DPOS(21), DPOS(22), 0, 0, 0
MPOS = DPOS(20), DPOS(21), DPOS(22), 0, 0, 0

' 叠加轴运动
' 虚拟工具轴叠加到凸轮轴
ADDAX(23) AXIS(26)
ADDAX(24) AXIS(27)
ADDAX(25) AXIS(28)
' 凸轮轴叠加到真实工具轴
ADDAX(26) AXIS(20)
ADDAX(27) AXIS(21)
ADDAX(28) AXIS(22)

' 轴速度
VR(0) = ABS(dx)
VR(1) = ABS(dy)
VR(2) = ABS(dz)
DIM minT = 99999999999
DIM primeAxis = 0
FOR i = 0 TO 3
	LOCAL t = VR(i) / SPEED(20+i)
	IF t < minT THEN
		minT = t
		primeAxis = i
	ENDIF
NEXT
' 主轴位移
DIM primeDist = VR(primeAxis)
' 摆动频率
DIM freq = 0.2
' 实际主轴索引
primeAxis = primeAxis + 23
' 周期数
DIM numPeriod = (primeDist \ SPEED(primeAxis)) * freq


' 正弦曲线写入Table
DIM sinTableBeg = 1500
FOR i = 0 TO 100
	TABLE(sinTableBeg + i)  = sin(2 * PI * i / 99)
NEXT

' 跟随凸轮表
?"numPeriod = "numPeriod
CAMBOX(sinTableBeg, sinTableBeg+99, 100000 * 0, primeDist / numPeriod, primeAxis, 4, 0) AXIS(26)
CAMBOX(sinTableBeg, sinTableBeg+99, 100000 * 1, primeDist / numPeriod, primeAxis, 4, 0) AXIS(27)
CAMBOX(sinTableBeg, sinTableBeg+99, 100000 * 0, primeDist / numPeriod, primeAxis, 4, 0) AXIS(28)


' 虚拟轴运动
BASE(23,24,25)
MOVE(dx, dy, dz)

ENDSUB
