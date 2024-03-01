GLOBAL SUB Swing_C()

DIM i

' 输入参数
' 摆动频率
DIM freq = 1
' 目标位置
DIM endX = 100, endY = 100, endZ = 100
' 中间位置
DIM viaX = 100, viaY = 100, viaZ = 100

' 读取当前位置
DIM curX = MPOS(20), curY = MPOS(21), curZ = MPOS(22)

' 计算圆心位置
DIM center(3)
LOCAL a2 = (curX-viaX)*(curX-viaX) + (curY-viaY)*(curY-viaY) + (curZ-viaZ)*(curZ-viaZ)
LOCAL b2 = (endX-viaX)*(endX-viaX) + (endY-viaY)*(endY-viaY) + (endZ-viaZ)*(endZ-viaZ)
center(0) = 1, center(1) = 1, center(2) = 1

' 计算旋转角度
DIM theta = 100
' 圆弧半径
DIM radius = 100

' 计算总运动距离
DIM dist = theta / 180 * PI * radius

' 运动时间
DIM totalT = dist / SPEED(20)

' 计算周期
DIM numPeriod = (dist \ SPEED(20)) * freq

' 计算凸轮
DIM numInterp = numPeriod * 99 + 1    ' 插值点数
LOCAL dq = theta / (numInterp - 1)    ' 插值点对应的角度之差
LOCAL curTheta = 0
DIM sinTableBeg = 1500                ' 凸轮表起始位置
FOR i = 0 TO numInterp
	' 旋转角度
	curTheta = curTheta + dq
    ' 切线方向
	' 偏移方向
	' 记录凸轮表
	TABLE(sinTableBeg + i)  = sin(2 * PI * i / 99)
NEXT

' 叠加运动

ENDSUB
