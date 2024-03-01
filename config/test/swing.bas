GLOBAL SUB Swing_L()

DIM dx = -200
DIM dy = 0
DIM dz = 0

DIM gx = 20
DIM gy = 21
DIM gz = 22

DIM i

' �л������ģʽ

' ����������
base(23,24,25,26,27,28)
atype = 0,0,0,0,0,0
UNITS = 1000,1000,1000,1000,1000,1000

' ͬ��������״̬
DPOS = DPOS(20), DPOS(21), DPOS(22), 0, 0, 0
MPOS = DPOS(20), DPOS(21), DPOS(22), 0, 0, 0

' �������˶�
' ���⹤������ӵ�͹����
ADDAX(23) AXIS(26)
ADDAX(24) AXIS(27)
ADDAX(25) AXIS(28)
' ͹������ӵ���ʵ������
ADDAX(26) AXIS(20)
ADDAX(27) AXIS(21)
ADDAX(28) AXIS(22)

' ���ٶ�
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
' ����λ��
DIM primeDist = VR(primeAxis)
' �ڶ�Ƶ��
DIM freq = 0.2
' ʵ����������
primeAxis = primeAxis + 23
' ������
DIM numPeriod = (primeDist \ SPEED(primeAxis)) * freq


' ��������д��Table
DIM sinTableBeg = 1500
FOR i = 0 TO 100
	TABLE(sinTableBeg + i)  = sin(2 * PI * i / 99)
NEXT

' ����͹�ֱ�
?"numPeriod = "numPeriod
CAMBOX(sinTableBeg, sinTableBeg+99, 100000 * 0, primeDist / numPeriod, primeAxis, 4, 0) AXIS(26)
CAMBOX(sinTableBeg, sinTableBeg+99, 100000 * 1, primeDist / numPeriod, primeAxis, 4, 0) AXIS(27)
CAMBOX(sinTableBeg, sinTableBeg+99, 100000 * 0, primeDist / numPeriod, primeAxis, 4, 0) AXIS(28)


' �������˶�
BASE(23,24,25)
MOVE(dx, dy, dz)

ENDSUB
