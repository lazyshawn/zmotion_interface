GLOBAL SUB Swing_C()

DIM i

' �������
' �ڶ�Ƶ��
DIM freq = 1
' Ŀ��λ��
DIM endX = 100, endY = 100, endZ = 100
' �м�λ��
DIM viaX = 100, viaY = 100, viaZ = 100

' ��ȡ��ǰλ��
DIM curX = MPOS(20), curY = MPOS(21), curZ = MPOS(22)

' ����Բ��λ��
DIM center(3)
LOCAL a2 = (curX-viaX)*(curX-viaX) + (curY-viaY)*(curY-viaY) + (curZ-viaZ)*(curZ-viaZ)
LOCAL b2 = (endX-viaX)*(endX-viaX) + (endY-viaY)*(endY-viaY) + (endZ-viaZ)*(endZ-viaZ)
center(0) = 1, center(1) = 1, center(2) = 1

' ������ת�Ƕ�
DIM theta = 100
' Բ���뾶
DIM radius = 100

' �������˶�����
DIM dist = theta / 180 * PI * radius

' �˶�ʱ��
DIM totalT = dist / SPEED(20)

' ��������
DIM numPeriod = (dist \ SPEED(20)) * freq

' ����͹��
DIM numInterp = numPeriod * 99 + 1    ' ��ֵ����
LOCAL dq = theta / (numInterp - 1)    ' ��ֵ���Ӧ�ĽǶ�֮��
LOCAL curTheta = 0
DIM sinTableBeg = 1500                ' ͹�ֱ���ʼλ��
FOR i = 0 TO numInterp
	' ��ת�Ƕ�
	curTheta = curTheta + dq
    ' ���߷���
	' ƫ�Ʒ���
	' ��¼͹�ֱ�
	TABLE(sinTableBeg + i)  = sin(2 * PI * i / 99)
NEXT

' �����˶�

ENDSUB
