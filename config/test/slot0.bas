	'''''''''''''''''''''''''
	'������룺��
	'�ļ����ܣ�EtherCAT����ɨ���ʼ��
	'���ݲ�������Ҫ��ǰ�޸����ã���������ֵ
	'�����������
	'����ֵ��ECAT_InitEnable    ���߳�ʼ��״̬   -1--δ���� 0--��ʼ������ 1--��ʼ�����
	'��ע�������� ʹ��ǰע�����ú�����ȫ�ֳ�����ֵ��SYS_ZFEATUREָ�֧�ֵĻ��� ���ֶ������Ӧ���� ������
	'��ע�������� ʹ��ǰע��鿴 Sub_SetNodePara �������Ƿ�����ʹ�õ������ͺ� ������
	'��ע�������� Sub_SetNodePara �����н�ÿȦ�������޸ĳ� 10000 ������
	'���ߣ�Shio
	'''''''''''''''''''''''''
global sub T_AxisInit()
	'�������������
	DIM ControlMaxAxis
	ControlMaxAxis = SYS_ZFEATURE(0)
	'GLOBAL CONST ControlMaxAxis = 
	'֧�ֵ������
	DIM RealAxisMax
	RealAxisMax = SYS_ZFEATURE(1)
	'GLOBAL CONST RealAxisMax = 
	
	'��λ�ţ��������߿�����ȱʡ0������鿴Ӳ���ֲᣩ
	CONST Bus_Slot = 0
	
	'������������ʼ���
	CONST LocalAxis_Start = 6
	
	'����������������
	CONST LocalAxis_Num = 0
	
	'��������ʼ���
	CONST BusAxis_Start = 0
	
	'���߳�ʼ��״̬ -1--δ���� 0--��ʼ������ 1--��ʼ�����
	DIM ECAT_InitEnable
		ECAT_InitEnable = -1
	'�ӳ�3�룬�ȴ��������ϵ磬��ͬ�����������ϵ�ʱ�䲻ͬ���������������������ʱ
	DELAY(5000)
	'SERVO_PERIOD = 500 'ͨѶ����
	? "SlOT0����ͨѶ���ڣ�",SERVO_PERIOD,"us"
	RAPIDSTOP(2)
	'��ʼ����ԭ������
	FOR i = 0 TO ControlMaxAxis - 1
		
		AXIS_ADDRESS(i) = 0
		AXIS_ENABLE(i) = 0
		ATYPE(i) = 0
		WAITIDLE(i)
		
	NEXT
	'����������ӳ��
	FOR i=0 TO LocalAxis_Num -1
		AXIS_ADDRESS(LocalAxis_Start+i)= (-1<<16) + i	'��������0-->iӳ�䵽��LocalAxis_Start-->LocalAxis_Start+i
		ATYPE(LocalAxis_Start+i)=1	'������
	NEXT
	ECAT_Init() '���ó�ʼ������
'	Move_Axis()	'ÿ8����岹
end sub

'/*************************************************************
'Description:		//�������ʼ��
'Input:				//
'Input:				//
'Input:				//
'Output:			// ECAT_InitEnable=ON -->��ʼ����ɱ�־
'Return:			//
'*************************************************************/
DIM NodeSum_Num ,BusAxis_Num ,NodeAxis_Num '�豸������������������ÿ���ڵ��ϵĵ����
DIM Drive_Vender,Drive_Device,Drive_Alias '�������̱�š������豸��š������豸����ID
SUB ECAT_Init()
	
'	LOCAL NodeSum_Num ,BusAxis_Num ,NodeAxis_Num '�豸������������������ÿ���ڵ��ϵĵ����
'	LOCAL Drive_Vender,Drive_Device,Drive_Alias '�������̱�š������豸��š������豸����ID
	
	RAPIDSTOP(2)
	'��ʼ����ԭ������
	FOR i = 0 TO ControlMaxAxis - 1
		
		AXIS_ADDRESS(i) = 0
		AXIS_ENABLE(i) = 0
		ATYPE(i) = 0
		WAITIDLE(i)
		
	NEXT
	'����������ӳ��
	FOR i=0 TO LocalAxis_Num -1
		AXIS_ADDRESS(LocalAxis_Start+i)= (-1<<16) + i	'��������0-->iӳ�䵽��LocalAxis_Start-->LocalAxis_Start+i
		ATYPE(LocalAxis_Start+i)=1	'������
	NEXT
	
	ECAT_InitEnable = -1
	
	SLOT_STOP(Bus_Slot)
	DELAY(200)
	
	SYSTEM_ZSET = SYSTEM_ZSET OR 128
	SLOT_SCAN(Bus_Slot)
	IF RETURN THEN
		NodeSum_Num = NODE_COUNT(Bus_Slot)
		? "slot0����ɨ��ɹ��������豸����",NodeSum_Num
		'��������������0��ʼ����
		BusAxis_Num = 0
		FOR i = 0 TO NodeSum_Num - 1
			
			NodeAxis_Num = NODE_AXIS_COUNT(Bus_Slot,i) '��ȡ�豸�����
			Drive_Vender = NODE_INFO(Bus_Slot,i,0) '��ȡ����������
			Drive_Device = NODE_INFO(Bus_Slot,i,1) '��ȡ�豸���
			Drive_Alias = NODE_INFO(Bus_Slot,i,3) '��ȡ�豸����ID
			
			FOR j = 0 TO NodeAxis_Num - 1
				
				AXIS_ADDRESS(BusAxis_Num+BusAxis_Start) = (Bus_Slot<<16)+ BusAxis_Num + 1'ӳ�����
				ATYPE(BusAxis_Num+BusAxis_Start) = 65 '���ÿ���ģʽ 65-λ�� 66-�ٶ� 67-ת�� ��ϸ����AXISSTATUS
				DRIVE_PROFILE(BusAxis_Num+BusAxis_Start) = 5 '������PDO����,������Ĭ������-- -1 λ��ģʽ--0  �ٶ�ģʽ--20+  ����ģʽ--30+
				DISABLE_GROUP(BusAxis_Num+BusAxis_Start) 'ÿ�ᵥ������
				
				Sub_SetNodePara(i,Drive_Vender,Drive_Device)	'�����������߲���
				
				BusAxis_Num = BusAxis_Num + 1 '���������+1
				
			NEXT
			
		NEXT
		
		? "SlOT0��ɨ��ӳ����ɣ���������������",BusAxis_Num
		
		DELAY(100)
		SLOT_START(Bus_Slot)
		WA(10) ' �ӳ�3�룬�ȴ�������ʱ��ͬ������ͬ������ʱ�䲻ͬ���������������������ʱ
		IF RETURN THEN 
			
			? "SlOT0��ʼ�������������"
			FOR i = BusAxis_Start TO  BusAxis_Start + BusAxis_Num - 1
				
				BASE(i)
				DRIVE_CLEAR(0)
				WA(10)
				DRIVE_CONTROLWORD(i) = 128 ' �ŷ��������
				WA(10)
				DRIVE_CONTROLWORD(i)=6 ' �ŷ�shutdown 
				WA(10)
				'DRIVE_CONTROLWORD(i)=7 ' �ŷ�disable voltage
				'WA(10)
				DRIVE_CONTROLWORD(i)=15 ' �ŷ�fault reset
				WA(10)		
				
			NEXT
			
			DELAY(100)	
			? "SlOT0����������������"
			DATUM(0) ' ���������Ĵ���״̬��
			DELAY(1000)
			?"SlOT0��ʼ�ŷ�ʹ��"
			WDOG = 1
			FOR i = BusAxis_Start TO  BusAxis_Start + BusAxis_Num - 1
				
				AXIS_ENABLE(i) = 1
				
			NEXT
			?"SlOT0�ŷ�ʹ�����"
			
			ECAT_InitEnable = 1
			
		ELSE
			
			?"SlOT0���߿���ʧ��"
			ECAT_InitEnable = 0
			
		ENDIF
	
	ELSE
		
		?"SlOT0����ɨ��ʧ��"
		ECAT_InitEnable = 0
		
	ENDIF
	
ENDSUB

'/*************************************************************
'Description:		//���߶Զ˽ڵ������������
'Input:				//iNode -> �豸��
'Input:				//iVender -> ���̱��
'Input:				//iDevice -> �豸���
'Output:			// 
'Return:			//
'*************************************************************/
SUB Sub_SetNodePara(iNode,iVender,iDevice)
	
	IF iVender = $41B AND iDevice = $1ab0 THEN ' ���˶�24088������չ��
		
		SDO_WRITE(Bus_Slot,iNode,$6011,j*$800,5,5)			'������չ������ATYPE����
		SDO_WRITE(Bus_Slot,iNode,$6012,j*$800,6,0)			'������չ������INVERT_STEP�������ģʽ
		NODE_IO(Bus_Slot,iNode) = 1200 + 32*iNode				'����240808��IO����ʼӳ���ַ
		'?"SlOT0�豸��"iNode,"��ʼIOΪ��"TOSTR (1200 + 32*iNode, 6, 0)
	
	ELSEIF iVender = $41B AND iDevice = $1918 THEN ' ���˶�EIO
		
		NODE_IO(Bus_Slot,iNode) = 512 + 32*iNode				'����240808��IO����ʼӳ���ַ
		'?"SlOT0�豸��"iNode,"��ʼIOΪ��"TOSTR (512 + 32*iNode, 6, 0)
		
	ELSEIF iVender = $41B AND iDevice = $0 THEN ' ���˶�EIO
		
		NODE_IO(Bus_Slot,iNode) = 1000 + 32*iNode				'����240808��IO����ʼӳ���ַ
		'?"SlOT0�豸��"iNode,"��ʼIOΪ��"TOSTR (1000 + 32*iNode, 6, 0)
		
	ELSEIF iVender = $66f THEN '����������
		
		SDO_WRITE(Bus_Slot,iNode,$6085,0,7,4290000000) '�쳣���ٶ�
		SDO_WRITE(Bus_Slot,iNode,$6092,1,7,10000) '����ÿȦ����
		
	ELSEIF iVender = $100000 AND iDevice = $c010d THEN '�㴨������
		
		SDO_WRITE(Bus_Slot,iNode,$6091,1,7,10000)
		SDO_WRITE(Bus_Slot,iNode,$6091,2,7,1) 'ÿȦ������
		
	ELSEIF iVender = $1DD AND iDevice = $6080 THEN '̨��B3E������
		
		SDO_WRITE(Bus_Slot,iNode,$6093,1,7,16777216)
		SDO_WRITE(Bus_Slot,iNode,$6093,2,7,10000) 'ÿȦ������
	
	ELSEIF iVender = $10001000 AND iDevice = $100 THEN ' �����ŷ�
		
		SDO_WRITE(Bus_Slot,iNode,$6091,1,7,8388608) '���õ��ת��һȦ��Ҫ��������Ϊ10000�������ŷ���
		SDO_WRITE(Bus_Slot,iNode,$6091,2,7,10000)
		
	ELSEIF iVender = $116c7 AND iDevice = $3e0402 THEN '�̴�X3E�ŷ�
		
		SDO_WRITE(Bus_Slot,iNode,$6092,1,7,10000) '����ÿȦ����
		
	ELSEIF iVender = $60a AND iDevice = $1 THEN ' ��˹���ŷ�
		
		SDO_WRITE(Bus_Slot,iNode,$6093,1,7,131072)
		SDO_WRITE(Bus_Slot,iNode,$6093,2,7,10000) 'ÿȦ������
		
	ELSEIF iVender = $4321 AND iDevice = $a100 THEN ' ����˫��
		
		SDO_WRITE(Bus_Slot,iNode,$6092,1,7,10000) '������1ÿȦ����
		
		SDO_WRITE(Bus_Slot,iNode,$6892,1,7,10000) '������2ÿȦ����
		
	ELSEIF iVender = $99998888 AND iDevice = $20001 THEN ' ��������
		
		SDO_WRITE(Bus_Slot,iNode,$6092,1,7,10000) '������ÿȦ����
		
	ELSEIF iVender = $9a AND iDevice = $30924 AND DRIVE_PROFILE(iNode) = -1 THEN ' ELMO(��Ī����)����csp��csv��cst�����л�ģʽ
		
		SDO_WRITE (Bus_Slot, iNode, $1c12, $0 ,5 ,$0) ' ����RxPDO,���ú�ſ����޸�����
		WA(50)
		SDO_WRITE (Bus_Slot, iNode, $1c13, $0 ,5 ,$0) ' ����TxPDO,���ú�ſ����޸�����
		WA(50)
		
		SDO_WRITE (Bus_Slot, iNode, $1a07, $0 ,5 ,$0) ' ����
		SDO_WRITE (Bus_Slot, iNode, $1a07, $1 ,7 ,$60410010) ' ״̬��
		SDO_WRITE (Bus_Slot, iNode, $1a07, $2 ,7 ,$60770010) ' ��ǰ����
		SDO_WRITE (Bus_Slot, iNode, $1a07, $3 ,7 ,$60640020) ' ����λ��
		SDO_WRITE (Bus_Slot, iNode, $1a07, $4 ,7 ,$60fd0020) ' ������IO����
		SDO_WRITE (Bus_Slot, iNode, $1a07, $5 ,7 ,$60b90010) ' probe״̬
		SDO_WRITE (Bus_Slot, iNode, $1a07, $6 ,7 ,$60ba0020) ' probeλ��1
		SDO_WRITE (Bus_Slot, iNode, $1a07, $7 ,7 ,$60bb0020) ' probeλ��2
		SDO_WRITE (Bus_Slot, iNode, $1a07, $0 ,5 ,$7) ' ���÷���
		
		SDO_WRITE (Bus_Slot, iNode, $1607, $0 ,5 ,$0) ' ����
		SDO_WRITE (Bus_Slot, iNode, $1607, $1 ,7 ,$60400010) ' ������
		SDO_WRITE (Bus_Slot, iNode, $1607, $2 ,7 ,$60710010) ' ��������
		SDO_WRITE (Bus_Slot, iNode, $1607, $3 ,7 ,$60ff0020) ' �����ٶ�
		SDO_WRITE (Bus_Slot, iNode, $1607, $4 ,7 ,$607a0020) ' Ŀ��λ��
		SDO_WRITE (Bus_Slot, iNode, $1607, $5 ,7 ,$60b80010) ' probe����
		SDO_WRITE (Bus_Slot, iNode, $1607, $6 ,7 ,$60720010) ' ��������
		SDO_WRITE (Bus_Slot, iNode, $1607, $7 ,7 ,$60600008) ' ����ģʽ
		SDO_WRITE (Bus_Slot, iNode, $1607, $0 ,5 ,$7) ' ���÷���
		
		SDO_WRITE (Bus_Slot, iNode, $1c12, $1 ,6 ,$1607) ' RxPDO�������
		SDO_WRITE (Bus_Slot, iNode, $1c12, $0 ,5 ,$1) ' ���÷���Elmo_EtherCAT Application Manual_Password_Removed
		SDO_WRITE (Bus_Slot, iNode, $1c13, $1 ,6 ,$1a07) ' TxPDO�������
		SDO_WRITE (Bus_Slot, iNode, $1c13, $0 ,5 ,$1) ' ���÷���
		
		SDO_WRITE (Bus_Slot, iNode, $6040, $0 ,6 ,$0) ' ״̬��ʼ��
		WA(50)
		SDO_WRITE (Bus_Slot, iNode, $6040, $0 ,6 ,$6) ' �ŷ�shutdown 
		WA(50)
		SDO_WRITE (Bus_Slot, iNode, $6040, $0 ,6 ,$7) ' �ŷ�disable voltage
		WA(50)
		SDO_WRITE (Bus_Slot, iNode, $6040, $0 ,6 ,$F) ' �ŷ�fault reset
		
	ENDIF
	
ENDSUB

SUB Move_Axis()
	BASE(0,1)
	DPOS = 0,0
	MPOS = 0,0
	UNITS = 500,500
	SPEED = 500,500
	DELAY(100)
'	vmove(1)
'	forward
	WHILE 1
		MOVE(5000,5000)
		WAIT UNTIL IDLE(0) and idle(1)
		MOVE(-5000,-5000)
		WAIT UNTIL IDLE(0) and idle(1)
	WEND
END SUB



