	'''''''''''''''''''''''''
	'任务编码：无
	'文件功能：EtherCAT总线扫描初始化
	'传递参数：需要提前修改设置，各变量的值
	'输出函数：无
	'返回值：ECAT_InitEnable    总线初始化状态   -1--未进行 0--初始化错误 1--初始化完成
	'备注：※※※ 使用前注意设置好以下全局常量的值，SYS_ZFEATURE指令不支持的机型 请手动输入对应参数 ※※※
	'备注：※※※ 使用前注意查看 Sub_SetNodePara 函数中是否有你使用的驱动型号 ※※※
	'备注：※※※ Sub_SetNodePara 函数中将每圈脉冲数修改成 10000 ※※※
	'作者：Shio
	'''''''''''''''''''''''''
global sub T_AxisInit()
	'控制器最大轴数
	DIM ControlMaxAxis
	ControlMaxAxis = SYS_ZFEATURE(0)
	'GLOBAL CONST ControlMaxAxis = 
	'支持电机个数
	DIM RealAxisMax
	RealAxisMax = SYS_ZFEATURE(1)
	'GLOBAL CONST RealAxisMax = 
	
	'槽位号，（单总线控制器缺省0，具体查看硬件手册）
	CONST Bus_Slot = 0
	
	'本地脉冲轴起始编号
	CONST LocalAxis_Start = 6
	
	'本地脉冲轴轴数量
	CONST LocalAxis_Num = 0
	
	'总线轴起始编号
	CONST BusAxis_Start = 0
	
	'总线初始化状态 -1--未进行 0--初始化错误 1--初始化完成
	DIM ECAT_InitEnable
		ECAT_InitEnable = -1
	'延迟3秒，等待驱动器上电，不同驱动器自身上电时间不同，具体根据驱动器调整延时
	DELAY(5000)
	'SERVO_PERIOD = 500 '通讯周期
	? "SlOT0总线通讯周期：",SERVO_PERIOD,"us"
	RAPIDSTOP(2)
	'初始化还原轴类型
	FOR i = 0 TO ControlMaxAxis - 1
		
		AXIS_ADDRESS(i) = 0
		AXIS_ENABLE(i) = 0
		ATYPE(i) = 0
		WAITIDLE(i)
		
	NEXT
	'本地轴重新映射
	FOR i=0 TO LocalAxis_Num -1
		AXIS_ADDRESS(LocalAxis_Start+i)= (-1<<16) + i	'将本地轴0-->i映射到轴LocalAxis_Start-->LocalAxis_Start+i
		ATYPE(LocalAxis_Start+i)=1	'轴类型
	NEXT
	ECAT_Init() '调用初始化函数
'	Move_Axis()	'每8个轴插补
end sub

'/*************************************************************
'Description:		//总线轴初始化
'Input:				//
'Input:				//
'Input:				//
'Output:			// ECAT_InitEnable=ON -->初始化完成标志
'Return:			//
'*************************************************************/
DIM NodeSum_Num ,BusAxis_Num ,NodeAxis_Num '设备总数、总线轴总数、每个节点上的电机数
DIM Drive_Vender,Drive_Device,Drive_Alias '驱动厂商编号、驱动设备编号、驱动设备拨码ID
SUB ECAT_Init()
	
'	LOCAL NodeSum_Num ,BusAxis_Num ,NodeAxis_Num '设备总数、总线轴总数、每个节点上的电机数
'	LOCAL Drive_Vender,Drive_Device,Drive_Alias '驱动厂商编号、驱动设备编号、驱动设备拨码ID
	
	RAPIDSTOP(2)
	'初始化还原轴类型
	FOR i = 0 TO ControlMaxAxis - 1
		
		AXIS_ADDRESS(i) = 0
		AXIS_ENABLE(i) = 0
		ATYPE(i) = 0
		WAITIDLE(i)
		
	NEXT
	'本地轴重新映射
	FOR i=0 TO LocalAxis_Num -1
		AXIS_ADDRESS(LocalAxis_Start+i)= (-1<<16) + i	'将本地轴0-->i映射到轴LocalAxis_Start-->LocalAxis_Start+i
		ATYPE(LocalAxis_Start+i)=1	'轴类型
	NEXT
	
	ECAT_InitEnable = -1
	
	SLOT_STOP(Bus_Slot)
	DELAY(200)
	
	SYSTEM_ZSET = SYSTEM_ZSET OR 128
	SLOT_SCAN(Bus_Slot)
	IF RETURN THEN
		NodeSum_Num = NODE_COUNT(Bus_Slot)
		? "slot0总线扫描成功，连接设备数：",NodeSum_Num
		'总线轴总数，从0开始计数
		BusAxis_Num = 0
		FOR i = 0 TO NodeSum_Num - 1
			
			NodeAxis_Num = NODE_AXIS_COUNT(Bus_Slot,i) '读取设备电机数
			Drive_Vender = NODE_INFO(Bus_Slot,i,0) '读取驱动器厂商
			Drive_Device = NODE_INFO(Bus_Slot,i,1) '读取设备编号
			Drive_Alias = NODE_INFO(Bus_Slot,i,3) '读取设备拨码ID
			
			FOR j = 0 TO NodeAxis_Num - 1
				
				AXIS_ADDRESS(BusAxis_Num+BusAxis_Start) = (Bus_Slot<<16)+ BusAxis_Num + 1'映射轴号
				ATYPE(BusAxis_Num+BusAxis_Start) = 65 '设置控制模式 65-位置 66-速度 67-转矩 详细参照AXISSTATUS
				DRIVE_PROFILE(BusAxis_Num+BusAxis_Start) = 5 '驱动器PDO设置,驱动器默认设置-- -1 位置模式--0  速度模式--20+  力矩模式--30+
				DISABLE_GROUP(BusAxis_Num+BusAxis_Start) '每轴单独分组
				
				Sub_SetNodePara(i,Drive_Vender,Drive_Device)	'设置特殊总线参数
				
				BusAxis_Num = BusAxis_Num + 1 '总线轴计数+1
				
			NEXT
			
		NEXT
		
		? "SlOT0轴扫描映射完成，连接总线轴数：",BusAxis_Num
		
		DELAY(100)
		SLOT_START(Bus_Slot)
		WA(10) ' 延迟3秒，等待驱动器时钟同步，不同驱动器时间不同，具体根据驱动器调整延时
		IF RETURN THEN 
			
			? "SlOT0开始清除驱动器报警"
			FOR i = BusAxis_Start TO  BusAxis_Start + BusAxis_Num - 1
				
				BASE(i)
				DRIVE_CLEAR(0)
				WA(10)
				DRIVE_CONTROLWORD(i) = 128 ' 伺服错误清除
				WA(10)
				DRIVE_CONTROLWORD(i)=6 ' 伺服shutdown 
				WA(10)
				'DRIVE_CONTROLWORD(i)=7 ' 伺服disable voltage
				'WA(10)
				DRIVE_CONTROLWORD(i)=15 ' 伺服fault reset
				WA(10)		
				
			NEXT
			
			DELAY(100)	
			? "SlOT0控制器报警清除完成"
			DATUM(0) ' 清除所有轴的错误状态。
			DELAY(1000)
			?"SlOT0开始伺服使能"
			WDOG = 1
			FOR i = BusAxis_Start TO  BusAxis_Start + BusAxis_Num - 1
				
				AXIS_ENABLE(i) = 1
				
			NEXT
			?"SlOT0伺服使能完成"
			
			ECAT_InitEnable = 1
			
		ELSE
			
			?"SlOT0总线开启失败"
			ECAT_InitEnable = 0
			
		ENDIF
	
	ELSE
		
		?"SlOT0总线扫描失败"
		ECAT_InitEnable = 0
		
	ENDIF
	
ENDSUB

'/*************************************************************
'Description:		//总线对端节点特殊参数设置
'Input:				//iNode -> 设备号
'Input:				//iVender -> 厂商编号
'Input:				//iDevice -> 设备编号
'Output:			// 
'Return:			//
'*************************************************************/
SUB Sub_SetNodePara(iNode,iVender,iDevice)
	
	IF iVender = $41B AND iDevice = $1ab0 THEN ' 正运动24088脉冲扩展轴
		
		SDO_WRITE(Bus_Slot,iNode,$6011,j*$800,5,5)			'设置扩展脉冲轴ATYPE类型
		SDO_WRITE(Bus_Slot,iNode,$6012,j*$800,6,0)			'设置扩展脉冲轴INVERT_STEP脉冲输出模式
		NODE_IO(Bus_Slot,iNode) = 1200 + 32*iNode				'设置240808上IO的起始映射地址
		'?"SlOT0设备："iNode,"起始IO为："TOSTR (1200 + 32*iNode, 6, 0)
	
	ELSEIF iVender = $41B AND iDevice = $1918 THEN ' 正运动EIO
		
		NODE_IO(Bus_Slot,iNode) = 512 + 32*iNode				'设置240808上IO的起始映射地址
		'?"SlOT0设备："iNode,"起始IO为："TOSTR (512 + 32*iNode, 6, 0)
		
	ELSEIF iVender = $41B AND iDevice = $0 THEN ' 正运动EIO
		
		NODE_IO(Bus_Slot,iNode) = 1000 + 32*iNode				'设置240808上IO的起始映射地址
		'?"SlOT0设备："iNode,"起始IO为："TOSTR (1000 + 32*iNode, 6, 0)
		
	ELSEIF iVender = $66f THEN '松下驱动器
		
		SDO_WRITE(Bus_Slot,iNode,$6085,0,7,4290000000) '异常减速度
		SDO_WRITE(Bus_Slot,iNode,$6092,1,7,10000) '设置每圈脉冲
		
	ELSEIF iVender = $100000 AND iDevice = $c010d THEN '汇川驱动器
		
		SDO_WRITE(Bus_Slot,iNode,$6091,1,7,10000)
		SDO_WRITE(Bus_Slot,iNode,$6091,2,7,1) '每圈脉冲数
		
	ELSEIF iVender = $1DD AND iDevice = $6080 THEN '台达B3E驱动器
		
		SDO_WRITE(Bus_Slot,iNode,$6093,1,7,16777216)
		SDO_WRITE(Bus_Slot,iNode,$6093,2,7,10000) '每圈脉冲数
	
	ELSEIF iVender = $10001000 AND iDevice = $100 THEN ' 大研伺服
		
		SDO_WRITE(Bus_Slot,iNode,$6091,1,7,8388608) '设置电机转动一圈需要的脉冲数为10000（大研伺服）
		SDO_WRITE(Bus_Slot,iNode,$6091,2,7,10000)
		
	ELSEIF iVender = $116c7 AND iDevice = $3e0402 THEN '禾川X3E伺服
		
		SDO_WRITE(Bus_Slot,iNode,$6092,1,7,10000) '设置每圈脉冲
		
	ELSEIF iVender = $60a AND iDevice = $1 THEN ' 埃斯顿伺服
		
		SDO_WRITE(Bus_Slot,iNode,$6093,1,7,131072)
		SDO_WRITE(Bus_Slot,iNode,$6093,2,7,10000) '每圈脉冲数
		
	ELSEIF iVender = $4321 AND iDevice = $a100 THEN ' 雷赛双驱
		
		SDO_WRITE(Bus_Slot,iNode,$6092,1,7,10000) '设置轴1每圈脉冲
		
		SDO_WRITE(Bus_Slot,iNode,$6892,1,7,10000) '设置轴2每圈脉冲
		
	ELSEIF iVender = $99998888 AND iDevice = $20001 THEN ' 雷赛单驱
		
		SDO_WRITE(Bus_Slot,iNode,$6092,1,7,10000) '设置轴每圈脉冲
		
	ELSEIF iVender = $9a AND iDevice = $30924 AND DRIVE_PROFILE(iNode) = -1 THEN ' ELMO(诶莫驱动)配置csp、csv、cst任意切换模式
		
		SDO_WRITE (Bus_Slot, iNode, $1c12, $0 ,5 ,$0) ' 禁用RxPDO,禁用后才可以修改内容
		WA(50)
		SDO_WRITE (Bus_Slot, iNode, $1c13, $0 ,5 ,$0) ' 禁用TxPDO,禁用后才可以修改内容
		WA(50)
		
		SDO_WRITE (Bus_Slot, iNode, $1a07, $0 ,5 ,$0) ' 禁用
		SDO_WRITE (Bus_Slot, iNode, $1a07, $1 ,7 ,$60410010) ' 状态字
		SDO_WRITE (Bus_Slot, iNode, $1a07, $2 ,7 ,$60770010) ' 当前力矩
		SDO_WRITE (Bus_Slot, iNode, $1a07, $3 ,7 ,$60640020) ' 反馈位置
		SDO_WRITE (Bus_Slot, iNode, $1a07, $4 ,7 ,$60fd0020) ' 驱动器IO输入
		SDO_WRITE (Bus_Slot, iNode, $1a07, $5 ,7 ,$60b90010) ' probe状态
		SDO_WRITE (Bus_Slot, iNode, $1a07, $6 ,7 ,$60ba0020) ' probe位置1
		SDO_WRITE (Bus_Slot, iNode, $1a07, $7 ,7 ,$60bb0020) ' probe位置2
		SDO_WRITE (Bus_Slot, iNode, $1a07, $0 ,5 ,$7) ' 启用分配
		
		SDO_WRITE (Bus_Slot, iNode, $1607, $0 ,5 ,$0) ' 禁用
		SDO_WRITE (Bus_Slot, iNode, $1607, $1 ,7 ,$60400010) ' 控制字
		SDO_WRITE (Bus_Slot, iNode, $1607, $2 ,7 ,$60710010) ' 周期力矩
		SDO_WRITE (Bus_Slot, iNode, $1607, $3 ,7 ,$60ff0020) ' 周期速度
		SDO_WRITE (Bus_Slot, iNode, $1607, $4 ,7 ,$607a0020) ' 目标位置
		SDO_WRITE (Bus_Slot, iNode, $1607, $5 ,7 ,$60b80010) ' probe设置
		SDO_WRITE (Bus_Slot, iNode, $1607, $6 ,7 ,$60720010) ' 力矩限制
		SDO_WRITE (Bus_Slot, iNode, $1607, $7 ,7 ,$60600008) ' 控制模式
		SDO_WRITE (Bus_Slot, iNode, $1607, $0 ,5 ,$7) ' 启用分配
		
		SDO_WRITE (Bus_Slot, iNode, $1c12, $1 ,6 ,$1607) ' RxPDO分配对象
		SDO_WRITE (Bus_Slot, iNode, $1c12, $0 ,5 ,$1) ' 启用分配Elmo_EtherCAT Application Manual_Password_Removed
		SDO_WRITE (Bus_Slot, iNode, $1c13, $1 ,6 ,$1a07) ' TxPDO分配对象
		SDO_WRITE (Bus_Slot, iNode, $1c13, $0 ,5 ,$1) ' 启用分配
		
		SDO_WRITE (Bus_Slot, iNode, $6040, $0 ,6 ,$0) ' 状态初始化
		WA(50)
		SDO_WRITE (Bus_Slot, iNode, $6040, $0 ,6 ,$6) ' 伺服shutdown 
		WA(50)
		SDO_WRITE (Bus_Slot, iNode, $6040, $0 ,6 ,$7) ' 伺服disable voltage
		WA(50)
		SDO_WRITE (Bus_Slot, iNode, $6040, $0 ,6 ,$F) ' 伺服fault reset
		
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



