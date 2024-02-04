Header header 

int32[] startRingIndex			#每个scanID起始特征点的索引
int32[] endRingIndex			#每个scanID终止特征点的索引

float32 startOrientation		#sweep的起始角度
float32 endOrientation			#sweep的终止角度
float32 orientationDiff			#sweep的角度差

bool[]    segmentedCloudGroundFlag	#是否是地面点
uint32[]  segmentedCloudColInd 		#在range image中的列索引
float32[] segmentedCloudRange		#每个点的range值



