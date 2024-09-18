
#define UC100_DLL_EXPORT 

#if defined(UC100_DLL_EXPORT) 
	#define UC100API   __declspec(dllexport)
#else 
	#define UC100API   __declspec(dllimport)
#endif  

//F�ggv�ny visszat�r�si �rt�kek:
enum ReturnVal{
	UC100_OK,					//0 Command executed without error.
	UC100_DEVICE_NOT_OPENED,	//1 Device was not opened yet.
	UC100_DEVICE_NOT_FOUND,		//2 Device not found.
	UC100_FIRMWARE_ERROR,		//3 Firmware error.
	UC100_IO_ERROR,				//4 Communication error.
	UC100_MOVEMENT_IN_PROGRESS,	//5// Motion in progress. Can be cleared with the Stop() function.
	UC100_HOME_IN_PROGRESS,		//6 Homing in progress. Can be cleared with the Stop() function.
	UC100_ESTOP,				//7 Controller is in estop state. Can be cleared with the Stop() function.
	UC100_LIMIT,				//8 Limit input(s) active. Can be cleared with the LimitOverRide() function.
	UC100_PARAMETER_ERROR,		//9 Function parameter error. The function was called with errorous parameter(s).
	UC100_COMMAND_BUFFER_FULL,	//10 Function buffer is full.
	UC100_FW_UPDATE,			//11 Firmware update is in progress.
	UC100_PROBEPIN_ACTIVE,		//12 Probe input is active.
	UC100_NO_LOADED_LASER_DATA,	//13 Laser data not present.
	UC100_TIMEOUT				//14 Function could not complete in time.
};

enum DeviceType{
	Demo_AXBB = -19,			//-19. Demo AXBB controller.
	Demo_UC300ETH_UB1 = -18,	//-18. Demo UC300ETH-UB1 motherboard.
	Demo_UC300ETH_M45 = -17,	//-17. Demo UC300ETH-M45 motherboard.
	Demo_UC300ETH_5441 = -16,	//-16. Demo UC300ETH-5441 motherboard.
	Demo_UC300_5441 = -15,		//-15. Demo UC300-5441 motherboard.
	Demo_UC300ETH_STEPPER = -14,//-14. Demo UC300ETH-STEPPER motherboard. Note: Not implemented.
	Demo_UC300ETH_ISOBOB = -13,	//-13. Demo UC300ETH-ISOBOB motherboard. Note: Not implemented.
	Demo_UC300ETH_M44 = -12,	//-12. Demo UC300ETH-MK44 motherboard.
	Demo_UC300ETH_5LPT = -11,	//-11. Demo UC300ETH-5LPT motherboard.
	Demo_UC300ETH_Low = -10,	//-10. Demo UC300ETH module with outputs low level.
	Demo_UC300ETH_Hi = -9,		//-9.  Demo UC300ETH module with outputs high level.
	Demo_UC400ETH = -8,			//-8.  Demo UC400ETH controller.
	Demo_UC300_STEPPER = -7,	//-7.  Demo UC300-STEPPER motherboard. Note: Not implemented.
	Demo_UC300_ISOBOB = -6,		//-6.  Demo UC300-ISOBOB motherboard. Note: Not implemented.
	Demo_UC300_M44 = -5,		//-5.  Demo UC300-MK44 motherboard.
	Demo_UC300_5LPT = -4,		//-4.  Demo UC300-5LPT motherboard.
	Demo_UC300_Low = -3,		//-3.  Demo UC300 module with outputs low level.
	Demo_UC300_Hi = -2,			//-2.  Demo UC300 module with outputs high level.
	Demo_UC100 = -1,			//-1.  Demo UC100 controller.
	Demo_mode = 0,				//0.   Demo UC100 controller.
	UC100 = 1,					//1.   UC100 controller.
	UC300_Hi = 2,				//2.   UC300 module with outputs high level.
	UC300_Low = 3,				//3.   UC300 module with outputs low level.
	UC300_5LPT = 4,				//4.   UC300-5LPT motherboard.
	UC300_M44 = 5,				//5.   UC300-MK44 motherboard.
	UC300_ISOBOB = 6,			//6.   UC300-ISOBOB motherboard. Note: Not implemented.
	UC300_STEPPER = 7,			//7.   UC300-STEPPER motherboard. Note: Not implemented.
	UC400ETH = 8,				//8.   UC400ETH motherboard.
	UC300ETH_Hi = 9,			//9.   UC300ETH module with outputs high level.
	UC300ETH_Low = 10,			//10.  UC300ETH module with outputs low level.
	UC300ETH_5LPT = 11,			//11.  UC300ETH-5LPT motherboard.
	UC300ETH_M44 = 12,			//12.  UC300ETH-MK44 motherboard.
	UC300ETH_ISOBOB = 13,		//13.  UC300ETH-ISOBOB motherboard. Note: Not implemented.
	UC300ETH_STEPPER = 14,		//14.  UC300ETH-STEPPER motherboard. Note: Not implemented.
	UC300_5441 = 15,			//15.  UC300-5441 motherboard.
	UC300ETH_5441 = 16,			//16.  UC300ETH-5441 motherboard.
	UC300ETH_M45 = 17,			//17.  UC300ETH-M45 motherboard.
	UC300ETH_UB1 = 18,			//18.  UC300ETH-UB1 motherboard.
	AXBB = 19					//19.  AXBB controller.
};

typedef struct AxisSetting {
	int		Axis;			//Axis number (X=0,Y=1,Z=2,A=3,B=4,C=5).
	bool	Enable;			//Enables the axis.
	int		StepPin;		//Step output pin.
	int		DirPin;			//Direction output pin.
	bool	StepNeg;		//Inverts the step pin.
	bool	DirNeg;			//Inverts the dir pin.
	double	MaxAccel;		//Acceleration parameter of the axis in Units/sec^2.
	double	MaxVel;			//Velocity parameter of the axis in Units/sec.
	double	StepPer;		//Steps per Units parameter.
	int		HomePin;		//Home input pin of the axis.
	bool	HomeNeg;		//Inverts the home input.
	int		LimitPPin;		//Positive side end-limit input pin.
	bool	LimitPNeg;		//Inverts the positive side end-limit input.
	int		LimitNPin;		//Negative side end-limit input pin.
	bool	LimitNNeg;		//Inverts the negative side end-limit input.
	double	SoftLimitP;		//Positive side software end-limit.
	double	SoftLimitN;		//Negative side software end-limit.
	int		SlaveAxis;		//Slave axis, only for the XYZ axes. (0=No slave, 3=A slave, 4=B slave, 5=C slave).
	bool	BacklashOn;		//Enables the backlash compensation for the axis.
	double	BacklashDist;	//Backlash distance in Units.
	double	CompAccel;		//Compensation acceleration for backlash and thread cutting in Units/sec^2.
	int		EnablePin;		//Axis enable output pin.
	bool	EnablePinNeg;	//Inverts the axis enable output.
	int		EnableDelay;	//Delays the enable output 0-255 value (x10msec).
	int		CurrentHiLowPin;	//Current hi/low output pin.
	bool	CurrentHiLowPinNeg;	//Inverts the current hi/low output.
	double	HomeBackOff;	//Home back off distance in Units.
	bool	RotaryAxis;		//Enables the rotary axis function for the axis. Works for the A,B,C axes only.
	bool	RotaryRollover;	//Enables the rollover for rotary axis on 360 degrees. Works for the A,B,C axes only and if the rotary axis function is enabled.
}AxisSetting;


//Status structure.
typedef struct Stat{
	bool	Idle;			//True when the motion controller is in idle.
	bool	Jog;			//True when jog command is being executed.
	bool	Dwell;			//True when dwell command is being executed.
	bool	Backlash;		//True when backlash compensation is being executed.
	bool	Home;			//True when homing command is being executed.
	bool	Probe;			//True when probing command is being executed.
	bool	Estop;			//True when the e-stop input is active.
	bool	SoftLimit;		//True when the motion controller is in software limits.
	bool	HardLimit;		//True when the hardware limit input is active.
	int		Puffer;			//The data size in the motion buffer.
	double	Feed;			//The actual feedrate of the motion in Units per seconds.
	double	SpindleRPM;		//The actual spindle RPM measured with the spindle index or with the spindle encoder.
	int		CurrentID;		//The identifier (ID) of the current motion command.
	bool	LimitOverride;	//True if the limit override is on.
	double	ProgrammedFeed;	//The programmed feedrate of the current motion command.
	bool	MPG1JogOn;		//True when there is a jog command on the MPG 1.
	bool	MPG2JogOn;		//True when there is a jog command on the MPG 2.
	bool	THCOnWaiting;	//True when the THC on delay is ongoing.
	bool	SyncThread;		//True when spindle syncronous motion is ongoing.
	bool	SpindleOn;		//True when the spindle is switched on.
	bool	SpindleDir;		//Shows the spindle rotation direction.
	bool	LaserRunning;	//True when laser engraving is ongoing.
	bool	LDataValid;		//True when laser data is loaded.
	bool	THCOn;			//True when the THC control is enabled.
	bool	THCAntiDive;	//True when the THC anti dive is active.
	bool	THCAntiDiveEnable;	//True when the THC anti dive function is enabled.
	bool	THCDelayEnable;		//True when the THC delay function is enabled.
	bool	THCAntiDownEnable;	//True when the THC anti down function is enabled.
	bool	ProbeActive;		//True when the probe input is active. The probejogmask applies.
}Stat;

//Encoder settings structure.
typedef struct EncoderSetting{
	bool	MPGEnable;				//Enables the MPG.
	int		MPGPinA;				//MPG encoder A channel input pin.
	int		MPGPinB;				//MPG encoder B channel input pin.
	int		MPGPrescale;			//MPG prescaler 0-100 integer.
	int		MPGFilter;				//MPG filter constant 0-100 integer.
	double	MPGSpeedMultiplier;		//MPG speed multiplier 0-1000 integer.
	bool	EnableJROToMPG;			//Attach JRO function for the MPG.
	int		EncoderPinA;			//Spindle encoder A channel input pin.
	int		EncoderPinB;			//Spindle encoder B channel input pin.
	bool	EncoderReverseDirection;//Inverts the encoder count direction.
	int		EncoderPPR;				//Encoder counts per spindle rotation.
	double	EncoderCorrection;		//Encoder to spindle gear ratio. Value=1 if there is no gearing.
	int		Aux1EncoderPinA;		//1. auxiliary encoder A channel input pin.
	int		Aux1EncoderPinB;		//1. auxiliary encoder B channel input pin.
	int		Aux2EncoderPinA;		//2. auxiliary encoder A channel input pin.
	int		Aux2EncoderPinB;		//2. auxiliary encoder B channel input pin.
	int		Aux3EncoderPinA;		//3. auxiliary encoder A channel input pin.
	int		Aux3EncoderPinB;		//3. auxiliary encoder B channel input pin.
	int		Aux4EncoderPinA;		//4. auxiliary encoder A channel input pin.
	int		Aux4EncoderPinB;		//4. auxiliary encoder B channel input pin.
	int		Aux5EncoderPinA;		//5. auxiliary encoder A channel input pin.
	int		Aux5EncoderPinB;		//5. auxiliary encoder B channel input pin.
	int		Aux6EncoderPinA;		//6. auxiliary encoder A channel input pin.
	int		Aux6EncoderPinB;		//6. auxiliary encoder B channel input pin.
	double	Aux1EncoderCountsPer;	//1. auxiliary encoder steps per unit value.
	double	Aux2EncoderCountsPer;	//2. auxiliary encoder steps per unit value.
	double	Aux3EncoderCountsPer;	//3. auxiliary encoder steps per unit value.
	double	Aux4EncoderCountsPer;	//4. auxiliary encoder steps per unit value.
	double	Aux5EncoderCountsPer;	//5. auxiliary encoder steps per unit value.
	double	Aux6EncoderCountsPer;	//6. auxiliary encoder steps per unit value.
}EncoderSetting;

//Spindle Setting structure.
typedef struct SPSetting{
	int		Mode;				//Spindle mode (0=Not used, 1=PWM spindle mode, 2=Step/dir spindle mode).
	int		SPPin;				//Spindle PWM or step output pin.
	bool	SPPinNeg;			//Inverts the spindle PWM output.
	int		SPDirPin;			//Spindle direction output pin.
	bool	SPDirPinNeg;		//Inverts the spindle direction output.
	int		PWMFreq;			//Spindle PWM frequency. (1 to 5000 Hz, resolution is 100000/value.)
	double	SPAccel;			//Spindle acceleration for step/dir spindle in rev/second^2.
	double	SPStepPerRev;		//Steps per spindle turn value for step/dir spindle.
	double	SPMinVel;			//Minimum spindle velocity (1/minute).
	double	SPMaxVel;			//Maximum spindle velocity (1/minute).
	double	SPMinPWMPercent;	//Minimum PWM duty cycle percentage for PWM spindle.
	double	SPMaxPWMPercent;	//Maximum PWM duty cycle percentage for PWM spindle.
	int		SPM3Pin;			//Spindle relay M3 output pin.
	bool	SPM3PinNeg;			//Inverts the spindle relay M3 output.
	int		SPM4Pin;			//Spindle relay M4 output pin.
	bool	SPM4PinNeg;			//Inverts the spindle relay M4 output.
	bool	EnableM3M4Output;	//Spindle relays M3 and M4 enable.		
}SPSetting;

//THC Setting structure.
typedef struct THCSetting{
	int		THCOnPin;		//THC on input pin.
	bool	THCOnPinNeg;	//Inverts the THC on input.
	int		THCUpPin;		//THC up input pin.
	bool	THCUpPinNeg;	//Inverts the THC up input.
	int		THCDownPin;		//THC down input pin.
	bool	THCDownPinNeg;	//Inverts the THC down input.
	double	THCmin;			//Z axis minimum height position for THC control.
	double	THCmax;			//Z axis maximum height position for THC control.
	double	THCFeed;		//THC control feedrate in Units per seconds.
	bool	THCAlwaysOn;	//THC control always on.
	double	THCOnTime;		//THC on delay time in seconds. (0-10sec range)
	double	THCAntiDiveVel;	//THC antidive velocity percentage trigger level. (0-100%)
	int		THCEnPin;		//THC enable output pin.
	bool	THCEnPinNeg;	//Inverts the THC enable output.
	int		THCAntiDivePin;		//THC antidive output pin.
	bool	THCAntiDivePinNeg;	//Inverts the THC antidive output.
	int		THCAntiDownPin;		//THC anti dive output pin.
	bool	THCAntiDownPinNeg;	//Inverts the THC anti down output.
}THCSetting;

//Hardware/software version structure.
typedef struct Versio{
	int	FWVersion;		//Motion controller firmware version number.
	int HWVersion;		//Motion controller hardware version number.
	int APIVersion;		//API dll version number.
	int SerialNumber;	//Motion controller serial number in decimal value. (Should be converted to hexadecimal.)
	int DeviceType;		//Eszk�z tipusa (UC100, UC300-5LPT stb.) ???
}Versio;

//Trajectory parameters structure.
typedef struct TrajParam{
	double	StopAngle;		//Below this vectors connection angle in degrees the controller executes the command without an exact stop.
	bool	ConstantVel;	//Constant velocity mode is used if true and exact stop mode is used if false.
	bool	Units;			//This paramter is currently not implemented.
	double	LinearErr;		//The maximum path deviation in Units. If 0 then all vectors are unified.
	double	MaxLinearUnificationLenght;	//The maximum total length in Units of the unified vectors.
	double	MaxLinearAddLenght;			//The maximum length of the vectors to be unified.
	double	CornerError;	//Maximum path error on corners.
	double	PufferSize;		//The communication buffer length in seconds (0.05 to 1 sec)
}TrajParam;

//Analog settings structure.
typedef struct AnalogSetting{
	bool	FROAnalogEnable;	//Attaches the feedrate override (FRO) to analog input channel.
	int		FROAnalogIn;		//FRO analog input channel number (0=no input, 1=channel 1., 2=channel 2., 3=channel 3., 4= channel 4.).
	double	FROMinPercent;		//FRO minimum percentage (0-300 range).
	double	FROMaxPercent;		//FRO maximum percentage (0-300 range).
	bool	SROAnalogEnable;	//Attaches the spindle rate override (SRO) to analog input channel.
	int		SROAnalogIn;		//SRO analog input channel number (0=no input, 1=channel 1., 2=channel 2., 3=channel 3., 4=channel 4.).
	double	SROMinPercent;		//SRO minimum percentage (0-300 range).
	double	SROMaxPercent;		//SRO maximum percentage (0-300 range).
	int		SpindlePWMEnable;	//Enables the spindle PWM proportional voltage output on the analog output channel (0=no output, 1=channel 1., 2=channel 2., 3=channel 3., 4= channel 4.).
	bool	JROAnalogEnable;	//Attaches the jog rate override (JRO) to analog input channel.
	int		JROAnalogIn;		//JRO analog input channel number (0=no input, 1=channel 1., 2=channel 2., 3=channel 3., 4=channel 4.).
	double	JROMinPercent;		//JRO minimum percentage (0-100 range).
	double	JROMaxPercent;		//JRO maximum percentage (0-100 range).
}AnalogSetting;

//Laser picture data structure.
typedef struct LaserPictureData{
	int     LaserMode;          //Laser mode (0=Grayscale with PWM output, 1=Halftone, 2=Black and white).     
    double	StartPosX;          //X start position for the laser picture engraving.
    double  StartPosY;          //Y start position for the laser picture engraving.
    double  Feed;				//Laser picture engraving feedrate in Units/sec.
    double  PointXsize;			//Point X dimension in Units.
    double  PointYsize;         //Point Y dimension in Units.
    char*	BrightnessData;		//Picture points brightness data in a byte array pointer.
    int     ArrayXsize;			//Picture array X dimension (number of points in X).
    int     ArrayYsize;			//Picture array Y dimension (number of points in Y).
}LaserPictureData;

//Statistics structure.
typedef struct Statistics{ 
	double	DistX;		//Total distance ran by the X axis.
	double	DistCutX;	//Total distance run by the X axis with running spindle.
	double	DistY;		//Total distance ran by the Y axis.
	double	DistCutY;	//Total distance run by the Y axis with running spindle.
	double	DistZ;		//Total distance ran by the Z axis.
	double	DistCutZ;	//Total distance run by the Z axis with running spindle.
	double	DistA;		//Total distance ran by the A axis.
	double	DistCutA;	//Total distance run by the A axis with running spindle.
	double	DistB;		//Total distance ran by the B axis.
	double	DistCutB;	//Total distance run by the B axis with running spindle.
	double	DistC;		//Total distance ran by the C axis.
	double	DistCutC;	//Total distance run by the C axis with running spindle.
}Statistics;

//Spindle PID controller structure.
typedef struct SpindlePID{
	bool			EnablePID;			//Enables the spindle PID controller.
	int				PIDTimePrescaler;	//PID loop time constant (10mseconds divided with 1 to 255).
	double			Kp;					//Proportional gain of the spindle PID controller.
	double			Ki;					//Integral gain of the spindle PID controller.
	double			Kd;					//Derivative gain of the spindle PID controller.
}SpindlePID;

//Fast output pin switch structure.
typedef struct OutPin{
	int		Out1Pin;		//Out 1 pin number.
	bool	Out1PinNeg;		//Out 1 pin invert.
	int		Out2Pin;		//Out 2 pin number.
	bool	Out2PinNeg;		//Out 2 pin invert.
	int		Out3Pin;		//Out 3 pin number.
	bool	Out3PinNeg;		//Out 3 pin invert.
	int		Out4Pin;		//Out 4 pin number.
	bool	Out4PinNeg;		//Out 4 pin invert.
	int		Out5Pin;		//Out 5 pin number.
	bool	Out5PinNeg;		//Out 5 pin invert.
	int		Out6Pin;		//Out 6 pin number.
	bool	Out6PinNeg;		//Out 6 pin invert.
	int		Out7Pin;		//Out 7 pin number.
	bool	Out7PinNeg;		//Out 7 pin invert.
	int		Out8Pin;		//Out 8 pin number.
	bool	Out8PinNeg;		//Out 8 pin invert.
	int		Out9Pin;		//Out 9 pin number.
	bool	Out9PinNeg;		//Out 9 pin invert.
	int		Out10Pin;		//Out 10 pin number.
	bool	Out10PinNeg;	//Out 10 pin invert.
}OutPin;

//Tangential cutter knife structure.
typedef struct TangentCutterParam{
	bool	Enable;				//Enables the tangential cutter knife function.
	double	StopAtAngle;		//Knife stops above this connection angle.
	double	RetractAtAngle;		//Knife retracts above this connection angle.
	double	Retract;			//Knife retracts with this distance.
	double	MoveBackFeedrate;	//Knife feedrate when put back to the workpiece after a retraction.
	double	PullOutFeedrate;	//???
}TangentCutterParam;


extern "C" UC100API int ListDevices(int *DevicesCount);		//Scans and lists all connected UC devices.
extern "C" UC100API int DeviceInfo(int BoardID, int *Type, int *SerialNumber);		//Returns the device properties defined by the boardID parameter. If the BoardID is lower or equals 0 then it is demo mode.
extern "C" UC100API int Open(int BoardID);		//Opens the connection with the device defined by the BoardID parameter. If BoardID lower or equal 0 then demo mode, otherwise device open.
extern "C" UC100API	int Close(void);			//Closes the connection with the device.
extern "C" UC100API	int SetStepRate(int Rate);	//Sets the kernel frequency.
extern "C" UC100API int GetStepRate(int *Rate);	//Reads the kernel frequency.
extern "C" UC100API int GetInput(unsigned __int64 *Inp);		//Reads the input pins states.
extern "C" UC100API int GetOutput(unsigned __int64 *Out);		//Reads the output pins states.
extern "C" UC100API int SetOutput(unsigned __int64 Out);		//Writes the output pins states.
extern "C" UC100API int SetMPGCount(int MPGx, int MPGCount);	//Sets the MPG parameters. (MPGx=0 -> MPG1, MPGx=1 -> MPG2)
extern "C" UC100API int GetMPGCount(int MPGx, int *MPGCount);	//Gets the MPG parameters. (MPGx=0 -> MPG1, MPGx=1 -> MPG2)
extern "C" UC100API int SetChargePumpSetting(int Pin1, bool Pin1Neg,int Pin2, bool Pin2Neg, bool ChargeAlwaysOn);	//Sets the charge pump settings.
extern "C" UC100API int GetChargePumpSetting(int *Pin1, bool *PinNeg1,int *Pin2, bool *PinNeg2, bool *ChargeAlwaysOn);	//Gets the charge pump settings.
extern "C" UC100API int SetDebounce(int Debounce, int THCDebounce, int HomeDebounce);	//Sets the debounce (inputs digital low pass filter).
extern "C" UC100API int GetDebounce(int *Debounce, int *THCDebounce, int *HomeDebounce);	//Gets the debounce (inputs digital low pass filter).
extern "C" UC100API int SetEstopSetting(int Pin, bool PinNeg);			//Sets the e-stop input pin.
extern "C" UC100API int GetEstopSetting(int *Pin, bool *PinNeg);		//Gets the e-stop input pin.
extern "C" UC100API int StopEndOfSegment();	//Stop command which stops motion at the end of each movement segment.
extern "C" UC100API int StopWithDeccel();	//Stop command which stops motion at the end of each movement.
extern "C" UC100API int Stop();				//Stop command which makes an instant stop (e-stop like stop).
extern "C" UC100API int SetEstopState();	//Sets the reset condition.
extern "C" UC100API int SetTrajParam(TrajParam *_TrajParam); //Sets the path trajectory (Constant velocity) parameters.
extern "C" UC100API int GetTrajParam(TrajParam *_TrajParam); //Gets the path trajectory (Constant velocity) parameters.
extern "C" UC100API int SetAxisSetting(AxisSetting *_AxisSetting);  //Sets the axes parameters.
extern "C" UC100API int GetAxisSetting(AxisSetting *_AxisSetting);	//Gets the axes parameters.
extern "C" UC100API int SetAxisPosition(double Xpos, double Ypos, double Zpos, double Apos, double Bpos, double Cpos); //Sets the position (coordinate counter) of the axes in Units.
extern "C" UC100API int GetAxisPosition(double *Xpos, double *Ypos, double *Zpos, double *Apos, double *Bpos, double *Cpos); //Gets the position (coordinate counter) of the axes in Units.
extern "C" UC100API int GetAxisDTG(double *Xpos, double *Ypos, double *Zpos, double *Apos, double *Bpos, double *Cpos); //Gets the DTG (distance to go) of the axes in Units.
extern "C" UC100API int SetAxisIndex(int Xpos,int Ypos, int Zpos, int Apos, int Bpos, int Cpos); //Sets the axis position in Steps.
extern "C" UC100API int GetAxisIndex(int *Xpos,int *Ypos, int *Zpos, int *Apos, int *Bpos, int *Cpos); //Gets the axis position in Steps.
extern "C" UC100API int AddLinearMove(double x,double y,double z,double a,double b,double c, double Feed, int ID); //Adds one linear movement to the motion buffer for execution.
extern "C" UC100API int AddCircularMove(double x,double y,double z, double rx, double ry, double rz, bool Dir, double Feed,int Plane, int ID, int NextID); //Adds one circular movement to the motion buffer for execution. Plane 0=XY, 1=XZ, 2=YZ. NextID is the ID of the next command for proper stop and re// start on the end of the arc.
extern "C" UC100API int AddDwell(double Time,int ID);	//Adds a dwell (wait) command to the motion buffer for execution.
extern "C" UC100API int AddDummyMove(int ID);			//Adds a dummy move command to the motion buffer for execution. (no move, but the returned ID changes).
extern "C" UC100API int SetSpindleSetting(SPSetting *_SPSetting); //Sets the spindle parameters.
extern "C" UC100API int GetSpindleSetting(SPSetting *_SPSetting); //Gets the spindle parameters.
extern "C" UC100API int SpindleOn(bool CCW);			//Turns the spindle on.
extern "C" UC100API int SetSpindleSpeed(double Speed);	//Sets the spindle speed.
extern "C" UC100API int GetSpindleSpeed(double *Speed);	//Gets the spindle speed.
extern "C" UC100API int SpindleOff();					//Turns the spindle off.
extern "C" UC100API int SetIndexSetting(int Pin, double Prescale);		//Sets the index prescaler and pin of the spindle index sensor.
extern "C" UC100API int GetIndexSetting(int *Pin, double *Prescale);	//Gets the index prescaler and pin of the spindle index sensor.
extern "C" UC100API int SetTHCSetting(THCSetting *_THCSetting);			//Sets the THC (torch height control for plasma) settings.
extern "C" UC100API int GetTHCSetting(THCSetting *_THCSetting);			//Gets the THC (torch height control for plasma) settings.
extern "C" UC100API int THCNullCorrection(void);						//Nulls the THC position correction value.
extern "C" UC100API int GetTHCCorrection(double *THCCorr);				//Gets the THC position correction value.
extern "C" UC100API int THCEnable(bool Enable);				//Enables the THC control
extern "C" UC100API int GetStatus(Stat *SStat);				//Gets the status structure with the status parameters.
extern "C" UC100API int HomeOn(int Axis, double SpeedUp, double SpeedDown, bool Dir);	//Homes one axis.(0=X, 1=Y, 2=Z, 3=A, 4=B, 5=C). Feedrate in Units/min.
extern "C" UC100API int JogOn(int Axis, bool Dir, bool MaxSpeed);	//Switches the jog on on one axis. Multiple calls on different axis is possible.(0=X, 1=Y, 2=Z, 3=A, 4=B, 5=C)
extern "C" UC100API int JogOff(int Axis);			//Switches the jog off on one axis. (0=X, 1=Y, 2=Z, 3=A, 4=B, 5=C, 6=all)
extern "C" UC100API int SetOffline(bool State);		//Sets the outputs to offline mode. (inhibits the outputs)
extern "C" UC100API int GetOffline(bool *State);	//Gets the offline state.
extern "C" UC100API int SetProbeSetting(int Pin1,bool Pin1Neg,int Pin2,bool Pin2Neg); //Sets the probe sensors settings.
extern "C" UC100API int GetProbeSetting(int *Pin,bool *PinNeg,int *Pin2, bool *Pin2Neg); //Gets the probe sensors settings.
extern "C" UC100API int ProbeStart(int Axis, double ProbePos, double Speed, int ID);	//Starts a probing operation.
extern "C" UC100API int ProbeStop(void);	//Stops a probing operation.
extern "C" UC100API int SetSoftLimit(bool SoftLimitEnable); //Enables the softlimits.
extern "C" UC100API int GetSoftLimit(bool *SoftLimitEnable); //Gets the softlimit enable.
extern "C" UC100API int LimitOverRide();	//Overrides the limit input pins if any of them are active. Not set and automatically cleared when they are not active and when they all become inactive.
extern "C" UC100API int GetVersio(Versio *Version); //Gets the API version datas.
extern "C" UC100API int SetFRO(double FRO);		//Sets the FRO (feedrate override). FRO percentage (0-300%).
extern "C" UC100API int GetFRO(double *FRO);	//Gets the FRO (feedrate override). FRO percentage (0-300%).
extern "C" UC100API int SetSRO(double SRO);		//Sets the SRO (spindle rate override). SRO percentage (0-300%).
extern "C" UC100API int GetSRO(double *SRO);	//Gets the SRO (spindle rate override). SRO percentage (0-300%).
extern "C" UC100API int SetJRO(double JRO);		//Sets the JRO (jog rate override). JRO percentage (0-100%).
extern "C" UC100API int GetJRO(double *JRO);	//Gets the JRO (jog rate override). JRO percentage (0-100%).
extern "C" UC100API int MPGJogOn(int Axis, int MPG, int Mode, double Step);	//Switches the MPG jog on. Axis to jog with the MPG.(0=X, 1=Y, 2=Z, 3=A, 4=B, 5=C)
extern "C" UC100API int MPGJogOff(int MPG);			//Switches the MPG jog off. MPG number. (Currently only 1 MPG is available)
extern "C" UC100API int GetAnalogInput(int *AnIn1, int * AnIn2, int *AnIn3, int *AnIn4);	//Gets the analog inputs values.
extern "C" UC100API int GetAnalogInputT(int *AnIn1, int * AnIn2, int *AnIn3, int *AnIn4);	//Gets the analog output channel values.
extern "C" UC100API int SetAnalogOut(int AnOut1, int AnOut2, int AnOut3, int AnOut4);		//Sets the analog output channel values.
extern "C" UC100API int GetAnalogOut(int *AnOut1, int *AnOut2, int *AnOut3, int *AnOut4);	//Gets the analog output channel values.
extern "C" UC100API int SetAnalogSetting(AnalogSetting *_AnalogSetting);	//Sets the analog channels parameters.
extern "C" UC100API int GetAnalogSetting(AnalogSetting *_AnalogSetting);	//Gets the analog channels parameters.

extern "C" UC100API int AddSyncMove(double x, double z, double Pitch, double Degree, int ID); //Adds a spindle syncronised motion (Thread cutting.) to the motion buffer for execution.
extern "C" UC100API int SetEncoderSetting(EncoderSetting *_EncoderSetting);				//Sets the spindle and AUX encoder and MPG settings.
extern "C" UC100API int GetEncoderSetting(EncoderSetting *_EncoderSetting);				//Gets the spindle and AUX encoder and MPG settings.
extern "C" UC100API int AddRigidTappingMove(double z, double Pitch, bool Dir, int ID);	//Adds a spindle syncronised Z axis movement (rigid tapping) to the motion buffer for execution.
extern "C" UC100API int ClearSoftlimitState();		//Clears the softlimit status if it was set.	
extern "C" UC100API int SetFeedHold(bool State);	//Sets the feedhold and pauses the motion.
extern "C" UC100API int GetFeedHold(bool *State);	//Gets the feedhold state.	
extern "C" UC100API int AddPinSwitch(int PinDuty, int ID);	//Adds a syncronous laser output pin PWM switching operation to the motion buffer for execution.
extern "C" UC100API int AddPinSwitch2(int Output, bool Active, int ID);	//Adds a syncronous pin on/off swtiching operation to the motion buffer for execution. The output pin number configured in the I/Os (value 1 to 10.)

extern "C" UC100API int SetLaserPictureData(LaserPictureData *_LaserPictureData);	//Loads the laser image data array for later execution.
extern "C" UC100API int StartLaserPicture();	//Starts the laser image data execution.
extern "C" UC100API int GetLaserProgress(double *Percent);		//Gets the done percentage of the laser image data execution.
extern "C" UC100API int UnloadLaserPictureData();		//Unloads the laser image data from the memory array.
extern "C" UC100API int SetLaserOutPin(int Pin,bool PinNeg);	//Sets the laser output pin.
extern "C" UC100API int GetLaserOutPin(int *Pin,bool *PinNeg);	//Gets the laser output pin.

extern "C" UC100API int GetDebugData(double *D1, double *D2, double *D3, double *D4, double *D5, double *D6, double *D7, double *D8, double *D9, double *D10);	//Gets the debug data from the API. (For internal use only, not for developers)

extern "C" UC100API int AddTHCEnable(bool Enable,int ID);			//Adds a THC on/off command to the motion buffer for execution.

extern "C" UC100API int JogOnSpeed(int Axis, bool Dir, double Speed);	//Starts a jogging operation on an axis. The axis to be jogged.(0=X, 1=Y, 2=Z, 3=A, 4=B, 5=C)
extern "C" UC100API int AddLinearMoveRel(int Axis,double Step,int StepCount,double Speed,bool Dir); //Adds a relative coordinate linear movement to the motion buffer.

extern "C" UC100API int AddOEMVar(int Num, double Val, int ID);		//Adds an OEM variable value change to the motion buffer. The number of the OEM var.(range:0-100)
extern "C" UC100API int GetOEMVar(int Num, double *Val);			//Gets the value of an OEM variable. The number of the OEM var.(range:0-100)

extern "C" UC100API int AddIvar(int Num, double Val, int ID);		//Adds a variable value change to the motion buffer. The number of the variable.(range:0-6000)
extern "C" UC100API int SetIvar(int Num, double Val);				//Sets the value of a variable. The number of the variable.(range:0-6000)
extern "C" UC100API int GetIvar(int Num, double *Val);				//Gets the value of a variable. The number of the variable.(range:0-6000)

extern "C" UC100API int GetEstopCause(int *Cause);					//Gets the reason for the last e-stop reset event. Reason for the e-stop reset. (0=no reset, 1=e-stop input, 2=limit input.)

extern "C" UC100API int THCAntiDiveEnable(bool Enable);				//Switches the THC anti dive on/off.
extern "C" UC100API int THCDelayEnable(bool Enable);				//Switches the THC arcon movement delay on/off.
extern "C" UC100API int THCAntiDownEnable(bool Enable);				//Switches the THC anti down on/off.

extern "C" UC100API int AddTHCAntiDiveEnable(bool Enable,int ID);	//Adds a THC antidive on/off switching operation to the motion buffer.
extern "C" UC100API int AddTHCDelayEnable(bool Enable,int ID);		//Adds a THC arcon movement delay on/off operation to the motion buffer.
extern "C" UC100API int AddTHCAntiDownEnable(bool Enable,int ID);	//Adds a THC anti down on/off operation to the motion buffer.
extern "C" UC100API int SetTHCVirtualSignal(bool THCArcOn, bool THCUp, bool THCDown); //Sets the THC virtual control signals
extern "C" UC100API int GetTHCVirtualSignal(bool* THCArcOn, bool* THCUp, bool* THCDown); //Gets the THC virtual control signals

extern "C" UC100API int GetProbePosition(double* Xpos,double* Ypos,double* Zpos,double* Apos,double* Bpos,double* Cpos);	//Gets the probed coordinates of a probing operation.
extern "C" UC100API int SafeProbeEnable(bool Enable);		//Switches the safe probe mode on.
extern "C" UC100API int JogSafeProbeEnable(bool Enable);	//Switches the jogsafe probe mode on.
extern "C" UC100API int GetProbeState(bool* Finish);		//Gets the probe input state. True when probing finished.

extern "C" UC100API int GetStatistics(Statistics * _Statistics);	//Gets the system statistics.
extern "C" UC100API int SetStatistics(Statistics * _Statistics);	//Sets the system statistics.
extern "C" UC100API int GetThreadUsage(BYTE ** _ThreadUsage);		//Gets the system communication latency. it returns a pointer with 500 data bytes. Each data bytes represents one latency point.

extern "C" UC100API int SetAuxEncoderCountsPer(int Num, double CountsPer);	//Reads the auxiliary encoder counts per turn. Number of aux encoder (0 to 5).
extern "C" UC100API int GetAuxEncoderCountsPer(int Num, double * CountsPer);//Sets the auxiliary encoder counts per turn. Number of aux encoder (0 to 5).
extern "C" UC100API int SetAuxEncoderPos(int Num, double Pos);				//Sets the auxiliary encoder position. Number of aux encoder (0 to 5).
extern "C" UC100API int GetAuxEncoderPos(int Num, double * Pos);			//Gets the auxiliary encoder position. Number of aux encoder (0 to 5).
extern "C" UC100API int SetSpindlePID(SpindlePID *_SpPID);		//Sets the spindle closed loop PID controller parameters.
extern "C" UC100API int GetSpindlePID(SpindlePID *_SpPID);		//Gets the spindle closed loop PID controller parameters.
extern "C" UC100API int GetPIDData(double ** _CommandedRPM, double ** _MeasuredRPM, double ** _PIDOut);		//Gets the spindle PID feedback data in an array of 500.
extern "C" UC100API int GetPIDOut(double * PidOut);		//Gets the actual spindle PID PWM output.

extern "C" UC100API int SetMotionProgressState(bool RUN);		//Tells the controller that motion is in progress. Set this function true before motion and set it false after all motion ended. Required for THC control operations only.

extern "C" UC100API int SetOutputPin(OutPin * _OutPin);		//Sets the state of 10 dedicated fast outputs.
extern "C" UC100API int GetOutputPin(OutPin * _OutPin);		//Gets the state of 10 dedicated fast outputs.

extern "C" UC100API int SetOutputBit(int Num);				//Sets the sate of the output pins to high level. Integer number in which every bit represents one output pin.(0-63)
extern "C" UC100API int ClearOutputBit(int Num);			//Sets the sate of the output pins to low level. Integer number in which every bit represents one output pin.(0-63)

extern "C" UC100API int SetFeedMode(int Num);				//Selects the feedrate mode. Use the FeedMode enum. Note: G95 is not implemented, is to be implemented in future releases of the API.
extern "C" UC100API int GetFeedMode(int *Num);				//Gets the feedrate mode. Use the FeedMode enum. Note: G95 is not implemented, is to be implemented in future releases of the API.

extern "C" UC100API int SetTangentialCutter(TangentCutterParam * _TangentCutterParam);	//Sets the tangential cutter knife parameters and enables the tangential cutter option.
extern "C" UC100API int GetTangentialCutter(TangentCutterParam * _TangentCutterParam);	//Gets the tangential cutter knife parameters and enables the tangential cutter option.

