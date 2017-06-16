/**
 * @brief		OIS system header for LC898123
 * 				API List for customers
 *
 * @author		Copyright (C) 2015, ON Semiconductor, all right reserved.
 *
 * @file		OisAPI.h
 * @date		svn:$Date:: 2016-06-17 16:42:32 +0900#$
 * @version		svn:$Revision: 54 $
 * @attention
 **/
#ifndef OISAPI_H_
#define OISAPI_H_
#include	"MeasurementLibrary.h"

#include	"Ois.h"

//****************************************************
//	extern selector for API
//****************************************************
#ifdef	__OISCMD__
	#define	__OIS_CMD_HEADER__
#else
	#define	__OIS_CMD_HEADER__		extern
#endif

#ifdef	__OISFLSH__
	#define	__OIS_FLSH_HEADER__
#else
	#define	__OIS_FLSH_HEADER__		extern
#endif

//****************************************************
//	MODE SELECTORS (Compile Switches)
//****************************************************
#define			__OIS_MODULE_CALIBRATION__		//!< for module maker to done the calibration.
//#define 		__CRC_VERIFY__					//!< select CRC16 for upload verify, if this comment out, MD5 is selected.
//#define		__OIS_BIG_ENDIAN__				//!< endian of MPU

//#define		__OIS_CLOSED_AF__

//****************************************************
//	STRUCTURE DEFINE
//****************************************************
typedef struct STRECALIB {
	INT_16	SsFctryOffX ;
	INT_16	SsFctryOffY ;
	INT_16	SsRecalOffX ;
	INT_16	SsRecalOffY ;
	INT_16	SsDiffX ;
	INT_16	SsDiffY ;
} stReCalib ;

typedef struct STMESRAM {
	INT_32	SlMeasureMaxValue ;
	INT_32	SlMeasureMinValue ;
	INT_32	SlMeasureAmpValue ;
	INT_32	SlMeasureAveValue ;
} stMesRam ;									// Struct Measure Ram

//****************************************************
//	API LIST
//****************************************************
/* Status Read and OIS enable [mandatory] */
__OIS_CMD_HEADER__	UINT_8	RdStatus( UINT_8 ) ;						//!< Status Read whether initialization finish or not.
__OIS_CMD_HEADER__	void	OisEna( void ) ;						//!< OIS Enable function
__OIS_CMD_HEADER__	void	OisDis( void ) ;						//!< OIS Disable function

/* Others [option] */
__OIS_CMD_HEADER__	UINT_8	RtnCen( UINT_8 ) ;						//!< Return to center function. Hall servo on/off
__OIS_CMD_HEADER__	void	OisEnaNCL( void ) ;						//!< OIS Enable function w/o delay clear
__OIS_CMD_HEADER__	void	OisEnaDrCl( void ) ;					//!< OIS Enable function force drift cancel
__OIS_CMD_HEADER__	void	OisEnaDrNcl( void ) ;					//!< OIS Enable function w/o delay clear and force drift cancel
__OIS_CMD_HEADER__	void	SetRec( void ) ;						//!< Change to recording mode function
__OIS_CMD_HEADER__	void	SetStill( void ) ;						//!< Change to still mode function

__OIS_CMD_HEADER__	void	SetPanTiltMode( UINT_8 ) ;				//!< Pan/Tilt control (default ON)
//__OIS_CMD_HEADER__	void	RdHallCalData( void ) ;					//!< Read Hall Calibration Data in Data Ram

__OIS_CMD_HEADER__	UINT_8	RunHea( void ) ;						//!< Hall Examination of Acceptance
__OIS_CMD_HEADER__	UINT_8	RunGea( void ) ;						//!< Gyro Examination of Acceptance
__OIS_CMD_HEADER__	UINT_8	RunGea2( UINT_8 ) ;						//!< Gyro Examination of Acceptance


__OIS_CMD_HEADER__	void	OscStb( void );							//!< Standby the oscillator
__OIS_CMD_HEADER__	UINT_8	GyroReCalib( stReCalib * ) ;			//!< Gyro offset re-calibration
__OIS_CMD_HEADER__	UINT_32	ReadCalibID( void ) ;					//!< Read calibration ID
//__OIS_CMD_HEADER__	UINT_16	GyrSlf( void ) ;						//!< Gyro self test

__OIS_CMD_HEADER__	UINT_8	GyrWhoAmIRead( void ) ;					//!< Gyro Who am I Read
__OIS_CMD_HEADER__	UINT_8	GyrWhoAmICheck( void ) ;				//!< Gyro Who am I Check
__OIS_CMD_HEADER__	UINT_8	GyrIdRead( UINT_8 * ) ;					//!< Gyro ID Read

__OIS_CMD_HEADER__	UINT_8	MesRam( INT_32 , INT_32 , INT_32 , stMesRam* , stMesRam* );

#ifdef	__OIS_MODULE_CALIBRATION__

 /* Calibration Main [mandatory] */
 #ifdef	__OIS_CLOSED_AF__
 __OIS_CMD_HEADER__	UINT_32	TneRunA( void ) ;						//!< calibration with close AF
 __OIS_CMD_HEADER__	UINT_32	AFHallAmp( void ) ;

 #else
 __OIS_CMD_HEADER__	UINT_32	TneRun( void );							//!< calibration for bi-direction AF
 #endif
// __OIS_CMD_HEADER__	UINT_8	TneADO( ) ;

 __OIS_CMD_HEADER__	void	TneSltPos( UINT_8 ) ;					//!< for NVC
 __OIS_CMD_HEADER__	void	TneVrtPos( UINT_8 ) ;					//!< for CROSS TALK
 __OIS_CMD_HEADER__ void	TneHrzPos( UINT_8 ) ;					//!< for CROSS TALK
 __OIS_CMD_HEADER__ UINT_16	TneADO( void ) ;						//!< adjust AD offset
 __OIS_CMD_HEADER__ UINT_32	TneAvc( UINT_8 ) ;						//!< calibration for 6 axis offset
 __OIS_CMD_HEADER__	UINT_8	FrqDet( void ) ;						//!< oscillation detect
 __OIS_CMD_HEADER__	UINT_8	FrqDetReTry( void ) ;					//!< oscillation detect
 __OIS_CMD_HEADER__	UINT_8	FrqDetWGyr( UINT_8 ) ;					//!< oscillation detect

 __OIS_CMD_HEADER__	UINT_8	WrHallCalData( void ) ;					//!< upload the calibration data except gyro gain to Flash
 __OIS_CMD_HEADER__	UINT_8	WrGyroGainData( void ) ;				//!< upload the gyro gain to Flash
 __OIS_CMD_HEADER__	UINT_8	WrGyroAngleData( void ) ;				//!< upload the gyto angle correction to Flash
 __OIS_CMD_HEADER__	UINT_8	WrCLAFData( void ) ;					//!< Flash Write CL-AF Calibration Data Function
 __OIS_CMD_HEADER__	UINT_8	WrMixingData( void ) ;					//!< Flash Write Mixing Data Function
 __OIS_CMD_HEADER__	UINT_8	WrFstData( void ) ;						//!< Flash Write FST calibration data Function
 __OIS_CMD_HEADER__	UINT_8	WrMixCalData( UINT_8, mlMixingValue * ) ;
 __OIS_CMD_HEADER__	UINT_8	WrGyroOffsetData( void ) ;

 #ifdef	HF_LINEAR_ENA
// __OIS_CMD_HEADER__	void	SetHalLnData( UINT_16 * );
// __OIS_CMD_HEADER__	INT_16	WrHalLnData( UINT_8 );
 #endif	// HF_LINEAR_ENA

 #ifdef	HF_MIXING_ENA
 __OIS_CMD_HEADER__	INT_8	WrMixCalData( UINT_8, mlMixingValue * ) ;//!< upload the mixing coefficient to Flash
 #endif	// HF_MIXING_ENA
 __OIS_CMD_HEADER__	UINT_8	WrAclOffsetData( void ) ;				//!< accelerator offset and matrix to Flash

 __OIS_CMD_HEADER__	UINT_8	WrLinCalData( UINT_8, mlLinearityValue * ) ;
 __OIS_CMD_HEADER__	UINT_8	WrLinMixCalData( UINT_8, mlMixingValue *, mlLinearityValue * ) ;
 __OIS_CMD_HEADER__	UINT_8	ErCalData( UINT_16 ) ;

 /* Flash Update */
 __OIS_FLSH_HEADER__	UINT_8	ReadWPB( void ) ;						//!< WPB level read
 __OIS_FLSH_HEADER__	UINT_8	UnlockCodeSet( void ) ;					//!< <Flash Memory> Unlock Code Set
 __OIS_FLSH_HEADER__	UINT_8	UnlockCodeClear(void) ;					//!< <Flash Memory> Clear Unlock Code
 __OIS_FLSH_HEADER__	void	FlashByteRead( UINT_32, UINT_8 *, UINT_8 ) ;
 __OIS_FLSH_HEADER__	void	FlashSectorRead( UINT_32, UINT_8 * ) ;
 __OIS_FLSH_HEADER__	UINT_8	FlashInt32Write( UINT_32, UINT_32 *, UINT_8 ) ;

 __OIS_FLSH_HEADER__	UINT_8	FlashBlockErase( UINT_32 ) ;
 __OIS_FLSH_HEADER__	UINT_8	FlashSectorErase( UINT_32 ) ;
 __OIS_FLSH_HEADER__	UINT_8	FlashSectorWrite( UINT_32, UINT_8 * ) ;
 __OIS_FLSH_HEADER__	UINT_8	FlashProtectStatus( void ) ;

 __OIS_FLSH_HEADER__	UINT_8	FlashDownload( UINT_8, UINT_8 , UINT_8 ) ;
 __OIS_FLSH_HEADER__	UINT_8	FlashUpdateF40( UINT_8, DOWNLOAD_TBL * ) ;			//!< Flash Update for LC898123F40
 __OIS_FLSH_HEADER__	UINT_8	EraseCalDataF40( void ) ;				//!< Flash erase calibration data(NVR2)
 __OIS_FLSH_HEADER__	void	ReadCalDataF40( UINT_32 *, UINT_32 * ) ;	//!< Flash read calibration data(NVR2)
 __OIS_FLSH_HEADER__	UINT_8	WriteCalDataF40( UINT_32 *, UINT_32 * ) ;	//!< Flash write calibration data(NVR2)
 __OIS_FLSH_HEADER__	void	CalcChecksum( const UINT_8 *, UINT_32, UINT_32 *, UINT_32 * ) ;
 __OIS_FLSH_HEADER__	void	CalcBlockChksum( UINT_8, UINT_32 *, UINT_32 * );

 // following functions are into boot rom mode. if use, be careful.
 __OIS_FLSH_HEADER__	UINT_8	FlashSectorRead_Burst( UINT_32, UINT_8 *, UINT_8 ) ;
 __OIS_FLSH_HEADER__	UINT_8	FlashSectorWrite_Burst( UINT_32, UINT_8 *, UINT_8 ) ;
 //HTC_START
 __OIS_FLSH_HEADER__	void	FlashSectorRead_htc( UINT_32, UINT_8 *, UINT_32 ) ;
 //HTC_END

#endif	// __OIS_MODULE_CALIBRATION__

#endif /* #ifndef OISAPI_H_ */
