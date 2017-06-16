//********************************************************************************
//
//		<< LC898123F40 Evaluation Soft >>
//	    Program Name	: FlsCmd_F40.c
//		Design			: K.abe
//		History			: First edition
//********************************************************************************
#define		__OISFLSH__
//**************************
//	Include Header File
//**************************
#include	"Ois.h"

#include	"FromCode_09_05_05.h"
#include	"FromCode_09_05_07.h"

#include	"PmemCode.h"

#define	USER_RESERVE			3		// Reserved for customer data blocks
#define	ERASE_BLOCKS			(16 - USER_RESERVE)


//#define BURST_LENGTH ( 32*5 )			// 160 必ず5の倍数で設定すること。最大160Byteまで
#define BURST_LENGTH ( 8*5 ) 			// 40 必ず5の倍数で設定すること。最大160Byteまで

#define DMB_COEFF_ADDRESS		0x21
#define BLOCK_UNIT				0x200
#define BLOCK_BYTE				2560
#define SECTOR_SIZE				320
#define HALF_SECTOR_ADD_UNIT	0x20
#define FLASH_ACCESS_SIZE		32		//2の乗数で設定。MAXは32
#define	USER_AREA_START			(BLOCK_UNIT * ERASE_BLOCKS)

//****************************************************
//	CUSTOMER NECESSARY CREATING FUNCTION LIST
//****************************************************
/* for I2C communication */
extern	void RamWrite32A( UINT_16, UINT_32 );
extern 	void RamRead32A( UINT_16, void * );
/* for I2C Multi Translation : Burst Mode*/
extern 	void CntWrt( void *, UINT_16) ;
extern	void CntRd3( UINT_32, void *, UINT_16 ) ;

/* WPB control for LC898123F40*/
extern void WPBCtrl( UINT_8 );
/* for Wait timer [Need to adjust for your system] */
extern void	WitTim( UINT_16 );

//**************************
//	Local Function Prototype
//**************************
UINT_8	FlashBurstWriteF40( const UINT_8 *, UINT_32, UINT_32 ) ;
UINT_8	FlashBurstReadF40( UINT_8 *, UINT_32, UINT_32 ) ;


//********************************************************************************
// Function Name 	: IOWrite32A
// Retun Value		: None
// Argment Value	: IOadrs, IOdata
// Explanation		: Write data to IO area Command
// History			: First edition
//********************************************************************************
void IORead32A( UINT_32 IOadrs, UINT_32 *IOdata )
{
	RamWrite32A( CMD_IO_ADR_ACCESS, IOadrs ) ;
	RamRead32A ( CMD_IO_DAT_ACCESS, IOdata ) ;
}

//********************************************************************************
// Function Name 	: IOWrite32A
// Retun Value		: None
// Argment Value	: IOadrs, IOdata
// Explanation		: Write data to IO area Command
// History			: First edition
//********************************************************************************
void IOWrite32A( UINT_32 IOadrs, UINT_32 IOdata )
{
	RamWrite32A( CMD_IO_ADR_ACCESS, IOadrs ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, IOdata ) ;
}

//********************************************************************************
// Function Name 	: WPB level read
// Retun Value		: 0: WPB active error , 1: WPB active
// Argment Value	: NON
// Explanation		: Read WPB level
// History			: First edition
//********************************************************************************
UINT_8 ReadWPB( void )
{
	UINT_32 UlReadVal, UlCnt=0;

	do{
		IORead32A( FLASHROM_F40_WPB, &UlReadVal );
		if( (UlReadVal & 0x00000004) != 0 )	return ( 1 ) ;
		WitTim( 1 );
	}while ( UlCnt++ < 10 );
	return ( 0 );
}

//********************************************************************************
// Function Name 	: UnlockCodeSet
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: <Flash Memory> Unlock Code Set
// History			: First edition
//********************************************************************************
UINT_8 UnlockCodeSet( void )
{
	UINT_32 UlReadVal;

	WPBCtrl(WPB_OFF) ;
	if ( ReadWPB() != 1 )	return ( 5 );								// WPB LOW ERROR

	IOWrite32A( FLASHROM_F40_UNLK_CODE1, 0xAAAAAAAA ) ;					// UNLK_CODE1(E0_7554h) = AAAA_AAAAh
	IOWrite32A( FLASHROM_F40_UNLK_CODE2, 0x55555555 ) ;					// UNLK_CODE2(E0_7AA8h) = 5555_5555h
	IOWrite32A( FLASHROM_F40_RSTB_FLA, 0x00000001 );					// RSTB_FLA_WR(E0_74CCh[0])=1
	IOWrite32A( FLASHROM_F40_CLK_FLAON, 0x00000010 );					// FLA_WR_ON(E0_7664h[4])=1
	// Additional Unllock Code Set
	IOWrite32A( FLASHROM_F40_UNLK_CODE3, 0x0000ACD5 );					// Unlock Code
	IOWrite32A( FLASHROM_F40_WPB, 1 );
//	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_F40_WPB );
	RamRead32A(  CMD_IO_DAT_ACCESS , &UlReadVal ) ;

	if ( (UlReadVal & 0x00000007) != 7 ) return(1);

	return(0);
}

//********************************************************************************
// Function Name 	: UnlockCodeClear
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: <Flash Memory> Clear Unlock Code
// History			: First edition
//********************************************************************************
UINT_8 UnlockCodeClear(void)
{
	UINT_32 UlReadVal;

	IOWrite32A( FLASHROM_F40_WPB, 0x00000010 );							// UNLOCK_CLR(E0_701Ch[4])=1

//	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_F40_WPB );					// UNLOCK (E0_701Ch[7])
	RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );


	if( (UlReadVal & 0x00000080) != 0 )	return (3);

	WPBCtrl(WPB_ON) ;

	return(0);
}


//********************************************************************************
// Function Name 	: FlashBurstWriteF40
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: <Flash Memory> Flash Burst Write
// History			: First edition
//********************************************************************************
UINT_8 FlashBurstWriteF40( const UINT_8 *NcDataVal, UINT_32 NcDataLength, UINT_32 ScNvrMan)
{
	UINT_32 i, j, UlCnt;
	UINT_8 data[163];			// ComdH + CmdL + Length + Data[40]
	UINT_32 UlReadVal;
	UINT_8 UcOddEvn;
	UINT_8 Remainder;	// 余り

	data[0] = 0xF0;					// Command High
	data[1] = 0x08;					// Command Low
	data[2] = BURST_LENGTH;			// Data Size, Max 160[Byte]

	for(i = 0 ; i< (NcDataLength / BURST_LENGTH) ; i++)
	{
		UlCnt = 3;

		UcOddEvn =i % 2;					// 奇数偶数Check
		data[1] = 0x08 + UcOddEvn;			// Command Low

		for(j = 0 ; j < BURST_LENGTH; j++)	data[UlCnt++] = *NcDataVal++;

		CntWrt( data, BURST_LENGTH + 3 );
		RamWrite32A( 0xF00A ,(UINT_32) ( ( BURST_LENGTH / 5 ) * i + ScNvrMan) ) ;	// set Flash write address
		RamWrite32A( 0xF00B ,(UINT_32) (BURST_LENGTH / 5) ) ;						// Word Address

		RamWrite32A( 0xF00C , 4 + 4 * UcOddEvn ) ;									// set write operation
	}

	Remainder = NcDataLength % BURST_LENGTH;
	if (Remainder != 0 ){
		data[2] = Remainder;				// Data Size, Max 160[Byte]

		UlCnt = 3;

		UcOddEvn =i % 2;					// 奇数偶数Check
		data[1] = 0x08 + UcOddEvn;			// Command Low

		for(j = 0 ; j < Remainder; j++)	data[UlCnt++] = *NcDataVal++;

		CntWrt( data, BURST_LENGTH + 3 );
		RamWrite32A( 0xF00A ,(UINT_32) ( ( BURST_LENGTH / 5 ) * i + ScNvrMan) ) ;	// set Flash write address
		RamWrite32A( 0xF00B ,(UINT_32) (Remainder /5) ) ;							// Word Address
		RamWrite32A( 0xF00C , 4 + 4 * UcOddEvn ) ;									// set write operation
	}

	UlCnt = 0;
	do{
		if( UlCnt++ > 100 ) return ( 1 );
		RamRead32A( 0xF00C, &UlReadVal );
	}while ( UlReadVal != 0 );
	return( 0 );
}

#if		MODULE_VENDOR == 9
#define			RXCMDBUFFER						0x0020
#endif	//MODULE_VENDOR == 9

//********************************************************************************
// Function Name 	: FlashBurstReadF40
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: <Flash Memory> Flash Burst Read
// History			: First edition
//********************************************************************************
UINT_8	FlashBurstReadF40( UINT_8 *NcDataVal, UINT_32 UcDataLength, UINT_32 ScNvrMan )
{
	UINT_8	i ;
	UINT_32	UlReadVal, UlCnt, UlCmdID ;

	for( i = 0 ; i < ( UcDataLength / BURST_LENGTH ) ; i++ )
	{
#if		MODULE_VENDOR == 9
		UlCmdID		= ( unsigned long )0xF008 + ( i % 2 ) ;
#else	//MODULE_VENDOR == 9
		UlCmdID		= ( ( ( UINT_32 )0xF008 + ( i % 2 ) ) << 8 ) + BURST_LENGTH ;
#endif	//MODULE_VENDOR == 9
		RamWrite32A( 0xF00A, ( UINT_32 )( ( BURST_LENGTH / 5 ) * i + ScNvrMan ) ) ;		// set Flash write address
		RamWrite32A( 0xF00B, ( BURST_LENGTH / 5 ) ) ;									// set Data Length
		RamWrite32A( 0xF00C, 1 + ( i % 2 ) ) ;											// set read operation

		UlCnt	= 0 ;
		do {																			// Check Busy flag ?
			if( UlCnt++ > 100 ) return( 1 ) ;
			RamRead32A( 0xF00C, &UlReadVal ) ;
		} while( UlReadVal != 0 ) ;
#if		MODULE_VENDOR == 9
		RamWrite32A( RXCMDBUFFER, ( unsigned long )BURST_LENGTH << 16 ) ;
		CntRd3( ( unsigned int )UlCmdID, ( char * )&NcDataVal[ i * BURST_LENGTH ], BURST_LENGTH ) ;
#else	//MODULE_VENDOR == 9
		CntRd3( ( UINT_32 )UlCmdID, ( char * )&NcDataVal[ i * BURST_LENGTH ], BURST_LENGTH ) ;
#endif	//MODULE_VENDOR == 9
	}

	return( 0 ) ;
}

//********************************************************************************
// Function Name 	: FlashDownload
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: 
// History			: Second edition
//********************************************************************************
const DOWNLOAD_TBL DTbl[] = {
	{0x0905, CcMagicCodeF40_09_05_05, sizeof(CcMagicCodeF40_09_05_05), CcFromCodeF40_09_05_05, sizeof(CcFromCodeF40_09_05_05) },
	{0x0907, CcMagicCodeF40_09_05_07, sizeof(CcMagicCodeF40_09_05_07), CcFromCodeF40_09_05_07, sizeof(CcFromCodeF40_09_05_07) },
	{0xFFFF, (void*)0,                0,                               (void*)0,               0                              }
};

UINT_8 FlashDownload( UINT_8 chiperase, UINT_8 ModuleVendor, UINT_8 MdlVer )
{
	DOWNLOAD_TBL* ptr;

	ptr = ( DOWNLOAD_TBL * )DTbl;
	while (ptr->Index != 0xFFFF ){
		if( ptr->Index == ( ((UINT_16)ModuleVendor<<8) + MdlVer) ) break;
		ptr++ ;
	}
	if (ptr->Index == 0xFFFF)	return(0xF0);

	return FlashUpdateF40( chiperase, ptr );
}


UINT_8 FlashUpdateF40( UINT_8 chiperase, DOWNLOAD_TBL* ptr )
{
	INT_32 SiWrkVl0 ,SiWrkVl1;
	INT_32 SiAdrVal;

	const UINT_8 *NcDatVal;
	UINT_32 UlReadVal, UlCnt;
	UINT_8 ans, i;

	UINT_16 UsChkBlocks;

	UINT_8 UserMagicCode[ptr->SizeMagicCode];
	



//--------------------------------------------------------------------------------
// 0. Start up to boot exection
//--------------------------------------------------------------------------------
	IOWrite32A( SYSDSP_REMAP, 0x00001440 ) ;				// CORE_RST[12], MC_IGNORE2[10] = 1 PRAMSEL[7:6]=01b
	WitTim( 25 ) ;											// Bootプログラムを回すのに25msec必要。

	IORead32A( SYSDSP_SOFTRES, (UINT_32 *)&SiWrkVl0 ) ;		// SPI 無効
	SiWrkVl0	&= 0xFFFFEFFF ;
	IOWrite32A( SYSDSP_SOFTRES, SiWrkVl0 ) ;

	// Bootで立ち上がるとOSCの設定は1/2で動いているが、Flashのアクセスは41MHzに変更しないといけないため。
	RamWrite32A( 0xF006 , 0x00000000 ) ;					// Clock set
	IOWrite32A( SYSDSP_DSPDIV, 1 ) ;						// DSPDIV

	// Pmem にFlash 関連のコマンドを追加
	RamWrite32A( 0x0344, 0x00000014 ) ;						// set Pmem.Lnegth
	SiAdrVal =0x00100000;

	for(UlCnt = 0 ;UlCnt < 25 ; UlCnt++ ){
		RamWrite32A( 0x0340, SiAdrVal ) ;					// アドレスセット
		SiAdrVal+= 0x00000008;								// addressのインクリメント
		RamWrite32A( 0x0348, UlPmemCodeF40[UlCnt*5] ) ;			// set Pmem.Data[0]
		RamWrite32A( 0x034C, UlPmemCodeF40[UlCnt*5+1] ) ;		// set Pmem.Data[1]
		RamWrite32A( 0x0350, UlPmemCodeF40[UlCnt*5+2] ) ;		// set Pmem.Data[2]
		RamWrite32A( 0x0354, UlPmemCodeF40[UlCnt*5+3] ) ;		// set Pmem.Data[3]
		RamWrite32A( 0x0358, UlPmemCodeF40[UlCnt*5+4] ) ;		// set Pmem.Data[4]
		RamWrite32A( 0x033c, 0x00000001 ) ;					// set Pmem.Control(write enable)
	}
	// Pmem テーブルのセット
	for(UlCnt = 0 ;UlCnt < 9 ; UlCnt++ ){
		CntWrt( (INT_8 *)&UpData_CommandFromTable[UlCnt*6], 6 );
	}

//--------------------------------------------------------------------------------
// 1. <Main area> Erase
//--------------------------------------------------------------------------------
	if( chiperase ) {
		//--------------------------------------------------------------------------------
		// CHIP ERASEを使用する（ユーザーエリアも消去する）
		//--------------------------------------------------------------------------------
		// Flash Memory40[KByte]のAll Eraseの実行
		ans = UnlockCodeSet();
		if ( ans != 0 ) return (ans);							// Unlock Code Set

		IOWrite32A( FLASHROM_F40_ADR, 0x00000000 ) ;			// ADR[31:17]=0000h, ADR[16]=1			// ADR[15:0]=0000h

		IOWrite32A( FLASHROM_F40_CMD, 5 );						// Chip Erase Start

		WitTim( 13 ) ;
		UlCnt=0;
		do {
			if( UlCnt++ > 100 ){ ans=0x10; break; }

			IORead32A( FLASHROM_F40_INT, &UlReadVal );
		}while ( (UlReadVal & 0x00000080) != 0 );

	} else {
		//--------------------------------------------------------------------------------
		// BLOCK ERASEを使用する（ユーザーエリアは消さない）
		//--------------------------------------------------------------------------------
		// Code AreaのBlock Erase実行
		for( i = 0 ; i < ERASE_BLOCKS ; i++ )
		{
			ans	= FlashBlockErase( i * BLOCK_UNIT );
			if( ans != 0 ) {
				return( ans ) ;
			}
		}
		ans = UnlockCodeSet();
		if ( ans != 0 ) return (ans);							// Unlock Code Set
	}
//--------------------------------------------------------------------------------
// 2. <NVR> NVR0:MagicCode area erase
//--------------------------------------------------------------------------------
	IOWrite32A( FLASHROM_F40_ADR, 0x00010000 ) ;			// set NVR1

	IOWrite32A( FLASHROM_F40_CMD, 4 ) ;  					// Sector Erase Start

	WitTim( 5 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 100 ) { ans=0x10; break; }

		IORead32A( FLASHROM_F40_INT, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );

//--------------------------------------------------------------------------------
// 3. <Main area> Program from_all.txt
//--------------------------------------------------------------------------------
	FlashBurstWriteF40( ptr->FromCode, ptr->SizeFromCode, 0 );

	ans |= UnlockCodeClear();					// Unlock Code Clear
	if ( ans != 0 ) return (ans);				//既にErrorが発生していたらWPB=LOWにしてから抜ける

//--------------------------------------------------------------------------------
// 4. <Main area> Write
//--------------------------------------------------------------------------------
	UsChkBlocks = ( ptr->SizeFromCode / 160 ) + 1 ;
	TRACE("Flash code block %x(%d)\n",UsChkBlocks, UsChkBlocks  ) ;
	RamWrite32A( 0xF00A , 0x00000000 ) ;		// set Flash write address
#if 1
	RamWrite32A( 0xF00B , UsChkBlocks ) ;		// Maxサイズの設定160*UsChkBlocks
#else
	RamWrite32A( 0xF00B , 0x00000100 ) ;		// Maxサイズの設定160*256=40960
#endif
	RamWrite32A( 0xF00C , 0x00000100 ) ;		// set checksum operation

	NcDatVal = ptr->FromCode;
	SiWrkVl0 = 0;
	for( UlCnt = 0; UlCnt < ptr->SizeFromCode ;UlCnt++){
		SiWrkVl0 += *NcDatVal++;
	}
#if 1
	UsChkBlocks *= 160 ;


	for( ; UlCnt < UsChkBlocks ; UlCnt++ )
	{
		SiWrkVl0 += 0xFF ;
	}
#else
	for(; UlCnt < 40960; UlCnt++){
		SiWrkVl0 += 0xFF;
	}
#endif

	UlCnt=0;
	do{																// Check Busy flag ?
		if( UlCnt++ > 100 ) return ( 6 );
		RamRead32A( 0xF00C, &UlReadVal );
	}while( UlReadVal != 0 );

	RamRead32A( 0xF00D, &SiWrkVl1 );								// read CheckSum Value
/*HTC_START*/
#if 0
	if( SiWrkVl0 != SiWrkVl1 ){
#else
	if( (int)SiWrkVl0 != (int)SiWrkVl1 ){
#endif
/*HTC_END*/
		return(0x20);
	}

//--------------------------------------------------------------------------------
// X. CHIP ERASEしない場合はユーザエリアのチェックサムを作成
//--------------------------------------------------------------------------------

	// ChipEraseしない場合
	if ( !chiperase ) {
		UINT_32 sumH, sumL;
		UINT_16 Idx;

		// if you can use memcpy(), modify code.
		for( UlCnt = 0; UlCnt < ptr->SizeMagicCode ;UlCnt++) {
			UserMagicCode[UlCnt] = ptr->MagicCode[UlCnt];
		}

		// ユーザエリアのチェックサム計算
		for( UlCnt = 0; UlCnt < USER_RESERVE; UlCnt++ ) {

			CalcBlockChksum( ERASE_BLOCKS + UlCnt, &sumH, &sumL );

			Idx =  (ERASE_BLOCKS + UlCnt) * 2 * 5 + 1 + 40;
			NcDatVal = (UINT_8 *)&sumH;

#ifdef _BIG_ENDIAN_
			// for BIG ENDIAN SYSTEM
			UserMagicCode[Idx++] = *NcDatVal++;
			UserMagicCode[Idx++] = *NcDatVal++;
			UserMagicCode[Idx++] = *NcDatVal++;
			UserMagicCode[Idx++] = *NcDatVal++;
			Idx++;
			NcDatVal = (UINT_8 *)&sumL;
			UserMagicCode[Idx++] = *NcDatVal++;
			UserMagicCode[Idx++] = *NcDatVal++;
			UserMagicCode[Idx++] = *NcDatVal++;
			UserMagicCode[Idx++] = *NcDatVal++;
#else
			// for LITTLE ENDIAN SYSTEM
			UserMagicCode[Idx+3] = *NcDatVal++;
			UserMagicCode[Idx+2] = *NcDatVal++;
			UserMagicCode[Idx+1] = *NcDatVal++;
			UserMagicCode[Idx+0] = *NcDatVal++;
			Idx+=5;
			NcDatVal = (UINT_8 *)&sumL;
			UserMagicCode[Idx+3] = *NcDatVal++;
			UserMagicCode[Idx+2] = *NcDatVal++;
			UserMagicCode[Idx+1] = *NcDatVal++;
			UserMagicCode[Idx+0] = *NcDatVal++;
#endif
		}
		NcDatVal = UserMagicCode;

	} else {
		NcDatVal = ptr->MagicCode;
	}



//--------------------------------------------------------------------------------
// 5. <NVR1> Program magic code again
//--------------------------------------------------------------------------------
	ans = UnlockCodeSet();
	if ( ans != 0 ) return (ans);							// Unlock Code Set

	FlashBurstWriteF40( NcDatVal, ptr->SizeMagicCode, 0x00010000 );					// Magic Code Write

	UnlockCodeClear();										// Unlock Code Clear

//--------------------------------------------------------------------------------
// 6. <NVR1> Verify
//--------------------------------------------------------------------------------
	RamWrite32A( 0xF00A , 0x00010000 ) ;				// set Flash write address
	RamWrite32A( 0xF00B , 0x00000002 ) ;				// Maxサイズの設定160*2=320
	RamWrite32A( 0xF00C , 0x00000100 ) ;				// set checksum operation

//	NcDatVal = ptr->MagicCode;
	SiWrkVl0 = 0;
	for( UlCnt = 0; UlCnt < ptr->SizeMagicCode ;UlCnt++){
		SiWrkVl0 += *NcDatVal++;
	}
	for(; UlCnt < 320; UlCnt++){
		SiWrkVl0 += 0xFF;
	}

	UlCnt=0;
	do{
		if( UlCnt++ > 100 ) return ( 6 );
		RamRead32A( 0xF00C, &UlReadVal );
	}while( UlReadVal != 0 );
	RamRead32A( 0xF00D, &SiWrkVl1 );						// read CheckSum Value

/*HTC_START*/
#if 0
	if( SiWrkVl0 != SiWrkVl1 ){
#else
	if( (int)SiWrkVl0 != (int)SiWrkVl1 ){
#endif
/*HTC_END*/

		return(0x30);
	}

//	ReMapコマンドだと、Check Sumを見逃す為ここではリーブートをさせます。
	IOWrite32A( SYSDSP_REMAP, 0x00001000 ) ;				// CORE_RST[12], MC_IGNORE2[10] = 1 PRAMSEL[7:6]=01b
	return( 0 );
}


//********************************************************************************
// Function Name 	: FlashByteRead
// Retun Value		: NON
// Argment Value	: Read Address, Read Data Pointer, Address for Byte Unit
// Explanation		: Read Byte Data for Flash Memory
// History			: First edition 									2016.02.10
//********************************************************************************
void	FlashByteRead( UINT_32 UlAddress, UINT_8 *PucData, UINT_8 UcByteAdd )
{
	UINT_8	UcReadDat[ 4 ] ;

	IOWrite32A( FLASHROM_F40_ADR, UlAddress ) ;

	IOWrite32A( FLASHROM_F40_ACSCNT, 0 ) ;

	IOWrite32A( FLASHROM_F40_CMD, 1 ) ;  					// Read Start

	if( UcByteAdd < 4 ) {
		RamWrite32A( CMD_IO_ADR_ACCESS , FLASHROM_F40_RDATL ) ;
		RamRead32A(  CMD_IO_DAT_ACCESS , UcReadDat ) ;

		*PucData	= UcReadDat[ UcByteAdd ] ;
	} else {
		RamWrite32A( CMD_IO_ADR_ACCESS , FLASHROM_F40_RDATH ) ;
		RamRead32A(  CMD_IO_DAT_ACCESS , UcReadDat ) ;
		*PucData	= UcReadDat[ 0 ] ;
		// dummy read
		RamWrite32A( CMD_IO_ADR_ACCESS , FLASHROM_F40_RDATL ) ;
		RamRead32A(  CMD_IO_DAT_ACCESS , UcReadDat ) ;
	}
}

//********************************************************************************
// Function Name 	: FlashInt32Write
// Retun Value		: NON
// Argment Value	: Write Address, Write Data Pointer, Data length
// Explanation		: Write Int32 Data to Flash Memory
// History			: First edition 									2016.05.19
//********************************************************************************
UINT_8	FlashInt32Write( UINT_32 UlAddress, UINT_32 *PuiData, UINT_8 UcLength )
{
	UINT_8	UcIndex ;
	UINT_8 * PucData = (UINT_8 *)PuiData ;
	UINT_8	UcResult = 0 ;
	UINT_32	UlOffset = UlAddress & 0x3F ;
	UINT_32	UlSecAddress = UlAddress & 0xFFFFFFC0 ;
	UINT_8	SectorData[ SECTOR_SIZE ];

	// Range check
	if( (UlOffset + UcLength) > 0x40 )
		return 9;

	// Prepare sector data
	FlashSectorRead( UlSecAddress, SectorData );

	// Copy data to sector buffer
	for( UcIndex = 0; UcIndex < UcLength; UcIndex++ )
	{
#ifdef _BIG_ENDIAN_
		SectorData[ (UlOffset + UcIndex) * 5 + 1 ] = *PucData++ ;
		SectorData[ (UlOffset + UcIndex) * 5 + 2 ] = *PucData++ ;
		SectorData[ (UlOffset + UcIndex) * 5 + 3 ] = *PucData++ ;
		SectorData[ (UlOffset + UcIndex) * 5 + 4 ] = *PucData++ ;
#else
		SectorData[ (UlOffset + UcIndex) * 5 + 4 ] = *PucData++ ;
		SectorData[ (UlOffset + UcIndex) * 5 + 3 ] = *PucData++ ;
		SectorData[ (UlOffset + UcIndex) * 5 + 2 ] = *PucData++ ;
		SectorData[ (UlOffset + UcIndex) * 5 + 1 ] = *PucData++ ;
#endif
	}

	UcResult = FlashSectorWrite( UlSecAddress, SectorData );
	if ( UcResult == 0 )
	{
		UINT_32	UlSumH, UlSumL ;
		UINT_8	UcBlockNum = (UlSecAddress >> 9) & 0x0F ;
		UINT_8 *	NcDatVal ;

		// Calculate check sum
		CalcBlockChksum( UcBlockNum, &UlSumH, &UlSumL ) ;

		// NVR1 sector read
		FlashSectorRead( 0x00010000, SectorData ) ;

		// Update block checksum
		UcIndex =  UcBlockNum * 2 * 5 + 1 + 40;
		NcDatVal = (UINT_8 *)&UlSumH;

#ifdef _BIG_ENDIAN_
			// for BIG ENDIAN SYSTEM
			SectorData[UcIndex++] = *NcDatVal++;
			SectorData[UcIndex++] = *NcDatVal++;
			SectorData[UcIndex++] = *NcDatVal++;
			SectorData[UcIndex++] = *NcDatVal++;
			UcIndex++;
			NcDatVal = (UINT_8 *)&UlSumL;
			SectorData[UcIndex++] = *NcDatVal++;
			SectorData[UcIndex++] = *NcDatVal++;
			SectorData[UcIndex++] = *NcDatVal++;
			SectorData[UcIndex++] = *NcDatVal++;
#else
			// for LITTLE ENDIAN SYSTEM
			SectorData[UcIndex+3] = *NcDatVal++;
			SectorData[UcIndex+2] = *NcDatVal++;
			SectorData[UcIndex+1] = *NcDatVal++;
			SectorData[UcIndex+0] = *NcDatVal++;
			UcIndex += 5;
			NcDatVal = (UINT_8 *)&UlSumL;
			SectorData[UcIndex+3] = *NcDatVal++;
			SectorData[UcIndex+2] = *NcDatVal++;
			SectorData[UcIndex+1] = *NcDatVal++;
			SectorData[UcIndex+0] = *NcDatVal++;
#endif

		// NVR1 sector write
		UcResult = FlashSectorWrite( 0x00010000, SectorData ) ;
	}

	return UcResult ;
}

//********************************************************************************
// Function Name 	: FlashSectorRead
// Retun Value		: NON
// Argment Value	: Read Address, Read Data Pointer
// Explanation		: Read Sector Data for Flash Memory
// History			: First edition 									2016.02.10
//********************************************************************************
void	FlashSectorRead( UINT_32 UlAddress, UINT_8 *PucData )
{
	UINT_8	UcIndex, UcNum ;
	UINT_8	UcReadDat[ 4 ] ;

	IOWrite32A( FLASHROM_F40_ADR, ( UlAddress & 0xFFFFFFC0 ) ) ;

	IOWrite32A( FLASHROM_F40_ACSCNT, 63 ) ;
	UcNum	= 64 ;

	IOWrite32A( FLASHROM_F40_CMD, 1 ) ;  					// Read Start

	for( UcIndex = 0 ; UcIndex < UcNum ; UcIndex++ )
	{
		RamWrite32A( CMD_IO_ADR_ACCESS , FLASHROM_F40_RDATH ) ;
		RamRead32A(  CMD_IO_DAT_ACCESS , UcReadDat ) ;
		*PucData++		= UcReadDat[ 0 ] ;
		RamWrite32A( CMD_IO_ADR_ACCESS , FLASHROM_F40_RDATL ) ;
		RamRead32A(  CMD_IO_DAT_ACCESS , UcReadDat ) ;
		*PucData++	= UcReadDat[ 3 ] ;
		*PucData++	= UcReadDat[ 2 ] ;
		*PucData++	= UcReadDat[ 1 ] ;
		*PucData++	= UcReadDat[ 0 ] ;
	}
}

//HTC_START
void	FlashSectorRead_htc( UINT_32 UlAddress, UINT_8 *PucData, UINT_32 version )
{
	UINT_8	UcIndex, UcNum ;
	UINT_8	UcReadDat[ 4 ] ;

	IOWrite32A( FLASHROM_F40_ADR, ( UlAddress & 0xFFFFFFC0 ) ) ;

	IOWrite32A( FLASHROM_F40_ACSCNT, 63 ) ;
	UcNum	= 64 ;

	IOWrite32A( FLASHROM_F40_CMD, 1 ) ;  					// Read Start

	for( UcIndex = 0 ; UcIndex < UcNum ; UcIndex++ )
	{
		RamWrite32A( CMD_IO_ADR_ACCESS , FLASHROM_F40_RDATH ) ;
		RamRead32A(  CMD_IO_DAT_ACCESS , UcReadDat ) ;
		if(version == 0)
			*PucData++		= UcReadDat[ 0 ] ;
		RamWrite32A( CMD_IO_ADR_ACCESS , FLASHROM_F40_RDATL ) ;
		RamRead32A(  CMD_IO_DAT_ACCESS , UcReadDat ) ;
		*PucData++	= UcReadDat[ 3 ] ;
		*PucData++	= UcReadDat[ 2 ] ;
		*PucData++	= UcReadDat[ 1 ] ;
		*PucData++	= UcReadDat[ 0 ] ;
	}
}
//HTC_END

//********************************************************************************
// Function Name 	: FlashSectorRead_Burst
// Retun Value		: NON
// Argment Value	: Read Address, Read Data Pointer
// Explanation		: Read Sector Data for Flash Memory
// History			: First edition 									2016.02.29
//********************************************************************************
UINT_8	FlashSectorRead_Burst( UINT_32 UlAddress, UINT_8 *PucData, UINT_8 UcSectorNum )
{
	UINT_8	UcIndex, UcStatus ;
	INT_32	SiWrkVal ;

	// Core Reset & Magic Code Ignore
	IOWrite32A( SYSDSP_REMAP, 0x00001440 ) ;				// CORE_RST[12], MC_IGNORE2[10] = 1 PRAMSEL[7:6]=01b
	WitTim( 1 ) ;											// Bootプログラムを回すのに1msec必要。

	IORead32A( SYSDSP_SOFTRES, (UINT_32 *)&SiWrkVal ) ;
	SiWrkVal	&= 0xFFFFEFFF ;
	IOWrite32A( SYSDSP_SOFTRES, SiWrkVal ) ;

	// Bootで立ち上がるとOSCの設定は1/2で動いているが、Flashのアクセスは41MHzに変更しないといけないため。
	RamWrite32A( 0xF006 , 0x00000000 ) ;					// Clock set
	IOWrite32A(  SYSDSP_DSPDIV, 1 ) ;						// DSPDIV

	for( UcIndex = 0 ; UcIndex < UcSectorNum ; UcIndex++ )
	{
		UcStatus	= FlashBurstReadF40( &PucData[ UcIndex * SECTOR_SIZE ], SECTOR_SIZE, ( UINT_32 )( UlAddress + ( UcIndex * ( SECTOR_SIZE / 5 ) ) ) ) ;
		if( UcStatus ) {
			return( UcStatus ) ;
		}
	}

//	ReMapコマンドだと、Check Sumを見逃す為ここではリブートをさせます。
	IOWrite32A( SYSDSP_REMAP, 0x00001000 ) ;				// CORE_RST[12], MC_IGNORE2[10] = 1 PRAMSEL[7:6]=01b
	WitTim( 10 ) ;

	return( 0 ) ;

}

//********************************************************************************
// Function Name 	: FlashBlockErase
// Retun Value		: 0 : Success, 1 : Unlock Error, 2 : Time Out Error
// Argment Value	: Flash Address
// Explanation		: <Flash Memory> Block Erase
// History			: First edition 						2016.02.17 Y.Tashita
//********************************************************************************
UINT_8	FlashBlockErase( UINT_32 SetAddress )
{
	UINT_32	UlReadVal, UlCnt;
	UINT_8	ans	= 0 ;

	// fail safe
	// reject command if address inner NVR3
	if( SetAddress & 0x00010000 )
		return 9;

	// Flash write準備
	ans	= UnlockCodeSet();
	if( ans != 0 )	return( ans ) ;							// Unlock Code Set

	IOWrite32A( FLASHROM_F40_ADR, SetAddress & 0xFFFFFE00 ) ;
	// Sector Erase Start
	IOWrite32A( FLASHROM_F40_CMD, 6	/* BLOCK ERASE */ ) ;

	WitTim( 5 ) ;

	UlCnt	= 0 ;

	do {
		if( UlCnt++ > 100 ){	ans = 2;	break;	} ;

		IORead32A( FLASHROM_F40_INT, &UlReadVal ) ;
	} while( ( UlReadVal & 0x00000080 ) != 0 ) ;

	UnlockCodeClear();										// Unlock Code Clear

	return( ans ) ;
}



//********************************************************************************
// Function Name 	: FlashSectorErase
// Retun Value		: 0 : Success, 1 : Unlock Error, 2 : Time Out Error
// Argment Value	: Flash Address
// Explanation		: <Flash Memory> Sector Erase
// History			: First edition 						2016.02.17 Y.Tashita
//********************************************************************************
UINT_8	FlashSectorErase( UINT_32 SetAddress )
{
	UINT_32	UlReadVal, UlCnt;
	UINT_8	ans	= 0 ;

	// fail safe
	// reject command if address inner NVR3
	if( (SetAddress & 0x000100FF) >  0x0001007F )
		return 9;

	// protect program area
	if( USER_AREA_START > (SetAddress & 0xFFFFFFC0) )
		return 9;

	// Flash write準備
	ans	= UnlockCodeSet();
	if( ans != 0 )	return( ans ) ;							// Unlock Code Set

	IOWrite32A( FLASHROM_F40_ADR, SetAddress & 0xFFFFFFC0 ) ;
	// Sector Erase Start
	IOWrite32A( FLASHROM_F40_CMD, 4 ) ;

	WitTim( 5 ) ;

	UlCnt	= 0 ;

	do {
		if( UlCnt++ > 100 ){	ans = 2;	break;	} ;

		IORead32A( FLASHROM_F40_INT, &UlReadVal ) ;
	} while( ( UlReadVal & 0x00000080 ) != 0 ) ;

	ans = UnlockCodeClear();								// Unlock Code Clear

	return( ans ) ;
}



//********************************************************************************
// Function Name 	: FlashSectorWrite
// Retun Value		: NON
// Argment Value	: Address, Write Data
// Explanation		: Program to Flash Memory
// History			: First Edition 									2016.02.15
//					  Second Edition									2016.03.10
//********************************************************************************
UINT_8	FlashSectorWrite( UINT_32 UlAddress, UINT_8 *PucData )
{
	UINT_8	UcNum, UcStatus = 0, UcSize = 0 ;
	UnDwdVal		UnWriteDat ;

	// fail safe
	// reject command if address inner NVR3
	if( (UlAddress & 0x000100FF) >  0x0001007F )
		return 9;

	// protect program area
	if( USER_AREA_START > (UlAddress & 0xFFFFFFC0) )
		return 9;

	UcStatus	= FlashSectorErase( UlAddress & 0xFFFFFFC0 ) ;
	if( UcStatus != 0 ) {
		return( UcStatus ) ;
	}

	// Flash write準備
	UcStatus	= UnlockCodeSet();
	if( UcStatus != 0 ) {
		return( UcStatus ) ;											// Unlock Code Set
	}

	do {
		// Count
		IOWrite32A( FLASHROM_F40_ACSCNT, ( FLASH_ACCESS_SIZE - 1 ) ) ;	// Access Count	// Max.32まで

		// NVR2 Addres Set
		IOWrite32A( FLASHROM_F40_ADR, UlAddress + UcSize ) ;
		// Write Start
		IOWrite32A( FLASHROM_F40_CMD, 2 ) ;  							// Program

		for( UcNum = 0 ; UcNum < FLASH_ACCESS_SIZE ; UcNum++ )
		{
			UnWriteDat.StCdwVal.UcRamVa0	= *PucData++ ;
			IOWrite32A( FLASHROM_F40_WDATH, UnWriteDat.UlDwdVal ) ;

			UnWriteDat.StCdwVal.UcRamVa3	= *PucData++ ;
			UnWriteDat.StCdwVal.UcRamVa2	= *PucData++ ;
			UnWriteDat.StCdwVal.UcRamVa1	= *PucData++ ;
			UnWriteDat.StCdwVal.UcRamVa0	= *PucData++ ;
			IOWrite32A( FLASHROM_F40_WDATL, UnWriteDat.UlDwdVal ) ;
			UcSize++ ;
		}
	} while( UcSize < 64 ) ;											// 64 * 5 = 320 : Sector Size

	UcStatus = UnlockCodeClear() ;										// Unlock Code Clear

	return( UcStatus ) ;
}



//********************************************************************************
// Function Name 	: FlashSectorWrite_Burst
// Retun Value		: NON
// Argment Value	: Address, Write Data, Write Sector Number
// Explanation		: Program to Flash Memory
// History			: First edition 									2016.02.29
//********************************************************************************
UINT_8	FlashSectorWrite_Burst( UINT_32 UlAddress, UINT_8 *PucData, UINT_8 UcSectorNum )
{
	UINT_8	UcStatus	= 0 ;
	UINT_8	UcRemainSecNum, UcBlockErased	= 0 ;
	UINT_32	UlCnt ;
	INT_32	SiAdrVal, SiWrkVal ;

	// Core Reset & Magic Code Ignore
	IOWrite32A( SYSDSP_REMAP, 0x00001440 ) ;					// CORE_RST[12], MC_IGNORE2[10] = 1 PRAMSEL[7:6]=01b
	WitTim( 1 ) ;												// Bootプログラムを回すのに1msec必要。

	IORead32A( SYSDSP_SOFTRES, (UINT_32 *)&SiWrkVal ) ;
	SiWrkVal	&= 0xFFFFEFFF ;
	IOWrite32A( SYSDSP_SOFTRES, SiWrkVal ) ;

	// Bootで立ち上がるとOSCの設定は1/2で動いているが、Flashのアクセスは41MHzに変更しないといけないため。
	RamWrite32A( 0xF006 , 0x00000000 ) ;						// Clock set
	IOWrite32A( SYSDSP_DSPDIV, 1 ) ;							// DSPDIV

	// Prepare Program Memory Code
	// Pmem にFlash 関連のコマンドを追加
	RamWrite32A( 0x0344, 0x00000014 ) ;							// set Pmem.Lnegth
	SiAdrVal	= 0x00100000 ;

	for( UlCnt = 0 ; UlCnt < 25 ; UlCnt++ ) {
		RamWrite32A( 0x0340, SiAdrVal ) ;						// アドレスセット
		SiAdrVal+= 0x00000008;									// addressのインクリメント
		RamWrite32A( 0x0348, UlPmemCodeF40[ UlCnt * 5 ] ) ;		// set Pmem.Data[0]
		RamWrite32A( 0x034C, UlPmemCodeF40[ UlCnt * 5 + 1 ] ) ;	// set Pmem.Data[1]
		RamWrite32A( 0x0350, UlPmemCodeF40[ UlCnt * 5 + 2 ] ) ;	// set Pmem.Data[2]
		RamWrite32A( 0x0354, UlPmemCodeF40[ UlCnt * 5 + 3 ] ) ;	// set Pmem.Data[3]
		RamWrite32A( 0x0358, UlPmemCodeF40[ UlCnt * 5 + 4 ] ) ;	// set Pmem.Data[4]
		RamWrite32A( 0x033c, 0x00000001 ) ;						// set Pmem.Control(write enable)
	}

	// Pmem テーブルのセット
	for( UlCnt = 0 ; UlCnt < 9 ; UlCnt++ ) {
		CntWrt( ( char * )&UpData_CommandFromTable[ UlCnt * 6 ], 6 ) ;
	}

	UcRemainSecNum	= UcSectorNum ;
	for( UlCnt = 0 ; UlCnt < UcSectorNum ; UlCnt++ )
	{
		if( !( UlCnt % 8 ) ) {
			UcBlockErased	= 0 ;
		}

		if( !UcBlockErased ) {
			if( UcRemainSecNum / 8 ) {
				UcStatus	= FlashBlockErase( ( UlAddress & 0xFFFFFFC0 ) + ( UlCnt * ( SECTOR_SIZE / 5 ) ) ) ;
				if( UcStatus != 0 ) {
					return( UcStatus ) ;
				}
				UcRemainSecNum	-= 8 ;
				UcBlockErased	= 1 ;
			} else if( UcRemainSecNum < 8 ) {
				UcStatus	= FlashSectorErase( ( UlAddress & 0xFFFFFFC0 ) + ( UlCnt * ( SECTOR_SIZE / 5 ) ) ) ;
				if( UcStatus != 0 ) {
					return( UcStatus ) ;
				}
				UcRemainSecNum-- ;
			}
		}

		UcStatus	= UnlockCodeSet() ;											// Unlock Code Set
		if( UcStatus != 0 ) {
			return( UcStatus ) ;
		}

		UcStatus	= FlashBurstWriteF40( &PucData[ UlCnt * SECTOR_SIZE ], SECTOR_SIZE, ( UINT_32 )( UlAddress + ( UlCnt * ( SECTOR_SIZE / 5 ) ) ) ) ;
		if( UcStatus ) {
			return( UcStatus ) ;
		}

		UcStatus = UnlockCodeClear() ;											// Unlock Code Clear
	}

//	ReMapコマンドだと、Check Sumを見逃す為ここではリーブートをさせます。
	IOWrite32A( SYSDSP_REMAP, 0x00001000 ) ;									// CORE_RST[12], MC_IGNORE2[10] = 1 PRAMSEL[7:6]=01b
	WitTim( 10 ) ;
	return( 0 );
}



//********************************************************************************
// Function Name 	: FlashProtectStatus
// Retun Value		: char 0:Protected 1: Not Protected
// Argment Value	: NON
// Explanation		: Flash Write Protection Status Check
// History			: First edition 									2016.02.17
//********************************************************************************
UINT_8	FlashProtectStatus( void )
{
	UINT_32	UlReadVal;

	IORead32A( FLASHROM_F40_WPB, &UlReadVal ) ;					// FLASHWPB(E0_701Ch)

	UlReadVal &= 0x00000007;									// Bit2 : PINFLASHWPB, Bit1:I2CFLASHWPB, Bit0:REGFLASHWPB

	if( !UlReadVal ){
		return ( 0 );											// Protected
	}else{
		return ( 1 );											// Not Protected
	}
}


//********************************************************************************
// Function Name 	: EraseCalDataF40
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Flash erase calibration data(NVR2)
// History			: First edition 									2015.7.10
//********************************************************************************
UINT_8 EraseCalDataF40( void )
{
	UINT_32	UlReadVal, UlCnt;
	UINT_8 ans = 0;

	// Flash write準備
	ans = UnlockCodeSet();
	if ( ans != 0 ) return (ans);								// Unlock Code Set

	// set NVR2 area
	IOWrite32A( FLASHROM_F40_ADR, 0x00010040 ) ;
	// Sector Erase Start
	IOWrite32A( FLASHROM_F40_CMD, 4	/* SECTOR ERASE */ ) ;

	WitTim( 5 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 100 ){	ans = 2;	break;	} ;
		IORead32A( FLASHROM_F40_INT, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );

	ans = UnlockCodeClear();									// Unlock Code Clear

	return(ans);
}

//********************************************************************************
// Function Name 	: ReadCalDataF40
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Flash read calibration data(NVR2)
// History			: First edition 									2015.7.10
//********************************************************************************

void ReadCalDataF40( UINT_32 * BufDat, UINT_32 * ChkSum )
{
	UINT_16	UsSize = 0, UsNum;

	*ChkSum = 0;

	do{
		// Count
		IOWrite32A( FLASHROM_F40_ACSCNT, (FLASH_ACCESS_SIZE-1) ) ;

		// NVR2 Addres Set
		IOWrite32A( FLASHROM_F40_ADR, 0x00010040 + UsSize ) ;		// set NVR2 area
		// Read Start
		IOWrite32A( FLASHROM_F40_CMD, 1 ) ;  						// Read Start

		RamWrite32A( CMD_IO_ADR_ACCESS , FLASHROM_F40_RDATL ) ;		// RDATL data

		for( UsNum = 0; UsNum < FLASH_ACCESS_SIZE; UsNum++ )
		{
			RamRead32A(  CMD_IO_DAT_ACCESS , &(BufDat[ UsSize ]) ) ;

			*ChkSum += BufDat[ UsSize++ ];
		}
	}while (UsSize < 64);	// 64*5 = 320 : NVR sector size
}

//********************************************************************************
// Function Name 	: WriteCalDataF40
// Retun Value		: CheckSum write data
// Argment Value	: NON
// Explanation		: Flash write calibration data(NVR2)
// History			: First edition 									2015.7.10
//********************************************************************************
UINT_8 WriteCalDataF40( UINT_32 * BufDat, UINT_32 * ChkSum )
{
	UINT_16	UsSize = 0, UsNum;
	UINT_8 ans = 0;
	UINT_32	UlReadVal = 0;

	*ChkSum = 0;

	// Flash write準備
	ans = UnlockCodeSet();
	if ( ans != 0 ) return (ans);									// Unlock Code Set

	IOWrite32A( FLASHROM_F40_WDATH, 0x000000FF ) ;

	do{
		// Count
		IOWrite32A( FLASHROM_F40_ACSCNT, (FLASH_ACCESS_SIZE - 1) ) ;// Max.32まで
		// NVR2 Addres Set
		IOWrite32A( FLASHROM_F40_ADR, 0x00010040 + UsSize ) ;
		// Write Start
		IOWrite32A( FLASHROM_F40_CMD, 2) ;  						// 1 byte program each flash

//		RamWrite32A( CMD_IO_ADR_ACCESS , FLASHROM_F40_WDATL ) ;

		for( UsNum = 0; UsNum < FLASH_ACCESS_SIZE; UsNum++ )
		{

//			RamWrite32A( CMD_IO_DAT_ACCESS , BufDat[ UsSize ] ) ;
			IOWrite32A( FLASHROM_F40_WDATL,  BufDat[ UsSize ] ) ;
			do {
				IORead32A( FLASHROM_F40_INT, &UlReadVal );
			}while ( (UlReadVal & 0x00000020) != 0 );

			*ChkSum += BufDat[ UsSize++ ];							// ChckSum計算
		}
	}while (UsSize < 64);											// 64*5 = 320 : NVR sector size

	ans = UnlockCodeClear();										// Unlock Code Clear

	return( ans );
}

//********************************************************************************
// Function Name 	: CalcChecksum
// Retun Value		: CheckSum data
// Argment Value	: pData, len,  *pSumH, *pSumL
// Explanation		: Calculation memory array
// History			: First edition 									2016.4.13
//********************************************************************************
void CalcChecksum( const UINT_8 *pData, UINT_32 len, UINT_32 *pSumH, UINT_32 *pSumL )
{
	UINT_64 sum = 0;
	UINT_32 dat;
	UINT_16 i;

	for( i = 0; i < len / 5; i++ ) {
		sum  += (UINT_64)*pData++ << 32;

		dat  = *pData++ << 24;
		dat += *pData++ << 16;
		dat += *pData++ << 8;
		dat += *pData++ ;
		sum += dat ;
	}

	*pSumH = (UINT_32)(sum >> 32);
	*pSumL = (UINT_32)(sum & 0xFFFFFFFF);
}

//********************************************************************************
// Function Name 	: CalcBlockChksum
// Retun Value		: CheckSum data
// Argment Value	: num, *pSumH, *pSumL
// Explanation		: Calculation flash block area
// History			: First edition 									2016.4.13
//********************************************************************************
void CalcBlockChksum( UINT_8 num, UINT_32 *pSumH, UINT_32 *pSumL )
{
	UINT_8 SectorData[SECTOR_SIZE];
	UINT_32 top;
	UINT_16 sec;
	UINT_64 sum = 0;
	UINT_32 datH, datL;

	top = num * BLOCK_UNIT;		// num * 0x200

	// 1blockループ 8sector
	for( sec = 0; sec < (BLOCK_BYTE / SECTOR_SIZE); sec++ ) {
		FlashSectorRead( top + sec * 64, SectorData );

		CalcChecksum( SectorData, SECTOR_SIZE, &datH, &datL );
		sum += ((UINT_64)datH << 32) + datL ;
	}

	*pSumH = (UINT_32)(sum >> 32);
	*pSumL = (UINT_32)(sum & 0xFFFFFFFF);

}
