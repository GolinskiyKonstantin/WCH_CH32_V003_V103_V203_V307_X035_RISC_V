/* 2014.09.09
*****************************************
**   Copyright  (C)  W.ch  1999-2019   **
**   Web:      http://wch.cn           **
*****************************************
**  USB-flash File Interface for CHRV3 **
**  KEIL423, gcc 8.20          **
*****************************************
*/
/* CHRV3 U�������ļ�ϵͳ�ӿ�, ֧��: FAT12/FAT16/FAT32 */

//#define DISK_BASE_BUF_LEN		512	/* Ĭ�ϵĴ������ݻ�������СΪ512�ֽ�(����ѡ��Ϊ2048����4096��֧��ĳЩ��������U��),Ϊ0���ֹ�ڱ��ļ��ж��建��������Ӧ�ó�����pDISK_BASE_BUF��ָ�� */
/* �����Ҫ���ô������ݻ������Խ�ԼRAM,��ô�ɽ�DISK_BASE_BUF_LEN����Ϊ0�Խ�ֹ�ڱ��ļ��ж��建����,����Ӧ�ó����ڵ���CHRV3LibInit֮ǰ��������������õĻ�������ʼ��ַ����pDISK_BASE_BUF���� */

//#define NO_DEFAULT_ACCESS_SECTOR	    1		/* ��ֹĬ�ϵĴ���������д�ӳ���,���������б�д�ĳ�������� */
//#define NO_DEFAULT_DISK_CONNECT		1		/* ��ֹĬ�ϵļ����������ӳ���,���������б�д�ĳ�������� */
//#define NO_DEFAULT_FILE_ENUMER		1		/* ��ֹĬ�ϵ��ļ���ö�ٻص�����,���������б�д�ĳ�������� */
//#include "CHRV3SFR.H"

#include "debug.h"
#include "ch32v30x.h"
#include "usb_host_config.h"
#include "CHRV3UFI.h"

uint8_t USBHostTransact( uint8_t endp_pid, uint8_t tog, uint32_t timeout )
{
#if DEF_USB_PORT_FS_EN
    uint8_t  r, trans_rerty;
    uint16_t i;
    USBOTG_H_FS->HOST_TX_CTRL = USBOTG_H_FS->HOST_RX_CTRL = 0;
    if( tog & 0x80 )
    {
        USBOTG_H_FS->HOST_RX_CTRL = 1<<2;
    }
    if( tog & 0x40 )
    {
        USBOTG_H_FS->HOST_TX_CTRL = 1<<2;
    }
    trans_rerty = 0;
    do
    {
        USBOTG_H_FS->HOST_EP_PID = endp_pid;       // Specify token PID and endpoint number
        USBOTG_H_FS->INT_FG = USBFS_UIF_TRANSFER;  // Allow transmission
        for( i = DEF_WAIT_USB_TOUT_200US; ( i != 0 ) && ( ( USBOTG_H_FS->INT_FG & USBFS_UIF_TRANSFER ) == 0 ); i-- )
        {
            Delay_Us( 1 );
        }
        USBOTG_H_FS->HOST_EP_PID = 0x00;  // Stop USB transfer
        if( ( USBOTG_H_FS->INT_FG & USBFS_UIF_TRANSFER ) == 0 )
        {
            return ERR_USB_UNKNOWN;
        }
        else
        {
            /* Complete transfer */
            if( USBOTG_H_FS->INT_ST & USBFS_UIS_TOG_OK )
            {
                return ERR_SUCCESS;
            }
            r = USBOTG_H_FS->INT_ST & USBFS_UIS_H_RES_MASK;  // USB device answer status
            if( r == USB_PID_STALL )
            {
                return ( r | ERR_USB_TRANSFER );
            }
            if( r == USB_PID_NAK )
            {
                if( timeout == 0 )
                {
                    return ( r | ERR_USB_TRANSFER );
                }
                if( timeout < 0xFFFF )
                {
                    timeout--;
                }
                --trans_rerty;
            }
            else switch ( endp_pid >> 4 )
            {
                case USB_PID_SETUP:
                case USB_PID_OUT:
                    if( r )
                    {
                        return ( r | ERR_USB_TRANSFER );
                    }
                    break;
                case USB_PID_IN:
                    if( ( r == USB_PID_DATA0 ) && ( r == USB_PID_DATA1 ) )
                    {
                        ;
                    }
                    else if( r )
                    {
                        return ( r | ERR_USB_TRANSFER );
                    }
                    break;
                default:
                    return ERR_USB_UNKNOWN;
            }
        }
        Delay_Us( 15 );
        if( USBOTG_H_FS->INT_FG & USBFS_UIF_DETECT )
        {
            Delay_Us( 200 );
            if( USBFSH_CheckRootHubPortEnable( ) == 0 )
            {
                return ERR_USB_DISCON;  // USB device disconnect event
            }
        }
    }while( ++trans_rerty < 10 );

    return ERR_USB_TRANSFER; // Reply timeout
#elif DEF_USB_PORT_HS_EN
    uint8_t   r, trans_retry;
    uint16_t  i;
    USBHSH->HOST_TX_CTRL = USBHSH->HOST_RX_CTRL = 0;
    if( tog & 0x80 )
    {
        USBHSH->HOST_RX_CTRL = 1<<3;
    }
    if( tog & 0x40 )
    {
        USBHSH->HOST_TX_CTRL = 1<<3;
    }
    trans_retry = 0;
    do
    {
        USBHSH->HOST_EP_PID = endp_pid; // Set the token for the host to send the packet
        USBHSH->INT_FG = USBHS_UIF_TRANSFER;
        for( i = DEF_WAIT_USB_TOUT_200US; ( i != 0 ) && ( ( USBHSH->INT_FG & USBHS_UIF_TRANSFER ) == 0 ); i-- )
        {
            Delay_Us( 1 );
        }
        USBHSH->HOST_EP_PID = 0x00;
        if( ( USBHSH->INT_FG & USBHS_UIF_TRANSFER ) == 0 )
        {
            return ERR_USB_UNKNOWN;
        }

        if( USBHSH->INT_FG & USBHS_UIF_DETECT )
        {
            USBHSH->INT_FG = USBHS_UIF_DETECT;
            Delay_Us( 200 );
            if( USBHSH->MIS_ST & USBHS_UIF_TRANSFER )
            {
                if( USBHSH->HOST_CTRL & USBHS_UH_SOF_EN )
                {
                    return ERR_USB_CONNECT;
                }
            }
            else
            {
                return ERR_USB_DISCON;
            }
        }
        else if( USBHSH->INT_FG & USBHS_UIF_TRANSFER ) // The packet transmission was successful
        {
            r = USBHSH->INT_ST & USBHS_UIS_H_RES_MASK;
            if( ( endp_pid >> 4 ) == USB_PID_IN )
            {
                if( USBHSH->INT_ST & USBHS_UIS_TOG_OK )
                {
                    return ERR_SUCCESS; // Packet token match
                }
            }
            else
            {
                if( ( r == USB_PID_ACK ) || ( r == USB_PID_NYET ) )
                {
                    return ERR_SUCCESS;
                }
            }
            if( r == USB_PID_STALL )
            {
                return ( r | ERR_USB_TRANSFER );
            }

            if( r == USB_PID_NAK )
            {
                if( timeout == 0 )
                {
                    return ( r | ERR_USB_TRANSFER );
                }
                if( timeout < 0xFFFF )
                {
                    timeout--;
                }
                --trans_retry;
            }
            else switch( endp_pid >> 4  )
            {
                case USB_PID_SETUP:

                case USB_PID_OUT:
                    if( r )
                    {
                        return ( r | ERR_USB_TRANSFER );
                    }
                    break;
                case USB_PID_IN:
                    if( ( r == USB_PID_DATA0 ) || ( r == USB_PID_DATA1 ) )
                    {
                        ;
                    }
                    else if( r )
                    {
                        return ( r | ERR_USB_TRANSFER );
                    }
                    break;
                default:
                    return ERR_USB_UNKNOWN;
            }
        }
        else
        {
            USBHSH->INT_FG = USBHS_UIF_DETECT | USBHS_UIF_TRANSFER | USBHS_UIF_SUSPEND | USBHS_UIF_HST_SOF | USBHS_UIF_FIFO_OV | USBHS_UIF_SETUP_ACT;
        }
        Delay_Us( 15 );
    } while( ++trans_retry < 10 );

    return ERR_USB_TRANSFER;
#endif
}

uint8_t HostCtrlTransfer( uint8_t *DataBuf, uint8_t *RetLen )
{
    uint8_t  ret;
    uint16_t retlen;
    retlen = (uint16_t)(*RetLen);
#if DEF_USB_PORT_FS_EN
    ret = USBFSH_CtrlTransfer( RootHubDev[ DEF_USB_PORT_HS ].bEp0MaxPks, DataBuf, &retlen );
#elif DEF_USB_PORT_HS_EN
    ret = USBHSH_CtrlTransfer( RootHubDev[ DEF_USB_PORT_HS ].bEp0MaxPks, DataBuf, &retlen );
#endif
    return ret;
}

void CopySetupReqPkg( const char * pReqPkt )
{
    uint8_t i;
    for(i = 0; i != sizeof(USB_SETUP_REQ); i++)
    {
#if DEF_USB_PORT_FS_EN
        ((char *)pUSBFS_SetupRequest)[i] = *pReqPkt;
#elif DEF_USB_PORT_HS_EN
        ((char *)pUSBHS_SetupRequest)[i] = *pReqPkt;
#endif
        pReqPkt++;
    }
}

uint8_t CtrlGetDeviceDescrTB( void )
{
    uint8_t ret;
#if DEF_USB_PORT_FS_EN
    ret = USBFSH_GetDeviceDescr( &RootHubDev[ DEF_USB_PORT_HS ].bEp0MaxPks, TxBuffer );
#elif DEF_USB_PORT_HS_EN
    ret = USBHSH_GetDeviceDescr( &RootHubDev[ DEF_USB_PORT_HS ].bEp0MaxPks, TxBuffer );
#endif
    return ret;
}

uint8_t CtrlGetConfigDescrTB( void )
{
    uint16_t len;
    uint8_t  ret;
#if DEF_USB_PORT_FS_EN
    ret = USBFSH_GetConfigDescr( RootHubDev[ DEF_USB_PORT_HS ].bEp0MaxPks, TxBuffer, 256, &len );
#elif DEF_USB_PORT_HS_EN
    ret = USBHSH_GetConfigDescr( RootHubDev[ DEF_USB_PORT_HS ].bEp0MaxPks, TxBuffer, 256, &len );
#endif
    return ret;
}

uint8_t CtrlSetUsbConfig( uint8_t cfg )
{
    uint8_t ret;
#if DEF_USB_PORT_FS_EN
    ret = USBFSH_SetUsbConfig( RootHubDev[ DEF_USB_PORT_HS ].bEp0MaxPks, cfg );
#elif DEF_USB_PORT_HS_EN
    ret = USBHSH_SetUsbConfig( RootHubDev[ DEF_USB_PORT_HS ].bEp0MaxPks, cfg );
#endif
    return ret;
}

uint8_t CtrlSetUsbAddress( uint8_t addr )
{
    uint8_t ret;
#if DEF_USB_PORT_FS_EN
    ret = USBFSH_SetUsbAddress( RootHubDev[ DEF_USB_PORT_HS ].bEp0MaxPks, addr );
#elif DEF_USB_PORT_HS_EN
    ret = USBHSH_SetUsbAddress( RootHubDev[ DEF_USB_PORT_HS ].bEp0MaxPks, addr );
#endif
    return ret;
}

uint8_t CtrlClearEndpStall( uint8_t endp )
{
    uint8_t ret;
#if DEF_USB_PORT_FS_EN
    ret = USBFSH_ClearEndpStall( RootHubDev[ DEF_USB_PORT_HS ].bEp0MaxPks, endp );
#elif DEF_USB_PORT_HS_EN
    ret = USBHSH_ClearEndpStall( RootHubDev[ DEF_USB_PORT_HS ].bEp0MaxPks, endp );
#endif
    return ret;
}

#ifndef FOR_ROOT_UDISK_ONLY
uint8_t CtrlGetHubDescr( void )
{

}

uint8_t HubGetPortStatus( uint8_t HubPortIndex )
{

}

uint8_t HubSetPortFeature( uint8_t HubPortIndex, uint8_t FeatureSelt )
{

}

uint8_t HubClearPortFeature( uint8_t HubPortIndex, uint8_t FeatureSelt )
{

}
#endif

CMD_PARAM_I	mCmdParam;						/* ������� */
#if		DISK_BASE_BUF_LEN > 0
//uint8_t	DISK_BASE_BUF[ DISK_BASE_BUF_LEN ] __attribute__((at(BA_RAM+SZ_RAM/2)));	/* �ⲿRAM�Ĵ������ݻ�����,����������Ϊһ�������ĳ��� */
uint8_t	DISK_BASE_BUF[ DISK_BASE_BUF_LEN ] __attribute__((aligned (4)));	        /* �ⲿRAM�Ĵ������ݻ�����,����������Ϊһ�������ĳ��� */
//UINT8	DISK_FAT_BUF[ DISK_BASE_BUF_LEN ] __attribute__((aligned (4)));	            /* �ⲿRAM�Ĵ���FAT���ݻ�����,����������Ϊһ�������ĳ��� */
#endif

/* ���³�����Ը�����Ҫ�޸� */

#ifndef	NO_DEFAULT_ACCESS_SECTOR		/* ��Ӧ�ó����ж���NO_DEFAULT_ACCESS_SECTOR���Խ�ֹĬ�ϵĴ���������д�ӳ���,Ȼ�������б�д�ĳ�������� */
//if ( use_external_interface ) {  // �滻U�������ײ��д�ӳ���
//    CHRV3vSectorSize=512;  // ����ʵ�ʵ�������С,������512�ı���,��ֵ�Ǵ��̵�������С
//    CHRV3vSectorSizeB=9;   // ����ʵ�ʵ�������С��λ����,512���Ӧ9,1024��Ӧ10,2048��Ӧ11
//    CHRV3DiskStatus=DISK_MOUNTED;  // ǿ�ƿ��豸���ӳɹ�(ֻ������ļ�ϵͳ)
//}

uint8_t	CHRV3ReadSector( uint8_t SectCount, uint8_t *DataBuf )  /* �Ӵ��̶�ȡ������������ݵ��������� */
{
    uint8_t	retry;
//	if ( use_external_interface ) return( extReadSector( CHRV3vLbaCurrent, SectCount, DataBuf ) );  /* �ⲿ�ӿ� */
	for( retry = 0; retry < 3; retry ++ ) {  /* �������� */
		pCBW -> mCBW_DataLen = (uint32_t)SectCount << CHRV3vSectorSizeB;  /* ���ݴ��䳤�� */
		pCBW -> mCBW_Flag = 0x80;
		pCBW -> mCBW_LUN = CHRV3vCurrentLun;
		pCBW -> mCBW_CB_Len = 10;
		pCBW -> mCBW_CB_Buf[ 0 ] = SPC_CMD_READ10;
		pCBW -> mCBW_CB_Buf[ 1 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 2 ] = (uint8_t)( CHRV3vLbaCurrent >> 24 );
		pCBW -> mCBW_CB_Buf[ 3 ] = (uint8_t)( CHRV3vLbaCurrent >> 16 );
		pCBW -> mCBW_CB_Buf[ 4 ] = (uint8_t)( CHRV3vLbaCurrent >> 8 );
		pCBW -> mCBW_CB_Buf[ 5 ] = (uint8_t)( CHRV3vLbaCurrent );
		pCBW -> mCBW_CB_Buf[ 6 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 7 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 8 ] = SectCount;
		pCBW -> mCBW_CB_Buf[ 9 ] = 0x00;
		CHRV3BulkOnlyCmd( DataBuf );  /* ִ�л���BulkOnlyЭ������� */
		if ( CHRV3IntStatus == ERR_SUCCESS ) {
			return( ERR_SUCCESS );
		}
		CHRV3IntStatus = CHRV3AnalyzeError( retry );
		if ( CHRV3IntStatus != ERR_SUCCESS ) {
			return( CHRV3IntStatus );
		}
	}
	return( CHRV3IntStatus = ERR_USB_DISK_ERR );  /* ���̲������� */
}

#ifdef	EN_DISK_WRITE
uint8_t	CHRV3WriteSector( uint8_t SectCount, uint8_t *DataBuf )  /* ���������еĶ�����������ݿ�д����� */
{
    uint8_t	retry;
//	if ( use_external_interface ) return( extWriteSector( CHRV3vLbaCurrent, SectCount, DataBuf ) );  /* �ⲿ�ӿ� */
	for( retry = 0; retry < 3; retry ++ ) {  /* �������� */
		pCBW -> mCBW_DataLen = (uint32_t)SectCount << CHRV3vSectorSizeB;  /* ���ݴ��䳤�� */
		pCBW -> mCBW_Flag = 0x00;
		pCBW -> mCBW_LUN = CHRV3vCurrentLun;
		pCBW -> mCBW_CB_Len = 10;
		pCBW -> mCBW_CB_Buf[ 0 ] = SPC_CMD_WRITE10;
		pCBW -> mCBW_CB_Buf[ 1 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 2 ] = (uint8_t)( CHRV3vLbaCurrent >> 24 );
		pCBW -> mCBW_CB_Buf[ 3 ] = (uint8_t)( CHRV3vLbaCurrent >> 16 );
		pCBW -> mCBW_CB_Buf[ 4 ] = (uint8_t)( CHRV3vLbaCurrent >> 8 );
		pCBW -> mCBW_CB_Buf[ 5 ] = (uint8_t)( CHRV3vLbaCurrent );
		pCBW -> mCBW_CB_Buf[ 6 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 7 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 8 ] = SectCount;
		pCBW -> mCBW_CB_Buf[ 9 ] = 0x00;
		CHRV3BulkOnlyCmd( DataBuf );  /* ִ�л���BulkOnlyЭ������� */
		if ( CHRV3IntStatus == ERR_SUCCESS ) {
			Delay_Us( 200 );  /* д��������ʱ */
			return( ERR_SUCCESS );
		}
		CHRV3IntStatus = CHRV3AnalyzeError( retry );
		if ( CHRV3IntStatus != ERR_SUCCESS ) {
			return( CHRV3IntStatus );
		}
	}
	return( CHRV3IntStatus = ERR_USB_DISK_ERR );  /* ���̲������� */
}
#endif
#endif  // NO_DEFAULT_ACCESS_SECTOR

#ifndef	NO_DEFAULT_DISK_CONNECT			/* ��Ӧ�ó����ж���NO_DEFAULT_DISK_CONNECT���Խ�ֹĬ�ϵļ����������ӳ���,Ȼ�������б�д�ĳ�������� */

/*
Լ��: USB�豸��ַ�������(�ο�USB_DEVICE_ADDR)
��ֵַ  �豸λ��
0x02    ����Root-HUB0�µ�USB�豸���ⲿHUB
0x03    ����Root-HUB1�µ�USB�豸���ⲿHUB
0x1x    ����Root-HUB0�µ��ⲿHUB�Ķ˿�x�µ�USB�豸,xΪ1~n
0x2x    ����Root-HUB1�µ��ⲿHUB�Ķ˿�x�µ�USB�豸,xΪ1~n
*/

#define		UHUB_DEV_ADDR	(USBHSH->DEV_AD)
#define		UHUB_MIS_STAT	(USBHSH->MIS_ST)
#define		UHUB_HOST_CTRL	(USBHSH->HOST_CTRL)
#define		UHUB_INT_FLAG	(USBHSH->INT_FG)
#define		bUMS_ATTACH		USBHS_UMS_DEV_ATTACH
#define		bUMS_SUSPEND	USBHS_UMS_SUSPEND

/* �������Ƿ����� */
uint8_t	CHRV3DiskConnect( void )
{
    uint8_t	ums, devaddr;
	UHUB_DEV_ADDR = UHUB_DEV_ADDR & 0x7F;
	ums = UHUB_MIS_STAT;
	devaddr = UHUB_DEV_ADDR;
	if ( devaddr == USB_DEVICE_ADDR+1 )
	{
	    /* ����Root-HUB�µ�USB�豸 */
		if ( ums & bUMS_ATTACH )
		{
		    /* ����Root-HUB�µ�USB�豸���� */
			if ( ( ums & bUMS_SUSPEND ) == 0 )
			{
			    /* ����Root-HUB�µ�USB�豸������δ��� */
				return( ERR_SUCCESS );  /* USB�豸�Ѿ�������δ��� */
			}
			else
			{
			    /* ����Root-HUB�µ�USB�豸���� */
				CHRV3DiskStatus = DISK_CONNECT;  /* �����Ͽ��� */
				return( ERR_SUCCESS );  /* �ⲿHUB��USB�豸�Ѿ����ӻ��߶Ͽ����������� */
			}
		}
		else
		{
		    /* USB�豸�Ͽ� */
mDiskDisconn:
			CHRV3DiskStatus = DISK_DISCONNECT;
			return( ERR_USB_DISCON );
		}
	}
	else
	{
		goto mDiskDisconn;
	}
}
#endif  // NO_DEFAULT_DISK_CONNECT

#ifndef	NO_DEFAULT_FILE_ENUMER			/* ��Ӧ�ó����ж���NO_DEFAULT_FILE_ENUMER���Խ�ֹĬ�ϵ��ļ���ö�ٻص�����,Ȼ�������б�д�ĳ�������� */
void xFileNameEnumer( void )			/* �ļ���ö�ٻص��ӳ��� */
{
/* ���ָ��ö�����CHRV3vFileSizeΪ0xFFFFFFFF�����FileOpen����ôÿ������һ���ļ�FileOpen������ñ��ص�����
   �ص�����xFileNameEnumer���غ�FileOpen�ݼ�CHRV3vFileSize������ö��ֱ�����������ļ�����Ŀ¼�����������ǣ�
   �ڵ���FileOpen֮ǰ����һ��ȫ�ֱ���Ϊ0����FileOpen�ص�������󣬱�������CHRV3vFdtOffset�õ��ṹFAT_DIR_INFO��
   �����ṹ�е�DIR_Attr�Լ�DIR_Name�ж��Ƿ�Ϊ�����ļ�������Ŀ¼������¼�����Ϣ������ȫ�ֱ�������������
   ��FileOpen���غ��жϷ���ֵ�����ERR_MISS_FILE��ERR_FOUND_NAME����Ϊ�����ɹ���ȫ�ֱ���Ϊ����������Ч�ļ�����
   ����ڱ��ص�����xFileNameEnumer�н�CHRV3vFileSize��Ϊ1����ô����֪ͨFileOpen��ǰ���������������ǻص��������� */
#if		0
    uint8_t			i;
    uint16_t	    FileCount;
	PX_FAT_DIR_INFO	pFileDir;
	uint8_t			*NameBuf;
	pFileDir = (PX_FAT_DIR_INFO)( pDISK_BASE_BUF + CHRV3vFdtOffset );  /* ��ǰFDT����ʼ��ַ */
	FileCount = (UINT16)( 0xFFFFFFFF - CHRV3vFileSize );  /* ��ǰ�ļ�����ö�����,CHRV3vFileSize��ֵ��0xFFFFFFFF,�ҵ��ļ�����ݼ� */
	if ( FileCount < sizeof( FILE_DATA_BUF ) / 12 ) {  /* ��黺�����Ƿ��㹻���,�ٶ�ÿ���ļ�����ռ��12���ֽڴ�� */
		NameBuf = & FILE_DATA_BUF[ FileCount * 12 ];  /* ���㱣�浱ǰ�ļ����Ļ�������ַ */
		for ( i = 0; i < 11; i ++ ) NameBuf[ i ] = pFileDir -> DIR_Name[ i ];  /* �����ļ���,����Ϊ11���ַ�,δ�����ո� */
//		if ( pFileDir -> DIR_Attr & ATTR_DIRECTORY ) NameBuf[ i ] = 1;  /* �ж���Ŀ¼�� */
		NameBuf[ i ] = 0;  /* �ļ��������� */
	}
#endif
}
#endif  // NO_DEFAULT_FILE_ENUMER

uint8_t	CHRV3LibInit( void )  /* ��ʼ��CHRV3�����,�����ɹ�����0 */
{
    uint8_t s;
    s = CHRV3GetVer( );
	if( s < CHRV3_LIB_VER )
	{
        return( 0xFF );  /* ��ȡ��ǰ�ӳ����İ汾��,�汾̫���򷵻ش��� */
	}
	printf( "lib vision:%02x\r\n",s );
#if		DISK_BASE_BUF_LEN > 0
	pDISK_BASE_BUF = & DISK_BASE_BUF[0]; /* ָ���ⲿRAM�Ĵ������ݻ����� */
	pDISK_FAT_BUF = & DISK_BASE_BUF[0];  /* ָ���ⲿRAM�Ĵ���FAT���ݻ�����,������pDISK_BASE_BUF�����Խ�ԼRAM */
//	pDISK_FAT_BUF = & DISK_FAT_BUF[0];   /* ָ���ⲿRAM�Ĵ���FAT���ݻ�����,������pDISK_BASE_BUF������ٶ� */
/* ���ϣ������ļ���ȡ�ٶ�,��ô�������������е���CHRV3LibInit֮��,��pDISK_FAT_BUF����ָ����һ�������������pDISK_BASE_BUFͬ����С�Ļ����� */
#endif
	CHRV3DiskStatus = DISK_UNKNOWN;  /* δ֪״̬ */
	CHRV3vSectorSizeB = 9;  /* Ĭ�ϵ��������̵�������512B */
	CHRV3vSectorSize = 512; /* Ĭ�ϵ��������̵�������512B,��ֵ�Ǵ��̵�������С */
	CHRV3vStartLba = 0;     /* Ĭ��Ϊ�Զ�����FDD��HDD */
	CHRV3vPacketSize = 512;  /* USB�洢���豸����������:64@FS,512@HS/SS,��Ӧ�ó����ʼ��,ö��U�̺�����Ǹ��ٻ��߳�����ô��ʱ����Ϊ512 */

    pTX_DMA_A_REG = (uint32_t *)&(USBHSH->HOST_TX_DMA);  /* ָ����DMA��ַ�Ĵ���,��Ӧ�ó����ʼ�� */
    pRX_DMA_A_REG = (uint32_t *)&(USBHSH->HOST_RX_DMA);  /* ָ�����DMA��ַ�Ĵ���,��Ӧ�ó����ʼ�� */
    pTX_LEN_REG = (uint16_t *)&(USBHSH->HOST_TX_LEN);    /* ָ���ͳ��ȼĴ���,��Ӧ�ó����ʼ�� */
    pRX_LEN_REG = (uint16_t *)&(USBHSH->RX_LEN);         /* ָ����ճ��ȼĴ���,��Ӧ�ó����ʼ�� */

	return( ERR_SUCCESS );
}

void mDelaymS( uint16_t n )
{
	Delay_Ms(n);
}
