#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

// Target platform as C library
typedef unsigned char            bit_t;
typedef unsigned char            u1_t;
typedef int8_t             s1_t;
typedef uint16_t           u2_t;
typedef int16_t            s2_t;
typedef uint32_t           u4_t;
typedef int32_t            s4_t;
typedef unsigned int       uint;
typedef const char*        str_t;

char fopts_content[20] = "test";
int temp = 12345;

#define UPLINK 1
#define DOWNLINK 0

#define TABLE_GET_U1(table, index) table_get_u1(RESOLVE_TABLE(table), index)
// Helper to add a prefix to the table name
#define RESOLVE_TABLE(table) constant_table_ ## table

// get number of entries in table
#define LENOF_TABLE(table) (sizeof(RESOLVE_TABLE(table)) / sizeof(RESOLVE_TABLE(table)[0]))

static inline u1_t table_get_u1(const u1_t *table, size_t index) { return table[index]; }

#define CONST_TABLE(type, name) const type RESOLVE_TABLE(name)

static CONST_TABLE(u1_t, macCmdSize_downlink)[] = {
    /* 2: LinkCheckAns */ 3,
    /* 3: LinkADRReq */ 5,
    /* 4: DutyCycleReq */ 2,
    /* 5: RXParamSetupReq */ 5,
    /* 6: DevStatusReq */ 1,
    /* 7: NewChannelReq */ 6,
    /* 8: RXTimingSetupReq */ 2,
    /* 9: TxParamSetupReq */ 2,
    /* 0x0A: DlChannelReq */ 5,
    /* B, C: RFU */ 0, 0,
    /* 0x0D: DeviceTimeAns */ 6,
    /* 0x0E, 0x0F */ 0, 0,
    /* 0x10: PingSlotInfoAns */ 1,
    /* 0x11: PingSlotChannelReq */ 5,
    /* 0x12: BeaconTimingAns */ 4,
    /* 0x13: BeaconFreqReq */ 4
};

static CONST_TABLE(u1_t, macCmdSize_uplink)[] = {
    /* 2: LinkCheckReq */ 1,
    /* 3: LinkADRAns */ 2,
    /* 4: DutyCycleAns */ 1,
    /* 5: RXParamSetupAns */ 2,
    /* 6: DevStatusAns */ 3,
    /* 7: NewChannelAns */ 2,
    /* 8: RXTimingSetupAns */ 1,
    /* 9: TxParamSetupAns */ 1,
    /* 0x0A: DlChannelAns */ 2,
    /* B, C: RFU */ 0, 0,
    /* 0x0D: DeviceTimeReq */ 1,
    /* 0x0E, 0x0F */ 0, 0,
    /* 0x10: PingSlotInfoReq */ 2,
    /* 0x11: PingSlotChannelAns */ 2,
    /* 0x12: BeaconTimingReq */ 1, //-----------------> check this
    /* 0x13: BeaconFreqAns */ 2
};

char datarate_array[6][40] = {"LoRa: SF12 / 125 kHz","LoRa: SF11 / 125 kHz","LoRa: SF10 / 125 kHz","LoRa: SF9 / 125 kHz","LoRa: SF8 / 125 kHz", "LoRa: SF7 / 125 kHz"};

char power_array[11][20] = {"Max EIRP","Max EIRP – 2dB","Max EIRP – 4dB","Max EIRP – 6dB","Max EIRP – 8dB","Max EIRP – 10dB","Max EIRP – 12dB","Max EIRP – 14dB","Max EIRP – 16dB","Max EIRP – 18dB","Max EIRP – 20dB"};

static u1_t getMacCmdSize(u1_t macCmd,char link_type) {
	if(link_type == 0)
	{
		if (macCmd >= 2) {
			const unsigned macCmdMinus2 = macCmd - 2u;
			if (macCmdMinus2 < LENOF_TABLE(macCmdSize_downlink)) {
				// macCmd in table, fetch it's size.
				return TABLE_GET_U1(macCmdSize_downlink, macCmdMinus2);
			}
		}		
	}
	else if (link_type == 1)
	{
		if (macCmd >= 2) {
			const unsigned macCmdMinus2 = macCmd - 2u;
			if (macCmdMinus2 < LENOF_TABLE(macCmdSize_uplink)) {
				// macCmd in table, fetch it's size.
				return TABLE_GET_U1(macCmdSize_uplink, macCmdMinus2);
			}
		}
	}
    // macCmd too small or too large: return zero. Zero is
    // never a legal command size, so it signals an error
    // to the caller.
    return 0;
}


enum {
    // Class A
    MCMD_LinkCheckAns = 0x02,       // u1:margin 0-254,255=unknown margin / u1:gwcnt         LinkCheckReq
    MCMD_LinkADRReq = 0x03,         // u1:DR/TXPow, u2:chmask, u1:chpage/repeat
    MCMD_DutyCycleReq = 0x04,       // u1:255 dead [7-4]:RFU, [3-0]:cap 2^-k
    MCMD_RXParamSetupReq = 0x05,    // u1:7-4:RFU/3-0:datarate, u3:freq
    MCMD_DevStatusReq = 0x06,       // -
    MCMD_NewChannelReq = 0x07,      // u1:chidx, u3:freq, u1:DRrange
    MCMD_RXTimingSetupReq = 0x08,   // u1: [7-4]:RFU [3-0]: Delay 1-15s (0 => 1)
    MCMD_TxParamSetupReq = 0x09,    // u1: [7-6]:RFU [5:4]: dl dwell/ul dwell [3:0] max EIRP
    MCMD_DlChannelReq = 0x0A,       // u1: channel, u3: frequency
    MCMD_DeviceTimeAns = 0x0D,      // u4: seconds since epoch, u1: fractional second

    // Class B
    MCMD_PingSlotInfoAns = 0x10,    // -
    MCMD_PingSlotChannelReq = 0x11, // u3: freq, u1:dr [7-4]:RFU [3:0]:datarate
    MCMD_BeaconTimingAns = 0x12,    // u2: delay(in TUNIT millis), u1:channel (DEPRECATED)
    MCMD_BeaconFreqReq = 0x13,      // u3: freq
};

int parseoptsfield(char *fopts_content_local,char link_type);

int is_mac_size_right(char mac_size, char* fopts_content_local)
{
	if(strlen(fopts_content_local)>=(mac_size*2)) 
	{
		//printf("The size of ADR Req is: %d \r\n",mac_size);
		return 0 ;	
	}
	else
	{
		printf("The data entered is not complete\r\n");
		return 1;
	}	
}

int main()

{
	int fopts_length = 0;
	int link_type = 0;
	while(1)
	{	
		while(1)
		{
			printf("\r\n\r\nEnter 0 Downlink, 1 for Uplink, 2 to exit \r\n");
			scanf("%s",fopts_content);
			if(strlen(fopts_content)==1)
			{
				link_type = atoi(fopts_content);
				if(link_type>1)
				{
					if(link_type==2)
					{
						return 0;						
					}
					printf("Wrong link type entered, Enter 0 for downlink and 1 for uplink\r\n");
					continue;
				}
				else
					break;
			}			
		}
		printf("Please enter the fopts field\r\n");
		scanf("%s",fopts_content);
		fopts_length = strlen(fopts_content);
		if(fopts_length%2!=0)
		{
			printf("The length is not right, possibly not 8 bit mulitiples!\r\n");
			continue;
		}
		printf("\r\n\r\n");
		//sprintf(fopts_content, "%02x\r\n",temp);
//		if(!strnicmp(fopts_content,"02", 2))
//		{
//			printf("fopts is: LinkCheckReq\r\n");
//		}
		int i = 0;
		int parse_result = 0;
		while(1)
		{
			if(i<fopts_length)
			{
				parse_result = parseoptsfield(fopts_content+i,link_type);
				printf("\r\n\r\n");
				if (parse_result==0)
				{				
					break;
				}
				i += (parse_result*2);
			}
			else
			{
				printf("Parsed all Mac Commands\r\n");
				break;				
			}
		}		
	}
	return 0;
}

int decode_MCMD_DevStatusReq(char *fopts_content_local, char link_type)
{
	int status = 0;
	int data_pwr = 0;
	char fopts_command[5] = "00";
	if(link_type == DOWNLINK)
	{
		strncpy(fopts_command, fopts_content_local+2,2);
		fopts_command[2] = 0x00;
		sscanf(fopts_command, "%x", &data_pwr);
	}
	else
	{
		
	}
}

int decode_MCMD_LinkADRReq(char *fopts_content_local, char link_type)
{
	long status = 0;
	int data_pwr = 0;
	char fopts_command[5] = "00";
	if(link_type == DOWNLINK)
	{
		strncpy(fopts_command, fopts_content_local+2,2);
		fopts_command[2] = 0x00;
		sscanf(fopts_command, "%x", &data_pwr);
		if((data_pwr&0x0F) < 6 && (data_pwr&0x0F) >= 0)
		{
			printf("Data rate: %s\r\n", datarate_array[data_pwr&0x0F]); 
		}
		else
		{
			printf("Data rate is invalid \r\n");
		}
		if((data_pwr&0xF0) < 0xB0 && (data_pwr&0xF0) >= 0x00)
		{
			printf("Power: %s\r\n", power_array[(data_pwr&0xF0)>>4]); 
		}
		else
		{
			printf("Power is invalid \r\n");
		}
		char *temp_string;
		strncpy(fopts_command, fopts_content_local+4,4);
		fopts_command[4] = 0x00;
		sscanf(fopts_command, "%x", &status);
		printf("Usable channels with LSB as Channel 0: %04x\r\n",status);
		strncpy(fopts_command, fopts_content_local+8,2);
		fopts_command[2] = 0x00;
		sscanf(fopts_command, "%x", &status);
		printf("The number of transmission for each uplink message %d\r\n", (status&0x0F));
		printf("The channel mask will be enabled as %x\r\n", (status&0x70)>>4);
		
	}
	else
	{
		strncpy(fopts_command, fopts_content_local+2,2);
		fopts_command[2] = 0x00;
		sscanf(fopts_command, "%x", &status);
		if(status&0x04)
			printf("Power Accepted by the end node\r\n");
		else
			printf("Power NOT Accepted by the end node\r\n");
		if(status&0x02)
			printf("Data rate accepted by node\r\n");
		else
			printf("Data rate NOT accepted by node\r\n");
		if(status&0x01)
			printf("Channel mask interpreted and all channel states set\r\n");
		else
			printf("Channel mask command discarded and state not changed\r\n");
	//	Power ACK Data rate ACK Channel mask
//ACK
	}
	return 0;
}

int parseoptsfield(char *fopts_content_local, char link_type)
{
	char fopts_command[5] = "00";
	int cmd,MacCmdSize,status;
	strncpy(fopts_command, fopts_content_local,2);
	fopts_command[2] = 0x00;
	sscanf(fopts_command, "%x", &cmd);
	MacCmdSize = getMacCmdSize(cmd,link_type);
	if(is_mac_size_right(MacCmdSize, fopts_content_local))
		return 0;
	switch( cmd ) {
		case MCMD_LinkCheckAns: {
			printf("MCMD_LinkCheckAns\r\n");
			printf("The size of ADR Req is: %d \r\n",MacCmdSize);
			break;
		}

		case MCMD_LinkADRReq: {
			printf("MCMD_LinkADRReq\r\n");
			decode_MCMD_LinkADRReq(fopts_content_local,link_type);
			break;
		}

		case MCMD_DevStatusReq: {
			printf("MCMD_DevStatusReq\r\n");
			if(link_type==DOWNLINK)
			{
				printf("Server is requesting status of the end device\r\n");
			}
			else
			{
				strncpy(fopts_command, fopts_content_local+2,2);
				fopts_command[2] = 0x00;
				sscanf(fopts_command, "%x", &status);
				int battery = (status&0xFF);
				if (battery == 0)
					printf("The device connected to external supply\r\n");
				else if (battery == 255)
					printf("The device was not able to measure the battery level\r\n");
				else
					printf("The battery level is(1 minimum level): %d\r\n",battery);
				strncpy(fopts_command, fopts_content_local+4,2);
				fopts_command[2] = 0x00;
				sscanf(fopts_command, "%x", &status);
				if(status&0x20)
					printf("The margin (SNR in dB): %d\r\n", (status&0x1F) - 32);
				else
					printf("The margin (SNR in dB): %d\r\n", (status&0x1F));
			}
			break;
		}

		case MCMD_RXParamSetupReq: {
			printf("MCMD_RXParamSetupReq\r\n");
			printf("The size of ADR Req is: %d \r\n",MacCmdSize);
			break;
		}

	   case MCMD_RXTimingSetupReq: {
			printf("MCMD_RXTimingSetupReq\r\n");
			break;
		}

		case MCMD_DutyCycleReq: {
			printf("MCMD_DutyCycleReq\r\n");
			break;
		}

		case MCMD_NewChannelReq: {
			printf("MCMD_NewChannelReq\r\n");
			break;
		}

	   case MCMD_DlChannelReq: {
			printf("MCMD_DlChannelReq\r\n");
			break;
		}

		case MCMD_PingSlotChannelReq: {
			printf("MCMD_PingSlotChannelReq\r\n");
			break;
		}

		case MCMD_BeaconTimingAns: {
			printf("MCMD_BeaconTimingAns\r\n");
			break;
		} /* end case */

		case MCMD_TxParamSetupReq: {
			printf("MCMD_TxParamSetupReq\r\n");
			break;
		} /* end case */

		case MCMD_DeviceTimeAns: {
			printf("MCMD_DeviceTimeAns\r\n");
			break;
		} /* end case */

		default: {
			printf("Didn't find any listed mac commands\r\n");
			break;
		} /* end case */
	} /* end switch */

	return MacCmdSize;
}





