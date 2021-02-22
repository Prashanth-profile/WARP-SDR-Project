#include "warpmac.h"
#include "warpphy.h"
#include "xparameters.h"

#include "wired_frame_formats.h"
#include "EMAC_Packing.h"

//14-byte packet header for ethernet 
ethernet_header txEthPktHeader;

//20-byte packet header for IP
ipv4_header txIPPktHeader;
	
//the MAC addresses of the 6 PCs in iNETS Lab
unsigned char EMAC_Addr[6][6];

unsigned short int EMAC_dest_mac_index;
unsigned short int EMAC_src_mac_index;

//Initialization of EMAC packing parameters
//This function should be exexuted in the main function before polling operations
void EMAC_init()
{
	unsigned short int i;
	
	EMAC_Addr[0][0]=0x00;
	EMAC_Addr[0][1]=0x1B;
	EMAC_Addr[0][2]=0xFC;
	EMAC_Addr[0][3]=0xE6;
	EMAC_Addr[0][4]=0x76;
	EMAC_Addr[0][5]=0xE2;

	EMAC_Addr[1][0]=0x00;
	EMAC_Addr[1][1]=0x1D;
	EMAC_Addr[1][2]=0x60;
	EMAC_Addr[1][3]=0x44;
	EMAC_Addr[1][4]=0x02;
	EMAC_Addr[1][5]=0x35;
	
	EMAC_Addr[2][0]=0x00;
	EMAC_Addr[2][1]=0x1D;
	EMAC_Addr[2][2]=0x60;
	EMAC_Addr[2][3]=0x60;
	EMAC_Addr[2][4]=0x6D;
	EMAC_Addr[2][5]=0x80;
	
	EMAC_Addr[3][0]=0x00;
	EMAC_Addr[3][1]=0x1D;
	EMAC_Addr[3][2]=0x60;
	EMAC_Addr[3][3]=0x60;
	EMAC_Addr[3][4]=0x6E;
	EMAC_Addr[3][5]=0x33;

	EMAC_Addr[4][0]=0x00;
	EMAC_Addr[4][1]=0x1D;
	EMAC_Addr[4][2]=0x60;
	EMAC_Addr[4][3]=0x60;
	EMAC_Addr[4][4]=0x6B;
	EMAC_Addr[4][5]=0x7B;
	
	EMAC_Addr[5][0]=0x00;
	EMAC_Addr[5][1]=0x1D;
	EMAC_Addr[5][2]=0x60;
	EMAC_Addr[5][3]=0x33;
	EMAC_Addr[5][4]=0x1B;
	EMAC_Addr[5][5]=0x1C;

	//default mac indexes for dest and source are 0 and 1
	EMAC_dest_mac_index = 1;
	EMAC_src_mac_index = 0;
	
	//Initialization of txEthPktHeader
	//default ethertype is IP
	txEthPktHeader.ethertype = ETHERTYPE_IP;
	
	//initialize the mac address
	for(i=0;i<6;i++)
		txEthPktHeader.dest_addr_mac[i]=EMAC_Addr[EMAC_dest_mac_index][i];

	for(i=0;i<6;i++)
		txEthPktHeader.src_addr_mac[i]=0;//EMAC_Addr[EMAC_src_mac_index][i];
		
	//Initialization of txIPPktHeader, no need to change this part 
	txIPPktHeader.version=0x45;
	txIPPktHeader.typeOfService=0x00;
	txIPPktHeader.frag_offset=0x00;
	txIPPktHeader.ttl=0x80;
	txIPPktHeader.protocol=0xFF;
	txIPPktHeader.identification= 0x0000; 
}

// Change the destinatin mac address
unsigned short int EMAC_change_dest_mac()
{
	unsigned short int i;
	
	EMAC_dest_mac_index = (++EMAC_dest_mac_index)%6;
	
	for(i=0;i<6;i++)
		txEthPktHeader.dest_addr_mac[i]=EMAC_Addr[EMAC_dest_mac_index][i];
	
	xil_printf("Current Destination MAC:  ");
	
	for(i=0;i<5;i++)
		xil_printf("%02X-",txEthPktHeader.dest_addr_mac[i]);
	xil_printf("%02X\r\n",txEthPktHeader.dest_addr_mac[i]);
	
	return EMAC_dest_mac_index;
}

//Checksumming function for IP packet header
unsigned short int cksum(ipv4_header* ipheader, unsigned size)
{
	unsigned long int sum = 0;
	int i;
	unsigned short int *buf = ipheader;
	unsigned short int ck;
	
	for (i = 0; i<size/2; i++)
		sum += buf[i];
	
	ck = (sum & 0xFFFF)+(sum >> 16);

	return ~ck;
}	

//Preparation of the EMAC header and IP header
//This function should be executed after copying the Macframe header to the Tx buffer and before radio transmission
//@param pktBuf_tx_DATA the Tx buffer index
//@param txMacframe is the address of the Macframe header, which should be already configured 
void EMAC_prepEMACPktForXmit(unsigned char pktBuf_tx_DATA, Macframe* txMacframe)
{	
   void* txPktPtr = (void *)warpphy_getBuffAddr(pktBuf_tx_DATA)+NUM_HEADER_BYTES; 
   	
	//copy the Ethernet header after the Macframe header
	memcpy(txPktPtr, (void *)&(txEthPktHeader), sizeof(ethernet_header));
	
	//the seqNum is copied to the source IP for packet loss checking in PC side
	txIPPktHeader.src_addr_ip = txMacframe->header.seqNum;		//the original ip address of the source: 167772160+destNode;
	txIPPktHeader.dest_addr_ip = 0xA000000 + txMacframe->header.destAddr;//the ip address of the destination
	txIPPktHeader.length=txMacframe->header.length-sizeof(ethernet_header)-4; //minus 4 bytes for the Ethernet CRC tail
	
	//The checksum bypes should be initialized as 0 for checksumming.
	txIPPktHeader.checksum=0;
	txIPPktHeader.checksum=cksum(&txIPPktHeader,20);
	
	//copy the IP packet header after the Ethernet header
	memcpy(txPktPtr+sizeof(ethernet_header), (void *)&(txIPPktHeader), sizeof(txIPPktHeader));
}
