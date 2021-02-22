/*! \file halfmac_server.c
 \brief Server for halfmac workshop exercise
 
 @version 16.1
 @author Chris Hunter
 
 This code is a simple manipulation of csmaMac.c that allows for addressing
 multiple WARP nodes based upon the IP address of the packet being sent.
 
 */

// ------------------------------------------------------------------------------------------------------------------------------

#include "warpmac.h"
#include "warpphy.h"
#include "receiversource.h"
#include "xparameters.h"
#include "ascii_characters.h"
#include "ofdm_txrx_mimo_regMacros.h"
#include "ofdm_agc_mimo_regMacros.h"
#include "wired_frame_formats.h"
#include "EMAC_Packing.h"



unsigned char pktBuf_rx_upload[25];
unsigned char pktBuf_tx_ACK;
unsigned char pktBuf_rx;

unsigned char maximumReSend;

///ID of this node
unsigned short int myID, RxID=4;

///Full rate modulation selection; QPSK by default
unsigned int pktFullRate;

//Payload code rate selection
unsigned int pktCodeRate;

///Buffer for holding a packet-to-xmit across multiple retransmissions
///Buffer to hold received packet
Macframe rxMacframe;
///Buffer to hold ACK
Macframe txACK;


///Current 802.11 channel
unsigned char chan;



unsigned int lastRxSeqNum, continuous_rx=1;
unsigned int pk=0,c=0;
unsigned int leng[25], buff_full[25];

unsigned int upload_period=80;

unsigned short pktDet_AC_corr, pktDet_AC_energy, pktDet_RSSI_thresh;


////unsigned short int should_send=0;


//Define handy macros for CSMA MAC packet types
///Data packet with payload meant for Ethernet transmission
#define PKTTYPE_DATA	1
#define PKTTYPE_ACK	0

// ------------------------------------------------------------------------------------------------------------------------------
void middleButton() {
	upload_period+=1000;
	warpmac_setTimeout(upload_period);
	xil_printf("Upload period incremented to %d \r\n", upload_period);
	return;
}

void rightButton() {
	upload_period-=1000;
	warpmac_setTimeout(upload_period);
	xil_printf("Upload period incremented to %d \r\n", upload_period);
	return;
}

void topButton() {
	upload_period+=100;
	warpmac_setTimeout(upload_period);
	xil_printf("Upload period incremented to %d \r\n", upload_period);
	return;
}

// Callback Function for Left Push Button:
// =======================================

///@brief Callback for the depression of the left push button
///
///This function is empty by default
void leftButton() {

switch (continuous_rx) {
	case 0:
		continuous_rx=1;
		warpmac_setTimer(TIMEOUT_TIMER);
		break;

	case 1:
		continuous_rx=0;
		warpmac_clearTimer(TIMEOUT_TIMER);
		break;
	}	
	return;
}

// ------------------------------------------------------------------------------------------------------------------------------

// Callback Function for reception of UART Bytes:
// ==============================================

///@brief Callback for the reception of UART bytes
///@param uartByte ASCII byte received from UART
///
///Provides the user with the bytes that was received over the serial port. This is useful for configuring
///PHY and MAC parameters in real time on a board.

void uartRecv_callback(unsigned char uartByte)
{
	if(uartByte != 0x0)
	{
		xil_printf("(%c)\t", uartByte);

		switch(uartByte)
		{
			case ASCII_1:
				pktFullRate = HDR_FULLRATE_BPSK;
				xil_printf("Tx Full Rate = BPSK\r\n");
				break;

			case ASCII_2:
				pktFullRate = HDR_FULLRATE_QPSK;
				xil_printf("Tx Full Rate = QPSK\r\n");
				break;

			case ASCII_4:
				pktFullRate = HDR_FULLRATE_QAM_16;
				xil_printf("Tx Full Rate = 16-QAM\r\n");
				break;

			case ASCII_6:
				pktFullRate = HDR_FULLRATE_QAM_64;
				xil_printf("Tx Full Rate = 64-QAM\r\n");
				break;

			case ASCII_7:
				pktCodeRate = HDR_CODE_RATE_12;
				xil_printf("Coding Rate = 1/2\r\n");
				break;
			case ASCII_8:
				pktCodeRate = HDR_CODE_RATE_23;
				xil_printf("Coding Rate = 2/3\r\n");
				break;
			case ASCII_9:
				pktCodeRate = HDR_CODE_RATE_34;
				xil_printf("Coding Rate = 3/4\r\n");
				break;
			case ASCII_0:
				pktCodeRate = HDR_CODE_RATE_NONE;
				xil_printf("Coding Rate = 1 (no coding)\r\n");
				break;
			case ASCII_F:
				if(chan<14) chan++;
				warpphy_setChannel(GHZ_2, chan);
				xil_printf("Current channel: %d\r\n",chan);
				break;
			case ASCII_f:
				if(chan>1) chan--;
				warpphy_setChannel(GHZ_2, chan);
				xil_printf("Current channel: %d\r\n",chan);
				break;

			case ASCII_C:
				pktDet_AC_corr = (pktDet_AC_corr < 250) ? (pktDet_AC_corr+5) : 255;
				xil_printf("AutoCorr_corr: %d\tAutoCorr_energy: %d\r\n", pktDet_AC_corr, pktDet_AC_energy);
				warpphy_setAutoCorrDetParams(pktDet_AC_corr, pktDet_AC_energy);
				break;
			case ASCII_c:
				pktDet_AC_corr = (pktDet_AC_corr > 4) ? (pktDet_AC_corr-5) : 0;
				xil_printf("AutoCorr_corr: %d\tAutoCorr_energy: %d\r\n", pktDet_AC_corr, pktDet_AC_energy);
				warpphy_setAutoCorrDetParams(pktDet_AC_corr, pktDet_AC_energy);
				break;

			case ASCII_E:
				pktDet_AC_energy = (pktDet_AC_energy<2047) ? (pktDet_AC_energy+1) : 2047;
				xil_printf("AutoCorr_corr: %d\tAutoCorr_energy: %d\r\n", pktDet_AC_corr, pktDet_AC_energy);
				warpphy_setAutoCorrDetParams(pktDet_AC_corr, pktDet_AC_energy);
				break;
			case ASCII_e:
				pktDet_AC_energy = (pktDet_AC_energy>0) ? (pktDet_AC_energy-1) : 0;
				xil_printf("AutoCorr_corr: %d\tAutoCorr_energy: %d\r\n", pktDet_AC_corr, pktDet_AC_energy);
				warpphy_setAutoCorrDetParams(pktDet_AC_corr, pktDet_AC_energy);
				break;
				
			case ASCII_R:
				pktDet_RSSI_thresh += 100;
				xil_printf("RSSI_thresh: %d\r\n", pktDet_RSSI_thresh);
				warpphy_setEnergyDetThresh(pktDet_RSSI_thresh);
				break;

			case ASCII_r:
				pktDet_RSSI_thresh -= 100;
				xil_printf("RSSI_thresh: %d\r\n", pktDet_RSSI_thresh);
				warpphy_setEnergyDetThresh(pktDet_RSSI_thresh);
				break;
		
			default:
				xil_printf("Undefined command\r\n");
				break;
		}
	}

	return;
}


/*
unsigned int function_to_find_seqnum() {
	unsigned int p, index=0;
	int smallest = seq_number_array[0];
	for (p = 0; p < 26; p++) {
		if (smallest >= seq_number_array[p]) {
			smallest = seq_number_array[p];
			index = p;
		}
	}
	return index;
}
*/
// ------------------------------------------------------------------------------------------------------------------------------

// Callback Function for Expiration of Timers:
// ===========================================

///@brief Callback for the expiration of timers
///
///This function is responsible for handling #TIMEOUT and #BACKOFF.
///The job responsibilities of this function are to:
///-increase the contention window upon the expiration of a #TIMEOUT
///-initiate a #BACKOFF timer upon the expiration of a #TIMEOUT
///-retransmit a packet upon the expiration of a #BACKOFF
///@param timerType #TIMEOUT or #BACKOFF

void timer_callback(unsigned char timerType) {
	
	switch(timerType) {
		case TIMEOUT_TIMER:
					uploadFromBuffer();
			
					warpmac_setTimer(TIMEOUT_TIMER);
						////xil_printf("Timer called \r\n");
		
			break;	
		case BACKOFF_TIMER:	
			break; //END BACKOFF_TIMER
			
	}
	return;
}


void uploadFromBuffer(){

	

		//if (seq_number_array[pk]==1)
			//one_cnt=1;
	
		if (buff_full[pk]!=0){
			warpmac_prepPktToNetwork((void *)warpphy_getBuffAddr(pktBuf_rx_upload[pk]), leng[pk]);
			warpmac_startPktToNetwork(leng[pk]); 
			buff_full[pk]=0;
			leng[pk]=0;


			pk=(pk+1)%25;
			warpmac_clearTimer(TIMEOUT_TIMER);
			

	}
	else{
	////xil_printf("No new packets to upload to network");
	}
	//}
	/*else{
			buff_full[pk]=0;
			leng[pk]=0;
			seq_number_array[pk]=0;
			pk=(pk+1)%26;
	}*/

return;
}



// ------------------------------------------------------------------------------------------------------------------------------

// Callback Function for reception of Ethernet Packets from MAC Layer:
// ===================================================================

///@brief Callback for the reception of Ethernet packets
///
///This function is called by the ethernet MAC drivers
///when a packet is available to send. This function fills
///the Macframe transmit buffer with the packet and sends
///it over the OFDM link
///@param length Length, in bytes, of received Ethernet frame


void dataFromNetworkLayer_callback(Xuint32 length, char* payload){

	
	return;
}

// ------------------------------------------------------------------------------------------------------------------------------

// Callback Function for reception of packets from PHY Layer (with bad headers):
// =============================================================================

///@brief Callback for the reception of bad wireless headers
///
///@param packet Pointer to received Macframe
void phyRx_badHeader_callback() {
	
	//Don't do anything with the packet (it had errors, and can't be trusted)
	
	//Increment the bottom LEDs
	warpmac_incrementLEDLow();
	
	return;
}

// ------------------------------------------------------------------------------------------------------------------------------

// Callback Function for reception of packets from PHY Layer (with good headers):
// ==============================================================================

///@brief Callback for the reception of good wireless headers
///
///This function then polls the PHY to determine if the entire packet passes checksum
///thereby triggering the transmission of the ACK and the transmission of the received
///data over Ethernet.
///@param packet Pointer to received Macframe
int phyRx_goodHeader_callback(Macframe* packet){
	
	
	unsigned char state = PHYRXSTATUS_INCOMPLETE;
	//unsigned char srcNode;
	unsigned char should_send = 0;
	
	//xil_printf("--- PHY Layer Server ---\r\n");
	
	////xil_printf("Destination Address: %d\r\n", packet->header.destAddr);
	//xil_printf("Host Address from where packet transitted (My Address...): %d\r\n", packet->header.srcAddr);	
	
	//Calculate the node ID from the packet's source MAC address
	//srcNode = ADDR_TO_NODEID( (packet->header.srcAddr) );
	
	//If the packet is addressed to this node
	if( packet->header.destAddr == (NODEID_TO_ADDR(myID)) ) {
		
		switch(packet->header.pktType) {
				//If received packet is data
			case PKTTYPE_DATA:
		//warpmac_leftHex(6);		
				
				
				if(buff_full[c] == 0){					
					
					txACK.header.reserved0 = 0;
				}
				else{
					
					txACK.header.reserved0 = 1;
				}
				
				warpmac_prepPhyForXmit(&txACK, pktBuf_tx_ACK);
				
				state = warpmac_finishPhyRecv();
				
				if(state & PHYRXSTATUS_GOOD){
					warpmac_startPhyXmit(pktBuf_tx_ACK);
					if (buff_full[c]!=0) {		
						
						break;
							
						}
					/////warpmac_startPhyXmit(pktBuf_tx_ACK);
					warpmac_leftHex(0xF & (packet->header.seqNum));
						
					if(packet->header.seqNum == lastRxSeqNum+1) {
						should_send = 1;
						lastRxSeqNum = packet->header.seqNum;}
						
					if (should_send) {
						
						
						memcpy((void *) warpphy_getBuffAddr(pktBuf_rx_upload[c]), (void *)warpphy_getBuffAddr(pktBuf_rx)+NUM_HEADER_BYTES, packet->header.length);
						
						leng[c]=packet->header.length;
						
						buff_full[c]=1;
					
						
						c=(c+1)%25;
						
		
					}
				
		
				warpmac_finishPhyXmit();
					
					//Waits until the DMA transfer is complete, then starts the EMAC
					//warpmac_startPktToNetwork((packet->header.length));  
				}
				
				if(state & PHYRXSTATUS_BAD) {
					///warpmac_incrementLEDLow();
				}
				
				//xil_printf("Acknowledgement to be received back for PHY Layer Packet from Destinition Address: %d\r\n", packet->header.destAddr);
				break; //END PKTTYPE_DATA
				
			case PKTTYPE_ACK:
	
				break; //END PKTTYPE_ACK
		}
	}
	else {
		state = warpmac_finishPhyRecv();
	}
	
	//Return 0, indicating we didn't clear the PHY status bits (WARPMAC will handle it)
	return 0;
}

// ------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------

// Main Function:
// --------------

///@brief Main function
///
///This function configures MAC parameters, enables the underlying frameworks, and then loops forever.
int main(){
	
	//Initialize global variables
	chan = 1;
	int i;
	//Assign the packet buffers in the PHY
	// The auto responder can't transmit from buffer 0, so we use it for Rx packets
	// The other assignments (DATA/ACK) are arbitrary; any buffer in [1,30] will work
	for ( i=0; i<26; i++){
		pktBuf_rx_upload[i] = i+3;
		buff_full[i]=0;
		leng[i]=0;
		}
		
	
	continuous_rx= 1;
	pktBuf_tx_ACK = 1;
	pktBuf_rx=0;
	//Set the full-rate modulation to QPSK by default
	pktFullRate = HDR_FULLRATE_QPSK;
	///pktFullRate = HDR_FULLRATE_QAM_16;
	
	//Set the payload coding rate to 3/4 rate by default
	pktCodeRate = HDR_CODE_RATE_34;
	
	//Initialize the MAC/PHY frameworks
	
	warpmac_init();
	maximumReSend = 8;
	warpmac_setMaxResend(maximumReSend);
   warpmac_setMaxCW(5);
	warpmac_setTimeout(7000);//120
	warpmac_setSlotTime(22);
	
	//Read Dip Switch value from FPGA board.
	//This value will be used as an index into the routing table for other nodes
	myID = (unsigned short int)warpmac_getMyId();
	warpmac_rightHex(myID);
	
	//Configure the PHY and radios for single antenna (SISO) mode
	warpphy_setAntennaMode(TX_ANTMODE_SISO_ANTA, RX_ANTMODE_SISO_ANTA);
	//warpphy_setAntennaMode(TX_ANTMODE_MULTPLX, RX_ANTMODE_MULTPLX);
	//warpphy_setAntennaMode(TX_ANTMODE_ALAMOUTI_ANTA, RX_ANTMODE_ALAMOUTI_ANTA);
	
	pktDet_RSSI_thresh = 7000;
	pktDet_AC_corr = 90;
	pktDet_AC_energy = 20;
	
	warpphy_setEnergyDetThresh(pktDet_RSSI_thresh);		//Min RSSI (in [0,16368])
	warpphy_setAutoCorrDetParams(pktDet_AC_corr, pktDet_AC_energy);	//Min auto-correlation (in [0,2047])
	warpphy_setLongCorrThresh(8000);		//Min cross-correlation (in [0,45e3])
	warpphy_setCarrierSenseThresh(12000); //Carrier sense thresh (in [0,16368])

	//Rx buffer is where the EMAC will DMA Wireless payloads from
	warpmac_setRxBuffers(&rxMacframe, pktBuf_rx);

	
	//Tx buffer is where the EMAC will send DMA Ethernet payloads to
	//warpmac_setPHYTxBuffer(pktBuf_tx_DATA);
	//warpmac_setEMACRxBuffer(pktBuf_tx_DATA);
	
	//TxACK buffer is where the EMAC will DMA Ethernet payloads to
   warpmac_setPHYTxBuffer(pktBuf_tx_ACK);
  	warpmac_setEMACRxBuffer(pktBuf_tx_ACK);
	
	//Set the modulation scheme use for base rate (header) symbols
	warpmac_setBaseRate(QPSK);
	
	//Copy this node's MAC address into the Tx buffer's source address field as well as TxACK's
	///txMacframe.header.srcAddr = (unsigned short int)(NODEID_TO_ADDR(myID));
	
	txACK.header.length = 0;
	txACK.header.pktType = PKTTYPE_ACK;
	txACK.header.fullRate = HDR_FULLRATE_QPSK;
   txACK.header.codeRate = HDR_CODE_RATE_34;
	txACK.header.srcAddr = NODEID_TO_ADDR(myID);
	txACK.header.destAddr = NODEID_TO_ADDR(RxID);
	warpmac_setTimer(TIMEOUT_TIMER);
	
	//Register callbacks
	warpmac_setCallback(EVENT_LEFTBUTTON, (void *)middleButton);
	warpmac_setCallback(EVENT_LEFTBUTTON, (void *)rightButton);
	warpmac_setCallback(EVENT_LEFTBUTTON, (void *)topButton);
	warpmac_setCallback(EVENT_LEFTBUTTON, (void *)leftButton);
	warpmac_setCallback(EVENT_TIMER, (void *)timer_callback);
	warpmac_setCallback(EVENT_DATAFROMNETWORK, (void *)dataFromNetworkLayer_callback);
	warpmac_setCallback(EVENT_PHYGOODHEADER, (void *)phyRx_goodHeader_callback);
	warpmac_setCallback(EVENT_PHYBADHEADER, (void *)phyRx_badHeader_callback);
	warpmac_setCallback(EVENT_UARTRX, (void *)uartRecv_callback);
	
	//Set the default center frequency
	warpphy_setChannel(GHZ_2, chan);
	
	//Enable carrier sensing
	warpmac_setCSMA(1);
	EMAC_init();
	lastRxSeqNum = -1;
	/*
	////warpmac_setTimer(TIMEOUT_TIMER);
	//------------------------------------------------------
	//autoResponder: set and send ACK automatically after reception
	autoResp_matchCond = PHY_AUTORESPONSE_MATCH_CONFIG(PKTHEADER_INDX_DSTADDR, 2, htons(NODEID_TO_ADDR(myID)));
	mimo_ofdmTxRx_setMatch0(autoResp_matchCond);

	//Match condition 1: received header's type is DATA
	autoResp_matchCond = PHY_AUTORESPONSE_MATCH_CONFIG(PKTHEADER_INDX_TYPE, 1, PKTTYPE_DATA);
	mimo_ofdmTxRx_setMatch1(autoResp_matchCond);

	
	PHY_HEADERTRANSLATE_SET(pktBuf_tx_ACK, (PKTHEADER_INDX_DSTADDR+0), pktBuf_rx, (PKTHEADER_INDX_SRCADDR+0));
	PHY_HEADERTRANSLATE_SET(pktBuf_tx_ACK, (PKTHEADER_INDX_DSTADDR+1), pktBuf_rx, (PKTHEADER_INDX_SRCADDR+1));

	templatePkt.header.fullRate = pktFullRate;
	templatePkt.header.codeRate = pktCodeRate;
	templatePkt.header.length = 0;
	templatePkt.header.srcAddr = (unsigned short)(NODEID_TO_ADDR(myID));
	templatePkt.header.pktType = PKTTYPE_ACK;
	

	//Copy the header down to the PHY's packet buffer
	// This doesn't actually send anything; the autoResponse system will use this template when sending ACKs
	warpmac_prepPhyForXmit(&templatePkt, pktBuf_tx_ACK);
	debug_goodHdrPrint = 0;
	
	autoResp_action = PHY_AUTORESPONSE_TXACTION_CONFIG(pktBuf_tx_ACK, PHY_AUTORESPONSE_ACT_TRANS_HDR, 0, (PHY_AUTORESPONSE_REQ_MATCH0 | PHY_AUTORESPONSE_REQ_MATCH1 | PHY_AUTORESPONSE_REQ_GOODHDR | PHY_AUTORESPONSE_REQ_GOODPKT));
	mimo_ofdmTxRx_setAction0(autoResp_action);
	*/
	//-------------------------------------------------------
	//Listen for new packets to send (either from Ethernet or local dummy packets)
	//debug_goodHdrPrint = 0;
	//xil_printf("Reference Design v16.1 CSMA\r\n");
	
	//xil_printf("Beginning main loop\r\n");
	
	while(1)
	{
		//Poll the timer, PHY and user I/O forever; actual processing will happen via callbacks above
		warpmac_pollPeripherals();
		////warpmac_setRxBuffers(&rxMacframe, pktBuf_rx);
		//warpmac_leftHex(warpmac_carrierSense());
		////warpmac_setRxBuffers(&rxMacframe, pktBuf_rx);
		///xil_printf("Is data being received continuosly");
	}
	
	return 0;
}

// ------------------------------------------------------------------------------------------------------------------------------