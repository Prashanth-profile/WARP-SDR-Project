// This header should be included in the main C source code file

#ifndef EMAC_PACK
#define EMAC_PACK

void EMAC_prepEMACPktForXmit(unsigned char pktBuf_tx_DATA, Macframe* txMacframe);
void EMAC_init();
unsigned short int EMAC_change_dest_mac();

#endif EMAC_PACK