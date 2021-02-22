int phyRx_goodHeader_callback(Macframe* packet);
void phyRx_badHeader_callback();
void dataFromNetworkLayer_callback(Xuint32 length, char* payload);
void timer_callback(unsigned char timerType);
void uartRecv_callback(unsigned char uartByte);
int main();
void leftButton();
