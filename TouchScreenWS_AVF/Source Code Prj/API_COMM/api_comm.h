
#ifndef _API_COMM_
#define _API_COMM_
void ModBusReceiver(uint8 data);
void AnswerApiReadFrequency(void);
void ModBusByteReceiver(uint8 data);
void ModBusCmdExtractor(void);
void AnswerApiReadErrors(void);
void AnswerApiReadOutlet(void);
void AnswerApiReadEntrance(void);
void AnswerApiReadPumps(void);
void AnswerApiReadPump(uint8 i);
void AnswerApiReadTargetPressure(void);
void AnswerApiReadDefaultPressure(void);

void AnswerApiReadPump1Temp(void);
void AnswerApiReadPump2Temp(void);
void AnswerApiReadPump3Temp(void);
void AnswerApiReadSmallPumpTemp(void);

void AnswerApiWriteDefaultPressure(uint16 data1, uint8 data2);
void AnswerApiSetNonNeagtive(void);
void AnswerApiSetSmallPumpOn(void);
void AnswerApiSetFactoryValues(void);

void RS485_Initialization(void);
void ReturnModBusCMD(void);

#ifdef DBG_LABVIEW
void ApiLabViewFeeder(void);
#endif

	void update_comm_data0(void);
	void update_comm_data1(void);
	void update_comm_data2(void);
	void update_comm_data3(void);
	void update_comm_data4(void);
	void update_comm_data5(void);
	void update_comm_data6(void);
	void update_comm_data7(void);
	void update_comm_data8(void);
	void update_comm_data9(void);
	void update_comm_data10(void);
	void update_comm_data11(void);
	void update_comm_data12(void);
	void update_comm_data13(void);
	void update_comm_data14(void);

	void UpdateCommData(void);	   //20210618

#endif
