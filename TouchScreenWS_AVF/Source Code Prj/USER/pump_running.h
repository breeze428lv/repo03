


#ifndef __PUMP_RUNNING_H
#define __PUMP_RUNNING_H

void PIDParament_Init(void);

void Pump_Manager(void);

void control_pump(uint8 i, uint8 mode, uint8 status);

void Start_Mod_n(void);

void Pump_Icons_Initialization(void);

void PidSetTargetPressure(float target_pressure);

void Manual_FC(float frequency);

void control_pump(uint8 i, uint8 mode, uint8 status);

void Manual_Control_Pump(uint8 i, uint8 mode, uint8 status);

void Start_Stopper(uint8 stop_type);

void save_pump_pointer(void);

void Fall_Asleep(void);

void take_cmd_pump(void);

uint8 GetCurrentFrequency(void);

uint8 SleepPressureConfirmed(void);

uint8 WakeUpPressureAlert(uint8 bias);

uint8 IsSmallPump(void);

uint8 Use_Backup(uint8 n);

void FuelMonitor(void);

void use_new_pump(uint8 old_index);

void select_algo(void);

void go_standby(void);

void StandbyMonitor(void);

void push_cmd_pump(uint8 cmd, uint16 screen_id, uint8 control_id, uint16 value);

void start_mod_1_2(void);

void update_pump_icon(uint8 ind, uint8 mode);

void update_focused_pump_index(void);

//void show_rt_info(char* info);

void Set_FC_StartValue(void);

void Set_FC_Zero(void);

void show_frequency(void);

void FatigePumpSwitcher(void);

void FatigueCancelMonitorAvf(void);

void UseSparePump(uint8 deffect_ind);

void StartSinglePumpRestore(uint8 ind);

void show_on_off(void);

int find_target_pump(uint8 ind);

void pid_calc(void);

void switch_to_fc(uint8 index);

#ifdef USE_CURVES
uint8 GetFrequencyDataForCurve(void);
#endif

void UpdateTargetPressure(void);

#ifdef DBG_PUMP_MANAGER	//	
void OSD_DbgInfo(void);
#endif

float LoadFrequency(void);

uint8 PressureInTargetRange(void);

void FC_Retreator(void);

void UpdateDac(float freq, uint8 ch);

void UpdatePumpFreq(float freq, uint8 n);

void RefreshTarget(void);	 //20210707

void PumpMileageMonitor(void); //20210805

int FindLeastUsedPump(void);   //20210805

#endif

#ifdef DBG_TIMING_CONSULTANT	
 	uint16 TimingConsultant(uint8 cmd);
#endif
