

/**********************************************************************************/


#ifndef __SETTINGS_H
#define __SETTINGS_H

void  BtnResponseSettings(uint16 control_id, uint8  state);
void LoadTargetTemperature(uint8 id);
void LoadAllSettingsInFlash(void);
void SaveDeviceAddress(void);
void UseSettingsInFlash(uint8 *_data,uint16 length);
void SaveSettingsToFlash(uint16 screen_id);
void FactoryCheck(void);
void FactorySettings(uint8 kw);
void WritePinToFlash(void);
void WriteFactoryPinToFlash(void);
void WriteMileageToFlash(void);
void WriteMileageSetToFlash(void);
void controls_update_SCREEN_TIME_SETTING_3(void);
void variables_update(uint8 *_data, uint16 screen_id);
void UpdateMileage(void);
void UseBKG_SettingsInFlash(uint8 *_data);
uint8 getValveControlGap(uint8 timing);
void controls_update_SCREEN_GROUP_SETTING_3(void);
void display(uint8* a, int len);
uint8 nbr_of_enabled_pump(void);
void SaveCompanyName(uint8* name);
void SavePhoneNbr(uint8* phone);
void SaveExpireDate(uint16 year, uint8 month, uint8 day, uint8 disable_manual, uint8 enable_expire_date);
void SaveOutletPressureBias(float bias);
void SaveWL_Bias(int bias);
void SaveEntrancePressureBias(float bias);
void SaveFcFreqOutBias(float bias);
void SavePidFactors(void);
void SavePidDefault(void);
void SavePowerSavingData(void);
void date_init_check(uint8* date);
void save_error(uint8* time, uint8 err_code);
void load_err_record(void);
void load_target_locker(void);
void save_target_locker(uint8 timing); 
void save_target_locker_delta(float delta) ;

void load_target_locker(void);
void save_pump_temp_setting(void);
void save_pump_temp_setting_default(void);
void SaveRemoteTargetEnable(uint8 x);
void SaveUseIoTargetPressure(uint8 x);


#ifdef DBG_DISP_INFO
void display(uint8* a, int len);
#endif

#ifdef USE_PIN_GENERATOR 
void SavePW_ErrIndex(uint8 ind);
#endif


typedef struct{
				uint16 year;
				uint8 month;
				uint8 day;
				uint8 disable_manual;
				uint8 enable_expire_date;
	
} MY_DATE;

uint8 DateCorrection(MY_DATE* date);

void ExpireDateManager(void);

#endif
