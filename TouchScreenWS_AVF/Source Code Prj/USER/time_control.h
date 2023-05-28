
#ifndef __TIME_CONTROL_H
#define __TIME_CONTROL_H


void start_task_scheduler(void);
void task_scheduler(void);
void update_schedule_data(uint8 row, uint8 column, int32 value);

void SaveScheduleToFlash(void);
void UpdatScheduleCell(uint8 data_byte, uint8 row, uint8 column);
void load_schedule(void);
void TaskWarning(uint8 level);

uint8 ReadScheduleCell(uint8 row, uint8 column);

void pump_pressure_check_startup_monitor(void);
void reset_pump_entrance_pressure_check_startup_monitor(void);
void pump_entrance_pressure_check_startup_monitor(void);
void reset_pump_pressure_check_startup_monitor(void);
float LoadTargetPressure(void);
void ShowTargetPressure(float target);

int check_schedule_ok(void);

void ScheduleUpdate(void);
uint8 CompressData(uint8* data, uint8 len);

#endif
