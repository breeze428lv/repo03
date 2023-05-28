

/**********************************************************************************/


#ifndef __USERINTERFACE_H
	#define __USERINTERFACE_H

	void NotifyButton(uint16 screen_id, uint16 control_id, uint8  state);
	uint8 is_fc_button(uint8 id);
	void PowerUpAutoRunManager(void);
	void UpdateUI(void);
	void SetButtonUsability(uint8 status);
	void ButtonUsabilityManager(uint8 cmd, uint8 status);
	uint8 TrialTimeOut(MY_DATE rtc, MY_DATE target);
	void UpdateRTC(void);
	void UiButtonManager(uint8 ind);

	void ApiCmdExecutor(void);

	void SetVisibiltyBumpTemp(void);

	#ifdef USE_CURVES
		uint8 ManualFcRunning(void);
	#endif
#endif
