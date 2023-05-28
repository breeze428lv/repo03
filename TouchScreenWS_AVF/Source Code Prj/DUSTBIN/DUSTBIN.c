 int UseSparePump(uint8 deffect_ind)	
{
#ifdef DBG_FUNC_INFO
	 func_dbg_info = ON;
	 if(func_dbg_info) printf("\n\n\n...............Start:【UseSparePump(uint8 deffect_ind)】................\n\n");
#endif

	err_pump_ind = deffect_ind; 
	
#ifdef DBG_FUNC_INFO
	 func_dbg_info = ON;
	 
#endif
	
	RefreshErrorInfo(err_pump_ind, ADD_ERROR_YES);
	if(pump_running_mode_tbl[deffect_ind] == MOD_PF)
	{
		old_pump_index = focused_pump_index;
	}
	else
	{
		old_pump_index = deffect_ind; 
	}

	
	if((pump_running_mode_tbl[deffect_ind] == MOD_PF) && (find_target_pump(deffect_ind) < 10))
	{

	}
	else
	{
		FC_SWITCH = OFF;	 
	}
	
	pump_switch_phase = 1;

	fast_mode = TRUE;
#ifdef DBG_FUNC_INFO
	if(func_dbg_info)
	{
		printf("变频转工频延时: %d ms\n", Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]);
	}
#endif
	timer_tick_count_rec = timer_tick_count; 


	pause(deffect_ind);


	if(amount_of_running_pumps > 0) amount_of_running_pumps--;

	printf("pause deffected pump%d\n", deffect_ind + 1);

 	//Disable Usability and Error table
	pump_usablilty_tbl[deffect_ind] = FALSE;  //pump_usablilty_tbl[i] == 0 : pump i has an error or disabled by user

    pump_error_tbl[deffect_ind] = TRUE; 

	if(pump_running_mode_tbl[deffect_ind] == MOD_PF) //PF pump has error, update pf_head 
	{
//	    update_index_pf_queue_head();
//		printf("PF pump has error, update pf_head to %d\n", index_pf_queue_head);
	}

	return PUMP_ERROR_SOLVED;

}
int UseSparePump(uint8 deffect_ind)	
{
#ifdef DBG_FUNC_INFO
	 func_dbg_info = ON;
	 if(func_dbg_info) printf("\n\n\n...............Start:【UseSparePump(uint8 deffect_ind)】................\n\n");
#endif

	err_pump_ind = deffect_ind; 
	
#ifdef DBG_FUNC_INFO
	 func_dbg_info = ON;
	 
#endif
	
	RefreshErrorInfo(err_pump_ind, ADD_ERROR_YES);

	if(pump_running_mode_tbl[deffect_ind] == MOD_PF)
	{
		pause(deffect_ind);
		if(amount_of_running_pumps > 0) amount_of_running_pumps--;
		if(focused_pump_index == deffect_ind)
		{
			 update_focused_pump_index();
			 printf("fc pump error!update_focused_pump_index()\n");
			 
		}
	 	pump_switch_phase = 0;
		fast_mode = FALSE;
		return PUMP_ERROR_SOLVED;

	}

	if(pump_running_mode_tbl[deffect_ind] == MOD_PF)
	{
		old_pump_index = focused_pump_index;
	}
	else
	{
		old_pump_index = deffect_ind; 
	}

	
	if((pump_running_mode_tbl[deffect_ind] == MOD_PF) && (find_target_pump(deffect_ind) < 10))
	{

	}
	else
	{
		FC_SWITCH = OFF;	 
	}
	
	pump_switch_phase = 1;

	fast_mode = TRUE;
#ifdef DBG_FUNC_INFO
	if(func_dbg_info)
	{
		printf("变频转工频延时: %d ms\n", Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]);
	}
#endif
	timer_tick_count_rec = timer_tick_count; 


	pause(deffect_ind);


	if(amount_of_running_pumps > 0) amount_of_running_pumps--;

	printf("pause deffected pump%d\n", deffect_ind + 1);

 	//Disable Usability and Error table
	pump_usablilty_tbl[deffect_ind] = FALSE;  //pump_usablilty_tbl[i] == 0 : pump i has an error or disabled by user
    pump_error_tbl[deffect_ind] = TRUE; 

	return PUMP_ERROR_SOLVED;

}  
	