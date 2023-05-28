


#ifndef __TABLES_H
	#define __TABLES_H
#define 	NTC	   		0
#define 	PM	   		1
#define 	PT   		2
#define 	HM_PM   	3
#define 	HM_PT   	4
#define 	WL_SWITCH   5

const uint8 SensoTypes[10] = {
								NTC,  
	    						NTC, 
	    						NTC, 
	    						NTC,      							
								NTC,  
	    						NTC, 
	    						NTC, 
	    						NTC,      							
								PM,     
								PT
								
} ;
 
const uint8 decimal_nbr[10] = {
								1,  
	    						1, 
	    				        1, 
	    					    1,      							
								1,  
	    						1, 
	    				        1, 
	    					    1,      							
						    	1,     
								1
								
} ;

const uint8 map_phybias_tempsetting[8] = {  
                19,        //0   ˮ���¶�
                24,        //1   ��ů�¶�
                17,        //2   ��ˮ���¶�
                18,        //3   ��ˮ�¶�
                20,        //4   �����¶�1
                21,        //5   �����¶�2
                22,        //6   �����¶�
                23,        //7   ��ȴҺ�¶�







};

typedef struct{
	uint8 sensor_type;
	int decimal_nbr;
	int id;
	
}PHY_INFO;


#endif
