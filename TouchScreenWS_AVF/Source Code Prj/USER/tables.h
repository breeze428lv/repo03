


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
                19,        //0   水箱温度
                24,        //1   供暖温度
                17,        //2   出水口温度
                18,        //3   回水温度
                20,        //4   室内温度1
                21,        //5   室内温度2
                22,        //6   室外温度
                23,        //7   冷却液温度







};

typedef struct{
	uint8 sensor_type;
	int decimal_nbr;
	int id;
	
}PHY_INFO;


#endif
