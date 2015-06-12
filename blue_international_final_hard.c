//////				 YEAR 2012 AUTO					//////
//////	  ENCODERS ON EXTERNAL INT WITH D-FFs		//////
//////	   SERIAL LCD AND KEYPAD AND EEPROM			////// 
//////				REVERSE NAVIGATION				//////

/*
	After picking basket: 	+90
	Back to start zone: 	-270
	Ramp turn: 				-270
	Back to operator:		-90
*/

/////////////////////////////////////////////////////////////////////////////
///  first time IMU on machine 1/22/2012 
/////////////////////////////////////////////////////////////////////////////

#include<avr/io.h>
#include<math.h> 
#include<util/delay.h>
#include <avr/interrupt.h>
#include<avrlibdefs.h>
#include <mit_tech/touch.h>
#include<avr/eeprom.h>
#include <mit_tech/seriallcd.h>
#include "eeprom_add.h"
#include<stdbool.h>

/// GENERAL
volatile int mean1=0,l1=0,r1=0,int_counter=0,arm_pos=150,fast_turn=0,collector_clamped=0;
volatile short int emergency_stop=0,hit_flag=0,trace=0,basket=0;
volatile float mean1_max,mean1_max_temp,totcorr_scale;
volatile long int ms_count=0;
volatile int var1,var2,var3,var4,var5,var6,var7,var8,var9,var10;
///		IMU
long imu_data=0,an_integer=0,reqa0=0,trace_about=0,shifted_angle;
float omega=0,parity_bit=0,prev_angle_parity=0;

///  ENCODER
volatile float ydest,xdest,speed=0,ydiff,xdiff;
volatile float xcurr=0,ycurr=0;
volatile long count=0;
///  GYROSCOPE
volatile float gp=0,gd=0,gi=0,gp_eep,gd_eep,gi_eep;
volatile float gp_back_eep=0,gd_back_eep=0,gi_back_eep=0;
volatile float temp,temp2=0,wcorr,temp3=0,reqa=0,angle=0,totcorr=0,prev_gyro_error=0;
volatile int gyro_turn=0;
volatile unsigned char gyro_on;
///  LINE TRACING     
volatile int s0,s1,s2,s3,s4,s5,s6,s7,s8,flagl=1,correction=0,countline=0,sen=0,crossline=0,Ki_pow=0;
volatile float error=0,prev_error=0,Ki=0,Kp=0,Kd=0,P=0,D=0,I=0,k=0;
volatile int diff_turn=0,nondiff_turn=0,freewheel=0;
/// KEYPAD
volatile long int ret_val,temp_val = 0;
volatile int choice=0;
/// Strategy
volatile int start_basket,basket_mid,mid_wall,wall_mid,mid_start,start_ramp,ramp_collector,boost_delay=1000,retard_delay=1000,enough=0;
volatile int start_basket_spd,basket_mid_spd,mid_wall_spd;
volatile int wall_mid_spd,mid_start_spd,start_ramp_spd,ramp_collector_spd;
///	NEW
volatile float fuse_error,d_error=0,prev_d_error=0,kp_fuse_front,kp_fuse_back;
bool fusion_flag = false,debuging = false,nbf = false,init_nbf = false;
volatile int dis,attempting_basket = 1;
 

#define BASE_BACK_L PIND&0x10	//D4
#define BASE_BACK_R PIND&0x02	//D1
#define BASE_FRONT_L PIND&0x10
#define BASE_FRONT_R PIND&0x02
#define SECOND_BASKET PINJ&0x20
#define EMERGENCY PIND&0x01
#define RAISE_FOR_BASKET PINJ&0x40
#define COLLECTOR_FULL PINJ&0x80
//#define NO_BASKET PING&0x20
//#define PODG PINE&0x20
#define COLLECTOR_S2 PINH&0x80
#define ARM_LOWER PINB&0x01
#define START PINJ&0x08

#define L 380

#define PISTONA_DDR DDRB
#define PISTONA_PORT PORTB
#define PISTONB_DDR DDRC
#define PISTONB_PORT PORTC
#define PISTONA 7
#define PISTONB 7

#define MAGNET_DDR DDRC
#define MAGNET_PORT PORTC
#define MAGNET1 4
#define MAGNET2 5

#define POWER_OCR OCR1A		//Raise Lower
#define POWER_OCR_DDR DDRB
#define POWER_OCR_PIN 5

#define POWER_PORT PORTC
#define POWER_DDR DDRC

#define POWER_D 2
#define POWER_B 3


#define MENUNUM 8
#define CODENUM 6
#define CONFIGNUM 15
#define DISPLAYNUM 17
#define COUNTSNUM 7
#define SPEEDNUM 7
#define GENNUM 10




char cmain_menu[][16]={"1) Codes",
"2) config",
"3) display",
"4) actuation",
"5) Counts",
"6) Speed",
"7) General",
"8) Test Elex"
};
char ccode_menu[][16]={"1) Only Gyro b1",
"2) only gyro b2",
"3) Gyro+line b1",
"4) Gyro+line b2",
"5) Retry",
"6) Back2start"
};
char cconfig_menu[][16]={"1) Gyro P",
"2) Gyro D",
"3) Gyro I",
"4) Line P",
"5) Line D",
"6) Line I",
"7) Line POW",
"8)	fuse front",
"9) fuse back",
"10) mean_max",
"11)bst_dely",
"12)rtd_dely",
"13) Gyro P_bk",
"14) Gyro D_bk",
"15) Gyro I_bk",
};
char ccounts_menu[][16]={"1. St_Bas",
"2) Bas_Mid",
"3) Mid_Wall",
"4) Wall_Mid",
"5) Mid_St",
"6) St_Ramp",
"7) Ramp_Coll"
};
char cspeed_menu[][16]={"1. St_Bas_Spd",
"2) Bas_Mid_Spd",
"3) Mid_Wall_Spd",
"4) Wall_Mid_Spd",
"5) Mid_St_Spd",
"6) St_Ramp_Spd",
"7) Ramp_Col_Spd"
};


char cgen_menu[][16]={"1)init angle",
"2) mid counts",
"3) end counts",
"4) b1,2 trn spd",
"5) back spd",
"6) var6",
"7) b2 return",
"8) b1 retard",
"9) b1 back cnt",
"10) b1 back spd"
};



void factor_init(void);
void code_menu(void);
long int eep(long int x,unsigned int *add);
void config_menu(void);
void display_menu(void);
void general_menu(void);
void actuation_init(void);
void counts_menu(void);
void speed_menu(void);
void count_display(void);
void speed_display(void);
void gen_display(void);
void full_nav(void); 
void retry_arm_adjust(int pwm_spd);
void release_basket(void);
void arm_adjust(int,int,int);
void test_elex(void);
	
long make_int(void)
{
	int a;
	long num=0;
	
	while(a!=12)
	{
		a=get_key();
		if(a==11)
			putc('0');
		else
			putc(a+48);
		if(a==12)
		{
			a=get_key();
			if(a==10)
				num=num/10;
			else
				break;		
		}
		if(a <= 9)
			num = num*10 + a;
		else if(a == 11)	//11 is zero
			num = num*10;
		else if(a == 10)	//* is for negation
			num = -num;
	}

return num;

}
/////////////////////////////////all menu for I key pad ///////////////////////////////
void main_menu(void)
{
int curr_menu=0,key_pressed;

	while(key_pressed!=12)
	{
		fprintf("\n");
		fprintf(">>");
		fprintf(cmain_menu[curr_menu]);
		lcd_num(angle*10,"");
		nextline();
		
		if(curr_menu==MENUNUM-1)
		{
			fprintf(cmain_menu[0]);
		}
		else
		{
			fprintf(cmain_menu[curr_menu+1]);
		}
		
		key_pressed=get_key();
		
		if(key_pressed==5)
		{
			if(curr_menu==0)
				code_menu();
			if(curr_menu==1)
				config_menu();
			if(curr_menu==2)
				display_menu();
			if(curr_menu==3)
				actuation_init();
			if(curr_menu==4)
				counts_menu();
			if(curr_menu==5)
				speed_menu();
			if(curr_menu==6)
				general_menu();
			if(curr_menu==7)
				test_elex();
		}
		if(key_pressed==2)
		{
			if(curr_menu==0)
				curr_menu=MENUNUM-1;
			else
				curr_menu--;
		}
		if(key_pressed==8)
		{
			if(curr_menu==MENUNUM-1)
				curr_menu=0;
			else
				curr_menu++;
		}

	}
	fprintf("\nFactor Init");
	factor_init();
}
void code_menu(void)
{
int curr_menu=0,key_pressed;
factor_init();
	while(key_pressed!=12)
	{
		fprintf("\n");
		fprintf(">>");
		fprintf(ccode_menu[curr_menu]);
		nextline();
		if(curr_menu==CODENUM-1)
			{
			fprintf(ccode_menu[0]);
			}
		else
			{
			fprintf(ccode_menu[curr_menu+1]);
			}
		key_pressed=get_key();
		
		/*if(key_pressed==5)
		{
			if(colour==1)
			{
				if(curr_menu==0)
				{
					full_nav();
				}
				if(curr_menu==1)
				{
					//g_bas2_r();
				}
				if(curr_menu==2)
				{
					//gl_bas1_r();
				}
				if(curr_menu==3)
				{
					//gl_bas2_r();
				}
				if(curr_menu==4)
				{
				//	retry_r();
				}
				if(curr_menu==5)
				{
				//	start_r();
				}

			}
			else if(colour==2)
			{
				if(curr_menu==0)
				{
				//	g_bas1_b();
				}
				if(curr_menu==1)
				{
				//	g_bas2_b();
				}
				if(curr_menu==2)
				{
				//	gl_bas1_b();
				}
				if(curr_menu==3)
				{
				//	gl_bas2_b();
				}
				if(curr_menu==4)
				{
				//	retry_b();
				}
				if(curr_menu==5)
				{
				//	start_b();
				}
			}
		}*/
	if(key_pressed==2)
		{
		if(curr_menu==0)
			curr_menu=CODENUM-1;
		else
			curr_menu--;
		}
	if(key_pressed==8)
		{
		if(curr_menu==CODENUM-1)
			curr_menu=0;
		else
			curr_menu++;
		}
	if(key_pressed==12)
		{
		main_menu();
		}
	}
}
void config_menu(void)
{
int curr_menu=0,key_pressed;
long temp_val,ret_val;

while(key_pressed!=12)
	{
	fprintf("\n");
	fprintf(">>");
	fprintf(cconfig_menu[curr_menu]);
	nextline();
	if(curr_menu==CONFIGNUM-1)
		{
		fprintf(cconfig_menu[0]);
		}
	else
		{
		fprintf(cconfig_menu[curr_menu+1]);
		}
	key_pressed=get_key();


	if(key_pressed==5)
		{
		fprintf("\n");
		fprintf("Value: ");
		temp_val = make_int();

		if(curr_menu==0)
			{
			ret_val = eep(temp_val,(unsigned int*)GP_ADD);
			lcd_num(ret_val,"\nGp");
			}
		if(curr_menu==1)
			{
			ret_val = eep(temp_val,(unsigned int*)GD_ADD);
			lcd_num(ret_val,"\nGd");
			}
		if(curr_menu==2)
			{
			ret_val = eep(temp_val,(unsigned int*)GI_ADD);
			lcd_num(ret_val,"\nGi");
			}
		if(curr_menu==3)
			{
			ret_val = eep(temp_val,(unsigned int*)KP_ADD);
			lcd_num(ret_val,"\nKp");
			}
		if(curr_menu==4)
			{
			ret_val = eep(temp_val,(unsigned int*)KD_ADD);
			lcd_num(ret_val,"\nKd");
			}
		if(curr_menu==5)
			{
			ret_val = eep(temp_val,(unsigned int*)KI_ADD);
			lcd_num(ret_val,"\nKi");
			}
		if(curr_menu==6)
			{
				ret_val = eep(temp_val,(unsigned int*)KI_POW_ADD);
			lcd_num(ret_val,"\nKi_POW");
			}
		if(curr_menu==7)
			{
			ret_val = eep(temp_val,(unsigned int*)KP_FUSE_FRONT_ADD);
			lcd_num(ret_val,"\nKp_fuse_front");
			}
		if(curr_menu==8)
			{
			ret_val = eep(temp_val,(unsigned int*)KP_FUSE_BACK_ADD);
			lcd_num(ret_val,"\nKp_fuse_back");
			}
		if(curr_menu==9)
			{
			ret_val = eep(temp_val,(unsigned int*)MEAN_MAX_ADD);
			lcd_num(ret_val,"\nmean1_max");
			}
		
		if(curr_menu==10)
			{
			ret_val = eep(temp_val,(unsigned int*)boost_delay_ADD);
			lcd_num(ret_val,"\nbst_dly");
			}
		if(curr_menu==11)
			{
			ret_val = eep(temp_val,(unsigned int*)retard_delay_ADD);
			lcd_num(ret_val,"\nrtd_dly");
			}
		if(curr_menu==12)
			{
			ret_val = eep(temp_val,(unsigned int*)GP_BACK_ADD);
			lcd_num(ret_val,"\nGp_back");
			}
		if(curr_menu==13)
			{
			ret_val = eep(temp_val,(unsigned int*)GD_BACK_ADD);
			lcd_num(ret_val,"\nGd_back");
			}
		if(curr_menu==14)
			{
			ret_val = eep(temp_val,(unsigned int*)GI_BACK_ADD);
			lcd_num(ret_val,"\nGi_back");
			}
			delay(2);
		}
	if(key_pressed==2)
		{
		if(curr_menu==0)
			curr_menu=CONFIGNUM-1;
		else
			curr_menu--;
		}
	if(key_pressed==8)
		{
		if(curr_menu==CONFIGNUM-1)
			curr_menu=0;
		else
			curr_menu++;
		}

	}
}
void display_menu(void)
{
int curr_menu=0,key_pressed=0;

factor_init();

while(key_pressed!=12)
	{
		if(curr_menu==0)
			{
			lcd_num(gp_eep*100,"\n1) Gp");
			nextline();
			lcd_num(gd_eep*100,"2) Gd");
			}
		if(curr_menu==1)
			{
			lcd_num(gd_eep*100,"\n2) Gd");
			nextline();
			lcd_num(gi_eep,"3) Gi");
			}
		if(curr_menu==2)
			{
			lcd_num(gi_eep,"\n3) Gi");
			nextline();
			lcd_num(Kp*10,"4) Lp");
			}
		if(curr_menu==3)
			{
			lcd_num(Kp*10,"\n4) Lp");
			nextline();
			lcd_num(Kd*10,"5) ld");
			}
		if(curr_menu==4)
			{
			lcd_num(Kd*10,"\n5) ld");
			nextline();
			lcd_num(Ki*pow(10,Ki_pow),"6) li");
			}
		if(curr_menu==5)
			{
			lcd_num(Ki*pow(10,Ki_pow),"\n6) li");
			nextline();
			lcd_num(Ki_pow,"7) lPOW");
			}
		if(curr_menu==6)
			{
			lcd_num(Ki_pow,"\n7) lPOW");
			nextline();
			lcd_num(kp_fuse_front*100,"8) fuse front");
			}
		if(curr_menu==7)
			{
			lcd_num(kp_fuse_front*100,"\n8) fuse front");
			nextline();
			lcd_num(kp_fuse_back*100,"9) fuse back");
			}
		if(curr_menu==8)
			{
			lcd_num(kp_fuse_back*100,"\n9) fuse back");
			nextline();
			fprintf("10)Count Menu");
			}
		if(curr_menu==9)
			{
			fprintf("\n10)Count Menu");
			nextline();
			fprintf("11)Speed Menu");			
			}
		if(curr_menu==10)
			{
			fprintf("\n11)Speed Menu");			
			nextline();
			lcd_num(boost_delay,"12) boost");
			}
		if(curr_menu==11)
			{
			lcd_num(boost_delay,"\n12) boost");
			nextline();
			lcd_num(retard_delay,"13) rtd");
			}
		if(curr_menu==12)
			{
			lcd_num(retard_delay,"\n13) rtd");
			nextline();
			lcd_num(gp_back_eep*100,"14) Gp_back");
			}
		if(curr_menu==13)
			{
			lcd_num(gp_back_eep*100,"\n14) Gp_back");
			nextline();
			lcd_num(gd_back_eep*100,"15) Gd_back");
			}
		if(curr_menu==14)
			{
			lcd_num(gd_back_eep*100,"\n15) Gd_back");
			nextline();
			lcd_num(gi_back_eep,"16) Gi_back");
			}
		if(curr_menu==15)
			{
			lcd_num(gi_back_eep,"\n16) Gi_back");
			nextline();
			fprintf("17) General");
			}
		if(curr_menu==16)
			{
			fprintf("\n17) General");
			nextline();
			lcd_num(gp_eep*100,"1) Gp");
			}
	key_pressed=get_key();
	
	if(key_pressed==2)
		{
		if(curr_menu==0)
			curr_menu=DISPLAYNUM-1;
		else
			curr_menu--;
		}
	if(key_pressed==8)
		{
		if(curr_menu==DISPLAYNUM-1)
			curr_menu=0;
		else
			curr_menu++;
		}
	if(key_pressed==5)
		{
		if(curr_menu==9)
			count_display();				

		if(curr_menu==10)
			speed_display();

		if(curr_menu==16)
			gen_display();				
		}
}	
}
void speed_menu(void)
{
int curr_menu=0,key_pressed;
long temp_val,ret_val;

while(key_pressed!=12)
	{
	fprintf("\n");
	fprintf(">>");
	fprintf(cspeed_menu[curr_menu]);
	nextline();
	if(curr_menu==SPEEDNUM-1)
		{
		fprintf(cspeed_menu[0]);
		}
	else
		{
		fprintf(cspeed_menu[curr_menu+1]);
		}
	key_pressed=get_key();


	if(key_pressed==5)
		{
		fprintf("\n");
		fprintf("Value: ");
		temp_val = make_int();

		if(curr_menu==0)
			{
			ret_val = eep(temp_val,(unsigned int*)START_BASKET_SPD_ADD);
			lcd_num(ret_val,"\nSt_Bas_Spd");
			}
		if(curr_menu==1)
			{
			ret_val = eep(temp_val,(unsigned int*)BASKET_MID_SPD_ADD);
			lcd_num(ret_val,"\nBas_Mid_Spd");
			}
		if(curr_menu==2)
			{
			ret_val = eep(temp_val,(unsigned int*)MID_WALL_SPD_ADD);
			lcd_num(ret_val,"\nMid_Wall_Spd");
			}
		if(curr_menu==3)
			{
			ret_val = eep(temp_val,(unsigned int*)WALL_MID_SPD_ADD);
			lcd_num(ret_val,"\nWall_Mid_Spd");
			}
		if(curr_menu==4)
			{
			ret_val = eep(temp_val,(unsigned int*)MID_START_SPD_ADD);
			lcd_num(ret_val,"\nMid_St_Spd");
			}
		if(curr_menu==5)
			{
			ret_val = eep(temp_val,(unsigned int*)START_RAMP_SPD_ADD);
			lcd_num(ret_val,"\nSt_Ramp_Spd");
			}
		if(curr_menu==6)
			{
				ret_val = eep(temp_val,(unsigned int*)RAMP_COLLECTOR_SPD_ADD);
			lcd_num(ret_val,"\nRamp_Coll_Spd");
			}
			delay(1);
		}
	if(key_pressed==2)
		{
		if(curr_menu==0)
			curr_menu=SPEEDNUM-1;
		else
			curr_menu--;
		}
	if(key_pressed==8)
		{
		if(curr_menu==SPEEDNUM-1)
			curr_menu=0;
		else
			curr_menu++;
		}

	}
}
void counts_menu(void)
{
int curr_menu=0,key_pressed;
long temp_val,ret_val;

while(key_pressed!=12)
	{
	fprintf("\n");
	fprintf(">>");
	fprintf(ccounts_menu[curr_menu]);
	nextline();
	if(curr_menu==COUNTSNUM-1)
		{
		fprintf(ccounts_menu[0]);
		}
	else
		{
		fprintf(ccounts_menu[curr_menu+1]);
		}
	key_pressed=get_key();


	if(key_pressed==5)
		{
		fprintf("\n");
		fprintf("Value: ");
		temp_val = make_int();

		if(curr_menu==0)
			{
			ret_val = eep(temp_val,(unsigned int*)START_BASKET_ADD);
			lcd_num(ret_val,"\nSt_Bas");
			}
		if(curr_menu==1)
			{
			ret_val = eep(temp_val,(unsigned int*)BASKET_MID_ADD);
			lcd_num(ret_val,"\nBas_Mid");
			}
		if(curr_menu==2)
			{
			ret_val = eep(temp_val,(unsigned int*)MID_WALL_ADD);
			lcd_num(ret_val,"\nMid_Wall");
			}
		if(curr_menu==3)
			{
			ret_val = eep(temp_val,(unsigned int*)WALL_MID_ADD);
			lcd_num(ret_val,"\nWall_Mid");
			}
		if(curr_menu==4)
			{
			ret_val = eep(temp_val,(unsigned int*)MID_START_ADD);
			lcd_num(ret_val,"\nMid_St");
			}
		if(curr_menu==5)
			{
			ret_val = eep(temp_val,(unsigned int*)START_RAMP_ADD);
			lcd_num(ret_val,"\nSt_Ramp");
			}
		if(curr_menu==6)
			{
				ret_val = eep(temp_val,(unsigned int*)RAMP_COLLECTOR_ADD);
			lcd_num(ret_val,"\nRamp_Coll");
			}
			delay(2);
		}
	if(key_pressed==2)
		{
		if(curr_menu==0)
			curr_menu=COUNTSNUM-1;
		else
			curr_menu--;
		}
	if(key_pressed==8)
		{
		if(curr_menu==COUNTSNUM-1)
			curr_menu=0;
		else
			curr_menu++;
		}

	}
}


void general_menu(void)
{
int curr_menu=0,key_pressed;
long temp_val,ret_val;

while(key_pressed!=12)
	{
	fprintf("\n");
	fprintf(">>");
	fprintf(cgen_menu[curr_menu]);
	nextline();
	if(curr_menu==GENNUM-1)
		{
		fprintf(cgen_menu[0]);
		}
	else
		{
		fprintf(cgen_menu[curr_menu+1]);
		}
	key_pressed=get_key();


	if(key_pressed==5)
		{
		fprintf("\n");
		fprintf("Value: ");
		temp_val = make_int();

		if(curr_menu==0)
			{
			ret_val = eep(temp_val,(unsigned int*)VAR1_ADD);
			lcd_num(ret_val,"\nVar1");
			}
		if(curr_menu==1)
			{
			ret_val = eep(temp_val,(unsigned int*)VAR2_ADD);
			lcd_num(ret_val,"\nVar2");
			}
		if(curr_menu==2)
			{
			ret_val = eep(temp_val,(unsigned int*)VAR3_ADD);
			lcd_num(ret_val,"\nVar3");
			}
		if(curr_menu==3)
			{
			ret_val = eep(temp_val,(unsigned int*)VAR4_ADD);
			lcd_num(ret_val,"\nVar4");
			}
		if(curr_menu==4)
			{
			ret_val = eep(temp_val,(unsigned int*)VAR5_ADD);
			lcd_num(ret_val,"\nVar5");
			}
		if(curr_menu==5)
			{
			ret_val = eep(temp_val,(unsigned int*)VAR6_ADD);
			lcd_num(ret_val,"\nVar6");
			}
		if(curr_menu==6)
			{
				ret_val = eep(temp_val,(unsigned int*)VAR7_ADD);
			lcd_num(ret_val,"\nVar7");
			}
		if(curr_menu==7)
			{
				ret_val = eep(temp_val,(unsigned int*)VAR8_ADD);
			lcd_num(ret_val,"\nVar8");
			}
		if(curr_menu==8)
			{
				ret_val = eep(temp_val,(unsigned int*)VAR9_ADD);
			lcd_num(ret_val,"\nVar9");
			}
		if(curr_menu==9)
			{
				ret_val = eep(temp_val,(unsigned int*)VAR10_ADD);
			lcd_num(ret_val,"\nVar10");
			}
			delay(2);
		}
	if(key_pressed==2)
		{
		if(curr_menu==0)
			curr_menu=GENNUM-1;
		else
			curr_menu--;
		}
	if(key_pressed==8)
		{
		if(curr_menu==GENNUM-1)
			curr_menu=0;
		else
			curr_menu++;
		}

	}
}

void count_display(void)
{
	int curr_menu=0,key_pressed=0;

factor_init();

while(key_pressed!=12)
	{
		if(curr_menu==0)
			{
			lcd_num(start_basket,"\n1) st_bsk");
			nextline();
			lcd_num(basket_mid,"2) bsk_mid");
			}
		if(curr_menu==1)
			{
			lcd_num(basket_mid,"\n2) bsk_mid");
			nextline();
			lcd_num(mid_wall,"3) mid_wall");
			}
		if(curr_menu==2)
			{
			lcd_num(mid_wall,"\n3) mid_wall");
			nextline();
			lcd_num(wall_mid,"4)wall_mid");
			}
		if(curr_menu==3)
			{
			lcd_num(wall_mid,"\n4)wall_mid");
			nextline();
			lcd_num(mid_start,"5)mid_st");
			}
		if(curr_menu==4)
			{
			lcd_num(mid_start,"\n5)mid_st");
			nextline();
			lcd_num(start_ramp,"6)start_ramp");
			}
		if(curr_menu==5)
			{
			lcd_num(start_ramp,"\n6)st_ramp");
			nextline();
			lcd_num(ramp_collector,"7)rmp_coll");
			}
		if(curr_menu==6)
			{
			lcd_num(ramp_collector,"\n7)rmp_coll");
			nextline();
			lcd_num(start_basket,"1)st_bsk");
			}
	key_pressed=get_key();
	
	if(key_pressed==2)
		{
		if(curr_menu==0)
			curr_menu=COUNTSNUM-1;
		else
			curr_menu--;
		}
	if(key_pressed==8)
		{
		if(curr_menu==COUNTSNUM-1)
			curr_menu=0;
		else
			curr_menu++;
		}
	}
}
void speed_display(void)
{
	int curr_menu=0,key_pressed=0;

factor_init();

while(key_pressed!=12)
	{
		if(curr_menu==0)
			{
			lcd_num(start_basket_spd,"\n1) st_bsk");
			nextline();
			lcd_num(basket_mid_spd,"2) bsk_mid");
			}
		if(curr_menu==1)
			{
			lcd_num(basket_mid_spd,"\n2) bsk_mid");
			nextline();
			lcd_num(mid_wall_spd,"3) mid_wall");
			}
		if(curr_menu==2)
			{
			lcd_num(mid_wall_spd,"\n3) mid_wall");
			nextline();
			lcd_num(wall_mid_spd,"4)wall_mid");
			}
		if(curr_menu==3)
			{
			lcd_num(wall_mid_spd,"\n4)wall_mid");
			nextline();
			lcd_num(mid_start_spd,"5)mid_st");
			}
		if(curr_menu==4)
			{
			lcd_num(mid_start_spd,"\n5)mid_st");
			nextline();
			lcd_num(start_ramp_spd,"6)start_ramp");
			}
		if(curr_menu==5)
			{
			lcd_num(start_ramp_spd,"\n6)st_ramp");
			nextline();
			lcd_num(ramp_collector_spd,"7)rmp_coll");
			}
		if(curr_menu==6)
			{
			lcd_num(ramp_collector_spd,"\n7)rmp_coll");
			nextline();
			lcd_num(start_basket_spd,"1)st_bsk");
			}
	key_pressed=get_key();
	
	if(key_pressed==2)
		{
		if(curr_menu==0)
			curr_menu=COUNTSNUM-1;
		else
			curr_menu--;
		}
	if(key_pressed==8)
		{
		if(curr_menu==COUNTSNUM-1)
			curr_menu=0;
		else
			curr_menu++;
		}
	}
}

void gen_display(void)
{
	int curr_menu=0,key_pressed=0;

factor_init();

while(key_pressed!=12)
	{
		if(curr_menu==0)
			{
			lcd_num(var1,"\n1)Var1");
			nextline();
			lcd_num(var2,"2)Var2");
			}
		if(curr_menu==1)
			{
			lcd_num(var2,"\n2)Var2");
			nextline();
			lcd_num(var3,"3)Var3");
			}
		if(curr_menu==2)
			{
			lcd_num(var3,"\n3)Var3");
			nextline();
			lcd_num(var4,"4)Var4");
			}
		if(curr_menu==3)
			{
			lcd_num(var4,"\n4)Var4");
			nextline();
			lcd_num(var5,"5)Var5");
			}
		if(curr_menu==4)
			{
			lcd_num(var5,"\n5)Var5");
			nextline();
			lcd_num(var6,"6)Var6");
			}
		if(curr_menu==5)
			{
			lcd_num(var6,"\n6)Var6");
			nextline();
			lcd_num(var7,"7)Var7");
			}
		if(curr_menu==6)
			{
			lcd_num(var7,"\n7)Var7");
			nextline();
			lcd_num(var8,"8)Var8");
			}
		if(curr_menu==7)
			{
			lcd_num(var8,"\n8)Var8");
			nextline();
			lcd_num(var9,"9)Var9");
			}
		if(curr_menu==8)
			{
			lcd_num(var9,"\n8)Var9");
			nextline();
			lcd_num(var10,"9)Var10");
			}
		if(curr_menu==9)
			{
			lcd_num(var10,"\n10)Var10");
			nextline();
			lcd_num(var1,"1)Var1");
			}
	key_pressed=get_key();
	
	if(key_pressed==2)
		{
		if(curr_menu==0)
			curr_menu=GENNUM-1;
		else
			curr_menu--;
		}
	if(key_pressed==8)
		{
		if(curr_menu==GENNUM-1)
			curr_menu=0;
		else
			curr_menu++;
		}
	}
}


/// Interrupts
ISR(TIMER5_COMPA_vect)
{	
	
	l1=mean1;
	r1=mean1;
	
	////small value of mean1 will have more error factors and correction while line tracing(while coming at start of ramp)
	if(mean1>30)
		k=(float)(mean1/5.0);
	else
		k=(float)(mean1/3.0);

	
	/// /////// l6 l5 l4 l3 l2 l1 l0//////////////////
	/// //////  s0 ...............s6/////////////////
	
	//sen=s0+s1+s2+s3+s4+s5+s6+s7+s8;
	
	sen = ((PINL&_BV(0))>>0) + ((PINL&_BV(1))>>1) + ((PINL&_BV(2))>>2)  + ((PINL&_BV(3))>>3)  + ((PINL&_BV(4))>>4)  + ((PINL&_BV(5))>>5)  + ((PINL&_BV(6))>>6 ) + ((PINL&_BV(7))>>7)  + ((PING&_BV(4))>>4);

	if((sen < 4)&&(!(flagl)))   ///////linecount
	{	
		crossline++;
		flagl=1;
	}
	if(sen>4)   /////////count of line
	{
		flagl=0;
	}
	
	/// GYRO
	if(gyro_on)		//only gyro
	{
		
		shifted_angle = (360 + angle - trace_about);
		
		if(shifted_angle >= 360)
			shifted_angle = shifted_angle - 360;
		
		if(reqa<=0 && gyro_turn)
			temp2 = -reqa - shifted_angle;//req = -90,temp2>0,//req = -270,temp2>0,
		else
			temp2=angle-(reqa + trace_about);
		
		if (temp2>180)			//think that original angle is 350 and reqa is 10 ...
			temp2=temp2-360;
		else if(temp2<(-180))	//think that o. ang is 10 and reqa is 340 ...
			temp2=360+temp2;
		
		//wcorr=temp2-prev_gyro_error;
		//prev_gyro_error=temp2;
		temp2*=gp;	//7.10;	
		
		//wcorr=wcorr*gd;//*3.25;					///////note this differial correction is according to error not 
												/////// according to omega
			
		
		//if(temp2==0)
			//temp3=0;
		
	//	if(gi != 0)
		//	temp3 +=temp2/(float)gi;	//7200.0
		//else
			//temp3 = 0;
		
		totcorr = temp2;//+wcorr;//+temp3;
			
		///////////////////////////////////////////
		if(!(gyro_turn))
		{
			if(fusion_flag)
			{
				if(PINL&_BV(3))
					d_error = 20; 
				else if(PINL&_BV(5))
					d_error = -20;
				else if(PINL&_BV(4))		
					d_error = 0;
				else if(PINL&_BV(2))  
					d_error = 40;    
				else if(PINL&_BV(6))
					d_error = -40;
				else if(PINL&_BV(1))   
					d_error = 65;
				else if(PINL&_BV(7))
					d_error = -65;
				else if(PINL&_BV(0))   
					d_error = 95;
				else if(PING&_BV(4))
					d_error = -95;
				
				//if(prev_d_error!=d_error || prev_angle_parity!=angle)
				//{
					if(mean1<0)
						fuse_error = -(kp_fuse_back*((float)(-d_error*cos(0.0174*shifted_angle)-(L*sin(0.0174*shifted_angle)))));
					else
						fuse_error = -kp_fuse_front*((float)d_error*cos(0.0174*(shifted_angle)));
				//}-kp_fuse_front*d_error;//
				
				totcorr = totcorr + fuse_error;
				
				//prev_d_error=d_error;
			}
			
			totcorr = ((float)mean1*totcorr/((float)(mean1_max)));//totcorr * totcorr_scale;	// CORRECTION PROPORTIONAL TO MEAN1
			
			if(basket)
			{
				totcorr_scale = (mean1/(float)mean1_max) * (mean1/(float)mean1_max);
				
				totcorr = totcorr * totcorr_scale;
			}
			if(totcorr>90)
				totcorr=90;
			else if(totcorr<-90)
				totcorr=-90;
		}
	
		//////////////////////////////////////////
		
		if(mean1>0)
		{
			if(totcorr>=0)
				l1-=totcorr;

			else
				r1+=totcorr;
		}
		else if(mean1<0)
		{
			if(totcorr>0)
				l1+=totcorr;
			else
				r1-=totcorr;
		}
		else if(mean1==0)
		{
			if(gyro_turn && fast_turn)
			{
				if((totcorr<25)&&(totcorr>0))
					totcorr=25;
				if((totcorr>(-25))&&(totcorr<0))
					totcorr=-25;	
			}
			//////////if reqa is +ve then 1 wheel will go back accordingly
			//////////if reqa is -ve then 1 wheel will go forward accordingly
			if(reqa>=0)//90 and 270
			{
				if(totcorr>=0)
					l1-=totcorr;
				else
					r1+=totcorr;
			}
			else
			{
				if(totcorr>=0)
					l1+=totcorr;
				else
					r1-=totcorr;
			}
		}
	}
	
	if(diff_turn==1)
		r1*=-1;
	else if(diff_turn==2)
		l1*=-1;

	if(freewheel)
	{
		if(nondiff_turn==1)// Right turn
		r1=255;
		else if(nondiff_turn==2)// left turn
		l1=255;
	}
	else
	{
		if(nondiff_turn==1)// Right turn
			r1=0;
		else if(nondiff_turn==2)// left turn
			l1=0;
	}
	
	if(r1<0)
	{
		r1*=-1;
		PORTD|=0b10000000;
	}
	else
		PORTD&=0b01111111;
	
	if(l1<0)
	{
		l1*=-1;
		PORTD&=0b10111111;
	}
	else
		PORTD|=0b01000000;

	if(l1>255)
		l1=254;

	if(r1>255)
		r1=254;
	
	if(PIND&0x04)
		UDR1=r1;	  	//lower 8 bits
	else
		UDR1=l1;
	
	if(emergency_stop)
	{
		int_counter++;
		if(int_counter<500)
		{
			mean1=0;
			l1=0;
			r1=0;
		}
		
		else
		{
			cli();
			full_nav();
		}
	}
}
ISR(TIMER3_COMPA_vect)
{
	speed=count-speed;

	xcurr=xcurr+speed*cos(0.01745*(360-(angle-trace_about)));
	ycurr=ycurr+speed*sin(0.01745*(angle-trace_about));///360-angle
	xdiff=xdest-xcurr;
	ydiff=ydest-ycurr;
	
	
	if((!gyro_turn) && (!reqa0))
	{
		reqa=((float)ydiff/(float)xdiff);
		reqa=atan(reqa);
		reqa*=57.295779;
	}
	if((reqa0)&&(!gyro_turn))
		reqa=0;
	
	if(hit_flag)
	{
		if((!(BASE_BACK_L))||(!(BASE_BACK_R)))
		{
			ms_count++;
			if(ms_count>15)
			{
				enough=1;
			}
		}
		else
		{
			ms_count = 0;
			enough = 0;
		}
	}
	else
	{
		enough=0;
		ms_count=0;
	}
	
	speed=count;
}
ISR (INT4_vect)
{
	if(PINE&_BV(3))
		count--;//Direction reversed
	else
		count++;
}
ISR (INT0_vect)	//We love you:)
{	
	emergency_stop = 1;
	int_counter=0;
	
	//fprintf("\nSelfKill!! :P");
}
ISR(INT6_vect)
{
	if(PINE&_BV(7))
		arm_pos++;
	else
		arm_pos--;
}


ISR(USART2_RX_vect)
{

	imu_data=UDR2;
	if(imu_data=='Y')
	{
		an_integer=0;
	}
	else if(imu_data=='*')
	{
		angle=(an_integer)/100.00;
		
		parity_bit=(angle-prev_angle_parity);			/// omega is +ve in angle positive direction
		
		if(parity_bit>180)
			parity_bit=parity_bit-360;
		else if(parity_bit<-180)
			parity_bit=parity_bit+360;
			
		if((parity_bit>20)||(parity_bit<-20))
		{
			parity_bit=0;
			angle=prev_angle_parity;
		}
		prev_angle_parity=angle;
	}
	else
	{
		imu_data-=48;		///conversion of ascii
		an_integer=an_integer*10+imu_data;
	}
}
/// Inits
void adc_init(void)
{
	ADMUX |= 0xE4;
	ADCSRA |= 0x87;
	ADCSRB |= 0x08;
}
void retry_clear(void)
{
	mean1=0,l1=0,r1=0,int_counter=0;
	emergency_stop=0,hit_flag=0,trace=0;
	///		IMU    /////////////////////////////////////
	reqa0=0,trace_about=0;
	omega=0;
	
	///  ENCODER
	speed=0;
	xcurr=0,ycurr=0;
	count=0;
	///  GYROSCOPE
	gp=0,gd=0,gi=0;
	gp_back_eep=0,gd_back_eep=0,gi_back_eep=0;
	temp2=0,temp3=0,reqa=0,angle=0,totcorr=0,prev_gyro_error=0;
	gyro_turn=0;
	///  LINE TRACING     
	flagl=1,correction=0,countline=0,sen=0,crossline=0,Ki_pow=0;
	error=0,prev_error=0,Ki=0,Kp=0,Kd=0,P=0,D=0,I=0,k=0;
	diff_turn=0,nondiff_turn=0,freewheel=0;
	/// KEYPAD
	temp_val = 0;
	choice=0;
	/// Strategy
	boost_delay=1000,retard_delay=1000;
	/// Fusion
	fusion_flag = false;
	
	init_nbf = nbf = false;
}
 

void actuation_init(void)
{
	int choice;
	do{
		fprintf("\n2:RAISE  8:LOWER");
		nextline();
		lcd_num(arm_pos,"ARM POS");
		choice  = return_key();
		if(choice == 2)
		{
			/////////////	RAISING 	///////////////////////////////
			sbi(POWER_PORT,POWER_B);	//brake low	for inside
			cbi(POWER_PORT,POWER_D);	//direction high for raising
			POWER_OCR=150;
		}
		else if(choice == 8)
		{
		/////////////	LOWERING 	///////////////////////////////
			cbi(POWER_PORT,POWER_B);	//brake low	for inside
			sbi(POWER_PORT,POWER_D);	//direction low for lowering
			POWER_OCR=150;
		}
		else if(choice==1)
		{
			cbi(PISTONA_PORT,PISTONA);//PISTON CLOSED
			sbi(PISTONB_PORT,PISTONB);
		}
		else if(choice==3)
		{
			sbi(PISTONA_PORT,PISTONA);//PISTON OPEN
			cbi(PISTONB_PORT,PISTONB);
		}
		else if(choice==7)
		{
			cbi(MAGNET_PORT,MAGNET1);
			cbi(MAGNET_PORT,MAGNET2);
		}
		else if(choice==9)
		{
			sbi(MAGNET_PORT,MAGNET1);
			sbi(MAGNET_PORT,MAGNET2);
		}
		else
		{
			POWER_OCR=0;
			cbi(POWER_PORT,POWER_B);	//brake
			cbi(POWER_PORT,POWER_D);	//brake
		}
		
	}
	while(choice!=10);
}

void init_timer(void)
{
	
	TCCR5A=0X00;			//////////	1k interrupt gyro+line tracin
	TCCR5B=0X0B;
	TCCR5C=0x00;
	TIMSK5=0X02;
	OCR5A=249;
	
	TCCR3A=0x00;		 	/////////    encoder read and reqa calc
	TCCR3B=0x0D;
	TCCR3C=0;
	TIMSK3=2;
	OCR3A=250;
	
	TCCR1A=0x81;					
	TCCR1B=0x01;	
	TCCR1C=0x00;
	OCR1A=0;
	
	EICRA=0X02;
	EICRB=0X3B;				//////////rising EDGE ON INT5 GENERATES INTERRUPT
	EIMSK=0X51;				//////////INT4 INITIALIZED
}

void init_ports(void)
{
	sbi(DDRD,7);		//right motor direction 
	sbi(DDRD,6);		//left motor direction
	cbi(DDRD,2);		//which motor value is in udr (rx)
	sbi(DDRD,3);			//TXD
	sbi(DDRD,5);			//CK	
	//// we have ads control in starting and low makes freedom
	sbi(PORTD,5);

	sbi(POWER_OCR_DDR,POWER_OCR_PIN);		//PWM				Power Window
	sbi(POWER_DDR,POWER_D);		//direction
	sbi(POWER_DDR,POWER_B);		//brake
	
	cbi(POWER_PORT,POWER_D);
	cbi(POWER_PORT,POWER_B);
	POWER_OCR=0;
	

	
	DDRL=0X00;				//sensors
	cbi(DDRG,4);			//Pole Sensor
	
	cbi(DDRE,3);		/////////  Navigation ENCODER 
	cbi(DDRE,4);
	
	cbi(DDRE,6);		/////////  Raising-lowering ENCODER 
	cbi(DDRE,7);	
	////////////////////////////////////////
	
	sbi(PISTONA_DDR,PISTONA);
	sbi(PISTONB_DDR,PISTONB);
	
	sbi(PISTONA_PORT,PISTONA);//PISTON OPEN
	cbi(PISTONB_PORT,PISTONB);
	
	sbi(MAGNET_DDR,MAGNET1);
	sbi(MAGNET_DDR,MAGNET2);
	
	cbi(MAGNET_PORT,MAGNET1);
	cbi(MAGNET_PORT,MAGNET2);
	
	/////////////////////////////////////////
	cbi(DDRD,1);	/// right base
	sbi(PORTD,1);
	
	cbi(DDRD,4);	/// left base
	sbi(PORTD,4);
	
	/////////////////////
	//cbi(DDRG,5);	//No basket
	//sbi(PORTG,5);
	
	cbi(DDRD,0);	//EXTERNAL INTERRUPT INT0
	sbi(PORTD,0);
	
	
	cbi(DDRJ,5);	//direct collector run
	sbi(PORTJ,5);
	
	cbi(DDRJ,3);	//full run
	sbi(PORTJ,3);
	
	cbi(DDRJ,6);	//Retry Arm Init
	sbi(PORTJ,6);
	
	cbi(DDRJ,7);	//ramp collector
	sbi(PORTJ,7);

	cbi(DDRB,0);	//Arm Lower
	sbi(PORTB,0);	
	
	cbi(DDRH,7);	//Collector without ramp from s2
	sbi(PORTH,7);
	
	cbi(DDRE,5);	//collector clamped
	sbi(PORTE,5);
	
	cbi(DDRK,4);
	cbi(PORTK,4);
}
void USART1_init(void)
{
	UCSR1A=0;
	// 7-Recieve complete
	// 6-Transmit complete
	// 5-Data Register empty(empty-1)
	// 4-Frame Error
	// 3-Data Over Run
	// 2-Parity error
	// 1-double Transmission speed
	// 0-Multi Processor Commmmunication mode
 
	UCSR1B=0b00001000;
	// 7-Receive Interrupt Enable
	// 6-Transmit Interrupt Enable
	// 5-Data Register Empty Interrupt enable
	// 4-Reciever Enable
	// 3-Transmitter Enable
	// 2-UCSZ 2 (MSB of character size)
	// 1-Recieve 9th bit
	// 0-Transmit 9th bit
	
	UCSR1C=0b00000110; // ASynchronous mode
	
	UBRR1=3;// Baud rate 9600
	
	sbi(DDRD,5);//Clock PIN D5 output
	sbi(DDRD,3);//Transmit PIN D3
}
void USART2_init(void)
{
	UCSR2A=0;
	// 7-Recieve complete
	// 6-Transmit complete
	// 5-Data Register empty(empty-1)
	// 4-Frame Error
	// 3-Data Over Run
	// 2-Parity error
	// 1-double Transmission speed
	// 0-Multi Processor Commmunication mode
 
	UCSR2B=0b10010000;
	// 7-Receive Interrupt Enable
	// 6-Transmit Interrupt Enable
	// 5-Data Register Empty Interrupt enable
	// 4-Reciever Enable
	// 3-Transmitter Enable
	// 2-UCSZ 2 (MSB of character size)
	// 1-Recieve 9th bit
	// 0-Transmit 9th bit
	
	UCSR2C=0b00000110; // Synchronous mode
	
	UBRR2=25;// Baud rate 9600
	
	cbi(DDRH,0);//receiver PIN H0
	
}
void init_hardware(void)
{
	init_ports();
	key_init();
	lcd_init();
	adc_init();
}
void init_all(void)
{
	init_hardware();
	
	emergency_stop=0;
	int_counter=0;
	USART1_init();
	init_timer();
	factor_init();	//INITIALIZING FACTORS  FROM EEPROM
	
	USART2_init();
}


void test_elex(void)
{
	mean1 = 0;
	sei();
	
	//	HIT SWITCHES
	
	fprintf("\nPRESS LEFT FRONT");
	while(BASE_FRONT_L);
	fprintf("\nRELEASE LEFT FRONT");
	while(!BASE_FRONT_L);
	
	
	fprintf("\nPRESS RIGHT FRONT");
	while(BASE_FRONT_R);
	fprintf("\nRELEASE RIGHT FRONT");
	while(!BASE_FRONT_R);
	
	fprintf("\nPRESS LEFT BACK");
	while(BASE_FRONT_L);
	fprintf("\nRELEASE LEFT BACK");
	while(!BASE_FRONT_L);
	
	fprintf("\nPRESS RIGHT BACK");
	while(BASE_FRONT_R);
	fprintf("\nRELEASE RIGHT BACK");
	while(!BASE_FRONT_L);
	
	fprintf("\nPRESS LOWERING");
	while(ARM_LOWER);
	fprintf("\nRELEASE LOWERING");
	while(!ARM_LOWER);
	
	// BUTTONS
	
	fprintf("\nPRESS J3");
	while(PINJ&0x08);
	fprintf("\nRELEASE J3");
	while(!(PINJ&0x08));
	
	fprintf("\nPRESS J5");
	while(PINJ&0x20);
	fprintf("\nRELEASE J5");
	while(!(PINJ&0x20));
	
	fprintf("\nPRESS J7");
	while(PINJ&0x80);
	fprintf("\nRELEASE J7");
	while(!(PINJ&0x80));
	
	
	fprintf("\nPRESS J6");
	while(PINJ&0x40);
	fprintf("\nRELEASE J6");
	while(!(PINJ&0x40));
	
	cli();
	SREG&=0x7F;
	
	fprintf("\nPRESS D0");
	while(PIND&0x01);
	fprintf("\nRELEASE D0");
	while(!(PIND&0x01));
	emergency_stop=0;
	
	fprintf("\nPRESS H7");
	while(PINH&0x80);
	fprintf("\nRELEASE H7");
	while(!(PINH&0x80));
	
	// ENCODER
	count = 0;
	sei();
	emergency_stop=0;
	while(count < 2000)
	{
		fprintf("\nENCODER FRONT");
		nextline();
		lcd_num(count,"counts");
		_delay_ms(10);
	}
	count = 0;
	fprintf("\nENCODER BACK");
	while(count > -2000)
	{
		fprintf("\nENCODER BACK");
		nextline();
		lcd_num(count,"counts");
		_delay_ms(10);
	}
	
	// RAISING & LOWERING
	
	retry_arm_adjust(100);
	fprintf("\nRAISING");
	arm_adjust(600,200,0);
	fprintf("\nLOWERING");
	
	
	// MOTOR TESTING
	mean1 = 0;
	fprintf("\nCHECK STALL");
	nextline();
	fprintf("PRESS J3");
	while(PINJ&0x08);
	fprintf("\nRELEASE J3");
	while(!(PINJ&0x08));
	
	mean1 = 50;
	fprintf("\nMOTORS FRONT");  
	fprintf("PRESS J3");
	while(PINJ&0x08);
	fprintf("\nRELEASE J3");
	while(!(PINJ&0x08));
	
	mean1 = -50;
	fprintf("\nMOTORS BACK");
	fprintf("PRESS J3");
	while(PINJ&0x08);
	fprintf("\nRELEASE J3");
	while(!(PINJ&0x08));
	
	// angle
	
	fprintf("\nCheck angle");
	nextline();
	fprintf("rotate to 90");
	freewheel = 1;
	while(angle < 90 || angle > 350)
	{
		fprintf("\nANGLE");
		nextline();
		lcd_num(angle*100,"angle");
		_delay_ms(10);
	}
	mean1 = 0;
	
	cli();
}
/// EEPROM functions 
long int eep(long int x,unsigned int *add)
{
	long int w;
	
	eeprom_write_word(add,x);
	_delay_ms(15);
	
	w=eeprom_read_word(add);
	_delay_ms(15);
	
	return w;
}
long int read_fac(unsigned int *add)
{
	long int w;
	
	w=eeprom_read_word(add);
	_delay_ms(15);
	
	return w;
}
/// i(key)pad 
void factor_init(void)
{
	Kp = read_fac((unsigned int*)KP_ADD);
	
	Kd = read_fac((unsigned int*)KD_ADD);
	
	Ki = read_fac((unsigned int*)KI_ADD);
	
	mean1_max = read_fac((unsigned int*)MEAN_MAX_ADD);
	
	Ki_pow = read_fac((unsigned int*)KI_POW_ADD);
	
	/////////////  GYRO  ///////////////
	gp_eep = 500;//read_fac((unsigned int*)GP_ADD);
	
	gd_eep = 0;//read_fac((unsigned int*)GD_ADD);
	
	gi_eep = 0;//read_fac((unsigned int*)GI_ADD);
	
	kp_fuse_front = 30;//read_fac((unsigned int*)KP_FUSE_FRONT_ADD);
	
	kp_fuse_back = 20;//read_fac((unsigned int*)KP_FUSE_BACK_ADD);
	
	boost_delay = 111;//read_fac((unsigned int*)boost_delay_ADD);
	
	retard_delay = 3550;//read_fac((unsigned int*)retard_delay_ADD);
	
	gp_back_eep = 250;//read_fac((unsigned int*)GP_BACK_ADD);
	
	gd_back_eep = 0;//read_fac((unsigned int*)GD_BACK_ADD);
	
	gi_back_eep = 0;//read_fac((unsigned int*)GI_BACK_ADD);
	
	///////////////////  STRATEGY  ////////////////////////
	start_basket = 5400;//read_fac((unsigned int*)START_BASKET_ADD);
	
	basket_mid = 5678;//read_fac((unsigned int*)BASKET_MID_ADD);
	
	mid_wall = 1111;//read_fac((unsigned int*)MID_WALL_ADD);
	
	wall_mid = 666;//read_fac((unsigned int*)WALL_MID_ADD);
	
	mid_start = 1450;//read_fac((unsigned int*)MID_START_ADD);
	
	start_ramp = 1450;//read_fac((unsigned int*)START_RAMP_ADD);
	
	ramp_collector = 13500;//read_fac((unsigned int*)RAMP_COLLECTOR_ADD);
	
	///////////////  SPEED  ////////////////
	start_basket_spd = 130;//read_fac((unsigned int*)START_BASKET_SPD_ADD);
	
	basket_mid_spd = 75;//read_fac((unsigned int*)BASKET_MID_SPD_ADD);
	
	mid_wall_spd = 35;//read_fac((unsigned int*)MID_WALL_SPD_ADD);
	
	wall_mid_spd = 13;//read_fac((unsigned int*)WALL_MID_SPD_ADD);
	
	mid_start_spd = 70;//read_fac((unsigned int*)MID_START_SPD_ADD);
	
	start_ramp_spd = 80;//read_fac((unsigned int*)START_RAMP_SPD_ADD);
	
	ramp_collector_spd = 60;//read_fac((unsigned int*)RAMP_COLLECTOR_SPD_ADD);
	
	var1 = read_fac((unsigned int*)VAR1_ADD);
	
	var2 = read_fac((unsigned int*)VAR2_ADD);
	
	var3 = read_fac((unsigned int*)VAR3_ADD);
	
	var4 = read_fac((unsigned int*)VAR4_ADD);
	
	var5 = read_fac((unsigned int*)VAR5_ADD);
	
	var6 = read_fac((unsigned int*)VAR6_ADD);
	
	var7 = read_fac((unsigned int*)VAR7_ADD);
	
	var8 = read_fac((unsigned int*)VAR8_ADD);
	
	var9 = read_fac((unsigned int*)VAR9_ADD);
	
	var10 = read_fac((unsigned int*)VAR10_ADD);

	gp_eep=gp_eep/100.0;
	gd_eep=gd_eep/100.0;
	
	Kp=Kp/10.0;
	Kd=Kd/10.0;
	Ki = 0;//Ki*pow(10,(Ki_pow*(-1)));
	
	kp_fuse_front/=100;
	kp_fuse_back/=100;

	gp_back_eep=gp_back_eep/100.0;
	gd_back_eep=gd_back_eep/100.0;
}
void clear(void)
{
	xcurr=0;
	ycurr=0;
	count = 0;
	speed = 0;
}
void g_turn(int an,float weight)
{
	trace = 0;
	gyro_on = 1;
	gyro_turn = 1;
	reqa0 = 0;
	clear();
	
	gp=weight;
	
	gd=0;
	gi=0;
	
	reqa = an;		
	mean1 = 0;
	if(an<0)
		an*=(-1);
	
	while(!((shifted_angle > (an - 1.5)) && (shifted_angle < (an + 1.5))))
	{
		lcd_num(totcorr,"t");
	}


	gyro_on=0;
	mean1=0;
	gyro_turn = 0;
}

/// Navigation sub functions
void power_ir(char ch,volatile int pwm)
{
	if(pwm > 230)
		pwm = 230;
		
	if(ch == 'R')
	{
		POWER_OCR = pwm;
		cbi(POWER_PORT,POWER_D);	//direction low for lowering
		sbi(POWER_PORT,POWER_B);	//brake low
	}
	else if(ch == 'L')
	{
		POWER_OCR = pwm;
		sbi(POWER_PORT,POWER_D);	//direction low for lowering
		cbi(POWER_PORT,POWER_B);	//brake low
	}
	else if(ch == 'B')
	{
		POWER_OCR = 0;
		cbi(POWER_PORT,POWER_D);	//direction low for lowering
		cbi(POWER_PORT,POWER_B);	//brake low
	}
}


void arm_adjust(int pos,int sp,int precision)
{
	do
	{
		if(pos<150)
			pos=150;
		
		
		if(!precision)
		{
			if(arm_pos > pos)
			{
				fprintf("\nLOWERING");
				power_ir('L',sp);
			}
			else if(arm_pos < pos)
			{
				fprintf("\nRAISING");
				power_ir('R',sp);
			}
		}
		else
		{
			if(arm_pos > pos)
			{
				sp=((arm_pos-pos)/3) + 40;
				fprintf("\nLOWERING");
				power_ir('L',sp);
			}
			else if(arm_pos < pos)
			{
				sp=((pos-arm_pos)/3) + 65;
				fprintf("\nRAISING");
				power_ir('R',sp);
			}
		}
		lcd_num(arm_pos,"\npos");
		nextline();
		
	}while(((arm_pos-pos)>7)||((pos-arm_pos)>7));
	
	power_ir('B',0);				//PWM 255 'ahem'!!!
}

void lcd_disp(void)
{
	if(debuging)
	{
		lcd_num(l1,"\nl");
		lcd_num(r1,"r");
		lcd_num(angle*100,"a");
		
		nextline();
		lcd_num(xcurr,"x");
		lcd_num(crossline,"cl");
		_delay_ms(5);
	}
}
void till_wall(bool dir_front)
{
	if(hit_flag)
	{
		count = 0;
		
		if(dir_front)
			mean1 = 10;
		else
			mean1 = -10;
		
		freewheel = 1;
		while(BASE_BACK_L||BASE_BACK_R)  //PORT J 3
		{
			if(enough)
				break;
			
			if(!(BASE_BACK_L))
				nondiff_turn = 2;		//l=255;
			if(!(BASE_BACK_R))
				nondiff_turn = 1;		//r=255;
			
			if(count>=0)
				lcd_num(count,"\ncount");
			else
				lcd_num(count,"\n-count");
			_delay_ms(50);
		}
		mean1=0;
		count = 0;
		freewheel = 0;
		nondiff_turn=0;
	}
	enough = 0;
}
void scale_factors(unsigned char direction,int sp)
{
	mean1_max = sp;
	
	if(direction == 'F')
	{
		gp=((sp*gp_eep)/250.0);
		gd=((sp*gd_eep)/250.0);
		gi=((250.0*gi_eep)/(float)sp);
	}
	else if(direction == 'B')
	{
		gp=((sp*gp_back_eep)/250.0);
		gd=((sp*gd_back_eep)/250.0);
		gi=((250.0*gi_back_eep)/(float)sp);
	}
}
void check_for_basket(void)
{
	int init_nb_count = 0;
	init_nbf = false;
	
	for(int i=1; i<=10; i++)
	{
		ADCSRA|=0x40;
		while(ADCSRA&0x40);
		dis=ADCH;
		lcd_num(dis*10,"\ndis");
		if(dis<120)
			init_nb_count++;
			
		_delay_ms(1);
	}
	if(init_nb_count > 6)
		init_nbf = true;
}

//Navigation
void to_the_basket(int roll_back,int roll_side,int sp)
{
	volatile int boost,retard;
	boost=(((float)sp/250.0)*boost_delay);//700
	retard=(((float)sp/250.0)*retard_delay);//1500
	
	basket = 1;

	fusion_flag = true;
	kp_fuse_back = 0.2;
	
	mean1_max=sp;
	mean1=0;
	hit_flag=0;
	
	trace = 0;
	gyro_on = 1;
	
	xdest=-roll_back;
	ydest=roll_side;
	clear();
	
	
	scale_factors('B',sp);
	
	while(mean1>-sp)
	{
		delay_10us(boost_delay);
		mean1--;
	}
	
	while(xcurr>xdest+retard)
		lcd_disp();
	
	gyro_on=1;
	hit_flag=1;
	
	while(((xcurr>xdest) && BASE_BACK_L && BASE_BACK_R))
	{
		if(enough)
			break;
		mean1=(((float)(xdest-xcurr)/(float)retard)*(sp-10)) - 10;
		lcd_disp();
		
	}
	//////////////////////////////////////////////////
	till_wall(false);
	
	//////////////////////////////////////////////////
	fusion_flag = false;
	enough=0;
	gyro_on=0;
	mean1=0;
	basket = 0;
	nondiff_turn=0;
	freewheel = 0;
}

void to_the_second_basket(int roll_back,int roll_side,int end_counts,int sp)
{
	volatile int boost,retard;
	
	basket = 1;
	boost=(((float)sp/250.0)*boost_delay);//700
	retard=(((float)sp/250.0)*retard_delay);//1500

	mean1_max=sp;
	mean1=0;
	
	trace_about = 360 - (57.2957*(atan((float)roll_side/(float)(roll_back))));
	
	trace = 0;
	fusion_flag = false;
	
	gyro_on = 1;
	reqa0 = 1;
	
	clear();
	xdest = -roll_back;
	ydest = 0;
	
	scale_factors('B',sp);
	
	while(mean1>-sp)
	{
		delay_10us(boost_delay);
		mean1--;
	}
	
	basket = 0;
	
	mean1=-sp;
	
	while(xcurr>xdest+1000)
		lcd_disp();
	
	while(xcurr>xdest)
	{
		mean1=((xdest-xcurr)*(sp-30)/1000)-30;
		lcd_disp();
	}
	
	mean1=-30;
	//////////////////////////////////////
	gyro_on=0;
	nondiff_turn = 2;
	
	while(!(angle > 357 || angle < 3));
	
	nondiff_turn = 0;
	trace_about = 0;

	gyro_on=1;
	///////////////////////////////////////
	fusion_flag = true;
	clear();
	xdest = -end_counts;
	
	while(xcurr>-300)
	{
		mean1=(xcurr*60/300)-5;
		lcd_disp();
	}
	
	mean1=-65;
	while(xcurr>xdest && BASE_BACK_L && BASE_BACK_R )
	{
		mean1=((xdest-xcurr)*65/end_counts)-5;
		lcd_disp();
		
		if(enough)
			break;
	}
	//////////////////////////////////////////////////
	
	till_wall(false);
	
	//////////////////////////////////////////////////
	gyro_on=0;
	mean1=0;
	fusion_flag = false;
	trace_about=0;
}
void pick_the_basket(void)
{
	mean1 = 0;
	
	/////////////	RAISING 	/////////////////
	fprintf("\nRAISING BASKET");
	arm_adjust(800,240,0);	

	/////////////	ACTVATING PISTON 	////////////////////////
	fprintf("\nACTIVATING PISTON");
	cbi(PISTONA_PORT,PISTONA);//PISTON CLOSED
	sbi(PISTONB_PORT,PISTONB);
}

void gyro_back(int roll_back,int roll_side,int sp)
{
	volatile int boost,retard;
	boost=(((float)sp/250.0)*2000);//700
	retard=(((float)sp/250.0)*2500);//1500
	
	if((boost+retard)>roll_back)
		boost=retard=(roll_back/2);
	
	trace = 0;
	gyro_on = 1;
	xdest=-roll_back;
	ydest=roll_side;
	clear();
	mean1=0;
	
	scale_factors('B',sp);
	
	mean1 = -10;
	while(xcurr > -boost)
	{
		mean1=(((sp+5)*xcurr)/boost)-5;
		lcd_disp();
	}	
	mean1=-sp;
	
	while((xcurr>xdest+retard))
		lcd_disp();

	while(xcurr>xdest  && BASE_BACK_L && BASE_BACK_R)
	{
		mean1=((xdest-xcurr)*sp/retard)-3;
		lcd_disp();
	}
	//////////////////////////////////////////////////
	
	till_wall(false);
	//////////////////////////////////////////////////

	gyro_on=0;
	mean1=0;
	enough=0;
}

void gyro_front(int roll_front,int roll_side,int sp)
{
	volatile int boost,retard;//,kp_fuse_front_temp;
	boost=(((float)sp/250.0)*1000);
	retard=(((float)sp/250.0)*1500);
	
	trace = 0;
	xdest=roll_front;
	ydest=roll_side; 
	clear();
	mean1=0;
	reqa0=1;
	
	gyro_on=1;
	//fusion_flag=false;

	scale_factors('F',sp);
	//kp_fuse_front_temp = kp_fuse_front;
	//kp_fuse_front = 0.1;
	
	while(xcurr<boost)
	{
		mean1 = ((xcurr*sp)/boost) + 3;
		
		lcd_disp();
		if(!(ARM_LOWER))
		{
			cbi(POWER_PORT,POWER_D);
			cbi(POWER_PORT,POWER_B);
			POWER_OCR=0;
			arm_pos=150;
		}
	}
	gyro_on=1;
	//fusion_flag=true;
	gp=0;
	mean1=sp;
	while(xcurr<(xdest-retard))
	{
		lcd_disp();
		if(!(ARM_LOWER))
		{
			cbi(POWER_PORT,POWER_D);
			cbi(POWER_PORT,POWER_B);
			POWER_OCR=0;
			arm_pos=150;
		}
	}
	
	while((xcurr<xdest)&&(BASE_FRONT_L)&&(BASE_FRONT_R))
	{
		mean1 = (((xdest-xcurr)*sp)/retard) + 3;
		if(enough)
			break;
		if(!(ARM_LOWER))
		{
			cbi(POWER_PORT,POWER_D);
			cbi(POWER_PORT,POWER_B);
			POWER_OCR=0;
			arm_pos=150;
		}
		lcd_disp();
	}
	fusion_flag=false;
	//////////////////////////////////////////////////
	till_wall(true);
	//////////////////////////////////////////////////
	enough = 0;
	//kp_fuse_front = kp_fuse_front_temp;
	gyro_on  = 0;
	mean1 = 0;
}
void with_the_basket(int roll_front,int roll_side,int sp)
{
	volatile int boost,retard,loop_executed=0,no_basket_count=0,xcurr_temp;
	
	boost=(((float)sp/250.0)*3300);
	retard=(((float)sp/250.0)*1800);
	
	if((boost+retard)>roll_front)
		boost=retard=(roll_front/2.0);
	
	mean1_max=sp;
	trace = 0;
	gyro_on = 1;
	xdest=roll_front;
	ydest=roll_side; 
	clear();
	mean1=0;
	
	scale_factors('F',sp);
	
	while(xcurr<boost)
	{
		mean1 = ((xcurr*(sp-10)/boost) + 10);
		
		lcd_disp();
	}

	mean1=sp;
	nbf = false;
	
	//selecting ADMUX channel 12
	ADMUX = 0xE4;
	
	while(xcurr<(xdest-retard))
	{
		if( (!loop_executed) && (xcurr>2500))
		{
			for(int i=1; i<=10; i++)
			{
				ADCSRA|=0x40;
				while(ADCSRA&0x40);
				dis=ADCH;
				lcd_num(dis*10,"\ndis");
				if(dis<120)
					no_basket_count++;
					
				_delay_ms(1);
			}
			loop_executed = 1;
			
			if(no_basket_count > 9)
			{
				nbf = true;
				break;
			}
		}
		lcd_disp();
	}
	
	//if initally no basket or currently no basket
	
	if(init_nbf || nbf)
	{
		if(attempting_basket == 1) //then attempt for 1
		{	
			//attempt for basket 2 now
			attempting_basket = 2;
			
			xcurr_temp=xcurr;
			
			
			while((xcurr-xcurr_temp)<1000)
			{
				mean1=(((1000-(xcurr-xcurr_temp))*(sp-15))/1000.0) + 15;
				lcd_disp();
			}
			
			mean1=0;
			
			//if initially no basket then do not lower arm
			if(!init_nbf)
			{
				arm_adjust(480,120,0);
				
				sbi(PISTONA_PORT,PISTONA);//PISTON OPEN
				cbi(PISTONB_PORT,PISTONB);
			}
			reqa0=1;
			gyro_turn=0;
			trace=0;
			gyro_on=0;
			
			//"var1) init angle",
			//"var2) mid counts",
			//"var3) end counts",
			//"var4) turn spd",
			//"var5) back spd",
			
			//************	TURN  *****************
			mean1 = -30;
			nondiff_turn = 1; // left behind
			gyro_on = 0;
			while(angle < 25 || angle > 335) // 13 to 0 to var1
				lcd_disp();
			nondiff_turn = 0;
			
			//************	BACK  *****************
			trace_about = 335;
			//gyro_back(var2,0,var5);
			gyro_on = 1;
			mean1 = -50;
			clear();
			while(xcurr>-2750)
				lcd_disp();
				
			//************	TURN  *****************
			mean1 = -30;
			gyro_on = 0;
			nondiff_turn = 2; // right behind
			while(angle > 3) // 360 to 0
				lcd_disp();
			trace_about = 0;
			nondiff_turn = 0;
			
			//************	BACK  *****************
			fusion_flag = 1;
			hit_flag = 1;
			//gyro_back(var3,0,var5);
			gyro_on = 1;
			mean1 = -25;
			clear();
			while(xcurr>-500 && BASE_BACK_L && BASE_BACK_R)
				lcd_disp();
			fusion_flag = 0;
			till_wall(false);
			//~//~//~//~//~//~//~//~//~//~//~//~//~//picking basket
			
			check_for_basket();
			
			if(!init_nbf)
			pick_the_basket();
			
			//~//~//~//~//~//~//~//~//~//~//~//~//~//diagonal after picking basket
			
			fprintf("\nGoing Diagonal");
			reqa0=1;
			trace_about = 347;//(360-((float)var7/100.0));
			
			with_the_basket(basket_mid,0,basket_mid_spd);
		}
		else if(attempting_basket == 2) // then attempt for 2
		{
			//attempt for basket 1 now
			attempting_basket = 1;
			
			if(nbf && !init_nbf)
			{
				mean1 = 0;
				fprintf("\nNO BASKET");
				while(1);
			}
			
			xcurr_temp=xcurr;
			
			
			//var 4) turn_speed
			//var 8) retard
			//var 9) back_counts
			//var 10) back_speed
			
			
			while((xcurr-xcurr_temp)<2000)
			{
				mean1=(((2000-(xcurr-xcurr_temp))*(sp-15))/2000) + 15;
				lcd_disp();
			}
			
			mean1=0;
			
			//if initially no basket then do not lower arm
			if(!init_nbf)
			{
				arm_adjust(480,120,0);
				
				sbi(PISTONA_PORT,PISTONA);//PISTON OPEN
				cbi(PISTONB_PORT,PISTONB);
			}
			
			
			reqa0=1;
			gyro_turn=0;
			trace=0;
			gyro_on=0;
			
			//*****  TURN  ****
			mean1 = -30;
			nondiff_turn = 2;	//right behind
			while(angle > 10)	//350 to 0 
				lcd_disp();
			nondiff_turn = 0;
				
			//**** BACK  *****
			trace_about = 0;
			gyro_on = 1;
			fusion_flag = true;
			hit_flag = 1;
			
			gyro_back(3750,0,75);
			
			hit_flag = 0;
			fusion_flag = false;
			//~//~//~//~//~//~//~//~//~//~//~//~//~//picking basket
			
			check_for_basket();
			
			if(!init_nbf)
				pick_the_basket();
			
			//~//~//~//~//~//~//~//~//~//~//~//~//~//diagonal after picking basket
			reqa0 = 0;
			with_the_basket(basket_mid,mid_wall,basket_mid_spd);	
		
		}
	}
	else
	{
		while((xcurr<xdest)&&(BASE_FRONT_L)&&(BASE_FRONT_R))
		{
			mean1 = (((xdest-xcurr)*sp)/retard) + 3;
			lcd_disp();
		}
		//////////////////////////////////////////////////
		//////////////////////////////////////////////////
		gyro_on  = 0;
		mean1 = 0;
	}
}
void release_basket(void)
{
	mean1 = 0;
	/////////////	DEACTVATING PISTON 	////////////////////////
	
	/*
	if(!start_basket)//slow
	{
		_delay_ms(50);
		sbi(PISTON_PORT,PISTONA);//PISTON OPEN
		cbi(PISTON_PORT,PISTONB);
		fprintf("\nDEACTIVATING PISTON");
		
		/////////////	LOWERING 	///////////////////////////////
		fprintf("\nLOWERING BASKET");
		
		arm_adjust(200,150,0);
	}
	//else//fast
	//{
		sbi(PISTON_PORT,PISTONA);//PISTON OPEN
		cbi(PISTON_PORT,PISTONB);
		fprintf("\nDEACTIVATING PISTON");*/
		
		
		sbi(MAGNET_PORT,MAGNET1);
		sbi(MAGNET_PORT,MAGNET2);
		
		POWER_OCR = 0;
		cbi(POWER_PORT,POWER_D);	//direction low for lowering
		cbi(POWER_PORT,POWER_B);	//brake low
	//
}
void clamp_collector(void)
{
	int choice1;
	if(!(collector_clamped))
	{
		retry_arm_adjust(80);
		
		//RAISE TILL HIT SWITCH RELEASE
		POWER_OCR = 175;
		cbi(POWER_PORT,POWER_B);	//BRAKE LOW
		cbi(POWER_PORT,POWER_D);	//RAISING
		
		while(!ARM_LOWER);
		arm_pos = 150;
		
		arm_adjust(400,230,1);		//380
		
		while((PINE&_BV(5))&&(PINJ&_BV(3)))
		{
			fprintf("\nTOGGLE MAGNET:J7");
			nextline();
			lcd_num(arm_pos,"pos");
			
			if(!(PINJ&_BV(7)))
			{
				while(!(PINJ&_BV(7)));
				_delay_ms(100);
				
				MAGNET_PORT^=((1<<MAGNET1)|(1<<MAGNET2));	
			}
			choice1  = return_key();
			if(choice1 == 2)
			{
				sbi(POWER_PORT,POWER_B);	//brake low	for inside
				cbi(POWER_PORT,POWER_D);	//direction low for raising
				POWER_OCR=150;
			}
			else if(choice1 == 8)
			{
				cbi(POWER_PORT,POWER_B);	//brake low	for inside
				sbi(POWER_PORT,POWER_D);	//direction high for lowering
				POWER_OCR=150;
			}
			else
			{
				POWER_OCR=0;
				cbi(POWER_PORT,POWER_B);	//brake
				cbi(POWER_PORT,POWER_D);	//brake
			}
		}
		sbi(MAGNET_PORT,MAGNET1);
		sbi(MAGNET_PORT,MAGNET2);
	}
	
	fprintf("\nLocked and loaded!!");
	collector_clamped = 1;
}


void to_the_ramp(int roll_back,int roll_side,int sp)
{
	int emergency_xcurr = -4900;
	int tspeed=60;
	volatile int temp_xdest,retard;
	
	mean1_max=sp;
	
	retard = roll_back;
	
	trace_about = 353;
	
	xdest = -9000;
	ydest = roll_side;
	clear();
	reqa0 = 1;
	mean1 = 0;
	/********************************************************************/
	gp = ((sp*8.5)/250.0);//8.5
	gd = 0;
	gi = 0; 
	
	trace = 0;
	gyro_on = 1;
	fusion_flag=false;
	
	crossline = 0;
	while((mean1>-sp) && (xcurr > emergency_xcurr))
	{
		if(xcurr>0)
			mean1=-5;
		else
			mean1=((xcurr*(sp+5))/500.0)-5;
		lcd_disp();
	}
	mean1=-sp;

	while((crossline<4) && (xcurr > emergency_xcurr))//(xcurr>xdest+retard)
	{
		lcd_disp();
		if(debuging)
		lcd_num(crossline,"cl");
	}
	
	temp_xdest = xcurr - retard;
	
	while(xcurr > (temp_xdest))
	{
		mean1 = ((temp_xdest - xcurr)/(float)retard)*(sp-tspeed) - tspeed;
		lcd_disp();
	}
	
	freewheel=0;
	nondiff_turn=2;
	mean1=-tspeed;
	gyro_on = 0;
	
	while(angle>340||angle<55)
	{
		lcd_disp();
		if(debuging)
			fprintf("in turn");
	}	
	
	while(angle<80)
	{
		mean1=(((angle-80)/25.0)*(tspeed-10))-10;
		lcd_disp();
		if(debuging)
			fprintf("in turn");
	}
	
	while(angle<87);
	nondiff_turn = 0;
	//////////////////////////////////////////////////
	gyro_on = 0;
	mean1 = 0;
}

void final_frontier(int roll_front,int roll_side,int sp)//LINE TRACING
{
	volatile int boost,retard,reduced_spd,increased_spd;
	boost=(((float)sp/250.0)*2000);
	retard=(((float)sp/250.0)*4000);
	
	if((boost+retard)>roll_front)
		boost=retard=(roll_front/2);
	
	mean1_max=sp;
	
	reduced_spd = 35;
	increased_spd = sp+30;
	
	fusion_flag=true;
	gyro_on=1;
	trace=0;
	
	reqa0 = 1;
	mean1 = 0;
	clear();
	
	scale_factors('F',sp);
	
	/**
	 *if(!((angle>80)&&(angle<100)))
	 *{
	 *	mean1=0;
	 *	while(1)
	 *	{
	 *	lcd_num(l1,"\nl");
	 *	lcd_num(r1,"r");
	 *	nextline();
	 *	lcd_num(fuse_error,"fe");
	 *	lcd_num(temp2,"t2");
	 *	lcd_num(angle,"a");
	 *	_delay_ms(5);
	 *	}
	 *}
	 */

	/*while(xcurr<300)
	{
		if(xcurr<0)
			mean1=10;
		else
			mean1=((reduced_spd-10)*xcurr/300)+10;
		//lcd_disp();
		lcd_num(l1,"\nl");
		lcd_num(r1,"r");
		nextline();
		lcd_num(fuse_error,"fe");
		lcd_num(temp2,"t2");
		lcd_num(angle,"a");
		_delay_ms(5);
	}*/
	
	crossline=0;
	//mean1=reduced_spd;
	while(crossline<1)
	{
		if(mean1<reduced_spd)
			mean1=((reduced_spd-10)*xcurr/300)+10;
		lcd_disp();
	}
	
	mean1=reduced_spd;
	xdest=roll_front;
	ydest=roll_side; 
	clear();
	
	scale_factors('F',sp);
	
	while(xcurr<700)		//Acc
	{
		mean1=((((sp-(reduced_spd))*xcurr)/700)+reduced_spd);
		lcd_disp();
	}
	
	mean1 = sp;
	
	while(xcurr<2000)
	{
		if(xcurr<1400)
			crossline=0;
		else
			if(crossline==1)
				xcurr=2000;
			
		lcd_disp();
	}
	
	while(xcurr<2700)
		lcd_disp();
	mean1=reduced_spd-7;
	
	while(xcurr<3800)
	{
		if(xcurr>3300)
			mean1=reduced_spd;
		lcd_disp();
	}
	
	scale_factors('F',increased_spd);
	
	while(xcurr<4500)
	{
		mean1=((((increased_spd-(reduced_spd))*(xcurr-3800))/700)+reduced_spd);
		lcd_disp();
	}
	
	mean1=increased_spd;

	while(xcurr<5300)		//Const on Ramp
		lcd_disp();

	while(xcurr<6000)
	{
		mean1=(((increased_spd-reduced_spd)*(6000-xcurr)/700)+reduced_spd);
		lcd_disp();
	}
	scale_factors('F',reduced_spd);
	//gp=0;
	//kp_fuse_front = 0.3;
	
	mean1 = reduced_spd;
	while(xcurr<6500)
		lcd_disp();
	
	scale_factors('F',sp);
	//gp=0;
	
	while(xcurr<7000)
	{
		mean1=((((sp-(reduced_spd))*(xcurr-6500))/500)+reduced_spd);
		lcd_disp();
	}
	
	mean1=sp;
	
	while(xcurr<7700)		//Down the Ramp
		lcd_disp();
	mean1=reduced_spd;
	while(xcurr<8200)
		lcd_disp();
	
	scale_factors('F',increased_spd);
	//gp=0;
	while(xcurr<9600)
	{
		mean1=(((increased_spd-reduced_spd)*(xcurr-8200)/1400)+reduced_spd);
		lcd_disp();
		lcd_num(fuse_error,"fe");
		if(xcurr<8700)
			crossline=0;
	}
	
	mean1 = increased_spd;
	
	while(crossline<4)
	{
		lcd_disp();
	}
	
	xcurr = 0;
	ycurr = 0;
	clear();
	xdest = 1700;
	
	gyro_on = 1;
	trace=0;
	reqa0=1;
	
	
	//selecting ADMUX channel 11
	ADMUX = 0xE3;
	
	bool save_collector = false;

	while(((xcurr<xdest))&&(BASE_FRONT_L)&&(BASE_FRONT_R))
	{
		ADCSRA|=0x40;
		while(ADCSRA&0x40);
		dis=ADCH;
		
		if(xcurr > 1000 && dis < 80) // if machine is one block behind
		{
			save_collector = true;
			break;
		}
		
		if(xcurr>1150)
		{
			cbi(MAGNET_PORT,MAGNET1);
			cbi(MAGNET_PORT,MAGNET2);
		}
		if(!(ARM_LOWER))
		{
			cbi(POWER_PORT,POWER_D);
			cbi(POWER_PORT,POWER_B);
			POWER_OCR=0;				//PWM 255 'ahem'!!!
			arm_pos=150;
		}
		else if(arm_pos>155 && xcurr>1400)
		{
			POWER_OCR=175;
			cbi(POWER_PORT,POWER_B);	//brake low
			sbi(POWER_PORT,POWER_D);	//direction low for lowering
			
		}
		mean1 = (((xdest-xcurr)*(increased_spd-10))/1700) + 10;
		lcd_disp();
	}
	if(save_collector)
	{
		mean1 = 30;
		clear();
		while((BASE_FRONT_L)&&(BASE_FRONT_R))
			lcd_disp();
		
		//release magnet
		cbi(MAGNET_PORT,MAGNET1);
		cbi(MAGNET_PORT,MAGNET2);
		
		//start lowering
		POWER_OCR=200;
		cbi(POWER_PORT,POWER_B);	//brake low
		sbi(POWER_PORT,POWER_D);	//direction low for lowering
		
		//wait till arm lowers
		while(!(ARM_LOWER));
		
		//break arm lowering
		cbi(POWER_PORT,POWER_D);
		cbi(POWER_PORT,POWER_B);
		POWER_OCR=0;
		arm_pos=150;
		
		
	}
	
	fusion_flag=false;
	gyro_on = 0;
	trace=0;
	/////////////////////////////////////////////////
	
	till_wall(true);
	
	/////////////////////////////////////////////////
	
	reqa0 = 1;
	gyro_on = 0;
	trace=0;
	mean1 = 0;
}
void line_front(int roll_front,int sp)
{
	volatile int boost,retard;
	boost=(((float)sp/250.0)*1000);
	retard=(((float)sp/250.0)*4000);
	
	if((boost+retard)>(roll_front))
		boost=retard=roll_front/2.0;
	
	reqa0=1;
	trace = 0;
	gyro_on = 1;
	clear();
	crossline=0;
	mean1=10;
	while(crossline<1);
	
	fusion_flag=true;

	while(count<boost)
	{
		mean1 = ((count*(sp-10))/(float)boost) + 10;
		
		lcd_disp();
		if(debuging)
		{
			lcd_num(boost,"\nb");
			lcd_num(retard,"r");
		}
		
	}
	mean1 = sp;
	while(count<2000)
	{
		lcd_disp();
		if(debuging)
		lcd_num(fuse_error,"fe");
	}

	clear();
	xdest = 1700;
	
	gyro_on = 1;
	trace=0;
	
	while(((xcurr<xdest))&&(BASE_FRONT_L)&&(BASE_FRONT_R))
	{
		if(xcurr>1150)
		{
			cbi(MAGNET_PORT,MAGNET1);
			cbi(MAGNET_PORT,MAGNET2);
		}
		if(!(ARM_LOWER))
		{
			cbi(POWER_PORT,POWER_D);
			cbi(POWER_PORT,POWER_B);
			POWER_OCR=0;				//PWM 255 'ahem'!!!
			arm_pos=150;
		}
		else if(arm_pos>155 && xcurr>1400)
		{
			POWER_OCR=175;
			cbi(POWER_PORT,POWER_B);	//brake low
			sbi(POWER_PORT,POWER_D);	//direction low for lowering
			
		}
		mean1 = (((xdest-xcurr)*(sp-10))/1700) + 10;
		lcd_disp();
	}

	fusion_flag=false;
	till_wall(true);
	
	trace=0;
	gyro_on=0;
	mean1 = 0;
}

void go_collector_go(void)
{
	_delay_ms(50);
	cbi(MAGNET_PORT,MAGNET1);
	cbi(MAGNET_PORT,MAGNET2);
	_delay_ms(50);
	
	///////////////////////////////////////////////////
	if(arm_pos >= 0)
		lcd_num(arm_pos,"\narm pos");
	else
		lcd_num(-arm_pos,"\n-arm pos");
	
	fprintf("\nLOWERING COLLECTOR");
	
	retry_arm_adjust(175);
}
void back_to_operator(void)
{
	trace=0;
	gyro_on=1;
	reqa0=1;
	clear();
	arm_adjust(400,150,1);
	trace_about=90;
	hit_flag=0;
	
	mean1 = -20;
	while(xcurr > -500);
	gyro_on=0;

	mean1 = 25;
	diff_turn = 1;
	while(angle < 267);
	diff_turn = 0;
	
	gyro_on=1;
	trace_about = 270;
	
	fusion_flag = true;
	gyro_front(ramp_collector+1000,0,50);
	fusion_flag = false;
	
	_delay_ms(100);
	//g_turn(-270,1.22);
	gyro_on=0;

	mean1 = 25;
	nondiff_turn = 1;
	while(angle>3);
	nondiff_turn=0;
	_delay_ms(100);
	gyro_on=1;
	trace_about=0;
	hit_flag=1;
	
	fusion_flag = true;
	gyro_front(start_ramp+3000,0,50);
	fusion_flag = false;
	
	hit_flag=0;
}
void full_nav(void)
{
	int adjustment=0,retry_flag=0;
	
	if(emergency_stop)
	{
		////// this will make it free
		cbi(PORTD,5);
		
		cbi(POWER_PORT,POWER_B);
		cbi(POWER_PORT,POWER_D);
		POWER_OCR=0;
		
		sbi(PISTONA_PORT,PISTONA);//PISTON CLOSED
		cbi(PISTONB_PORT,PISTONB);
		
		emergency_stop=0;
		int_counter=0;
		
		retry_clear();
		factor_init();
		retry_flag=1;
		mean1 = 0;
		
		sei();
	}
		
	
	while(1)
	{
		
		lcd_num(angle*100,"\nang");
		_delay_ms(1);
		
		if( (!(PINJ&0x08)) )	//J3
		{
			choice=1;
			break;
		}
		if(!(PINJ&0x20))	//J5
		{
			choice=2;
			break;
		}
		if(!(PINH&0x80))	//H7
		{
			choice=4;
			break;
		}		
		if(!(PINJ&0x80))	//J7
		{
			choice=3;
			break;
		}
		if(!(PINJ&0x40))
		{
			retry_arm_adjust(100);
			arm_adjust(480,120,0);	//520
		}
		adjustment = return_key();
		
		if(adjustment == 2)
		{
			/////////////	RAISING 	///////////////////////////////
			POWER_OCR=150;
			sbi(POWER_PORT,POWER_B);	//brake low	for inside
			cbi(POWER_PORT,POWER_D);	//direction high for raising
		}
		else if(adjustment == 8)
		{
		/////////////	LOWERING 	///////////////////////////////
			cbi(POWER_PORT,POWER_B);	//brake low	for inside
			sbi(POWER_PORT,POWER_D);	//direction low for lowering
			POWER_OCR=150;	
		}
		else
		{
			POWER_OCR=0;
			cbi(POWER_PORT,POWER_B);	//brake
			cbi(POWER_PORT,POWER_D);	//brake
		}
		
		if(adjustment==1)
		{
			cbi(PISTONA_PORT,PISTONA);//PISTON CLOSED
			sbi(PISTONB_PORT,PISTONB);
		}
		else if(adjustment==3)
		{
			sbi(PISTONA_PORT,PISTONA);//PISTON OPEN
			cbi(PISTONB_PORT,PISTONB);
		}
		else if(adjustment==7)
		{
			cbi(MAGNET_PORT,MAGNET1);
			cbi(MAGNET_PORT,MAGNET2);
		}
		else if(adjustment==9)
		{
			sbi(MAGNET_PORT,MAGNET1);
			sbi(MAGNET_PORT,MAGNET2);
		}
	}
	mean1=0;
	sbi(PORTD,5);

	fprintf("\nRELEASE ME");
	while((!(PINJ&0x08)) || (!(PINJ&0x20)) || (!(PINJ&0x80)));
	
	if(retry_flag)
		trace_about=angle;
	
	if(choice == 1)
	{		
		attempting_basket = 1;
		
		reqa0=1;
		to_the_basket(start_basket,0,start_basket_spd);//start_basket_spd
		trace_about = 0;
		hit_flag=0;
		//~//~//~//~//~//~//~//~//~//~//~//~//~//picking basket
		
		check_for_basket();
		// sets the init_nbf (initially no basket flag) 
		
		if(!init_nbf)
			pick_the_basket();
		
		//~//~//~//~//~//~//~//~//~//~//~//~//~//diagonal after picking basket
		
		fprintf("\nGoing Diagonal");
		reqa0=0;
		
		with_the_basket(basket_mid,mid_wall,basket_mid_spd);		
		
		trace_about = 0;
		
		fast_turn = 1;
		g_turn(277,0.7);
		fast_turn = 0;
		
		trace_about = 270;
		
		sbi(PISTONA_PORT,PISTONA);//PISTON OPEN
		cbi(PISTONB_PORT,PISTONB);
		fprintf("\nDEACTIVATING PISTON");
		
		if(attempting_basket==1)
			POWER_OCR=100;
		else
			POWER_OCR=70;
		
		sbi(POWER_PORT,POWER_D);	//direction low for lowering
		cbi(POWER_PORT,POWER_B);	//brake low
		
		hit_flag=1;
		nondiff_turn = 0;
		diff_turn = 0;
		count = 0;
		mean1 = -35;
		
		freewheel = 1;
		
		while((BASE_FRONT_L)||(BASE_FRONT_R))
		{
			if(enough)
				break;
			if(!(BASE_FRONT_L))
				nondiff_turn = 2;		//l=255;
			if(!(BASE_FRONT_R))
				nondiff_turn = 1;		//r=255;
			lcd_num(count,"\ncount");
			
			if(arm_pos<400)
				break;
		}
		
		enough=0;
		mean1=0;
		count = 0;
		freewheel = 0;
		nondiff_turn=0;
		release_basket();
		hit_flag=0;
		
		//~//~//~//~//~//~//~//~//~//~//~//~//~//back after keeping basket
		
		//~//~//~//~//~//~//~//~//~//~//~//~//~// 90 after keeping basket
		
		fprintf("\nGOING FRONT");
		xcurr=0;
		mean1=wall_mid_spd;
		while(xcurr<wall_mid)
		{
			lcd_num(xcurr,"\nxc");
			_delay_ms(1);
		}
		crossline=0;
		mean1=0;
		fast_turn = 1;
		
		reqa0 = 0;
		trace_about = 270;
		g_turn(-87,1.9);//LEFT STALLED , RIGHT FORWARD (left fwd 90)
		
		trace_about = 0;
		fast_turn = 0;
		//~//~//~//~//~//~//~//~//~//~//~//~//~//back after keeping basket
		
		hit_flag=1;
		fprintf("\nGOING FRONT");
		enough = 0;
		POWER_OCR=40;
		cbi(POWER_PORT,POWER_B);	//brake low
		sbi(POWER_PORT,POWER_D);	//direction low for lowering
		
		if(attempting_basket == 1)
			gyro_front(mid_start,0,mid_start_spd);//2375,70
		else
			gyro_front(1300,0,mid_start_spd);
			
		hit_flag=0;
		
	}
	
	
	else if(choice == 2)
	{
		attempting_basket = 2;
		
		hit_flag=1;
		reqa0=1;
		
		
		to_the_second_basket(5555,3333,2600,100);
		
		hit_flag=0;
		
		//~//~//~//~//~//~//~//~//~//~//~//~//~//picking basket
		
		check_for_basket();
		
		if(init_nbf)
			trace_about=330;
		else
		{
			pick_the_basket();
			trace_about = 347;
		}
		
		//~//~//~//~//~//~//~//~//~//~//~//~//~//diagonal after picking basket
		
		fprintf("\nGoing Diagonal");
		reqa0=1;
		
		with_the_basket(basket_mid,0,basket_mid_spd);	
		
		reqa0=0;
		trace_about=0;
		
		fast_turn = 1;
		g_turn(277,0.7);
		fast_turn = 0;
		
		trace_about=270;
		
		sbi(PISTONA_PORT,PISTONA);//PISTON OPEN
		cbi(PISTONB_PORT,PISTONB);
		fprintf("\nDEACTIVATING PISTON");
		if(attempting_basket == 2)
			POWER_OCR = 70;//135
		else
			POWER_OCR = 100;
		sbi(POWER_PORT,POWER_D);	//direction low for lowering
		cbi(POWER_PORT,POWER_B);	//brake low
		
		hit_flag=1;
		count = 0;
		mean1 = -mid_wall_spd;
		
		freewheel = 1;
		
		while((BASE_FRONT_L)||(BASE_FRONT_R))
		{
			if(enough)
				break;
			if(!(BASE_FRONT_L))
				nondiff_turn = 2;		//l=255;
			if(!(BASE_FRONT_R))
				nondiff_turn = 1;		//r=255;
			lcd_num(count,"\ncount");
			
			if(arm_pos<400)
				break;
		}
		
		enough=0;
		mean1=0;
		count = 0;
		freewheel = 0;
		nondiff_turn=0;
		release_basket();
		hit_flag=0;
		
		//~//~//~//~//~//~//~//~//~//~//~//~//~//back after keeping basket
	  	
		fprintf("\n90 turn");
		fprintf("\nGOING FRONT");
		xcurr=0;
		mean1=wall_mid_spd;
		while(xcurr<wall_mid)
		{
			lcd_num(xcurr,"\nxc");
			_delay_ms(1);
		}
		
		crossline=0;
		//mean1=15;
		/*while(flagl)
		{
			lcd_num(flagl,"\ncl");
			_delay_ms(1);
		}*/
		mean1=0;
		fast_turn = 1;
		g_turn(-87,1.9);//LEFT STALLED , RIGHT FORWARD (left fwd 90)
		fast_turn = 0;
		//~//~//~//~//~//~//~//~//~//~//~//~//~//back after keeping basket
		
		trace_about = 0;
		hit_flag=1;
		fprintf("\nGOING FRONT");
		enough = 0;
		POWER_OCR=40;
		cbi(POWER_PORT,POWER_B);	//brake low
		sbi(POWER_PORT,POWER_D);	//direction low for lowering
		
		if(attempting_basket == 2)
			gyro_front(1300,0,mid_start_spd);//2375,70
		else
			gyro_front(mid_start,0,mid_start_spd);//2375,70
		
		hit_flag=0;		
	}
	//~//~//~//~//~//~//~//~//~//~//~//~//~//~//
	
	if(choice != 4)
	{
		clamp_collector();
		fprintf("\nin full nav");
		//~//~//~//~//~//~//~//~//~//~//~//~//~//
		
		to_the_ramp(start_ramp,0,start_ramp_spd);
	}	
	hit_flag = 1;
	trace_about = 90 ;
	fprintf("\nCLIMBING RAMP");
	if(choice != 4)
		final_frontier(ramp_collector,0,ramp_collector_spd);
	else
	{
		clamp_collector();
		line_front(3700,60);
	}
	
	hit_flag = 0;
	
	//~//~//~//~//~//~//~//~//~//~//~//~//~//
	mean1 = 0;
	
	go_collector_go();
	collector_clamped=0;
	fprintf("\nTask Complete!");
	nextline();
	fprintf("PRESS KEY TO GO BACK");
	get_key();
	_delay_ms(500);
	back_to_operator();
	fprintf("\nBACK TO START ZONE");
	//~//~//~//~//~//~//~//~//~//~//~//~//~/*/
}


void retry_arm_adjust(int pwm_spd) 
{
	POWER_OCR=pwm_spd;
	cbi(POWER_PORT,POWER_B);	//brake low
	sbi(POWER_PORT,POWER_D);	//direction low for lowering
	while(ARM_LOWER)
	{
		//lcd_num(arm_pos,"\narm_pos");
		//_delay_ms(1);
	}
	
	cbi(POWER_PORT,POWER_D);
	cbi(POWER_PORT,POWER_B);
	POWER_OCR=0;				//PWM 255 'ahem'!!!
	arm_pos=150;
	//eep(arm_pos,(unsigned int*)COLOUR_ADD);
}

int main(void)
{
	init_all();
	UDR1='M';
	while(!(UCSR1A&0x20));
	mean1=0; 
	sei();
	
	main_menu();
	retry_arm_adjust(80);
	full_nav();


	return 0;
}