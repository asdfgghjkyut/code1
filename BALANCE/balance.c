#include "balance.h"
#include "usbh_hid_joy.h"

//int Time_count=0; //Time variable //¼ÆÊ±±äÁ¿

u32 Buzzer_count1 = 0;

// Robot mode is wrong to detect flag bits
//»úÆ÷ÈËÄ£Ê½ÊÇ·ñ³ö´í¼ì²â±êÖ¾Î»
int robot_mode_check_flag=0; 

short test_num;
u8 command_lost_count=0;//

Encoder OriginalEncoder; //Encoder raw data //±àÂëÆ÷Ô­Ê¼Êı¾İ     

//========== PWMÇå³ıÊ¹ÓÃ±äÁ¿ ==========//
u8 start_check_flag = 0;//±ê¼ÇÊÇ·ñĞèÒªÇå¿ÕPWM
u8 wait_clear_times = 0;
u8 start_clear = 0;     //±ê¼Ç¿ªÊ¼Çå³ıPWM
u8 clear_done_once = 0; //Çå³ıÍê³É±êÖ¾Î»
u16 clear_again_times = 0;
float debug_show_diff = 0;
void auto_pwm_clear(void);
volatile u8 clear_state = 0x00;
/*------------------------------------*/


/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
º¯Êı¹¦ÄÜ£ºÔË¶¯Ñ§Äæ½â£¬¸ù¾İÈıÖáÄ¿±êËÙ¶È¼ÆËã¸÷³µÂÖÄ¿±ê×ªËÙ
Èë¿Ú²ÎÊı£ºXºÍY¡¢ZÖá·½ÏòµÄÄ¿±êÔË¶¯ËÙ¶È
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
	
	//Speed smoothing is enabled when moving the omnidirectional trolley
	//È«ÏòÒÆ¶¯Ğ¡³µ²Å¿ªÆôËÙ¶ÈÆ½»¬´¦Àí
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
	{
		if(Allow_Recharge==0)
			Smooth_control(Vx,Vy,Vz); //Smoothing the input speed //¶ÔÊäÈëËÙ¶È½øĞĞÆ½»¬´¦Àí
		else
			smooth_control.VX=Vx,     
			smooth_control.VY=Vy,
			smooth_control.VZ=Vz;

		//Get the smoothed data 
		//»ñÈ¡Æ½»¬´¦ÀíºóµÄÊı¾İ			
		Vx=smooth_control.VX;     
		Vy=smooth_control.VY;
		Vz=smooth_control.VZ;
	}
		
	//Mecanum wheel car
	//Âó¿ËÄÉÄ·ÂÖĞ¡³µ

	//Omni car
	//È«ÏòÂÖĞ¡³µ

		
	//Ackermann structure car
	//°¢¿ËÂüĞ¡³µ




	
		
	//Differential car
	//²îËÙĞ¡³µ
if (Car_Mode==Diff_Car) {
// ç¡¬ä»¶å‚æ•°é…ç½®
// ç¡¬ä»¶å‚æ•°é…ç½®
// ç¡¬ä»¶å‚æ•°é…ç½®
// ç¡¬ä»¶å‚æ•°é…ç½®
const float Wheel_Spacing = 0.43f;     // è½®è·(m)
const float Wheel_Radius = 0.0735f;    // è½®åŠå¾„(m)
const float Gear_Ratio = 40.0f;        // å‡é€Ÿæ¯”
const float Max_Motor_RPM = 3000.0f;   // ç”µæœºé¢å®šè½¬é€Ÿ
const int SERVO_NEUTRAL = 1500;        // PWMä¸­ä½å€¼
const int SERVO_MIN = 1200;            // PWMæœ€å°å€¼
const int SERVO_MAX = 1800;            // PWMæœ€å¤§å€¼

// è®¡ç®—ç†è®ºæœ€å¤§çº¿é€Ÿåº¦
float Max_Speed = (Max_Motor_RPM / Gear_Ratio) * (2 * 3.141592f * Wheel_Radius) / 60.0f;
const float DEADZONE = 0.05f;

// è¾“å…¥å¤„ç†
if (fabs(Vx) < DEADZONE) Vx = 0;
if (fabs(Vz) < DEADZONE) Vz = 0;

// æ‘‡æ†è¾“å…¥æ˜ å°„
if (fabs(Move_X * 128) > DEADZONE) {
    Vx = (Move_X > 0) ? 
        (Move_X * 200 - DEADZONE) / (1 - DEADZONE) : 
        (Move_X * 200 + DEADZONE) / (1 - DEADZONE);
} else {
    Vx = 0;
}

if (fabs(Move_Z * 128) > DEADZONE) {
    Vz = (Move_Z > 0) ? 
        (Move_Z * 1500 - DEADZONE) / (1 - DEADZONE) : 
        (Move_Z * 1500 + DEADZONE) / (1 - DEADZONE);
} else {
    Vz = 0;
}

// è¿åŠ¨å­¦è®¡ç®—
float linear_speed = Vx * Max_Speed;
float angular_speed = Vz * (2 * Max_Speed) / Wheel_Spacing;

float v_left = 0.0f;
float v_right = 0.0f;

// å…³é”®ä¿®å¤ï¼šç¡®ä¿å¼§çº¿è¿åŠ¨æ­£å¸¸å·¥ä½œ
// æ£€æŸ¥æ˜¯å¦æ˜¯å¼§çº¿è¿åŠ¨ï¼ˆåŒæ—¶æœ‰çº¿é€Ÿåº¦å’Œè§’é€Ÿåº¦ï¼‰
if (fabs(linear_speed) > 0.001f && fabs(angular_speed) > 0.001f) {
    // å¼§çº¿è¿åŠ¨æ¨¡å¼
    // æ ‡å‡†å·®é€Ÿæ¨¡å‹ - ç¡®ä¿ä¸¤ä¸ªè½®å­åŒå‘ä½†é€Ÿåº¦ä¸åŒ
    v_left = linear_speed - (angular_speed * Wheel_Spacing / 2);
    v_right = linear_speed + (angular_speed * Wheel_Spacing / 2);
    
    // ç¡®ä¿ä¸¤ä¸ªè½®å­åŒå‘ï¼ˆé¿å…ä¸€ä¸ªå‘å‰ä¸€ä¸ªå‘åï¼‰
    // å¦‚æœv_leftå’Œv_rightç¬¦å·ç›¸åï¼Œè°ƒæ•´è®¡ç®—
    if ((v_left > 0 && v_right < 0) || (v_left < 0 && v_right > 0)) {
        // å¦‚æœç¬¦å·ç›¸åï¼Œé‡æ–°è®¡ç®—ç¡®ä¿åŒå‘
        // ä¿æŒçº¿é€Ÿåº¦æ–¹å‘ï¼Œè°ƒæ•´è§’é€Ÿåº¦å½±å“
        float adjustment = fmin(fabs(v_left), fabs(v_right));
        if (linear_speed > 0) {
            v_left = linear_speed - adjustment;
            v_right = linear_speed + adjustment;
        } else {
            v_left = linear_speed + adjustment;
            v_right = linear_speed - adjustment;
        }
    }
} 
// æ£€æŸ¥æ˜¯å¦æ˜¯çº¯æ—‹è½¬ï¼ˆåªæœ‰è§’é€Ÿåº¦ï¼Œæ²¡æœ‰çº¿é€Ÿåº¦ï¼‰
else if (fabs(linear_speed) < 0.001f && fabs(angular_speed) > 0.001f) {
    // çº¯æ—‹è½¬æ¨¡å¼
    float rotation_speed = fabs(angular_speed) * Wheel_Spacing / 2;
    // ç¡®ä¿æœ€å°æ—‹è½¬é€Ÿåº¦
    float min_rotation_speed = 0.2f * Max_Speed;
    if (rotation_speed < min_rotation_speed) {
        rotation_speed = min_rotation_speed;
    }
    
    if (angular_speed > 0) {
        v_left = -rotation_speed;
        v_right = rotation_speed;
    } else {
        v_left = rotation_speed;
        v_right = -rotation_speed;
    }
}
// æ£€æŸ¥æ˜¯å¦æ˜¯çº¯ç›´çº¿ï¼ˆåªæœ‰çº¿é€Ÿåº¦ï¼Œæ²¡æœ‰è§’é€Ÿåº¦ï¼‰
else if (fabs(linear_speed) > 0.001f && fabs(angular_speed) < 0.001f) {
    // çº¯ç›´çº¿æ¨¡å¼
    v_left = linear_speed;
    v_right = linear_speed;
}
else {
    // åœæ­¢
    v_left = 0;
    v_right = 0;
}

// PWMæ˜ å°„
Servo1 = SERVO_NEUTRAL + (v_left / Max_Speed) * (SERVO_MAX - SERVO_NEUTRAL);
Servo = SERVO_NEUTRAL + (v_right / Max_Speed) * (SERVO_MAX - SERVO_NEUTRAL);

// PWMé™å¹…
if (Servo < SERVO_MIN) Servo = SERVO_MIN;
if (Servo > SERVO_MAX) Servo = SERVO_MAX;
if (Servo1 < SERVO_MIN) Servo1 = SERVO_MIN;
if (Servo1 > SERVO_MAX) Servo1 = SERVO_MAX;

// è¾“å‡ºPWMåˆ°ç”µæœº
Set_Pwm(0, 0, 0, 0, Servo, Servo1);
}
	//Omni car
	//È«Ï²Â–Ğ¡Â³Âµ

		
	//Ackermann structure car
	//Â°Â¢Â¿Ë‚Ã¼Ğ¡Â³Âµ

else if (Car_Mode == Akm_Car) 
{
const float Wheel_Spacing = 0.43f;
const float Wheel_Radius = 0.0735f;
const float Gear_Ratio = 40.0f;
const float Max_Motor_RPM = 3000.0f;
const int SERVO_NEUTRAL = 1500;
const int SERVO_MIN = 1100;
const int SERVO_MAX = 1900;

// è®¡ç®—ç†è®ºæœ€å¤§çº¿é€Ÿåº¦
float Max_Speed = (Max_Motor_RPM / Gear_Ratio) * (2 * 3.141592f * Wheel_Radius) / 60.0f;
const float DEADZONE = 0.05f;

// è¾“å…¥å¤„ç†
if (fabs(Vx) < DEADZONE) Vx = 0;
if (fabs(Vz) < DEADZONE) Vz = 0;

// æ‘‡æ†è¾“å…¥æ˜ å°„
if (fabs(Move_X * 128) > DEADZONE) {
    Vx = (Move_X > 0) ? 
        (Move_X * 200 - DEADZONE) / (1 - DEADZONE) : 
        (Move_X * 200 + DEADZONE) / (1 - DEADZONE);
} else {
    Vx = 0;
}

if (fabs(Move_Z * 128) > DEADZONE) {
    Vz = (Move_Z > 0) ? 
        (Move_Z * 1500 - DEADZONE) / (1 - DEADZONE) : 
        (Move_Z * 1500 + DEADZONE) / (1 - DEADZONE);
} else {
    Vz = 0;
}

// è¿åŠ¨å­¦è®¡ç®—
float linear_speed = Vx * Max_Speed;
float angular_speed = Vz * (2 * Max_Speed) / Wheel_Spacing;

// ç›´æ¥è®¾ç½®PWMå€¼ï¼Œè·³è¿‡å¤æ‚çš„è¿åŠ¨å­¦è®¡ç®—
int Servo1, Servo;

// è¿åŠ¨æ¨¡å¼å¤„ç†
if (fabs(linear_speed) > 0.001f && fabs(angular_speed) > 0.001f) {
    // å¼§çº¿è¿åŠ¨æ¨¡å¼
    
    if (linear_speed > 0) {
        // å‰è¿›å¼§çº¿
        int base_pwm = 1600;
        int fast_wheel = 1850;
        int slow_wheel = 1550;
        
        // æ ¹æ®çº¿é€Ÿåº¦å¼ºåº¦è°ƒæ•´åŸºç¡€é€Ÿåº¦
        float speed_factor = linear_speed / Max_Speed;
        if (speed_factor > 0.3f) {
            base_pwm = 1650;
            fast_wheel = 1880;
            slow_wheel = 1580;
        }
        
        if (angular_speed > 0) {
            // å‰è¿›å³è½¬å¼¯ï¼šå·¦è½®å¿«ï¼Œå³è½®æ…¢
            Servo1 = fast_wheel;
            Servo = slow_wheel;
        } else {
            // å‰è¿›å·¦è½¬å¼¯ï¼šå³è½®å¿«ï¼Œå·¦è½®æ…¢
            Servo1 = slow_wheel;
            Servo = fast_wheel;
        }
    } else {
        // åé€€å¼§çº¿
        int base_pwm = 1400;
        int fast_wheel = 1450;  // åé€€æ—¶"å¿«"è½®æ›´æ¥è¿‘1500
        int slow_wheel = 1350;  // åé€€æ—¶"æ…¢"è½®æ›´è¿œç¦»1500
        
        // æ ¹æ®çº¿é€Ÿåº¦å¼ºåº¦è°ƒæ•´åŸºç¡€é€Ÿåº¦
        float speed_factor = fabs(linear_speed) / Max_Speed;
        if (speed_factor > 0.3f) {
            base_pwm = 1350;
            fast_wheel = 1400;
            slow_wheel = 1300;
        }
        
        if (angular_speed > 0) {
            // åé€€å³è½¬å¼¯ï¼šå·¦è½®æ…¢ï¼Œå³è½®å¿«ï¼ˆåé€€æ–¹å‘ï¼‰
            Servo1 = slow_wheel;  // å·¦è½®åé€€æ›´å¿«
            Servo = fast_wheel;   // å³è½®åé€€è¾ƒæ…¢
        } else {
            // åé€€å·¦è½¬å¼¯ï¼šå³è½®æ…¢ï¼Œå·¦è½®å¿«ï¼ˆåé€€æ–¹å‘ï¼‰
            Servo1 = fast_wheel;  // å·¦è½®åé€€è¾ƒæ…¢
            Servo = slow_wheel;   // å³è½®åé€€æ›´å¿«
        }
    }
} 
else if (fabs(linear_speed) < 0.001f && fabs(angular_speed) > 0.001f) {
    // çº¯æ—‹è½¬æ¨¡å¼
    if (angular_speed > 0) {
        // å³è½¬ï¼šå·¦è½®åé€€ï¼Œå³è½®å‰è¿›
        Servo1 = 1400;
        Servo = 1600;
    } else {
        // å·¦è½¬ï¼šå·¦è½®å‰è¿›ï¼Œå³è½®åé€€
        Servo1 = 1600;
        Servo = 1400;
    }
}
else if (fabs(linear_speed) > 0.001f && fabs(angular_speed) < 0.001f) {
    // çº¯ç›´çº¿æ¨¡å¼
    if (linear_speed > 0) {
        // å‰è¿›
        int straight_pwm = 1600 + (int)(linear_speed / Max_Speed * 100);
        Servo1 = straight_pwm;
        Servo = straight_pwm;
    } else {
        // åé€€
        int straight_pwm = 1400 + (int)(linear_speed / Max_Speed * 100);
        Servo1 = straight_pwm;
        Servo = straight_pwm;
    }
}
else {
    // åœæ­¢
    Servo1 = 1500;
    Servo = 1500;
}

// æœ€ç»ˆå¼ºåˆ¶æ£€æŸ¥ï¼šç¡®ä¿å¼§çº¿è¿åŠ¨æ—¶æœ‰æ­£ç¡®çš„PWMæ–¹å‘
if (fabs(linear_speed) > 0.001f && fabs(angular_speed) > 0.001f) {
    if (linear_speed > 0) {
        // å‰è¿›å¼§çº¿ï¼šç¡®ä¿ä¸¤ä¸ªPWMéƒ½å¤§äº1500
        if (Servo1 < 1550) Servo1 = 1550;
        if (Servo < 1550) Servo = 1550;
        
        // ç¡®ä¿æ˜æ˜¾çš„PWMå·®å¼‚
        int current_diff = abs(Servo1 - Servo);
        int min_diff = 150;
        if (current_diff < min_diff) {
            if (angular_speed > 0) {
                Servo1 = 1800;
                Servo = 1600;
            } else {
                Servo1 = 1600;
                Servo = 1800;
            }
        }
    } else {
        // åé€€å¼§çº¿ï¼šç¡®ä¿ä¸¤ä¸ªPWMéƒ½å°äº1500
        if (Servo1 > 1450) Servo1 = 1450;
        if (Servo > 1450) Servo = 1450;
        
        // ç¡®ä¿æ˜æ˜¾çš„PWMå·®å¼‚
        int current_diff = abs(Servo1 - Servo);
        int min_diff = 100;
        if (current_diff < min_diff) {
            if (angular_speed > 0) {
                Servo1 = 1300;
                Servo = 1400;
            } else {
                Servo1 = 1400;
                Servo = 1300;
            }
        }
    }
}

// PWMé™å¹…
if (Servo < SERVO_MIN) Servo = SERVO_MIN;
if (Servo > SERVO_MAX) Servo = SERVO_MAX;
if (Servo1 < SERVO_MIN) Servo1 = SERVO_MIN;
if (Servo1 > SERVO_MAX) Servo1 = SERVO_MAX;

// è¾“å‡ºPWMåˆ°ç”µæœº
Set_Pwm(0, 0, 0, 0, Servo, Servo1);
}
 

    // PWMé™å¹…ä¿æŠ¤



	}
	

	//FourWheel car
	//ËÄÇı³µ

	//Tank Car
	//ÂÄ´ø³µ


/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
º¯Êı¹¦ÄÜ£ºFreeRTOSÈÎÎñ£¬ºËĞÄÔË¶¯¿ØÖÆÈÎÎñ
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Balance_task(void *pvParameters)
{ 
	u32 lastWakeTime = getSysTickCnt();

    while(1)
    {	
		// This task runs at a frequency of 100Hz (10ms control once)
		//´ËÈÎÎñÒÔ100HzµÄÆµÂÊÔËĞĞ£¨10ms¿ØÖÆÒ»´Î£©
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); 

		//Time count is no longer needed after 30 seconds
		//Ê±¼ä¼ÆÊı£¬30Ãëºó²»ÔÙĞèÒª
		if(SysVal.Time_count<3000) SysVal.Time_count++;
			Buzzer_count1++;
		//Get the encoder data, that is, the real time wheel speed, 
		//and convert to transposition international units
		//»ñÈ¡±àÂëÆ÷Êı¾İ£¬¼´³µÂÖÊµÊ±ËÙ¶È£¬²¢×ª»»Î»¹ú¼Êµ¥Î»
		Get_Velocity_Form_Encoder();   
		
			//Click the user button to update the gyroscope zero
			//µ¥»÷ÓÃ»§°´¼ü¸üĞÂÍÓÂİÒÇÁãµã
			Key(); 
			
		if( Allow_Recharge==1 )
			if( Get_Charging_HardWare==0 ) Allow_Recharge=0,Find_Charging_HardWare();
		
		if(Check==0) //If self-check mode is not enabled //Èç¹ûÃ»ÓĞÆô¶¯×Ô¼ìÄ£Ê½
		{
//			command_lost_count++;
//			if(command_lost_count>RATE_100_HZ && APP_ON_Flag==0 && Remote_ON_Flag==0 && PS2_ON_Flag==0)
//				Move_X=0,Move_Y=0,Move_Z=0;
			if(Allow_Recharge==1)
			{
				if(Get_Charging_HardWare==1)
				{   //´æÔÚ»Ø³ä×°±¸Ê±£¬¶Ô»Ø³ä×°±¸µÄ×´Ì¬½øĞĞ¼ì²â
					charger_check++;
					if( charger_check>RATE_100_HZ) charger_check=RATE_100_HZ+1,Allow_Recharge=0,RED_STATE=0,Recharge_Red_Move_X = 0,Recharge_Red_Move_Y = 0,Recharge_Red_Move_Z = 0;
				}
				//Èç¹û¿ªÆôÁËµ¼º½»Ø³ä£¬Í¬Ê±Ã»ÓĞ½ÓÊÕµ½ºìÍâĞÅºÅ£¬½ÓÊÕÀ´×ÔÉÏÎ»»úµÄµÄ»Ø³ä¿ØÖÆÃüÁî
				if      (nav_walk==1 && RED_STATE==0) Drive_Motor(Recharge_UP_Move_X,0,Recharge_UP_Move_Z);
				//½ÓÊÕµ½ÁËºìÍâĞÅºÅ£¬½ÓÊÕÀ´×Ô»Ø³ä×°±¸µÄ»Ø³ä¿ØÖÆÃüÁî
				else if (RED_STATE!=0) nav_walk = 0,Drive_Motor(Recharge_Red_Move_X,0,Recharge_Red_Move_Z);
				//·ÀÖ¹Ã»ÓĞºìÍâĞÅºÅÊ±Ğ¡³µÔË¶¯
				if (nav_walk==0&&RED_STATE==0) Drive_Motor(0,0,0);
			}
			else
			{			
				if      (APP_ON_Flag)      Get_RC();         //Handle the APP remote commands //´¦ÀíAPPÒ£¿ØÃüÁî
				else if (Remote_ON_Flag)   Remote_Control(); //Handle model aircraft remote commands //´¦Àíº½Ä£Ò£¿ØÃüÁî
				else if (PS2_ON_Flag)      PS2_control();    //Handle PS2 controller commands //´¦ÀíPS2ÊÖ±ú¿ØÖÆÃüÁî

				//CAN, Usart 1, Usart 3, Uart5 control can directly get the three axis target speed, 
				//without additional processing
				//CAN¡¢´®¿Ú1¡¢´®¿Ú3(ROS)¡¢´®¿Ú5¿ØÖÆÖ±½ÓµÃµ½ÈıÖáÄ¿±êËÙ¶È£¬ÎŞĞë¶îÍâ´¦Àí
				else                      Drive_Motor(Move_X, Move_Y, Move_Z);
			}


			

			//If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
			//and the software failure flag is 0
			//Èç¹ûµç³ØµçÑ¹²»´æÔÚÒì³££¬¶øÇÒÊ¹ÄÜ¿ª¹ØÔÚONµµÎ»£¬¶øÇÒÈí¼şÊ§ÄÜ±êÖ¾Î»Îª0
			if(Turn_Off(Voltage)==0||(Allow_Recharge&&EN&&!Flag_Stop)) 
			{ 			
				//Speed closed-loop control to calculate the PWM value of each motor, 
				//PWM represents the actual wheel speed					 
				//ËÙ¶È±Õ»·¿ØÖÆ¼ÆËã¸÷µç»úPWMÖµ£¬PWM´ú±í³µÂÖÊµ¼Ê×ªËÙ
				MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
				MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
				MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
				MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);

				Limit_Pwm(16700);

				//¼ì²âÊÇ·ñĞèÒªÇå³ıPWM²¢×Ô¶¯Ö´ĞĞÇåÀí
				auto_pwm_clear();
				
				//Set different PWM control polarity according to different car models
				//¸ù¾İ²»Í¬Ğ¡³µĞÍºÅÉèÖÃ²»Í¬µÄPWM¿ØÖÆ¼«ĞÔ
				switch(Car_Mode)
				{
					case Mec_Car:       Set_Pwm( MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ,0); break; //Mecanum wheel car       //Âó¿ËÄÉÄ·ÂÖĞ¡³µ
					case Omni_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ,0); break; //Omni car  
			//È«ÏòÂÖĞ¡³µ
			//Ackermann structure car //°¢¿ËÂüĞ¡³µ
					case Diff_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0   ,0 ); break; //Differential car        //Á½ÂÖ²îËÙĞ¡³µ
					case FourWheel_Car: Set_Pwm( MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0   ,0 ); break; //FourWheel car           //ËÄÇı³µ 
					case Tank_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0   ,0 ); break; //Tank Car                //ÂÄ´ø³µ
				}
			}
			//If Turn_Off(Voltage) returns to 1, the car is not allowed to move, and the PWM value is set to 0
			//Èç¹ûTurn_Off(Voltage)·µ»ØÖµÎª1£¬²»ÔÊĞí¿ØÖÆĞ¡³µ½øĞĞÔË¶¯£¬PWMÖµÉèÖÃÎª0
			else	Set_Pwm(0,0,0,0,0,0); 
		}	
		else							//ÓÃ»§×Ô¼ì´úÂë
			{
				if(Proc_Flag==3)						//×Ô¼ìµç»ú
				{
					 if(check_time_count_motor_forward>0)
					 {	 
						 check_time_count_motor_forward--;
						 Full_rotation=16799;
					 }
					 else if(check_time_count_motor_retreat>0) 
					 {	 
							check_time_count_motor_retreat--;
						 Full_rotation=-16799;
					 }		

					 switch(Car_Mode)
					 {
							case Mec_Car:       Set_Pwm( Full_rotation, -Full_rotation, -Full_rotation, Full_rotation, 0   ,0 ); break; //Mecanum wheel car       //Âó¿ËÄÉÄ·ÂÖĞ¡³µ
							case Omni_Car:      Set_Pwm(-Full_rotation,  Full_rotation, -Full_rotation, Full_rotation, 0   ,0 ); break; //Omni car                //È«ÏòÂÖĞ¡³µ
							case Akm_Car:       Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation, 0   ,0 ); break; //Ackermann structure car //°¢¿ËÂüĞ¡³µ
							case Diff_Car:      Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation, 0   ,0 ); break; //Differential car        //Á½ÂÖ²îËÙĞ¡³µ
							case FourWheel_Car: Set_Pwm( Full_rotation, -Full_rotation, -Full_rotation, Full_rotation, 0   ,0 ); break; //FourWheel car           //ËÄÇı³µ 
							case Tank_Car:      Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation, 0    ,0); break; //Tank Car                //ÂÄ´ø³µ
					 } 
					 if(!(check_time_count_motor_retreat>0) && !(check_time_count_motor_forward>0))
					 {	 
						 Set_Pwm(0,0,0,0,0,0);		 
					 }
				}
				if(Proc_Flag==4)		Set_Pwm(0,0,0,0,0,0);
				if(Proc_Flag==6)		TIM8_SERVO_Init(9999,168-1);					//ÁùÂ·¶æ»ú
				if(Proc_Flag==7)																					//¿ØÖÆ¶æ»ú
				{
					if(servo_direction[0]==0&&Servo_Count[0]<2500) Servo_Count[0]=Servo_Count[0]+5;
				 if(servo_direction[0]==0&&Servo_Count[0]>=2500) servo_direction[0]=1;
				 if(Servo_Count[0]>500&&servo_direction[0]==1)  Servo_Count[0]=Servo_Count[0]-5;
				 if(Servo_Count[0]<=500&&servo_direction[0]==1)  Servo_Count[0]=500,servo_direction[0] = 2;
				TIM12->CCR2=Servo_Count[0];
				 
				}
				if(Proc_Flag==8)
				{
					if(servo_direction[0]!=2)					Servo_Count[0]=500; TIM12->CCR2=Servo_Count[0];
					if(servo_direction[1]==0&&Servo_Count[1]<2500) Servo_Count[1]=Servo_Count[1]+5;
				 if(servo_direction[1]==0&&Servo_Count[1]>=2500) servo_direction[1]=1;
				 if(Servo_Count[1]>500&&servo_direction[1]==1)  Servo_Count[1]=Servo_Count[1]-5;
				 if(Servo_Count[1]<=500&&servo_direction[1]==1)  Servo_Count[1]=500,servo_direction[1] = 2;
				TIM12->CCR1=Servo_Count[1];
					
				}
				if(Proc_Flag==9)
				{
					if(servo_direction[1]!=2)					Servo_Count[1]=500;TIM12->CCR1=Servo_Count[1];
					if(servo_direction[2]==0&&Servo_Count[2]<2500) Servo_Count[2]=Servo_Count[2]+5;
				 if(servo_direction[2]==0&&Servo_Count[2]>=2500) servo_direction[2]=1;
				 if(Servo_Count[2]>500&&servo_direction[2]==1)  Servo_Count[2]=Servo_Count[2]-5;
				 if(Servo_Count[2]<=500&&servo_direction[2]==1)  Servo_Count[2]=500,servo_direction[2] = 2;
					TIM8->CCR4=Servo_Count[2];
				}
				if(Proc_Flag==10)
				{
					if(servo_direction[2]!=2)					Servo_Count[2]=500,TIM8->CCR4=Servo_Count[2];
					if(servo_direction[3]==0&&Servo_Count[3]<2500) Servo_Count[3]=Servo_Count[3]+5;
				 if(servo_direction[3]==0&&Servo_Count[3]>=2500) servo_direction[3]=1;
				 if(Servo_Count[3]>500&&servo_direction[3]==1)  Servo_Count[3]=Servo_Count[3]-5;
				 if(Servo_Count[3]<=500&&servo_direction[3]==1)  Servo_Count[3]=500,servo_direction[3] = 2;
					TIM8->CCR3=Servo_Count[3];
				}
				if(Proc_Flag==11)
				{
					if(servo_direction[3]!=2)					Servo_Count[3]=500,TIM8->CCR3=Servo_Count[3];
					if(servo_direction[4]==0&&Servo_Count[4]<2500) Servo_Count[4]=Servo_Count[4]+5;
				 if(servo_direction[4]==0&&Servo_Count[4]>=2500) servo_direction[4]=1;
				 if(Servo_Count[4]>500&&servo_direction[4]==1)  Servo_Count[4]=Servo_Count[4]-5;
				 if(Servo_Count[4]<=500&&servo_direction[4]==1)  Servo_Count[4]=500,servo_direction[4] = 2;
					TIM8->CCR2=Servo_Count[4];
				}
				if(Proc_Flag==12)
				{
					if(servo_direction[4]!=2)					Servo_Count[4]=500,TIM8->CCR2=Servo_Count[4];
					if(servo_direction[5]==0&&Servo_Count[5]<2500) Servo_Count[5]=Servo_Count[5]+5;
				 if(servo_direction[5]==0&&Servo_Count[5]>=2500) servo_direction[5]=1;
				 if(Servo_Count[5]>500&&servo_direction[5]==1)  Servo_Count[5]=Servo_Count[5]-5;
				 if(Servo_Count[5]<=500&&servo_direction[5]==1)  Servo_Count[5]=500,servo_direction[5] = 2;
					TIM8->CCR1=Servo_Count[5];
				}
				
				if(Proc_Flag==13)																	//
				{
					servo_direction[0] = servo_direction[1] = servo_direction[2] = servo_direction[3] = servo_direction[4] = servo_direction[5] = 0;
					Servo_Count[0] = Servo_Count[1] = Servo_Count[2] = Servo_Count[3] = Servo_Count[4] = Servo_Count[5] = 500;
					 TIM8->CCR1=Servo_Count[5];
					 TIM8->CCR2=Servo_Count[4];
					 TIM8->CCR3=Servo_Count[3];
					 TIM8->CCR4=Servo_Count[2];
					 TIM12->CCR1=Servo_Count[1];
					 TIM12->CCR2=Servo_Count[0];
				}
				if(Proc_Flag==14)																	//·äÃùÆ÷¼ä¸ô1sÏìÒ»´Î
				{
					if((Buzzer_count1/100)%2)			Buzzer = 1;
					else													Buzzer = 0;
				}
				if(Proc_Flag==15)			Buzzer = 0;
//				if(Proc_Flag==17)																	//ÏòAPP·¢ËÍWHEELTEC
//				{
//					if(uart2_send_flag==1)
//					{
//						USART2_Return();
//						uart2_send_flag = 0;
//						app_count = 0;
//					}
//				}
				if(Proc_Flag==19)
				{
					if(uart3_send_flag==1)
					{
						USART3_Return();
						uart3_send_flag = 0;
						message_count = 0;
					}
				}
			}
	}  
}
/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
º¯Êı¹¦ÄÜ£º¸³Öµ¸øPWM¼Ä´æÆ÷£¬¿ØÖÆ³µÂÖ×ªËÙÓë·½Ïò
Èë¿Ú²ÎÊı£ºPWM
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo,int servo1)
{
	//Forward and reverse control of motor
	//µç»úÕı·´×ª¿ØÖÆ
	if(motor_a<0)			PWMA1=16799,PWMA2=16799+motor_a;
	else 	            PWMA2=16799,PWMA1=16799-motor_a;
	
	//Forward and reverse control of motor
	//µç»úÕı·´×ª¿ØÖÆ	
	if(motor_b<0)			PWMB1=16799,PWMB2=16799+motor_b;
	else 	            PWMB2=16799,PWMB1=16799-motor_b;
//  PWMB1=10000,PWMB2=5000;

	//Forward and reverse control of motor
	//µç»úÕı·´×ª¿ØÖÆ	
	if(motor_c<0)			PWMC1=16799,PWMC2=16799+motor_c;
	else 	            PWMC2=16799,PWMC1=16799-motor_c;
	
	//Forward and reverse control of motor
	//µç»úÕı·´×ª¿ØÖÆ
	if(motor_d<0)			PWMD1=16799,PWMD2=16799+motor_d;
	else 	            PWMD2=16799,PWMD1=16799-motor_d;
	
	//Servo control
	//¶æ»ú¿ØÖÆ
	Servo_PWM =servo;
	Servo_PWM1 =servo1;
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
º¯Êı¹¦ÄÜ£ºÏŞÖÆPWMÖµ 
Èë¿Ú²ÎÊı£º·ùÖµ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		  MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	    
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
º¯Êı¹¦ÄÜ£ºÏŞ·ùº¯Êı
Èë¿Ú²ÎÊı£º·ùÖµ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
º¯Êı¹¦ÄÜ£º¼ì²éµç³ØµçÑ¹¡¢Ê¹ÄÜ¿ª¹Ø×´Ì¬¡¢Èí¼şÊ§ÄÜ±êÖ¾Î»×´Ì¬
Èë¿Ú²ÎÊı£ºµçÑ¹
·µ»Ø  Öµ£ºÊÇ·ñÔÊĞí¿ØÖÆ£¬1£º²»ÔÊĞí£¬0ÔÊĞí
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<10||EN==0||Flag_Stop==1)
			{	                                                
				temp=1;      
				PWMA1=0;PWMA2=0;
				PWMB1=0;PWMB2=0;		
				PWMC1=0;PWMC1=0;	
				PWMD1=0;PWMD2=0;					
      }
			else
			temp=0;
			return temp;			
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
º¯Êı¹¦ÄÜ£ºÇó¾ø¶ÔÖµ
Èë¿Ú²ÎÊı£ºlong int
·µ»Ø  Öµ£ºunsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e£¨k£©-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e£¨k£©-e(k-1)]+Ki*e(k)

º¯Êı¹¦ÄÜ£ºÔöÁ¿Ê½PI¿ØÖÆÆ÷
Èë¿Ú²ÎÊı£º±àÂëÆ÷²âÁ¿Öµ(Êµ¼ÊËÙ¶È)£¬Ä¿±êËÙ¶È
·µ»Ø  Öµ£ºµç»úPWM
¸ù¾İÔöÁ¿Ê½ÀëÉ¢PID¹«Ê½ 
pwm+=Kp[e£¨k£©-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)´ú±í±¾´ÎÆ«²î 
e(k-1)´ú±íÉÏÒ»´ÎµÄÆ«²î  ÒÔ´ËÀàÍÆ 
pwm´ú±íÔöÁ¿Êä³ö
ÔÚÎÒÃÇµÄËÙ¶È¿ØÖÆ±Õ»·ÏµÍ³ÀïÃæ£¬Ö»Ê¹ÓÃPI¿ØÖÆ
pwm+=Kp[e£¨k£©-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //¼ÆËãÆ«²î
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //±£´æÉÏÒ»´ÎÆ«²î 
	
	//Çå³ıPWM±êÖ¾Î»£¬¸ÃÎ»Îª1Ê±´ú±íĞèÒªÇå³ıPWM
	if( start_clear ) 
	{
		//PWMÖğ½¥µİ¼õµÄ·½Ê½Çå³ı£¬¼õ»ºĞ¡³µÓÉÓÚµç»úÊÍ·Å¶øÔì³ÉÇáÎ¢ÒÆ¶¯µÄÓ°Ïì
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		//ÈôÇå³ıÍê±Ï£¬Ôò±ê¼Ç±êÖ¾Î»£¬4¸öµç»ú·Ö±ğÓÃ4¸öbit±íÊ¾
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<0;
		else clear_state &= ~(1<<0);
	}
	
	 return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //¼ÆËãÆ«²î
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //±£´æÉÏÒ»´ÎÆ«²î 
	if( start_clear ) 
	{
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<1;
		else clear_state &= ~(1<<1);
	}
	 return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //¼ÆËãÆ«²î
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //±£´æÉÏÒ»´ÎÆ«²î 
	
	if(Car_Mode==Diff_Car || Car_Mode==Akm_Car || Car_Mode==Tank_Car) Pwm = 0;
	if( start_clear ) 
	{
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<2;
		else clear_state &= ~(1<<2);
	}
	 return Pwm; 
}
int Incremental_PI_D (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	
	 Bias=Target-Encoder; //Calculate the deviation //¼ÆËãÆ«²î
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //±£´æÉÏÒ»´ÎÆ«²î 
	
	if(Car_Mode==Diff_Car || Car_Mode==Akm_Car || Car_Mode==Tank_Car || Car_Mode==Omni_Car ) Pwm = 0;
	if( start_clear ) 
	{
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<3;
		else clear_state &= ~(1<<3);
		
		//4¸öµç»ú¾ùÇå³ıÍê±Ï£¬Ôò¹Ø±ÕÇå³ıÈÎÎñ
		if( (clear_state&0xff)==0x0f ) start_clear = 0,clear_done_once=1,clear_state=0;
	}
	 return Pwm; 
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
º¯Êı¹¦ÄÜ£º¶ÔAPPÍ¨¹ı´®¿Ú2·¢ËÍ¹ıÀ´µÄÃüÁî½øĞĞ´¦Àí
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car) //The omnidirectional wheel moving trolley can move laterally //È«ÏòÂÖÔË¶¯Ğ¡³µ¿ÉÒÔ½øĞĞºáÏòÒÆ¶¯
	{
	 switch(Flag_Direction)  //Handle direction control commands //´¦Àí·½Ïò¿ØÖÆÃüÁî
	 { 
			case 1:      Move_X=RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 2:      Move_X=RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 3:      Move_X=0;      		     Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 4:      Move_X=-RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
			case 5:      Move_X=-RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 6:      Move_X=-RC_Velocity;  	 Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 7:      Move_X=0;     	 		     Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 8:      Move_X=RC_Velocity; 	   Move_Y=RC_Velocity;   Flag_Move=1;    break; 
			default:     Move_X=0;               Move_Y=0;             Flag_Move=0;    break;
	 }
	 if(Flag_Move==0)		
	 {	
		 //If no direction control instruction is available, check the steering control status
		 //Èç¹ûÎŞ·½Ïò¿ØÖÆÖ¸Áî£¬¼ì²é×ªÏò¿ØÖÆ×´Ì¬
		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //×ó×Ô×ª  
		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //ÓÒ×Ô×ª
		 else 		               Move_Z=0;                       //stop           //Í£Ö¹
	 }
	}	
	else //Non-omnidirectional moving trolley //·ÇÈ«ÏòÒÆ¶¯Ğ¡³µ
	{
	 switch(Flag_Direction) //Handle direction control commands //´¦Àí·½Ïò¿ØÖÆÃüÁî
	 { 
			case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/2;   	 break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //×ó×Ô×ª 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //ÓÒ×Ô×ª	
	}
	
	//Z-axis data conversion //ZÖáÊı¾İ×ª»¯
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//°¢¿ËÂü½á¹¹Ğ¡³µ×ª»»ÎªÇ°ÂÖ×ªÏò½Ç¶È
		
	}
	
	
	//Unit conversion, mm/s -> m/s
  //µ¥Î»×ª»»£¬mm/s -> m/s	
  Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//µÃµ½¿ØÖÆÄ¿±êÖµ£¬½øĞĞÔË¶¯Ñ§·ÖÎö
	Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
º¯Êı¹¦ÄÜ£º¶ÔPS2ÊÖ±ú¿ØÖÆÃüÁî½øĞĞ´¦Àí
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void PS2_control(void)
{
   	int LX,LY,RY,RX,LX1,LY1,RY1,RX1;
		int Threshold=20; //Threshold to ignore small movements of the joystick //ãĞÖµ£¬ºöÂÔÒ¡¸ËĞ¡·ù¶È¶¯×÷
	static u8 acc_dec_filter = 0;		
	
	  //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
	  //128ÎªÖĞÖµ¡£PS2×ø±êÏµÓëROS×ø±êÏµ¶ÔX¡¢YµÄ¶¨Òå²»Ò»Ñù
		LY=-(PS2_LX-128);  
		LX=-(PS2_LY-128); 
		RY=-(PS2_RX-128); 


	  
	  //Ignore small movements of the joystick //ºöÂÔÒ¡¸ËĞ¡·ù¶È¶¯×÷
		if(LX>-Threshold&&LX<Threshold)LX=0; 
		if(LY>-Threshold&&LY<Threshold)LY=0; 
		if(RY>-Threshold&&RY<Threshold)RY=0;

	
	if(++acc_dec_filter==15)
	{
		acc_dec_filter=0;
	  if (PS2_KEY==11)	    RC_Velocity+=5;  //To accelerate//¼ÓËÙ
	  else if(PS2_KEY==9)	RC_Velocity-=5;  //To slow down //¼õËÙ	
	}

	
		if(RC_Velocity<0)   RC_Velocity=0;
	
	  //Handle PS2 controller control commands
	  //¶ÔPS2ÊÖ±ú¿ØÖÆÃüÁî½øĞĞ´¦Àí
		Move_X=LX*(PI/2)/128; 
		Move_Y=LY*(PI/2)/128; 
	  Move_Z=RY*(PI/2)/128;      
	
	  //Z-axis data conversion //ZÖáÊı¾İ×ª»¯
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		else if(Car_Mode==Akm_Car)
		{
			//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		  //°¢¿ËÂü½á¹¹Ğ¡³µ×ª»»ÎªÇ°ÂÖ×ªÏò½Ç¶È

		}
		else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
		{

		}	
		 
	  //Unit conversion, mm/s -> m/s
    //µ¥Î»×ª»»£¬mm/s -> m/s	
	
		Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
		//Control target value is obtained and kinematics analysis is performed
	  //µÃµ½¿ØÖÆÄ¿±êÖµ£¬½øĞĞÔË¶¯Ñ§·ÖÎö
		Drive_Motor(Move_X,Move_Y,Move_Z);		 			
} 

/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
º¯Êı¹¦ÄÜ£º¶Ôº½Ä£Ò£¿Ø¿ØÖÆÃüÁî½øĞĞ´¦Àí
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Remote_Control(void)
{
	  //Data within 1 second after entering the model control mode will not be processed
	  //¶Ô½øÈëº½Ä£¿ØÖÆÄ£Ê½ºó1ÃëÄÚµÄÊı¾İ²»´¦Àí
    static u8 thrice=100; 
    int Threshold=100; //Threshold to ignore small movements of the joystick //ãĞÖµ£¬ºöÂÔÒ¡¸ËĞ¡·ù¶È¶¯×÷

	  //limiter //ÏŞ·ù
    int LX,LY,RY,RX,Remote_RCvelocity; 
		Remoter_Ch1=target_limit_int(Remoter_Ch1,1000,2000);
		Remoter_Ch2=target_limit_int(Remoter_Ch2,1000,2000);
		Remoter_Ch3=target_limit_int(Remoter_Ch3,1000,2000);
		Remoter_Ch4=target_limit_int(Remoter_Ch4,1000,2000);

	  // Front and back direction of left rocker. Control forward and backward.
	  //×óÒ¡¸ËÇ°ºó·½Ïò¡£¿ØÖÆÇ°½øºóÍË¡£
    LX=Remoter_Ch2-1500; 
	
	  //Left joystick left and right.Control left and right movement. Only the wheelie omnidirectional wheelie will use the channel.
	  //Ackerman trolleys use this channel as a PWM output to control the steering gear
	  //×óÒ¡¸Ë×óÓÒ·½Ïò¡£¿ØÖÆ×óÓÒÒÆ¶¯¡£ÂóÂÖÈ«ÏòÂÖ²Å»áÊ¹ÓÃµ½¸ÄÍ¨µÀ¡£°¢¿ËÂüĞ¡³µÊ¹ÓÃ¸ÃÍ¨µÀ×÷ÎªPWMÊä³ö¿ØÖÆ¶æ»ú
    LY=Remoter_Ch4-1500;

    //Front and back direction of right rocker. Throttle/acceleration/deceleration.
		//ÓÒÒ¡¸ËÇ°ºó·½Ïò¡£ÓÍÃÅ/¼Ó¼õËÙ¡£
	  RX=Remoter_Ch3-1500;

    //Right stick left and right. To control the rotation. 
		//ÓÒÒ¡¸Ë×óÓÒ·½Ïò¡£¿ØÖÆ×Ô×ª¡£
    RY=Remoter_Ch1-1500; 

    if(LX>-Threshold&&LX<Threshold)LX=0;
    if(LY>-Threshold&&LY<Threshold)LY=0;
    if(RX>-Threshold&&RX<Threshold)RX=0;
	  if(RY>-Threshold&&RY<Threshold)RY=0;
		
		//Throttle related //ÓÍÃÅÏà¹Ø
		Remote_RCvelocity=RC_Velocity+RX;
	  if(Remote_RCvelocity<0)Remote_RCvelocity=0;
		
		//The remote control command of model aircraft is processed
		//¶Ôº½Ä£Ò£¿Ø¿ØÖÆÃüÁî½øĞĞ´¦Àí
    Move_Z= LX*(PI/2)/500; 
		Move_Z=-LY*(PI/2)/500;
		Move_Z=-RY*(PI/2)/500;      
			 
		//ZÖáÊı¾İ×ª»¯
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Move_Z=Move_Z*Remote_RCvelocity/500;
		}	


		
	  //Unit conversion, mm/s -> m/s
    //µ¥Î»×ª»»£¬mm/s -> m/s	

		
	  //Data within 1 second after entering the model control mode will not be processed
	  //¶Ô½øÈëº½Ä£¿ØÖÆÄ£Ê½ºó1ÃëÄÚµÄÊı¾İ²»´¦Àí
    if(thrice>0) Move_X=0,Move_Z=0,thrice--;
			Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;	
		//Control target value is obtained and kinematics analysis is performed
	  //µÃµ½¿ØÖÆÄ¿±êÖµ£¬½øĞĞÔË¶¯Ñ§·ÖÎö
		Drive_Motor(Move_X,Move_Y,Move_Z);
}
/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
º¯Êı¹¦ÄÜ£ºµ¥»÷ÓÃ»§°´¼ü¸üĞÂÍÓÂİÒÇÁãµã
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Key(void)
{	
    u8 tmp;
    //´«ÈëÈÎÎñµÄÆµÂÊ
    tmp=KEY_Scan(RATE_100_HZ,0);
		if(Check==0)
		{
    //µ¥»÷ »ò ÊÖ±úÍ¬Ê±°´ÏÂÁ½±ßµÄÏÂ°â»ú£¬¿ªÆô×Ô¶¯»Ø³ä
    if(tmp==single_click || ( Get_PS2_KEY(L2_KEY) && Get_PS2_KEY(R2_KEY) ) )
	{
		Allow_Recharge=!Allow_Recharge;
		ImuData_copy(&imu.Deviation_gyro,&imu.gyro);
        ImuData_copy(&imu.Deviation_accel,&imu.accel);
	}		

    //Ë«»÷ »ò ÊÖ±úÍ¬Ê±°´ÏÂÁ½±ßµÄÒ¡¸Ë,¸üĞÂÍÓÂİÒÇ
    else if(tmp==double_click || ( Get_PS2_KEY(LF_ROCKER_KEY) && Get_PS2_KEY(RT_ROCKER_KEY) )  ) 
	{
		ImuData_copy(&imu.Deviation_gyro,&imu.gyro);
        ImuData_copy(&imu.Deviation_accel,&imu.accel);
	}

    //³¤°´ ÇĞ»»Ò³Ãæ
    else if(tmp==long_click )
    {
        oled_refresh_flag=1;
        oled_page++;
        if(oled_page>OLED_MAX_Page-1) oled_page=0;
    }
	}
		else if(Check==1)
		{
			if(tmp==single_click)		
			{
				Proc_Flag++;
				if(Proc_Flag==21)			
				{
					Check = 0;
					Buzzer = 0;
					Proc_Flag = 0;
					check_time_count_motor_forward=300;
					check_time_count_motor_retreat=500;
					Servo_Count[0] = Servo_Count[1] = Servo_Count[2] = Servo_Count[3] = Servo_Count[4] = Servo_Count[5] = 500;
					servo_direction[0] = servo_direction[1] = servo_direction[2] = servo_direction[3] = servo_direction[4] = servo_direction[5] = 0;
					TIM_ITConfig(TIM8, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,	ENABLE); 
				}
			}
			else if(tmp==double_click)
			{
				Check = 0;
				Buzzer = 0;
				Proc_Flag = 0;
				check_time_count_motor_forward=300;
				check_time_count_motor_retreat=500;
				Servo_Count[0] = Servo_Count[1] = Servo_Count[2] = Servo_Count[3] = Servo_Count[4] = Servo_Count[5] = 500;
				servo_direction[0] = servo_direction[1] = servo_direction[2] = servo_direction[3] = servo_direction[4] = servo_direction[5] = 0;
				TIM_ITConfig(TIM8, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,	ENABLE); 
			}
		}
}
/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
º¯Êı¹¦ÄÜ£º¶ÁÈ¡±àÂëÆ÷ÊıÖµ²¢¼ÆËã³µÂÖËÙ¶È£¬µ¥Î»m/s
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	  //Retrieves the original data of the encoder
	  //»ñÈ¡±àÂëÆ÷µÄÔ­Ê¼Êı¾İ
		float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; 
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		OriginalEncoder.C=Read_Encoder(4);	
		OriginalEncoder.D=Read_Encoder(5);	

	//test_num=OriginalEncoder.B;
	
	  //Decide the encoder numerical polarity according to different car models
		//¸ù¾İ²»Í¬Ğ¡³µĞÍºÅ¾ö¶¨±àÂëÆ÷ÊıÖµ¼«ĞÔ
		switch(Car_Mode)
		{
			case Mec_Car:       Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr=-OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break; 
			case Omni_Car:      Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr=-OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break;
			case Akm_Car:       Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break;
			case Diff_Car:      Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
			case FourWheel_Car: Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr=-OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break; 
			case Tank_Car:      Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
		}
		
		//The encoder converts the raw data to wheel speed in m/s
		//±àÂëÆ÷Ô­Ê¼Êı¾İ×ª»»Îª³µÂÖËÙ¶È£¬µ¥Î»m/s
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
º¯Êı¹¦ÄÜ£º¶ÔÈıÖáÄ¿±êËÙ¶È×öÆ½»¬´¦Àí
Èë¿Ú²ÎÊı£ºÈıÖáÄ¿±êËÙ¶È
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.005;
	
	if(PS2_ON_Flag)
	{
		step=0.05;
	}
	else
	{
		step=0.01;
	}
	
	if	   (vx>0) 	smooth_control.VX+=step;
	else if(vx<0)		smooth_control.VX-=step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9f;
	
	if	   (vy>0)   smooth_control.VY+=step;
	else if(vy<0)		smooth_control.VY-=step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9f;
	
	if	   (vz>0) 	smooth_control.VZ+=step;
	else if(vz<0)		smooth_control.VZ-=step;
	else if(vz==0)	smooth_control.VZ=smooth_control.VZ*0.9f;
	
	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
º¯Êı¹¦ÄÜ£º¸¡µãĞÍÊı¾İ¼ÆËã¾ø¶ÔÖµ
Èë¿Ú²ÎÊı£º¸¡µãÊı
·µ»Ø  Öµ£ºÊäÈëÊıµÄ¾ø¶ÔÖµ
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}

u32 int_abs(int a)
{
	u32 temp;
	if(a<0) temp=-a;
	else temp = a;
	return temp;
}

/**************************************************************************
Function: Prevent the potentiometer to choose the wrong mode, resulting in initialization error caused by the motor spinning.Out of service
Input   : none
Output  : none
º¯Êı¹¦ÄÜ£º·ÀÖ¹µçÎ»Æ÷Ñ¡´íÄ£Ê½£¬µ¼ÖÂ³õÊ¼»¯³ö´íÒı·¢µç»úÂÒ×ª¡£ÒÑÍ£Ö¹Ê¹ÓÃ
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void robot_mode_check(void)
{
	static u8 error=0;

	if(abs(MOTOR_A.Motor_Pwm)>2500||abs(MOTOR_B.Motor_Pwm)>2500||abs(MOTOR_C.Motor_Pwm)>2500||abs(MOTOR_D.Motor_Pwm)>2500)   error++;
	//If the output is close to full amplitude for 6 times in a row, it is judged that the motor rotates wildly and makes the motor incapacitated
	//Èç¹ûÁ¬Ğø6´Î½Ó½üÂú·ùÊä³ö£¬ÅĞ¶ÏÎªµç»úÂÒ×ª£¬ÈÃµç»úÊ§ÄÜ	
	if(error>6) EN=0,Flag_Stop=1,robot_mode_check_flag=1;  
}

//PWMÏû³ıº¯Êı
void auto_pwm_clear(void)
{
	//Ğ¡³µ×ËÌ¬¼òÒ×ÅĞ¶Ï
	float y_accle = (float)(imu.accel.y/1671.84f);//YÖá¼ÓËÙ¶ÈÊµ¼ÊÖµ
	float z_accle = (float)(imu.accel.z/1671.84f);//ZÖá¼ÓËÙ¶ÈÊµ¼ÊÖµ
	float diff;
	
	//¼ÆËãY¡¢Z¼ÓËÙ¶ÈÈÚºÏÖµ£¬¸ÃÖµÔ½½Ó½ü9.8£¬±íÊ¾Ğ¡³µ×ËÌ¬Ô½Ë®Æ½
	if( y_accle > 0 ) diff  = z_accle - y_accle;
	else diff  = z_accle + y_accle;
	
//	debug_show_diff = diff;
	
	//PWMÏû³ı¼ì²â
	if( MOTOR_A.Target !=0.0f || MOTOR_B.Target != 0.0f || MOTOR_C.Target != 0.0f || MOTOR_D.Target != 0.0f )
	{
		start_check_flag = 1;//±ê¼ÇĞèÒªÇå¿ÕPWM
		wait_clear_times = 0;//¸´Î»Çå¿Õ¼ÆÊ±
		start_clear = 0;     //¸´Î»Çå³ı±êÖ¾
		
		
		//ÔË¶¯Ê±Ğ±ÆÂ¼ì²âµÄÊı¾İ¸´Î»
		clear_done_once = 0;
		clear_again_times=0;
	}
	else //µ±Ä¿±êËÙ¶ÈÓÉ·Ç0±ä0Ê±£¬¿ªÊ¼¼ÆÊ± 2.5 Ãë£¬ÈôĞ¡³µ²»ÔÚĞ±ÆÂ×´Ì¬ÏÂ£¬Çå¿Õpwm
	{
		if( start_check_flag==1 )
		{
			wait_clear_times++;
			if( wait_clear_times >= 250 )
			{
				//Ğ¡³µÔÚË®Æ½ÃæÉÏÊ±²Å±ê¼ÇÇå¿Õpwm£¬·ÀÖ¹Ğ¡³µÔÚĞ±ÆÂÉÏÔË¶¯³öÏÖÁïÆÂ
				if( diff > 8.8f )	start_clear = 1,clear_state = 0;//¿ªÆôÇå³ıpwm
				else clear_done_once = 1;//Ğ¡³µÔÚĞ±ÆÂÉÏ£¬±ê¼ÇÒÑÍê³ÉÇå³ı
				
				start_check_flag = 0;
			}
		}
		else
		{
			wait_clear_times = 0;
		}
	}

	//Íê³ÉÁËÇå³ıºó£¬Èô³öÏÖÍÆ³µĞĞÎª£¬pwm»ıÀÛÒ»¶¨ÊıÖµºó½«ÔÚ10ÃëºóÔÙ´ÎÇå¿Õ
	if( clear_done_once )
	{
		//Ğ¡³µ½Ó½üÓÚË®Æ½ÃæÊ±²Å×÷»ıÀÛÏû³ı£¬·ÀÖ¹Ğ¡³µÔÚĞ±ÆÂÉÏÁï³µ
		if( diff > 8.8f )
		{
			//Íê³ÉÇå³ıºópwmÔÙ´Î»ıÀÛ£¬ÖØĞÂÇå³ı
			if( int_abs(MOTOR_A.Motor_Pwm)>300 || int_abs(MOTOR_B.Motor_Pwm)>300 || int_abs(MOTOR_C.Motor_Pwm)>300 || int_abs(MOTOR_D.Motor_Pwm)>300 )
			{
				clear_again_times++;
				if( clear_again_times>1000 )
				{
					clear_done_once = 0;
					start_clear = 1;//¿ªÆôÇå³ıpwm
					clear_state = 0;
				}
			}
			else
			{
				clear_again_times = 0;
			}
		}
		else
		{
			clear_again_times = 0;
		}

	}
}

