#include "AP_MotorCheck.h"
#include <AP_Logger/AP_Logger.h>

bool MotorCheck::first_check=false;
bool MotorCheck::second_check=false;
bool MotorCheck::atk_flag[4]={false};
int16_t MotorCheck::atk_para=0;
double MotorCheck::delta[3] = {0,0,0};
bool MotorCheck::door=false;

float MotorCheck::x[12]={0};
float MotorCheck::true_x[12]={0};
float MotorCheck::dx[12]={0};
float MotorCheck::y[12]={0};
float MotorCheck::u[4]={0};
int16_t MotorCheck::last_atk[6]={0};
void MotorCheck::quadrotor_m()
{
    int i;
    float b_x[16];

    // IDNLGREY model file (discrete-time nonlinear model) :
    // xn = x(t+Ts) : state update values in discrete-time case (A column vector with Nx entries)
    // y : outputs values (A column vector with Ny entries)
    //  gravity acceleration constant (m/s^2)
    //  inputs
    //      x(13:16)=u;
    for (i = 0; i < 12; i++)
    {
        b_x[i] = x[i];
    }

    for (i = 0; i < 4; i++)
    {
        b_x[i + 12] = u[i];
    }

    // -------------------------------------------------
    dx[0] = b_x[6];
    dx[1] = b_x[7];
    dx[2] = b_x[8];
    dx[3] = (b_x[9] + sinf(b_x[3]) * tanf(b_x[4]) * b_x[10]) + cosf(b_x[3]) * tanf(b_x[4]) * b_x[11];
    dx[4] = cosf(b_x[3]) * b_x[10] - sinf(b_x[3]) * b_x[11];
    dx[5] = sinf(b_x[3]) / cosf(b_x[4]) * b_x[10] + cosf(b_x[3]) / cosf(b_x[4]) * b_x[11];
    dx[6] = K_T / m * (((b_x[12] + b_x[13]) + b_x[14]) + b_x[15]) * (cosf(b_x[3]) * sinf(b_x[4]) * cosf(b_x[5]) + sinf(b_x[3]) * sinf(b_x[5]));
    dx[7] = K_T / m * (((b_x[12] + b_x[13]) + b_x[14]) + b_x[15]) * (cosf(b_x[3]) * sinf(b_x[4]) * sinf(b_x[5]) - sinf(b_x[3]) * cosf(b_x[5]));
    dx[8] = K_T / m * (((b_x[12] + b_x[13]) + b_x[14]) + b_x[15]) * (cosf(b_x[3]) * cosf(b_x[4])) - 9.80665F;
    dx[9] = (I_y - I_z) / I_x * b_x[10] * b_x[11] + K_T / I_x * (((-a * b_x[12] + d * b_x[13]) + a * b_x[14]) - d * b_x[15]);
    dx[10] = (I_x - I_z) / I_y * b_x[9] * b_x[11] + K_T / I_y * (((-b * b_x[12] + c * b_x[13]) - b * b_x[14]) + c * b_x[15]);
    dx[11] = (I_x - I_y) / I_z * b_x[9] * b_x[10] + K_Q / I_z * (((-b_x[12] - b_x[13]) + b_x[14]) + b_x[15]);

    //
    for (i = 0; i < 12; i++)
    {
        y[i] = x[i];
    }

    //  update outputs
}
/*
void MotorCheck::transfromNED2ENU(float state[12])
{
    float x[12] = {0};
    x[0] = state[1];
    x[1] = state[0];
    x[2] = -state[2];
    x[3] = state[3];
    x[4] = -state[4];
    x[5] = wrap_2PI(-state[5] + M_PI / 2);
    x[6] = state[7];
    x[7] = state[6];
    x[8] = -state[8];
    x[9] = state[9];
    x[10] = -state[10];
    x[11] = -state[11];
    for (int i = 0; i < 12; i++)
    {
        state[i] = x[i];
    }
}
*/
void MotorCheck::transfromNED2ENU(float state[12])
{
    x[0] = state[1];
    x[1] = state[0];
    x[2] = -state[2];
    x[3] = state[3];
    x[4] = -state[4];
    x[5] = wrap_2PI(-state[5] + M_PI / 2);
    x[6] = state[7];
    x[7] = state[6];
    x[8] = -state[8];
    x[9] = state[9];
    x[10] = -state[10];
    x[11] = -state[11];
}


void MotorCheck::transfromENU2NED(float in[12], float out[12])
{
    out[0] = in[1];
    out[1] = in[0];
    out[2] = -in[2];
    out[3] = in[3];
    out[4] = -in[4];
    out[5] = wrap_2PI(-(in[5] - M_PI / 2));
    out[6] = in[7];
    out[7] = in[6];
    out[8] = -in[8];
    out[9] = in[9];
    out[10] = -in[10];
    out[11] = -in[11];
}

void MotorCheck::updateState()
{
    for (int i = 0; i < 12; i++)
    {
        x[i] += dx[i] * dt;
    }
    x[5] = wrap_2PI(x[5]);
}

void MotorCheck::firstCheck(const struct log_Bob_EKF1 &sensor_pkt, const struct log_motors &motor_pkt)
{
    uint64_t time_us = sensor_pkt.time_us;
    
    //prepare dt, unit: s
    dt = (float)((double)(time_us - last_time_us) * 1e-6);
    last_time_us = time_us;

    //prepare input
    u[0] = transformInput(motor_pkt.motor1);
    u[1] = transformInput(motor_pkt.motor2);
    u[2] = transformInput(motor_pkt.motor3);
    u[3] = transformInput(motor_pkt.motor4);

    // update old states to predict current states.
    for (int i = 0; i < 12; i++)
    {
        last_x[i]=x[i];
    }
    MotorCheck::updateState();

    true_x[0] = sensor_pkt.posN;
    true_x[1] = sensor_pkt.posE;
    true_x[2] = sensor_pkt.posD;
    true_x[3] = sensor_pkt.roll;
    true_x[4] = sensor_pkt.pitch;
    true_x[5] = wrap_2PI(sensor_pkt.yaw);
    true_x[6] = sensor_pkt.velN;
    true_x[7] = sensor_pkt.velE;
    true_x[8] = sensor_pkt.velD;
    true_x[9] = sensor_pkt.gyrX;
    true_x[10] = sensor_pkt.gyrY;
    true_x[11] = sensor_pkt.gyrZ;
    MotorCheck::transfromNED2ENU(true_x);


    // get current output and calculate dx for next time.
    MotorCheck::quadrotor_m();
    //air resistence in static air
    for (int i = 0; i < 3; i++)
    {
        dx[6 + i] += -x[6 + i] * 9.80665F / 15.0F;
        dx[9 + i] += -x[9 + i] * 6.98131704F / 25.1327419F;
    }
    //on ground check
    float frame_height = 0.1;
    if ((x[2] - frame_height < 0.001F) && (dx[8] <= 0.0F))
    {
        //  on ground
        dx[8] = 0.0F;
    }

    //
   
    residual[0] = abs(true_x[6] - y[6]);
    residual[1] = abs(true_x[7] - y[7]);
    residual[2] = abs(true_x[8] - y[8]);

    cusum();
}

void MotorCheck::cusum()
{
	
	for(int i =0;i<3;i++)
	{
		if(delta[i]+residual[i]-bb[i]<0)
			delta[i]=0;
		else
			delta[i]+=(residual[i]-bb[i]);
	}

	for(int i=0;i<3;i++)
	{
		if(delta[i]>threshold1[i])
			first_check = true;
	}

}
void MotorCheck::secondCheck()
{
    e[0]=abs((u[0]-u[1])*900);
    e[1]=abs((u[2]-u[3])*900);
    e[2]=abs((u[0]+u[1]-u[2]-u[3])*900);
    e[3]=(u[0]+u[1]+u[2]+u[3])*900+4400;
    if(e[0]>threshold2[0] and e[1]<threshold2[1] and e[2]<threshold2[2])
    {
        second_check = true;
        getPara();
	u[0]>u[1]?atk_flag[0]=true:atk_flag[1]=true;
	if(door==false)
	    atk_para=(atk[0]+atk[1]+atk[2])/3; 
    }else if(e[0]<threshold2[0] and e[1]>threshold2[1] and e[2]<threshold2[2])
    {
	second_check = true;
        getPara();
	u[2]>u[3]?atk_flag[2]=true:atk_flag[3]=true;
	if(door==false)
	    atk_para=(atk[0]+atk[1]+atk[2])/3; 
    }else if(e[0]>threshold2[0] and e[1]>threshold2[1] and e[2]<threshold2[2])
    {
	second_check = true;
        getPara();
	u[0]>u[1]?atk_flag[0]=true:atk_flag[1]=true;
	u[2]>u[3]?atk_flag[2]=true:atk_flag[3]=true;
	if(door==false)
	    atk_para=(atk[0]+atk[1]+atk[2])/3/1.4; 
    }else if(e[0]<threshold2[0] and e[1]<threshold2[1] and e[2]>threshold2[2])
    {
	second_check = true;
        getPara();
	(u[0]+u[1])>(u[2]+u[3])?(atk_flag[0]=true,atk_flag[1]=true):(atk_flag[2]=true,atk_flag[3]=true);
	if(door==false)
	    atk_para=(atk[0]+atk[1]+atk[2])/3*1.8; //1.8,1.9 is OK
    }else if(e[0]>threshold2[0] and e[1]<threshold2[1] and e[2]>threshold2[2])
    {
	second_check = true;
        getPara();
	u[0]>u[1]?atk_flag[0]=true:atk_flag[1]=true;
	atk_flag[2]=true;
        atk_flag[3]=true;
	if(door==false)
	    atk_para=(atk[0]+atk[1]+atk[2])/3*1.2; //1.2 is OK
    }else if(e[0]<threshold2[0] and e[1]>threshold2[1] and e[2]>threshold2[2])
    {
	second_check = true;
        getPara();
	u[2]>u[3]?atk_flag[2]=true:atk_flag[3]=true;
	atk_flag[0]=true;
        atk_flag[1]=true;
	if(door==false)
	    atk_para=(atk[0]+atk[1]+atk[2])/3*1.2; 
    }else if(e[3]>threshold2[3])
    {
	second_check = true;
        getPara();
	atk_flag[0]=true;
        atk_flag[1]=true;
	atk_flag[2]=true;
        atk_flag[3]=true;
	if(door==false)
	    atk_para=(atk[0]+atk[1]+atk[2])/3/2+250; 
    }

    /*
    if(abs((u[0]-u[1])*900)>threshold2[0] or abs((u[2]-u[3])*900)>threshold2[1])
    {
	second_check = true;

        getPara();
	if(((u[0]-u[1])*900)>threshold2[0])
		u[0]>u[1]?atk_flag[0]=true:atk_flag[1]=true;
	if(((u[2]-u[3])*900)>threshold2[1])
		u[2]>u[3]?atk_flag[2]=true:atk_flag[3]=true;
        int atk_num=0;
	for(int i=0;i<4;i++)
	{
	    if(atk_flag[i]==true)
		atk_num++;	
	}
	if(atk_num==1)
	    if(door==false)
	    {
		atk_para=(atk[0]+atk[1]+atk[2])/3;
	    }   
	if(atk_num==2)
	    if(door==false)
	    {
		atk_para=(atk[0]+atk[1]+atk[2])/3*1.25;
	    }   
    }
    else
    {
        if((u[0]*900+1100)>1500 and (y[5]-true_x[5])>1)
	{

	    getPara();
            if(first_check == true)
	    {
	            //getPara();
		    second_check = true;
		    atk_flag[0]=true;
		    atk_flag[1]=true;
	    	    if(door==false) 
		    {
			atk_para=(atk[0]+atk[1]+atk[2]+last_atk[0]+last_atk[1]+last_atk[2])/3/2*2;
		    }   
    	    }
	}
	else if((u[0]*900+1100)>1500 and (y[5]-true_x[5])<1)
	{

            getPara();
            if(first_check == true)
	    {
	            //getPara();
		    second_check = true;
		    atk_flag[2]=true;
		    atk_flag[3]=true;
	    	    if(door==false) 
		    {
			atk_para=(atk[0]+atk[1]+atk[2]+last_atk[0]+last_atk[1]+last_atk[2])/3/2*1.25;
		    }   
    	    }	
	}
    }/*
    if((u[0]*900+1100)>1500 and first_check == true and second_check == true)
    {
	getPara();
	if(abs(atk[0]-last_atk[0])>300)
	    first_check =second_check =door= false;
    }*/
}
void MotorCheck::getPara()
{	
    atk[0]=(y[6] - true_x[6]) * m / K_T / dt / (cosf(last_x[3]) * sinf(last_x[4]) * cosf(last_x[5]) + sinf(last_x[3]) * sinf(last_x[5])) * 900;	
    atk[1]=(y[7] - true_x[7]) * m / K_T / dt / (cosf(last_x[3]) * sinf(last_x[4]) * sinf(last_x[5]) - sinf(last_x[3]) * cosf(last_x[5])) * 900;	
    atk[2]=(y[8] - true_x[8]) * m / K_T / dt / (cosf(last_x[3]) * cosf(last_x[4])) * 900;	
    atk[3]=(y[9] - true_x[9]) * I_x / K_T / dt / 0.128364 * 900;
    atk[4]=(y[10] - true_x[10]) * I_y / K_T / dt / 0.128364 * 900;
    atk[5]=(y[11] - true_x[11]) * I_z / K_Q / dt  * 900;	
    for(int i=0;i<6;i++)
    {
	last_atk[i]=atk[i];
    }			
}
float MotorCheck::transformInput(uint16_t pwm)
{
    return constrain_float(((float)(pwm - 1100)) / 900.0, 0.0f, 1.0f);
}
