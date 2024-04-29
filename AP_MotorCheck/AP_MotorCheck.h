#pragma once

#include <cstdlib>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>


class MotorCheck{
    public:
        static bool first_check;
	static bool second_check;
	static bool atk_flag[4];
	static int16_t atk_para;
        static bool door;

    	static float x[12];
    	static float true_x[12];
    	static float dx[12];
    	static float y[12];
    	static float u[4];
	static int16_t last_atk[6];
	float last_x[12]={0};
    	float a = 0.128364;
    	float b = 0.128364;
    	float c = 0.128364;
    	float d = 0.128364;
    	float m = 1.5;
    	float I_x = 0.015;
    	float I_y = 0.015;
    	float I_z = 0.015;
    	float K_T = 7.21077;
    	float K_Q = 0.10472;
    	uint64_t last_time_us = 0;
    	double residual[3];
    	double bb[3] = {0.000297694,0.000356379,0.000261503};
    	
	static double delta[3];
    	int16_t atk[6];

    	float dt=0;
    	double threshold1[3] = {0.00297694/2.4, 0.00356379/2.4, 0.00261503/2.4};
        //double threshold2[2] = {50, 50};
    	double threshold2[4] = {50, 50,200,7200};
	double e[4];

    	

	void quadrotor_m();
    	void transfromNED2ENU(float x[12]);
    	//void transfromNED2ENU(float in[12], float out[12]);
    	void transfromENU2NED(float in[12], float out[12]);
    	void updateState();
    	void firstCheck(const struct log_Bob_EKF1 & sensor_pkt, const struct log_motors & motor_pkt);
    	void secondCheck();
    	void getPara();
    	float transformInput(uint16_t pwm);
    	void cusum();
};
