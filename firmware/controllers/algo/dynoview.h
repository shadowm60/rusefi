/*
 * @file dynoview.h
 *
 * @date Nov 29, 2020
 * @author Alexandru Miculescu, (c) 2012-2020
 */

#pragma once

#include "biquad.h"

#define AIR_DENSITY 	1.225 /* kg/m3 */

void  updateDynoView();
void  updateDynoViewCan();
float getDynoviewAcceleration();
int   getDynoviewPower();
void  initDynoView();

typedef enum{
    ICU = 0,
    CAN,
}vssSrc;

class DynoView {
public:
	// init function
	void init();
	// Update inputs and calculated data
	void update(vssSrc src);
    void updateAcceleration(efitick_t deltaTime, float deltaSpeed);
    void updateHP();
    float getAcceleration();
    int getEngineForce();
    //in KW
    int getEnginePower();

    int getEngineHP();
    //in NM
    int getEngineTorque();
#if EFI_UNIT_TEST
    void setAcceleration(float a);
#endif
private:
	efitimeus_t timeStamp = 0;
    //km/h unit
    float vss = 0;
    //m/s/s unit
    float acceleration = 0;
	//vehicle force in N
	int vehicleForce;
    //engine force in N
    int engineForce;
    //engine power in W
    int enginePower;
    //engine powerin HP
    int engineHP;
    //Torque in lb-ft
    int engineTorque;
    //sign
    uint8_t direction;
	//filter
	Biquad m_filter;
};