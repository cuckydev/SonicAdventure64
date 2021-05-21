#include "sonic_ground.h"

#include "../engine/math_util.h"

void Sonic_GroundMovement(SonicState *s)
{
	u8 hasControl;
	s16 controlAngle;
	f32 controlMagnitude;
	f32 weight;
	Vector gravity;
	f32 forwardY;
	f32 frict, frictFactor;
	
	//Get our control state
	hasControl = Sonic_CheckPower(s, &controlAngle, &controlMagnitude);
	
	//Get our weight
	if (s->task.flag & SF_UNDERWATER)
		weight = s->p->weight * 0.45f;
	else
		weight = s->p->weight;
	
	//Get our gravity direction
	gravity.x = weight * s->gravity.x;
	gravity.y = weight * s->gravity.y;
	gravity.z = weight * s->gravity.z;
	Sonic_ConvertVector_G2P(s, &gravity);
	
	forwardY = -gravity.x / weight;
	
	//Amplify our gravity force
	if (abs(forwardY) >= 0.5f && gravity.x < 0.0001f && s->spd.x < 0.0001f)
	{
		//Factor depends on speed when gravity is working against us
		frictFactor = -s->spd.x / 5.0;
		if (frictFactor > 1.0f)
			frictFactor = 1.0f;
		gravity.x *= 1.5f - frictFactor;
	}
	else
	{
		//Constant 0.8 factor when gravity is working with us
		gravity.x *= 0.8f;
	}
	gravity.z *= 1.0f + sqrtf(abs(gravity.z) / weight) * 0.35f;
	
	//Apply gravity onto our speed
	s->spd.x += gravity.x;
	s->spd.y += gravity.y;
	s->spd.z += gravity.z;
	
	//Drag
	s->spd.x += s->spd.x * s->p->air_resist * 1.7f;
	s->spd.y += s->spd.y * s->p->air_resist_y * ((s->task.flag & SF_UNDERWATER) ? 1.5f : 1.0f);
	s->spd.z += s->spd.z * s->p->air_resist_z * 0.8f;
	
	//Get our abs(controlAngle) and check if we should skid
	if (s->task.mode != SM_SKID && ((s->spd.x <= s->p->run_speed) || (abs(controlAngle) <= 0x6000)))
	{
		//Turn
		if (abs(s->spd.x) < 0.1f && abs(controlAngle) >= 0x1000)
		{
			//Turn immediately if standing still
			Sonic_Turn(s, controlAngle);
			controlMagnitude = 0.0f;
		}
		else
		{
			if (s->spd.x < (s->p->jog_speed + s->p->run_speed) * 0.5f)
			{
				if (s->spd.x < 0.0f || abs(controlAngle) >= 0x1000)
				{
					if (s->spd.x < s->p->dash_speed)
					{
						if (s->spd.x >= s->p->jog_speed && s->spd.x <= s->p->rush_speed && abs(controlAngle) >= 0x2000)
							controlMagnitude *= 0.8f;
						Sonic_AdjustAngleY(s, controlAngle);
					}
					else
					{
						Sonic_AdjustAngleYS(s, controlAngle);
					}
				}
				else
				{
					Sonic_AdjustAngleYS(s, controlAngle);
				}
			}
			else
			{
				Sonic_AdjustAngleY(s, controlAngle);
			}
		}
		
		if (controlMagnitude != 0.0f)
		{
			//Acceleration
			frictFactor = (((abs(forwardY) >= 0.5f && s->spd.x < 0.2f) || s->spd.x < -0.1f) ?
							(0.1f - sqrtf(abs(forwardY)) * 0.1f) :
							(1.0f - (forwardY > 0 ? sqr(forwardY) : forwardY) * 0.125f));
			s->spd.x += ((0.0608f - ((s->task.flag & SF_UNDERWATER) ? (s->p->run_accel * (1.0f - 0.65f)) : 0.0f)) - s->p->grd_frict) * controlMagnitude * frictFactor;
		}
	}
	else
	{
		if (abs(controlAngle) < 0x4000 && controlMagnitude != 0.0f)
		{
			//Stop skidding if holding forward
			s->task.mode = SM_WALK;
		}
		else
		{
			//Slow down
			s->spd.x += s->p->run_break;
			
			if (s->spd.x <= 0.0f)
			{
				if (s->task.mode == SM_SKID)
				{
					//Stop skidding
					s->task.mode = SM_WALK;
					if (controlMagnitude != 0.0f)
					{
						//Turn around if still holding on the analogue stick
						Sonic_Turn(s, 0x8000);
						s->anim = SA_TURN_AROUND;
						s->spd.x = -s->p->run_break;
					}
					else
					{
						//Stop skidding if analogue stick is neutral
						s->anim = SA_STOP_SKID;
						s->spd.x = 0.0f;
					}
				}
			}
			else if (s->task.mode != SM_SKID && abs(forwardY) < 0.5f)
			{
				//Enter skid state
				s->task.mode = SM_SKID;
			}
		}
	}
	
	//Friction X
	if (s->spd.x > 0.0001f && gravity.x < 0.0001f)
		frictFactor = 1.0f; //Gravity should work against us going up a slope
	else
		frictFactor = 1.0f - (abs(gravity.x) / weight);
	frict = s->p->grd_frict * frictFactor;
	
	if (s->spd.x > 0.0f)
		s->spd.x = (s->spd.x < -frict) ? 0.0f : s->spd.x + frict;
	else if (s->spd.x < 0.0f)
		s->spd.x = (s->spd.x > frict) ? 0.0f : s->spd.x - frict;
	
	//Friction Z
	frictFactor = 1.0f - (abs(gravity.z) / weight);
	frict = s->p->grd_frict_z * frictFactor;
	
	if (s->spd.z > 0.0f)
		s->spd.z = (s->spd.z < -frict) ? 0.0f : s->spd.z + frict;
	else if (s->spd.z < 0.0f)
		s->spd.z = (s->spd.z > frict) ? 0.0f : s->spd.z - frict;
	
	//Push us out of being stuck on ramps
	if (controlMagnitude != 0.0f && abs(forwardY) >= 0.5f && abs(s->spd.x) < 0.2f)
		s->spd.x += ((gravity.x > 0.0f) ? 0.1f : -0.1f);
}

void Sonic_SpindashMovement(SonicState *s)
{
	u8 hasControl;
	s16 controlAngle;
	f32 controlMagnitude;
	f32 weight;
	Vector gravity;
	
	//Get our control state
	hasControl = Sonic_CheckPower(s, &controlAngle, &controlMagnitude);
	
	//Get our weight
	if (s->task.flag & SF_UNDERWATER)
		weight = s->p->weight * 0.45f;
	else
		weight = s->p->weight;
	
	//Get our gravity direction
	gravity.x = weight * s->gravity.x;
	gravity.y = weight * s->gravity.y;
	gravity.z = weight * s->gravity.z;
	Sonic_ConvertVector_G2P(s, &gravity);
	
	//Amplify gravity force
	gravity.x *= 2.0f;
	gravity.z *= 20.0f;
	
	//Apply gravity onto our speed
	s->spd.x += gravity.x;
	s->spd.y += gravity.y;
	s->spd.z += gravity.z;
	
	//Drag
	s->spd.x += s->spd.x * s->p->air_resist;
	s->spd.y += s->spd.y * s->p->air_resist_y * ((s->task.flag & SF_UNDERWATER) ? 1.5f : 1.0f);
	s->spd.z += s->spd.z * s->p->air_resist_z;
	
	//Friction X
	if (s->spd.x > 0.0f)
		s->spd.x = (s->spd.x < -s->p->grd_frict) ? 0.0f : s->spd.x + s->p->grd_frict;
	else if (s->spd.x < 0.0f)
		s->spd.x = (s->spd.x > s->p->grd_frict) ? 0.0f : s->spd.x - s->p->grd_frict;
	
	//Friction Z
	if (s->spd.z > 0.0f)
		s->spd.z = (s->spd.z < -s->p->grd_frict_z) ? 0.0f : s->spd.z + s->p->grd_frict_z;
	else if (s->spd.z < 0.0f)
		s->spd.z = (s->spd.z > s->p->grd_frict_z) ? 0.0f : s->spd.z - s->p->grd_frict_z;
	
	//Turning
	if (controlAngle < -0x1000)
		controlAngle = -0x1000;
	if (controlAngle > 0x1000)
		controlAngle = 0x1000;
	Sonic_Turn(s, controlAngle);
}

void Sonic_RollMovement(SonicState *s)
{
	u8 hasControl;
	s16 controlAngle;
	f32 controlMagnitude;
	f32 weight;
	Vector gravity;
	
	//Get our control state
	hasControl = Sonic_CheckPower(s, &controlAngle, &controlMagnitude);
	
	//Get our weight
	if (s->task.flag & SF_UNDERWATER)
		weight = s->p->weight * 0.45f;
	else
		weight = s->p->weight;
	
	//Get our gravity direction
	gravity.x = weight * s->gravity.x;
	gravity.y = weight * s->gravity.y;
	gravity.z = weight * s->gravity.z;
	Sonic_ConvertVector_G2P(s, &gravity);
	
	//Amplify gravity force depending if going up or down slope
	if ((gravity.x > 0.0001f && s->spd.x > 0.0001f) || (gravity.x < 0.0001f && s->spd.x < 0.0001f))
		gravity.x *= 1.0f + sqrtf(abs(gravity.x) / weight) * 2.5f;
	else
		gravity.x *= 0.8f;
	gravity.z *= 1.0f + sqr(gravity.z / weight) * 6.0f;
	
	//Apply gravity onto our speed
	s->spd.x += gravity.x;
	s->spd.y += gravity.y;
	s->spd.z += gravity.z;
	
	//Drag
	s->spd.x += s->spd.x * s->p->air_resist;
	s->spd.y += s->spd.y * s->p->air_resist_y * ((s->task.flag & SF_UNDERWATER) ? 1.5f : 1.0f);
	s->spd.z += s->spd.z * s->p->air_resist_z;
	
	//Turning
	if (controlAngle < -0x800)
		controlAngle = -0x800;
	if (controlAngle > 0x800)
		controlAngle = 0x800;
	Sonic_Turn(s, controlAngle);
}
