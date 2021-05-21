#include "sonic_air.h"

#include "../engine/math_util.h"

void Sonic_AirMovement(SonicState *s)
{
	u8 hasControl;
	s16 controlAngle;
	f32 controlMagnitude;
	f32 weight;
	f32 accel;
	
	//Get our control state
	hasControl = Sonic_CheckPower(s, &controlAngle, &controlMagnitude);
	
	//Get our weight
	if (s->task.flag & SF_UNDERWATER)
		weight = s->p->weight * 0.45f;
	else
		weight = s->p->weight;
	
	//Apply gravity onto our speed
	Sonic_ConvertVector_P2G(s, &s->spd);
	s->spd.x += weight * s->gravity.x;
	s->spd.y += weight * s->gravity.y;
	s->spd.z += weight * s->gravity.z;
	Sonic_ConvertVector_G2P(s, &s->spd);
	
	//Air drag
	s->spd.x += s->spd.x * s->p->air_resist_air;
	s->spd.y += s->spd.y * s->p->air_resist_y * ((s->task.flag & SF_UNDERWATER) ? 1.5f : 1.0f);
	s->spd.z += s->spd.z * s->p->air_resist_z;
	
	//Use lighter gravity if A is held
	if (hasControl && s->jumpTimer > 0 && (s->task.flag & SF_BALL_AURA) && (s->held & A_BUTTON))
	{
		s->jumpTimer--;
		s->spd.y += s->p->jmp_addit * 0.8f;
	}
	
	//Get our acceleration
	if (!hasControl)
	{
		//No acceleration
		accel = 0.0f;
	}
	else
	{
		//Check if we should "skid"
		if ((s->spd.x <= s->p->run_speed) || (abs(controlAngle) <= 0x6000))
		{
			if (abs(controlAngle) <= 0x1000)
			{
				if (s->spd.y >= 0.0f)
					accel = s->p->air_accel * controlMagnitude;
				else
					accel = s->p->air_accel * 2.0f * controlMagnitude;
			}
			else
			{
				accel = 0.0f;
			}
			
			Sonic_AdjustAngleY(s, controlAngle);
		}
		else
		{
			//Air brake
			accel = s->p->air_break * controlMagnitude;
		}
	}
	
	//Accelerate
	s->spd.x += accel;
}
