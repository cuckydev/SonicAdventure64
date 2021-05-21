#include "sonic.h"

#include <stdio.h>
#include <string.h>
#include "../engine/math_util.h"
#include "../audio/external.h"

#include "sonic_ground.h"
#include "sonic_air.h"
#include "sonic_collision.h"
#include "sonic_math.h"
#include "sonic_trig.h"

#include "area.h"
#include "camera.h"
#include "mario.h"
#include "mario_animation_ids.h"
#include "behavior_data.h"
#include "object_helpers.h"
#include "object_list_processor.h"
#include "level_update.h"
#include "sm64.h"

SonicState gSonicState;

//Sonic physics params (ripped straight from SA1)
static const PhysicsParam sSonicParam = {
	60,
	2.0f,
	16.0f,
	16.0f,
	3.0f,
	0.6f,
	1.66f,
	3.0f,
	0.23f,
	0.46f,
	1.39f,
	2.3f,
	3.7f,
	5.09f,
	0.076f,
	0.05f,
	0.031f,
	-0.06f,
	-0.18f,
	-0.17f,
	-0.028f,
	-0.008f,
	-0.01f,
	-0.4f,
	-0.1f,
	-0.6f,
	-0.2825f,
	0.3f,
	4.0f,
	10.0f,
	0.08f,
	7.0f,
	5.4f
};

//Homable objects
static const BehaviorScript *sHomableBhv[] = {
	bhvSpindrift,
	bhvPiranhaPlant,
	bhvScuttlebug,
	bhvBobomb,
	bhvSmallBully,
	bhvBigBully,
	bhvMoneybag,
	bhvKoopa,
	bhvSwoop,
	bhvFlyGuy,
	bhvGoomba,
	bhvWigglerHead,
	bhvEnemyLakitu,
	bhvMontyMole,
	bhvSnufit,
	bhvEyerokHand,
	bhvSkeeter,
	NULL,
};

//Quaternion
f32 Sonic_QuaternionDot(const Quaternion *lhs, const Quaternion *rhs)
{
	return lhs->x * rhs->x + lhs->y * rhs->y + lhs->z * rhs->z + lhs->w * rhs->w;
}

s16 Sonic_QuaternionAngle(const Quaternion *lhs, const Quaternion *rhs)
{
	f32 dot = Sonic_QuaternionDot(lhs, rhs);
	if (dot > 1.0f)
		return 0;
	else if (dot < -1.0f)
		return 0;
	else
		return Sonic_Acos(dot) * 0x8000 / M_PI * 2;
}

//Coordinate system conversion
void Sonic_GetInvertedQuaternion(Quaternion *res, const Quaternion *orig)
{
	f32 invsqr = 1.0f / (sqr(orig->x) + sqr(orig->y) + sqr(orig->z) + sqr(orig->w));
	res->x = orig->x * -invsqr;
	res->y = orig->y * -invsqr;
	res->z = orig->z * -invsqr;
	res->w = orig->w * invsqr;
}

void Sonic_AngleToMatrix(Matrix_ptr m, const Quaternion *q)
{
	Quaternion qi;
	f32 n, qx, qy, qz, qw, qxs, qys, qzs, *p;
	Sonic_GetInvertedQuaternion(&qi, q);
	
	n = 1.0f / sqrtf(qi.x * qi.x + qi.y * qi.y + qi.z * qi.z + qi.w * qi.w);
	qx = qi.x * n;
	qy = qi.y * n;
	qz = qi.z * n;
	qw = qi.w * n;
	qxs = sqr(qx);
	qys = sqr(qy);
	qzs = sqr(qz);
	
	p = &m[0];
	*p++ = 1.0f - 2.0f * qys - 2.0f * qzs;
	*p++ = 2.0f * qx * qy - 2.0f * qz * qw;
	*p++ = 2.0f * qx * qz + 2.0f * qy * qw;
	*p++ = 0.0f;
	
	*p++ = 2.0f * qx * qy + 2.0f * qz * qw;
	*p++ = 1.0f - 2.0f * qxs - 2.0f * qzs;
	*p++ = 2.0f * qy * qz - 2.0f * qx * qw;
	*p++ = 0.0f;
	
	*p++ = 2.0f * qx * qz - 2.0f *qy * qw;
	*p++ = 2.0f * qy * qz + 2.0f *qx * qw;
	*p++ = 1.0f - 2.0f * qxs - 2.0f * qys;
	*p++ = 0.0f;
	
	*p++ = 0.0f;
	*p++ = 0.0f;
	*p++ = 0.0f;
	*p++ = 1.0f;
}

void Sonic_QuaternionProduct(Quaternion *res, const Quaternion *lhs, const Quaternion *rhs)
{
	Quaternion temp;
	temp.x =  lhs->x * rhs->w + lhs->y * rhs->z - lhs->z * rhs->y + lhs->w * rhs->x;
	temp.y = -lhs->x * rhs->z + lhs->y * rhs->w + lhs->z * rhs->x + lhs->w * rhs->y;
	temp.z =  lhs->x * rhs->y - lhs->y * rhs->x + lhs->z * rhs->w + lhs->w * rhs->z;
	temp.w = -lhs->x * rhs->x - lhs->y * rhs->y - lhs->z * rhs->z + lhs->w * rhs->w;
	//temp.x = (lhs->w * rhs->x + lhs->x * rhs->w + lhs->y * rhs->z - lhs->z * rhs->y);
	//temp.y = (lhs->w * rhs->y + lhs->y * rhs->w + lhs->z * rhs->x - lhs->x * rhs->z);
	//temp.z = (lhs->w * rhs->z + lhs->z * rhs->w + lhs->x * rhs->y - lhs->y * rhs->x);
	//temp.w = (lhs->w * rhs->w - lhs->x * rhs->x - lhs->y * rhs->y - lhs->z * rhs->z);
	res->x = temp.x;
	res->y = temp.y;
	res->z = temp.z;
	res->w = temp.w;
}

void Sonic_QuaternionVectorProduct(Vector *res, const Quaternion *q)
{
	Quaternion qv, qm, t1, t2;
	qv.x = res->x;
	qv.y = res->y;
	qv.z = res->z;
	qv.w = 0.0f;
	
	//Quaternion qm = (*this) * qv * (*this).inverse();
	Sonic_QuaternionProduct(&t1, q, &qv);
	Sonic_GetInvertedQuaternion(&t2, q);
	Sonic_QuaternionProduct(&qm, &t1, &t2);
	
	res->x = qm.x;
	res->y = qm.y;
	res->z = qm.z;
}

void Sonic_ConvertVector_G2P(SonicState *s, Vector *destination)
{
	Quaternion invert;
	Sonic_GetInvertedQuaternion(&invert, &s->task.ang);
	Sonic_QuaternionVectorProduct(destination, &invert);
}

void Sonic_ConvertVector_P2G(SonicState *s, Vector *destination)
{
	Sonic_QuaternionVectorProduct(destination, &s->task.ang);
}

//Planes
f32 Sonic_PlaneProject(Vector *res, const Vector *normal, const Vector *point)
{
	f32 pointToPlaneDistance = njInnerProduct(normal, point);
	res->x = point->x - (normal->x * pointToPlaneDistance);
	res->y = point->y - (normal->y * pointToPlaneDistance);
	res->z = point->z - (normal->z * pointToPlaneDistance);
	return pointToPlaneDistance;
}

//Input functions
u8 Sonic_CheckPower(SonicState *s, s16 *angle, f32 *magnitude)
{
	s16 cameraYaw, glAngle;
	Vector up, invGrav, forward, right, inputVector;
	Quaternion finalRotation, camRotation, charRotation;
	
	//Get the controller used
	struct Controller *controller = s->marioState->controller;
	
	//Get magnitude
	if (magnitude != NULL)
		*magnitude = controller->stickMag / 64.0f;
	
	//Get angle (facing direction if 0 magnitude)
	if (angle != NULL)
	{
		if (controller->stickMag != 0.0f)
		{
			//Get vectors
			cameraYaw = s->marioState->area->camera->yaw;
			glAngle = -atan2s(controller->stickY, -controller->stickX);
			
			inputVector.x = coss(-glAngle);
			inputVector.y = 0.0f;
			inputVector.z = sins(glAngle);
			
			forward.x = 0.0f;
			forward.y = 0.0f;
			forward.z = 1.0f;
			Sonic_ConvertVector_P2G(s, &forward);
			
			right.x = 1.0f;
			right.y = 0.0f;
			right.z = 0.0f;
			Sonic_ConvertVector_P2G(s, &right);
			
			up.x = 0.0f;
			up.y = 1.0f;
			up.z = 0.0f;
			Sonic_ConvertVector_P2G(s, &up);
			
			//Get final rotation
			invGrav.x = -s->gravity.x;
			invGrav.y = -s->gravity.y;
			invGrav.z = -s->gravity.z;
			Sonic_QuaternionFromToRotation(&finalRotation, &invGrav, &s->lastUp);
			Sonic_QuaternionAngleAxis(&camRotation, cameraYaw, &invGrav);
			Sonic_QuaternionProduct(&finalRotation, &finalRotation, &camRotation);
			Sonic_QuaternionFromToRotation(&charRotation, &s->lastUp, &up);
			Sonic_QuaternionProduct(&finalRotation, &charRotation, &finalRotation);
			
			//Update last rotation
			if (njInnerProduct(&invGrav, &up) > -0.999f)
			{
				s->lastUp.x = up.x;
				s->lastUp.y = up.y;
				s->lastUp.z = up.z;
			}
			
			//Get final input vector
			Sonic_QuaternionVectorProduct(&inputVector, &finalRotation);
			*angle = Sonic_VectorAngle(&forward, &inputVector) * (njInnerProduct(&inputVector, &right) > 0.0f ? 1 : -1);
		}
		else
		{
			//No turn
			*angle = 0;
		}
		
		//Remember this turn for animation
		s->lastTurn = *angle;
	}
	return 1;
}

//Rotation
void Sonic_Turn(SonicState *s, s16 turn)
{
	Vector up;
	Quaternion turnQuat;
	
	if (turn)
	{
		//Get turn quaternion
		up.x = 0.0f;
		up.y = 1.0f;
		up.z = 0.0f;
		Sonic_ConvertVector_P2G(s, &up);
		Sonic_QuaternionAngleAxis(&turnQuat, turn, &up);
		Sonic_QuaternionProduct(&s->task.ang, &turnQuat, &s->task.ang);
	}
}

void Sonic_AdjustAngleY(SonicState *s, s16 turn)
{
	Vector prevSpeed;
	s16 maxTurn;
	f32 inertia;
	u8 hasControl;
	
	//Remember our global speed
	prevSpeed.x = s->spd.x;
	prevSpeed.y = s->spd.y;
	prevSpeed.z = s->spd.z;
	Sonic_ConvertVector_P2G(s, &prevSpeed);
	
	//Change our angle
	maxTurn = abs(turn);
	if (maxTurn <= 0x2000)
	{
		if (maxTurn <= 0x1000)
			maxTurn >>= 3;
		else
			maxTurn >>= 2;
	}
	else
	{
		maxTurn = 0x800;
	}
	
	if (turn > maxTurn)
		turn = maxTurn;
	else if (turn < -maxTurn)
		turn = -maxTurn;
	Sonic_Turn(s, turn);
	
	//Handle inertia
	Sonic_ConvertVector_G2P(s, &prevSpeed);
	
	if (!(s->task.flag & SF_GROUNDED))
	{
		s->spd.x = s->spd.x * 0.9f + prevSpeed.x * 0.1f;
		s->spd.y = s->spd.y * 0.9f + prevSpeed.y * 0.1f;
		s->spd.z = s->spd.z * 0.9f + prevSpeed.z * 0.1f;
	}
	else
	{
		hasControl = Sonic_CheckPower(s, NULL, NULL);
		if (hasControl)
		{
			if (s->dotp <= 0.4f)
				inertia = 0.5f;
			else
				inertia = 0.99f;
		}
		else
		{
			inertia = 0.05f;
		}
		
		if (s->frict < 1.0f)
			inertia *= s->frict;
		
		s->spd.x = s->spd.x * (1.0f - inertia) + prevSpeed.x * inertia;
		s->spd.y = s->spd.y * (1.0f - inertia) + prevSpeed.y * inertia;
		s->spd.z = s->spd.z * (1.0f - inertia) + prevSpeed.z * inertia;
	}
}

void Sonic_AdjustAngleYQ(SonicState *s, s16 turn)
{
	Vector prevSpeed;
	
	//Remember our global speed
	prevSpeed.x = s->spd.x;
	prevSpeed.y = s->spd.y;
	prevSpeed.z = s->spd.z;
	Sonic_ConvertVector_P2G(s, &prevSpeed);
	
	//Change our angle
	if (turn > 0x2000)
		turn = 0x2000;
	else if (turn < -0x2000)
		turn = -0x2000;
	Sonic_Turn(s, turn);
	
	//Handle inertia
	Sonic_ConvertVector_G2P(s, &prevSpeed);
	s->spd.x = prevSpeed.x;
	s->spd.y = prevSpeed.y;
	s->spd.z = prevSpeed.z;
}

void Sonic_AdjustAngleYS(SonicState *s, s16 turn)
{
	Vector prevSpeed;
	s16 maxTurn;
	f32 inertia;
	
	//Remember our global speed
	prevSpeed.x = s->spd.x;
	prevSpeed.y = s->spd.y;
	prevSpeed.z = s->spd.z;
	Sonic_ConvertVector_P2G(s, &prevSpeed);
	
	//Change our angle
	if (s->spd.x > s->p->dash_speed)
		maxTurn = 0x100 - (u16)(sqrtf(((s->spd.x - s->p->dash_speed) * 0.0625f)) * 256.0f);
	else
		maxTurn = 0x100;
	
	if (turn > maxTurn)
		turn = maxTurn;
	else if (turn < -maxTurn)
		turn = -maxTurn;
	Sonic_Turn(s, turn);
	
	//Handle inertia
	Sonic_ConvertVector_G2P(s, &prevSpeed);
	
	if (s->dotp <= 0.4f)
		inertia = 0.5f;
	else
		inertia = 0.99f;
	
	s->spd.x = s->spd.x * inertia + prevSpeed.x * (1.0f - inertia);
	s->spd.y = s->spd.y * inertia + prevSpeed.y * (1.0f - inertia);
	s->spd.z = s->spd.z * inertia + prevSpeed.z * (1.0f - inertia);
}

//Sonic state functions
s16 Sonic_Animate()
{
	SonicState *s = &gSonicState;
	Vector tilt;
	s16 turn, visRotate = 0;
	
	//Reset state if animation's changed
	if (s->anim != s->prevAnim)
	{
		s->prevAnim = s->anim;
		s->animMode = 0;
	}
	
	//Set Mario animation
	switch (s->anim)
	{
		case SA_IDLE:
			switch (s->animMode)
			{
				case 0:
					set_mario_animation(s->marioState, MARIO_ANIM_IDLE_HEAD_LEFT);
					break;
				case 1:
					set_mario_animation(s->marioState, MARIO_ANIM_IDLE_HEAD_RIGHT);
					break;
				case 2:
					set_mario_animation(s->marioState, MARIO_ANIM_IDLE_HEAD_CENTER);
					break;
			}
			
			if (is_anim_at_end(s->marioState))
			{
				s->animMode++;
				if (s->animMode >= 3)
					s->animMode = 0;
			}
			break;
		case SA_PUSH:
			set_mario_animation(s->marioState, MARIO_ANIM_PUSHING);
			break;
		case SA_WALK:
			set_mario_anim_with_accel(s->marioState, MARIO_ANIM_TIPTOE, (u32)(s->moveAnimSpeed * 5.0f * 0x10000));
			break;
		case SA_JOG:
			set_mario_anim_with_accel(s->marioState, MARIO_ANIM_WALKING, (u32)(s->moveAnimSpeed * 3.5f * 0x10000));
			break;
		case SA_RUN:
			set_mario_anim_with_accel(s->marioState, MARIO_ANIM_RUNNING, (u32)(s->moveAnimSpeed * 3.0f * 0x10000));
			break;
		case SA_SKID:
			set_mario_animation(s->marioState, MARIO_ANIM_TURNING_PART1);
			break;
		case SA_TURN_AROUND:
			set_mario_animation(s->marioState, MARIO_ANIM_TURNING_PART2);
			if (is_anim_at_end(s->marioState))
				s->anim = SA_IDLE;
			visRotate = 0x8000;
			break;
		case SA_STOP_SKID:
			set_mario_animation(s->marioState, MARIO_ANIM_STOP_SKID);
			if (is_anim_at_end(s->marioState))
				s->anim = SA_IDLE;
			break;
		case SA_BALL:
			set_mario_anim_with_accel(s->marioState, MARIO_ANIM_FORWARD_SPINNING, 0x10000 + s->moveAnimSpeed * 0x4000);
			break;
		case SA_UNCURL:
			set_mario_anim_with_accel(s->marioState, MARIO_ANIM_STOP_CROUCHING, 0x20000);
			if (s->animMode == 0)
				s->marioState->marioBodyState->torsoAngle[0] = -0x4000;
			if (++s->animMode > 3)
				s->animMode = 0x80;
			else
				s->marioState->marioBodyState->torsoAngle[0] += 0x600;
			break;
		case SA_HOMING_END:
		case SA_FALL:
			set_mario_animation(s->marioState, MARIO_ANIM_GENERAL_FALL);
			break;
		case SA_HURT:
			set_mario_animation(s->marioState, MARIO_ANIM_BACKWARD_AIR_KB);
			break;
		case SA_DEAD:
			set_mario_animation(s->marioState, MARIO_ANIM_DYING_ON_BACK);
			
			if (s->marioState->marioObj->header.gfx.unk38.animFrame < 30)
				s->marioState->marioBodyState->eyeState = MARIO_EYES_HALF_CLOSED;
			else
				s->marioState->marioBodyState->eyeState = MARIO_EYES_CLOSED;
				
			if (s->marioState->marioObj->header.gfx.unk38.animFrame == 54)
				level_trigger_warp(s->marioState, WARP_OP_DEATH);
			break;
		case SA_DROWNING:
			switch (s->animMode)
			{
				case 0:
					set_mario_animation(s->marioState, MARIO_ANIM_DROWNING_PART1);
					s->marioState->marioBodyState->eyeState = MARIO_EYES_HALF_CLOSED;
					if (is_anim_at_end(s->marioState))
						s->animMode = 1;
					break;
				case 1:
					set_mario_animation(s->marioState, MARIO_ANIM_DROWNING_PART2);
					s->marioState->marioBodyState->eyeState = MARIO_EYES_CLOSED;
					if (s->marioState->marioObj->header.gfx.unk38.animFrame == 30)
						level_trigger_warp(s->marioState, WARP_OP_DEATH);
					break;
			}
			break;
	}
	
	if (s->anim >= SA_WALK && s->anim <= SA_RUN)
	{
		//Tilt Mario body
		turn = s->lastTurn / 30;
		if (turn > 0x2800)
			turn = 0x2800;
		else if (turn < -0x2800)
			turn = -0x2800;
		
		if (s->anim >= SA_WALK && s->anim <= SA_RUN)
		{
			tilt.x = 0.0f;
			tilt.y = 1.0f;
			tilt.z = 0.0f;
			Sonic_ConvertVector_G2P(s, &tilt);
			
			s->marioState->marioBodyState->torsoAngle[0] = approach_s32(s->marioState->marioBodyState->torsoAngle[0], s->spd.x * 0x400, 0x800, 0x800);
			s->marioState->marioBodyState->torsoAngle[2] = approach_s32(s->marioState->marioBodyState->torsoAngle[2], tilt.z * s->spd.x * 0x300, 0x800, 0x800);
		}
		else
		{
			//Reset tilt
			s->marioState->marioBodyState->torsoAngle[0] = approach_s32(s->marioState->marioBodyState->torsoAngle[0], 0, 0x800, 0x800);
			s->marioState->marioBodyState->torsoAngle[2] = approach_s32(s->marioState->marioBodyState->torsoAngle[2], 0, 0x800, 0x800);
		}
		
		//Turn Mario body
		s->marioState->marioBodyState->torsoAngle[1] = approach_s32(s->marioState->marioBodyState->torsoAngle[1], turn * 10, 0x800, 0x800);
		s->marioState->marioBodyState->headAngle[1] = approach_s32(s->marioState->marioBodyState->headAngle[1], turn * 10, 0x800, 0x800);
	}
	else if (s->anim == SA_IDLE)
	{
		//Tilt based off floor angle
		tilt.x = 1.0f;
		tilt.y = 0.0f;
		tilt.z = 0.0f;
		Sonic_ConvertVector_P2G(s, &tilt);
		s->marioState->marioBodyState->torsoAngle[0] = approach_s32(s->marioState->marioBodyState->torsoAngle[0], Sonic_Asin(tilt.y) / M_PI * 0x6000, 0x800, 0x800);
		
		//Reset tilt and turn
		s->marioState->marioBodyState->torsoAngle[2] = approach_s32(s->marioState->marioBodyState->torsoAngle[2], 0, 0x800, 0x800);
		s->marioState->marioBodyState->torsoAngle[1] = approach_s32(s->marioState->marioBodyState->torsoAngle[1], 0, 0x800, 0x800);
		s->marioState->marioBodyState->headAngle[1] = approach_s32(s->marioState->marioBodyState->headAngle[1], 0, 0x800, 0x800);
	}
	else
	{
		//Reset tilt and turn
		s->marioState->marioBodyState->torsoAngle[0] = approach_s32(s->marioState->marioBodyState->torsoAngle[0], 0, 0x800, 0x800);
		s->marioState->marioBodyState->torsoAngle[2] = approach_s32(s->marioState->marioBodyState->torsoAngle[2], 0, 0x800, 0x800);
		s->marioState->marioBodyState->torsoAngle[1] = approach_s32(s->marioState->marioBodyState->torsoAngle[1], 0, 0x800, 0x800);
		s->marioState->marioBodyState->headAngle[1] = approach_s32(s->marioState->marioBodyState->headAngle[1], 0, 0x800, 0x800);
	}
	return visRotate;
}

//Water function
void Sonic_Water(SonicState *s)
{
	//Check if we're underwater
	s32 terrainIsSnow = (s->marioState->area->terrainType & TERRAIN_MASK) == TERRAIN_SNOW;
	
	if (s->task.pos.y < (s->marioState->waterLevel - 50.0f))
		s->task.flag |= SF_UNDERWATER;
	else
		s->task.flag &= ~SF_UNDERWATER;
	
	if (s->task.pos.y < (s->marioState->waterLevel - 100.0f))
		s->task.flag |= SF_DROWNING;
	else
		s->task.flag &= ~SF_DROWNING;
	
	if (s->task.flag & SF_UNDERWATER)
	{
		//Drag from being in water
		if (s->task.mode == SM_ROLL)
			s->spd.x *= 0.94f;
		else
			s->spd.x *= 0.99f;
	}
	
	if (s->task.flag & SF_DROWNING)
	{
		//Lose air
		if ((s->task.mode == SM_HURT || (s->task.flag & SF_DEAD) == 0) && s->subAirTime-- == 0)
		{
			if (s->airTime == 0)
			{
				//Drown
				s->task.mode = SM_DROWNING;
				s->task.flag |= SF_DEAD;
			}
			else
			{
				//Lose a second of air
				s->airTime--;
				s->subAirTime = 60;
				
				//Spawn a number if low on air
				u8 number = (s->airTime + 1) / 2;
				if ((s->airTime & 0x1) && number <= 5)
					spawn_orange_number(number, 0, 80, 0);
			}
		}
	}
	else
	{
		//Reset air
		s->airTime = terrainIsSnow ? 10 : 30;
		s->subAirTime = 60;
	}
}

//Homing attack detection
u8 Sonic_HomingAttack(SonicState *s)
{
	static const enum ObjectList lists[] = {OBJ_LIST_DESTRUCTIVE, OBJ_LIST_GENACTOR, OBJ_LIST_PUSHABLE, OBJ_LIST_LEVEL, OBJ_LIST_DEFAULT, -1};
	enum ObjectList list;
	u8 i, v, result = 0;
	struct ObjectNode *listHead;
	struct Object *obj;
	const struct BehaviorScript *bhv;
	Vector forward, angleToHome;
	f32 distTmp, distInvSqrt, dist = sqr(100.0f);
	
	//Get our forward vector
	forward.x = 1.0f;
	forward.y = 0.0f;
	forward.z = 0.0f;
	Sonic_ConvertVector_P2G(s, &forward);
	
	//Iterate through each object list
	for (i = 0; (list = lists[i]) != -1; i++)
	{
		//Iterate through each object in list
		listHead = &gObjectLists[list];
		for (obj = (struct Object*)listHead->next; obj != (struct Object*)listHead; obj = obj->header.next)
		{
			//Ignore if object is deactivated
			if (obj->activeFlags == ACTIVE_FLAGS_DEACTIVATED)
				continue;
			
			//Check if this object is homable
			for (v = 0; (bhv = sHomableBhv[v]) != NULL; v++)
			{
				//If this behaviour isn't it, check the next one
				if (obj->behavior != segmented_to_virtual(bhv))
					continue;
				
				//Get the position difference between us and the object
				angleToHome.x = (obj->oPosX - s->task.pos.x) / s->unitScale;
				angleToHome.y = ((obj->oPosY + obj->hitboxHeight / 2.0f) - s->task.pos.y) / s->unitScale;
				angleToHome.z = (obj->oPosZ - s->task.pos.z) / s->unitScale;
				
				//Check if this object is in range
				if (angleToHome.y > 20.0f)
					break;
				distTmp = sqr(angleToHome.x) + sqr(angleToHome.y) + sqr(angleToHome.z);
				if (distTmp == 0.0f || distTmp > dist)
					break;
				
				//Normalize angle to home
				distInvSqrt = 1.0f / sqrtf(distTmp);
				angleToHome.x *= distInvSqrt;
				angleToHome.y *= distInvSqrt;
				angleToHome.z *= distInvSqrt;
				
				//Check if object isn't behind us
				if (abs(Sonic_VectorAngle(&forward, &angleToHome)) > 0x5000)
					break;
				
				//Home into this object
				s->task.mode = SM_HOMING;
				s->homeObj = obj;
				s->jdTimer = 0;
				dist = distTmp;
				result = 1;
				break;
			}
		}
	}
	
	return result;
}

//Main update function
void Sonic_Update()
{
	SonicState *s = &gSonicState;
	f32 stickMag;
	Vector forward, angleToHome;
	Quaternion turn;
	
	//Clear turn in case we don't
	s->lastTurn = 0;
	
	//Update control state
	s->held = s->marioState->controller->buttonDown;
	s->press = (s->held ^ s->last) & s->held;
	s->last = s->held;
	
	//A button actions
	if (s->press & A_BUTTON)
	{
		switch (s->task.mode)
		{
			case SM_WALK:
			case SM_SKID:
			case SM_ROLL:
				//Jump
				s->spd.y = s->p->jmp_y_spd;
				s->jumpTimer = s->p->jump2_timer;
				s->task.flag &= ~SF_GROUNDED;
				s->task.flag |= SF_BALL_AURA;
				s->task.mode = SM_AIRBORNE;
				s->anim = SA_BALL;
				s->moveAnimSpeed = abs(s->spd.x);
				play_sound(SOUND_MARIO_HOOHOO, s->marioState->marioObj->header.gfx.cameraToObject);
				break;
			case SM_AIRBORNE:
				if (s->task.flag & SF_BALL_AURA)
				{
					//Attempt to homing attack, otherwise jump dash
					if (!Sonic_HomingAttack(s))
					{
						//Jump dash
						s->spd.x = 5.0f;
						s->anim = SA_HOMING_END;
						s->task.flag &= ~SF_BALL_AURA;
						s->task.flag |= SF_JD_AURA;
						s->jdTimer = 15;
					}
					
					play_sound(SOUND_MARIO_HRMM, s->marioState->marioObj->header.gfx.cameraToObject);
				}
				break;
		}
	}
	
	//Jump dash timer
	if (s->task.mode != SM_HOMING && s->jdTimer > 0)
	{
		if (s->task.flag & SF_GROUNDED)
		{
			//Cancel if grounded
			s->jdTimer = 0;
		}
		else
		{
			//Decrement jump dash timer and reduce speed
			s->jdTimer--;
			s->spd.x *= 0.98f;
			s->spd.y += s->spd.y * s->p->air_resist_y;
			s->spd.z += s->spd.z * s->p->air_resist_z;
		}
		
		//Clear jump dash aura if timer is 0
		if (s->jdTimer <= 0)
			s->task.flag &= ~SF_JD_AURA;
	}
	
	//B button actions
	if (s->press & B_BUTTON)
	{
		switch (s->task.mode)
		{
			case SM_WALK:
			case SM_SKID:
				//Start spindashing
				s->task.mode = SM_SPINDASH;
				s->task.flag |= SF_BALL_AURA;
				s->spindashSpeed = s->spd.x;
				if (s->spindashSpeed < 2.0f)
					s->spindashSpeed = 2.0f;
				play_sound(SOUND_MARIO_YAHOO, s->marioState->marioObj->header.gfx.cameraToObject);
				break;
			case SM_ROLL:
				//Stop rolling
				s->task.mode = SM_WALK;
				s->task.flag &= ~SF_BALL_AURA;
				s->anim = SA_UNCURL;
				break;
			case SM_AIRBORNE:
				//If in ball form, stop and enter freefall
				if (s->task.flag & SF_BALL_AURA)
				{
					s->spd.x = 0.0f;
					s->spd.y = 0.0f;
					s->spd.z = 0.0f;
					s->task.flag &= ~SF_BALL_AURA;
					s->anim = SA_FALL;
				}
				break;
		}
	}
	
	//Main mode updates
	switch (s->task.mode)
	{
		case SM_WALK:
		case SM_SKID:
			//Grounded movement
			Sonic_GroundMovement(s);
			
			//Collision
			Sonic_PerformCollision(s);
			
			if (!(s->task.flag & SF_GROUNDED))
			{
				//Start fall animation and enter fall state
				s->anim = SA_FALL;
				s->task.mode = SM_AIRBORNE;
			}
			else
			{
				//Update animation
				if (s->task.mode == SM_SKID)
				{
					//Skid
					s->anim = SA_SKID;
				}
				else
				{
					Sonic_CheckPower(s, NULL, &stickMag);
					if (stickMag != 0.0f && (s->task.flag & SF_PUSHING))
					{
						//Do pushing animation
						s->anim = SA_PUSH;
					}
					else
					{
						if (s->anim != SA_TURN_AROUND && !(s->anim == SA_UNCURL && (s->animMode & 0x80) == 0))
						{
							if (abs(s->spd.x) > 0.01f)
							{
								//Walk animations
								if (s->spd.x > (s->p->run_speed + s->p->dash_speed) / 2.0f)
									s->anim = SA_RUN;
								else if (s->spd.x > (s->p->jog_speed + s->p->run_speed) / 2.0f)
									s->anim = SA_JOG;
								else
									s->anim = SA_WALK;
							}
							else if (s->anim != SA_STOP_SKID)
							{
								//Idle
								s->anim = SA_IDLE;
							}
						}
					}
				}
				
				//Remember our ground movement speed
				s->moveAnimSpeed = abs(s->spd.x);
			}
			break;
		case SM_SPINDASH:
			//Movement
			Sonic_SpindashMovement(s);
			
			//Collision
			Sonic_PerformCollision(s);
			
			if (!(s->task.flag & SF_GROUNDED))
			{
				//Unground if left ground
				s->task.mode = SM_AIRBORNE;
			}
			else
			{
				if (s->held & B_BUTTON)
				{
					//Increase spindash speed
					if (s->spindashSpeed < 10.0f)
						s->spindashSpeed += 0.4f;
				}
				else
				{
					//Release spindash
					s->task.mode = SM_ROLL;
					s->spd.x = s->spindashSpeed;
					play_sound(SOUND_MARIO_UH, s->marioState->marioObj->header.gfx.cameraToObject);
				}
			}
			
			if (s->task.mode != SM_ROLL)
			{
				//Use rolling animation
				s->anim = SA_BALL;
				s->moveAnimSpeed = abs(s->spindashSpeed);
				break;
			}
			//Fallthrough
		case SM_ROLL:
			//Movement
			Sonic_RollMovement(s);
			
			//Collision
			Sonic_PerformCollision(s);
			
			if (!(s->task.flag & SF_GROUNDED))
			{
				//Unground if left ground
				s->task.mode = SM_AIRBORNE;
			}
			else
			{
				if (s->spd.x < s->p->run_speed)
				{
					//Uncurl if moving slow enough
					s->task.mode = SM_WALK;
					s->task.flag &= ~SF_BALL_AURA;
					s->anim = SA_UNCURL;
					break;
				}
			}
			
			//Use rolling animation
			s->anim = SA_BALL;
			s->moveAnimSpeed = abs(s->spd.x);
			break;
		case SM_AIRBORNE:
			//Airborne movement
			Sonic_AirMovement(s);
			
			//Collision
			Sonic_PerformCollision(s);
			if (s->task.flag & SF_GROUNDED)
			{
				//Enter walking state if landed
				s->task.mode = SM_WALK;
				s->task.flag &= ~SF_BALL_AURA;
			}
			break;
		case SM_HOMING:
			//Get our position difference from target
			angleToHome.x = s->homeObj->oPosX - s->task.pos.x;
			angleToHome.y = (s->homeObj->oPosY + s->homeObj->hitboxHeight / 2.0f) - s->task.pos.y;
			angleToHome.z = s->homeObj->oPosZ - s->task.pos.z;
			
			if (s->jdTimer == 0 && (sqr(angleToHome.x) + sqr(angleToHome.y) + sqr(angleToHome.z)) >= sqr(5.0f * s->unitScale))
			{
				//Get our forward vector in world space
				forward.x = 1.0f;
				forward.y = 0.0f;
				forward.z = 0.0f;
				Sonic_ConvertVector_P2G(s, &forward);
				
				//Rotate towards target
				Sonic_NormalizeVectorTo(&angleToHome, &angleToHome);
				Sonic_QuaternionFromToRotation(&turn, &forward, &angleToHome);
				Sonic_QuaternionProduct(&s->task.ang, &turn, &s->task.ang);
				
				//Thrust forward towards target
				s->spd.x = 5.0f;
				s->spd.y = 0.0f;
				s->spd.z = 0.0f;
			}
			else if (s->jdTimer == 0)
			{
				//Begin countdown to stop homing
				s->jdTimer++;
			}
			else
			{
				//Stop homing after 2 frames
				s->jdTimer++;
				if (s->jdTimer > 2)
				{
					s->jdTimer = 0;
					s->task.mode = SM_AIRBORNE;
				}
			}
			
			//Collision
			if (s->jdTimer != 0 || s->task.mode == SM_AIRBORNE)
			{
				s->tspd.x = s->spd.x;
				s->tspd.y = s->spd.y;
				s->tspd.z = s->spd.z;
				Sonic_PerformCollision(s);
				s->spd.x = s->tspd.x;
				s->spd.y = s->tspd.y;
				s->spd.z = s->tspd.z;
				s->task.flag |= SF_BALL_AURA;
				s->task.flag &= ~SF_GROUNDED;
			}
			else
			{
				Sonic_PerformCollision(s);
			}
			
			if (s->task.flag & SF_GROUNDED && (s->jdTimer == 0 || s->task.mode == SM_AIRBORNE))
			{
				if (s->floorNormal.y > 0.2f)
				{
					//Land on ground without rolling
					s->task.mode = SM_WALK;
					s->task.flag &= ~SF_BALL_AURA;
					break;
				}
				else
				{
					//Stop homing since we hit a wall, but keep rolling
					s->task.mode = SM_AIRBORNE;
					s->spd.x = 0.0f;
				}
			}
			else if (s->task.flag & SF_PUSHING)
			{
				//Stop homing since we hit a wall, but keep rolling
				s->task.mode = SM_AIRBORNE;
			}
			
			//Use rolling animation
			s->anim = SA_BALL;
			s->moveAnimSpeed = abs(s->spd.x);
			break;
		case SM_HURT:
			//Use hurt animation
			s->anim = SA_HURT;
			
			//Apply gravity onto our speed
			Sonic_ConvertVector_P2G(s, &s->spd);
			s->spd.x += s->p->weight * s->gravity.x;
			s->spd.y += s->p->weight * s->gravity.y;
			s->spd.z += s->p->weight * s->gravity.z;
			Sonic_ConvertVector_G2P(s, &s->spd);
			
			//Air drag
			s->spd.x += s->spd.x * s->p->air_resist * 2.7f;
			s->spd.y += s->spd.y * s->p->air_resist_y;
			s->spd.z += s->spd.z * s->p->air_resist_z * 2.7f;
			
			//Perform collision
			Sonic_PerformCollision(s);
			
			if (s->task.flag & SF_GROUNDED)
			{
				if (s->task.flag & SF_DEAD)
				{
					//If dead, enter death state
					s->task.mode = SM_DEAD;
				}
				else
				{
					//Land and kill all speed
					s->task.mode = SM_WALK;
					s->spd.x = 0.0f;
					s->spd.y = 0.0f;
					s->spd.z = 0.0f;
				}
			}
			break;
		case SM_DEAD:
			//Apply gravity onto our speed
			Sonic_ConvertVector_P2G(s, &s->spd);
			s->spd.x += s->p->weight * s->gravity.x;
			s->spd.y += s->p->weight * s->gravity.y;
			s->spd.z += s->p->weight * s->gravity.z;
			Sonic_ConvertVector_G2P(s, &s->spd);
			
			//Air drag
			s->spd.x += s->spd.x * s->p->air_resist * 0.5f;
			s->spd.y += s->spd.y * s->p->air_resist_y;
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
			
			//Perform collision
			Sonic_PerformCollision(s);
			
			//Use death animation
			s->anim = SA_DEAD;
			break;
		case SM_DROWNING:
			//Just do basic collision
			s->spd.x = 0.0f;
			s->spd.y += s->spd.y * s->p->air_resist_y * 2.7f;
			s->spd.z = 0.0f;
			
			s->task.flag &= ~SF_GROUNDED;
			Sonic_PerformCollision(s);
			
			if (s->marioState->waterLevel <= -11000.0f)
			{
				//Stop NOW
				s->spd.x = 0.0f;
				s->spd.y = 0.0f;
				s->spd.z = 0.0f;
			}
			else
			{
				if (s->task.pos.y < s->marioState->waterLevel - 120.0f)
				{
					//Float to surface
					s->spd.y += 0.02f;
				}
				else
				{
					//Stop but move towards surface if way above somehow
					s->spd.y *= 0.9f;
					s->spd.y -= (s->task.pos.y - (s->marioState->waterLevel - 120.0f)) / s->unitScale * 0.05f;
				}
			}
			
			//Use drowning animation
			s->anim = SA_DROWNING;
			break;
	}
	
	//Handle water stuff
	Sonic_Water(s);
}

void Sonic_Init(struct MarioState *marioState)
{
	SonicState *s;
	u64 i;
	
	//Clear Sonic state memory (fuck you not IEEE-754)
	s = &gSonicState;
	bzero(s, sizeof(SonicState));
	
	//Attach given Mario state to Sonic state
	s->marioState = marioState;
	
	//Use Sonic's physics params
	s->p = &sSonicParam;
	
	//Initialize state
	s->task.ang.x = 0.0f;
	s->task.ang.y = 0.0f;
	s->task.ang.z = 0.0f;
	s->task.ang.w = 1.0f;
	
	s->task.mode = SM_AIRBORNE;
	
	s->floorNormal.x = 0.0f;
	s->floorNormal.y = 1.0f;
	s->floorNormal.z = 0.0f;
	
	s->gravity.x = 0.0f;
	s->gravity.y = -1.0f;
	s->gravity.z = 0.0f;
	//Sonic_NormalizeVectorTo(&s->gravity, &s->gravity);
	
	s->lastUp.x = 0.0f;
	s->lastUp.y = 1.0f;
	s->lastUp.z = 0.0f;
	
	s->airTime = 30;
	s->subAirTime = 60;
	
	s->frict = 1.0f;
	s->unitScale = 10.0f;
}
