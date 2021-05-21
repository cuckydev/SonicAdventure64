#ifndef _SONIC_H
#define _SONIC_H

#include "sonic_math.h"
#include "types.h"

//Sonic enumeration and defines
typedef enum
{
	SM_WALK,
	SM_SKID,
	SM_SPINDASH,
	SM_ROLL,
	SM_AIRBORNE,
	SM_HOMING,
	SM_HURT,
	SM_DEAD,
	SM_DROWNING,
} SonicMode;

typedef enum
{
	SA_IDLE,
	SA_PUSH,
	SA_WALK,
	SA_JOG,
	SA_RUN,
	SA_SKID,
	SA_TURN_AROUND,
	SA_STOP_SKID,
	SA_BALL,
	SA_UNCURL,
	SA_HOMING_END,
	SA_FALL,
	SA_HURT,
	SA_DEAD,
	SA_DROWNING,
} SonicAnim;

#define SF_GROUNDED		(1 << 0)
#define SF_JD_AURA		(1 << 1)
#define SF_BALL_AURA	(1 << 2)
#define SF_PUSHING		(1 << 3)
#define SF_UNDERWATER	(1 << 4)
#define SF_DROWNING		(1 << 5)
#define SF_DEAD			(1 << 6)

//Physics struct
typedef struct
{
	s32 jump2_timer;
	f32 pos_error;
	f32 lim_h_spd;
	f32 lim_v_spd;
	f32 max_x_spd;
	f32 max_psh_spd;
	f32 jmp_y_spd;
	f32 nocon_speed;
	f32 slide_speed;
	f32 jog_speed;
	f32 run_speed;
	f32 rush_speed;
	f32 crash_speed;
	f32 dash_speed;
	f32 jmp_addit;
	f32 run_accel;
	f32 air_accel;
	f32 slow_down;
	f32 run_break;
	f32 air_break;
	f32 air_resist_air;
	f32 air_resist;
	f32 air_resist_y;
	f32 air_resist_z;
	f32 grd_frict;
	f32 grd_frict_z;
	f32 lim_frict;
	f32 rat_bound;
	f32 rad;
	f32 height;
	f32 weight;
	f32 eyes_height;
	f32 center_height;
} PhysicsParam;

//Sonic state structures
typedef struct
{
	//Current mode and flags
	SonicMode mode : 8;
	u8 flag;
	
	//Orientation, position, etc
	Quaternion ang;
	Vector pos;
} SonicTask;

typedef struct
{
	//Mario state we belong to
	struct MarioState *marioState;
	
	//Task state
	SonicTask task;
	
	//Animation state
	SonicAnim anim : 8;
	SonicAnim prevAnim : 8;
	u8 animMode;
	
	//Velocity and orientation, that kind of stuff
	Vector spd, gspd, tspd;
	f32 moveAnimSpeed;
	
	//Collision
	f32 dotp;
	Vector floorNormal;
	struct Surface *floor;
	
	//Movement state
	u8 jumpTimer, jdTimer;
	s16 lastTurn;
	f32 spindashSpeed;
	u8 airTime, subAirTime;
	
	struct Object *homeObj;
	
	//Control state
	u16 held, last, press;
	Vector lastUp;
	
	//Physics state
	const PhysicsParam *p;
	Vector gravity;
	f32 frict;
	f32 unitScale;
} SonicState;

//Sonic state
extern SonicState gSonicState;

//Coordinate system conversion
void Sonic_AngleToMatrix(Matrix_ptr m, const Quaternion *q);
void Sonic_QuaternionProduct(Quaternion *res, const Quaternion *lhs, const Quaternion *rhs);
void Sonic_QuaternionVectorProduct(Vector *res, const Quaternion *q);
void Sonic_ConvertVector_G2P(SonicState *s, Vector *destination);
void Sonic_ConvertVector_P2G(SonicState *s, Vector *destination);

//Input functions
u8 Sonic_CheckPower(SonicState *s, s16 *angle, f32 *magnitude);

//Rotation
void Sonic_Turn(SonicState *s, s16 turn);
void Sonic_AdjustAngleY(SonicState *s, s16 turn);
void Sonic_AdjustAngleYQ(SonicState *s, s16 turn);
void Sonic_AdjustAngleYS(SonicState *s, s16 turn);

//Sonic state functions
s16 Sonic_Animate();
void Sonic_Update();
void Sonic_Init(struct MarioState *marioState);

#endif
