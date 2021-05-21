#include "sonic_collision.h"

#include "../engine/math_util.h"
#include "../engine/surface_collision.h"
#include "sonic_math.h"
#include "sonic_trig.h"

//Vector operations
void Sonic_NormalizeVectorTo(Vector *destination, const Vector *source)
{
	f32 invsqrt = 1.0f / sqrtf(sqr(source->x) + sqr(source->y) + sqr(source->z));
	destination->x = source->x * invsqrt;
	destination->y = source->y * invsqrt;
	destination->z = source->z * invsqrt;
}

s16 Sonic_VectorAngle(const Vector *from, const Vector *to)
{
	Vector nFrom, nTo;
	f32 dot;
	
	//Angle difference is acos of the dot product between the two vectors
	Sonic_NormalizeVectorTo(&nFrom, from);
	Sonic_NormalizeVectorTo(&nTo, to);
	dot = njInnerProduct(&nFrom, &nTo);
	if (dot > 1.0f)
		return 0x0000;
	else if (dot < -1.0f)
		return -0x8000;
	else
		return Sonic_Acos(dot) * 0x8000 / M_PI;
}

//Quaternion operations
void Sonic_NormalizeQuaternion(Quaternion *q)
{
	f32 invsqrt = 1.0f / sqrtf(sqr(q->x) + sqr(q->y) + sqr(q->z) + sqr(q->w));
	q->x *= invsqrt;
	q->y *= invsqrt;
	q->z *= invsqrt;
	q->w *= invsqrt;
}

void Sonic_QuaternionAngleAxis(Quaternion *q, s16 angle, const Vector *axis)
{
	Vector normalizedAxis;
	f32 sin, cos;
	
	//Normalize axis vector
	Sonic_NormalizeVectorTo(&normalizedAxis, axis);
	
	//Get sine and cosine of angle
	angle /= 2;
	sin = sins(angle);
	cos = coss(angle);
	
	//Create quaternion from axis and angle
	q->x = axis->x * sin;
	q->y = axis->y * sin;
	q->z = axis->z * sin;
	q->w = cos;
}

void Sonic_QuaternionFromToRotation(Quaternion *quaternion, const Vector *from, const Vector *to)
{
	Vector axis;
	s16 angle;
	f32 axisMag;
	
	//Get our axis and angle
	axisMag = njOuterProduct(from, to, &axis);
	angle = Sonic_VectorAngle(from, to);
	
	//Create quaternion from axis and angle
	if (axisMag != 0.0f)
	{
		Sonic_QuaternionAngleAxis(quaternion, angle, &axis);
		return;
	}
	
	//Return identity matrix (axis magnitude was 0)
	quaternion->x = 0.0f;
	quaternion->y = 0.0f;
	quaternion->z = 0.0f;
	quaternion->w = 1.0f;
}

//Normal alignment
void Sonic_AlignNormal(SonicState *s, const Vector *up, const Vector *normal)
{
	Vector upl;
	Quaternion rot;
	
	if (up == NULL)
	{
		//Get our up vector
		upl.x = 0.0f;
		upl.y = 1.0f;
		upl.z = 0.0f;
		Sonic_ConvertVector_P2G(s, &upl);
	}
	
	//Get the quaternion rotation from our up vector to the normal
	Sonic_QuaternionFromToRotation(&rot, (up != NULL) ? up : &upl, normal);
	
	//Get the product of our angle onto the rotate quaternion
	Sonic_QuaternionProduct(&s->task.ang, &rot, &s->task.ang);
	
	//Set other collision properties
	s->floorNormal.x = normal->x;
	s->floorNormal.y = normal->y;
	s->floorNormal.z = normal->z;
	s->dotp = s->floorNormal.y;
}

void Sonic_AlignNormalInv(SonicState *s, const Vector *up, const Vector *normal)
{
	Vector inv;
	inv.x = normal->x * -1.0f;
	inv.y = normal->y * -1.0f;
	inv.z = normal->z * -1.0f;
	Sonic_AlignNormal(s, up, &inv);
}

//Wall collision function
u8 Sonic_DoWallCollision(SonicState *s, const Vector *up, const Vector *direction, f32 *vel, f32 dir, u8 velStop)
{
	struct Surface *hit;
	Vec3f ro, rd, hp;
	f32 ups, rads, back;
	Vector localNor;
	f32 dot, maxVel;
	
	//Raycast in the direction given with Sonic's radius and offset by his height
	ups = s->p->height * s->unitScale;
	rads = (s->p->rad + (((*vel * dir) < 0.0f) ? 0.0f : (*vel * dir))) * s->unitScale;
	back = s->p->rad * -0.5f * s->unitScale;
	
	vec3f_set(ro, s->task.pos.x + (up->x * ups) + (back * direction->x * dir), s->task.pos.y + (up->y * ups) + (back * direction->y * dir), s->task.pos.z + (up->z * ups) + (back * direction->z * dir));
	vec3f_set(rd, (rads - back) * direction->x * dir, (rads - back) * direction->y * dir, (rads - back) * direction->z * dir);
	find_surface_on_ray(ro, rd, &hit, hp);
	
	if (hit != NULL)
	{
		//Get the normal local to us
		localNor.x = hit->normal.x;
		localNor.y = hit->normal.y;
		localNor.z = hit->normal.z;
		Sonic_ConvertVector_G2P(s, &localNor);
		
		if (!(s->task.flag & SF_GROUNDED) || localNor.y < 0.4f)
		{
			s->task.pos.x = hp[0] - (direction->x * dir * (s->p->rad * s->unitScale)) - (ups * up->x);
			s->task.pos.y = hp[1] - (direction->y * dir * (s->p->rad * s->unitScale)) - (ups * up->y);
			s->task.pos.z = hp[2] - (direction->z * dir * (s->p->rad * s->unitScale)) - (ups * up->z);
			dot = njInnerProduct(direction, (Vector*)&hit->normal.x);
			if (velStop && dot < 0.0f)
			{
				maxVel = s->spd.y * (sqrtf(abs(localNor.y)) * ((localNor.y >= 0.0f) ? 1.0f : -1.0f));
				if (maxVel < 0.0f)
					maxVel = 0.0f;
				if (*vel > maxVel)
					*vel = maxVel;
			}
		}
		else
		{
			s->task.pos.x = hp[0];
			s->task.pos.y = hp[1];
			s->task.pos.z = hp[2];
		}
		return 1;
	}
	return 0;
}

//Collision function
void Sonic_PerformCollision(SonicState *s)
{
	struct Surface *hit;
	Vector up, forward, right, cornerRem;
	Vec3f ro, rd, hp;
	f32 fup, fdown;
	u8 wallContact = 0, didFall = FALSE;
	
	//Convert local speed to global speed
	s->gspd.x = s->spd.x;
	s->gspd.y = s->spd.y;
	s->gspd.z = s->spd.z;
	Sonic_ConvertVector_P2G(s, &s->gspd);
	
	//Get some orientation vectors
	up.x = 0.0f;
	up.y = 1.0f;
	up.z = 0.0f;
	Sonic_ConvertVector_P2G(s, &up);
	
	forward.x = 1.0f;
	forward.y = 0.0f;
	forward.z = 0.0f;
	Sonic_ConvertVector_P2G(s, &forward);
	
	right.x = 0.0f;
	right.y = 0.0f;
	right.z = 1.0f;
	Sonic_ConvertVector_P2G(s, &right);
	
	//Check collision in front of us
	if (Sonic_DoWallCollision(s, &up, &forward, &s->spd.x, 1.0f, TRUE))
	{
		//Begin pushing and set wall contact
		s->task.flag |= SF_PUSHING;
		wallContact = 1;
		cornerRem.x = s->task.pos.x;
		cornerRem.y = s->task.pos.y;
		cornerRem.z = s->task.pos.z;
	}
	else
	{
		//Stop pushing
		s->task.flag &= ~SF_PUSHING;
	}
	
	//Check collision behind us
	if (Sonic_DoWallCollision(s, &up, &forward, &s->spd.x, -1.0f, FALSE) && wallContact)
	{
		s->task.pos.x = (s->task.pos.x + cornerRem.x) / 2.0f;
		s->task.pos.y = (s->task.pos.y + cornerRem.y) / 2.0f;
		s->task.pos.z = (s->task.pos.z + cornerRem.z) / 2.0f;
	}
	else if (!wallContact && s->spd.x != 0.0f)
	{
		//Move forwards
		s->task.pos.x += forward.x * (s->spd.x * s->unitScale);
		s->task.pos.y += forward.y * (s->spd.x * s->unitScale);
		s->task.pos.z += forward.z * (s->spd.x * s->unitScale);
	}
	
	//Collide and move left and right
	if (Sonic_DoWallCollision(s, &up, &right, &s->spd.z, 1.0f, TRUE))
	{
		//Set wall contact
		wallContact = 1;
		cornerRem.x = s->task.pos.x;
		cornerRem.y = s->task.pos.y;
		cornerRem.z = s->task.pos.z;
	}
	else
	{
		//Clear wall contact
		wallContact = 0;
	}
	
	if (Sonic_DoWallCollision(s, &up, &right, &s->spd.z, -1.0f, TRUE) && wallContact)
	{
		s->task.pos.x = (s->task.pos.x + cornerRem.x) / 2.0f;
		s->task.pos.y = (s->task.pos.y + cornerRem.y) / 2.0f;
		s->task.pos.z = (s->task.pos.z + cornerRem.z) / 2.0f;
		s->task.pos.x -= forward.x * s->spd.x * s->unitScale;
		s->task.pos.y -= forward.y * s->spd.x * s->unitScale;
		s->task.pos.z -= forward.z * s->spd.x * s->unitScale;
		s->spd.x = 0.0f;
	}
	else if (!wallContact && s->spd.z != 0.0f)
	{
		s->task.pos.x += right.x * (s->spd.z * s->unitScale);
		s->task.pos.y += right.y * (s->spd.z * s->unitScale);
		s->task.pos.z += right.z * (s->spd.z * s->unitScale);
	}
	
	//Ceiling collision
	if (!(s->task.flag & SF_GROUNDED))
	{
		fdown = s->spd.y;
		fup = (s->p->height + (fdown >= 0.0f ? fdown : 0.0f)) * s->unitScale;
		if (fdown < 0.0f)
			fdown = 0.0f;
		fdown += s->p->height * s->unitScale;
		
		vec3f_set(ro, s->task.pos.x + (up.x * fdown), s->task.pos.y + (up.y * fdown), s->task.pos.z + (up.z * fdown));
		vec3f_set(rd, up.x * fup, up.y * fup, up.z * fup);
		find_surface_on_ray(ro, rd, &hit, hp);
		
		if (hit != NULL)
		{
			//Clip out of ceiling
			if (s->spd.y > 0.0f)
				s->spd.y = 0.0f;
			fdown = s->p->height * 2.0f * s->unitScale;
			s->task.pos.x = hp[0] - (up.x * fdown);
			s->task.pos.y = hp[1] - (up.y * fdown);
			s->task.pos.z = hp[2] - (up.z * fdown);
		}
	}
	
	//Floor collision
	fup = s->p->height * s->unitScale;
	fdown = -(fup + ((s->task.flag & SF_GROUNDED) ? (s->p->pos_error * s->unitScale) : 0.0f));
	
	if (s->spd.y <= 0.0f)
		fdown += s->spd.y * s->unitScale; //Moving downwards, extend raycast downwards
	else
		fup += s->spd.y * s->unitScale; //Moving upwards, move raycast upwards
	
	vec3f_set(ro, s->task.pos.x + (up.x * fup), s->task.pos.y + (up.y * fup), s->task.pos.z + (up.z * fup));
	vec3f_set(rd, up.x * fdown, up.y * fdown, up.z * fdown);
	find_surface_on_ray(ro, rd, &s->floor, hp);
	
	if (s->floor != NULL && s->floor->normal.y < 0.3f && (sqr(s->spd.x) + sqr(s->spd.z)) < sqr(1.16f))
		didFall = TRUE;
	
	if (s->floor != NULL && !didFall)
	{
		//Clip out of ground
		s->task.pos.x = hp[0];
		s->task.pos.y = hp[1];
		s->task.pos.z = hp[2];
		
		if (!(s->task.flag & SF_GROUNDED))
		{
			//Become grounded (previous velocity is converted to newly oriented global velocity)
			s->task.flag |= SF_GROUNDED;
			Sonic_ConvertVector_P2G(s, &s->spd);
			Sonic_AlignNormal(s, &up, (Vector*)&s->floor->normal.x);
			Sonic_ConvertVector_G2P(s, &s->spd);
		}
		else
		{
			//Align with floor
			Sonic_AlignNormal(s, &up, (Vector*)&s->floor->normal.x);
		}
		
		//Stop vertical speed
		s->spd.y = 0.0f;
	}
	else
	{
		//Move vertically
		if (s->spd.y != 0.0f)
		{
			s->task.pos.x += up.x * (s->spd.y * s->unitScale);
			s->task.pos.y += up.y * (s->spd.y * s->unitScale);
			s->task.pos.z += up.z * (s->spd.y * s->unitScale);
		}
		
		//Reset orientation if moving slow
		if ((sqr(s->spd.x) + sqr(s->spd.y) + sqr(s->spd.z)) < sqr(s->p->dash_speed))
		{
			if ((s->task.flag & SF_GROUNDED) && didFall)
			{
				//Move upwards
				s->task.pos.x += up.x * (s->p->height * s->unitScale);
				s->task.pos.y += up.y * (s->p->height * s->unitScale);
				s->task.pos.z += up.z * (s->p->height * s->unitScale);
			}
			
			//Reset falling direction
			Sonic_ConvertVector_P2G(s, &s->spd);
			Sonic_AlignNormalInv(s, &up, &s->gravity);
			Sonic_ConvertVector_G2P(s, &s->spd);
			
			if ((s->task.flag & SF_GROUNDED) && didFall)
			{
				//Move back downwards
				s->task.pos.x += s->gravity.x * (s->p->height * s->unitScale);
				s->task.pos.y += s->gravity.y * (s->p->height * s->unitScale);
				s->task.pos.z += s->gravity.z * (s->p->height * s->unitScale);
			}
		}
		
		//Unground
		s->task.flag &= ~SF_GROUNDED;
	}
	
	//Normalize quaternion for next frame
	Sonic_NormalizeQuaternion(&s->task.ang);
}
