#include "sonic_trig.h"
#include "../engine/math_util.h"

#ifdef TARGET_N64

static const f64
pio2_hi = 1.57079632679489655800e+00, /* 0x3FF921FB, 0x54442D18 */
pio2_lo = 6.12323399573676603587e-17, /* 0x3C91A626, 0x33145C07 */
/* coefficients for R(x^2) */
pS0 =  1.66666666666666657415e-01, /* 0x3FC55555, 0x55555555 */
pS1 = -3.25565818622400915405e-01, /* 0xBFD4D612, 0x03EB6F7D */
pS2 =  2.01212532134862925881e-01, /* 0x3FC9C155, 0x0E884455 */
pS3 = -4.00555345006794114027e-02, /* 0xBFA48228, 0xB5688F3B */
pS4 =  7.91534994289814532176e-04, /* 0x3F49EFE0, 0x7501B288 */
pS5 =  3.47933107596021167570e-05, /* 0x3F023DE1, 0x0DFDF709 */
qS1 = -2.40339491173441421878e+00, /* 0xC0033A27, 0x1C8A2D4B */
qS2 =  2.02094576023350569471e+00, /* 0x40002AE5, 0x9C598AC8 */
qS3 = -6.88283971605453293030e-01, /* 0xBFE6066C, 0x1B8D0159 */
qS4 =  7.70381505559019352791e-02; /* 0x3FB3B8C5, 0xB12E9282 */

typedef union
{
  f64 value;
  struct
  {
    u32 msw;
    u32 lsw;
  } parts;
  u64 word;
} ieee_f64_shape_type;


# define GET_HIGH_WORD(i,d)                                        \
do {                                                                \
  ieee_f64_shape_type gh_u;                                        \
  gh_u.value = (d);                                                \
  (i) = gh_u.parts.msw;                                                \
} while (0)

# define GET_LOW_WORD(i,d)                                        \
do {                                                                \
  ieee_f64_shape_type gl_u;                                        \
  gl_u.value = (d);                                                \
  (i) = gl_u.parts.lsw;                                                \
} while (0)

#define SET_HIGH_WORD(d,v)                                        \
do {                                                                \
  ieee_f64_shape_type sh_u;                                        \
  sh_u.value = (d);                                                \
  sh_u.parts.msw = (v);                                                \
  (d) = sh_u.value;                                                \
} while (0)

# define SET_LOW_WORD(d,v)                                        \
do {                                                                \
  ieee_f64_shape_type sl_u;                                        \
  sl_u.value = (d);                                                \
  sl_u.parts.lsw = (v);                                                \
  (d) = sl_u.value;                                                \
} while (0)

static f64 R(f64 z)
{
	f64 p, q;
	p = z*(pS0+z*(pS1+z*(pS2+z*(pS3+z*(pS4+z*pS5)))));
	q = 1.0+z*(qS1+z*(qS2+z*(qS3+z*qS4)));
	return p/q;
}

f64 Sonic_Asin(f64 x)
{
	f64 z,r,s;
	u32 hx,ix;

	GET_HIGH_WORD(hx, x);
	ix = hx & 0x7fffffff;
	/* |x| >= 1 or nan */
	if (ix >= 0x3ff00000) {
		u32 lx;
		GET_LOW_WORD(lx, x);
		if (((ix-0x3ff00000) | lx) == 0)
			/* asin(1) = +-pi/2 with inexact */
			return x*pio2_hi + 7.52316e-37;
		return 0/(x-x);
	}
	/* |x| < 0.5 */
	if (ix < 0x3fe00000) {
		/* if 0x1p-1022 <= |x| < 0x1p-26, avoid raising underflow */
		if (ix < 0x3e500000 && ix >= 0x00100000)
			return x;
		return x + x*R(x*x);
	}
	/* 1 > |x| >= 0.5 */
	z = (1 - abs(x))*0.5;
	s = sqrtf(z);
	r = R(z);
	if (ix >= 0x3fef3333) {  /* if |x| > 0.975 */
		x = pio2_hi-(2*(s+s*r)-pio2_lo);
	} else {
		f64 f,c;
		/* f+c = sqrtf(z) */
		f = s;
		SET_LOW_WORD(f,0);
		c = (z-f*f)/(s+f);
		x = 0.5*pio2_hi - (2*s*r - (pio2_lo-2*c) - (0.5*pio2_hi-2*f));
	}
	if (hx >> 31)
		return -x;
	return x;
}

f64 Sonic_Acos(f64 x)
{
	f64 z,w,s,c,df;
	u32 hx,ix;

	GET_HIGH_WORD(hx, x);
	ix = hx & 0x7fffffff;
	/* |x| >= 1 or nan */
	if (ix >= 0x3ff00000) {
		u32 lx;

		GET_LOW_WORD(lx,x);
		if (((ix-0x3ff00000) | lx) == 0) {
			/* acos(1)=0, acos(-1)=pi */
			if (hx >> 31)
				return 2*pio2_hi + 7.52316e-37;
			return 0;
		}
		return 0/(x-x);
	}
	/* |x| < 0.5 */
	if (ix < 0x3fe00000) {
		if (ix <= 0x3c600000)  /* |x| < 2**-57 */
			return pio2_hi + 7.52316e-37;
		return pio2_hi - (x - (pio2_lo-x*R(x*x)));
	}
	/* x < -0.5 */
	if (hx >> 31) {
		z = (1.0+x)*0.5;
		s = sqrtf(z);
		w = R(z)*s-pio2_lo;
		return 2*(pio2_hi - (s+w));
	}
	/* x > 0.5 */
	z = (1.0-x)*0.5;
	s = sqrtf(z);
	df = s;
	SET_LOW_WORD(df,0);
	c = (z-df*df)/(s+df);
	w = R(z)*s+c;
	return 2*(df+w);
}

#else

#include <math.h>

f64 Sonic_Asin(f64 x)
{
	return asin(x);
}

f64 Sonic_Acos(f64 x)
{
	return acos(x);
}

#endif
