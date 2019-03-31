#ifndef Utils_h
#define Utils_h

#ifdef __cplusplus
extern "C" {
#endif

float rad2deg(float rad);

float deg2rad(float deg);


/*
 *  This is quite a computationally expensive routine,
 *  so you might want to consider not using it.  But
 *  gaussian random numbers are really nice for a random
 *  walk behaviour :)
 *  From: http://www.taygeta.com/random/gaussian.html
 */
float randGaussian( float mean, float sd );

#ifdef __cplusplus
}
#endif

#endif /* Utils_h */
