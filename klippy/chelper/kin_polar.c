// Polar kinematics stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

#define PI 3.14159265358979323846F
#define ARM_LENGHT 126.0f


static double
polar_stepper_radius_calc_position(struct stepper_kinematics *sk, struct move *m
                                   , double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    return 2 * asinf(sqrtf(c.x*c.x+c.y*c.y) / (2 * ARM_LENGHT) );
}

static double
polar_stepper_angle_calc_position(struct stepper_kinematics *sk, struct move *m
                                  , double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    float theta1 = polar_stepper_radius_calc_position(sk, m, move_time);
    // XXX - handle x==y==0
    double angle = (PI-theta1)/2 - atan2f(c.y,c.x);
    if (angle - sk->commanded_pos > PI)
        angle -= 2. * PI;
    else if (angle - sk->commanded_pos < -PI)
        angle += 2. * PI;
    return angle;
}

static void
polar_stepper_angle_post_fixup(struct stepper_kinematics *sk)
{
    // Normalize the stepper_bed angle
    if (sk->commanded_pos < -PI)
        sk->commanded_pos += 2 * PI;
    else if (sk->commanded_pos > PI)
        sk->commanded_pos -= 2 * PI;
}

struct stepper_kinematics * __visible
polar_stepper_alloc(char type)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));
    if (type == 'r') {
        sk->calc_position_cb = polar_stepper_radius_calc_position;
    } else if (type == 'a') {
        sk->calc_position_cb = polar_stepper_angle_calc_position;
        sk->post_cb = polar_stepper_angle_post_fixup;
    }
    sk->active_flags = AF_X | AF_Y;
    return sk;
}
