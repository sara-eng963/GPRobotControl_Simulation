#include "../admittance_control.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static AdmittanceParams fixture(void)
{
    AdmittanceParams p = {0};
    for (size_t i = 0U; i < ADMITTANCE_AXES; ++i) {
        p.virtual_mass[i] = (i < 3U) ? 2.0 : 0.5;
        p.damping[i] = (i < 3U) ? 4.0 : 1.0;
        p.speed_limit[i] = 2.0;
        p.acceleration_limit[i] = 100.0;
    }
    p.maximum_step_s = 0.004; /* synthetic test fault threshold, not approved timing */
    return p;
}

static void close_to(double actual, double expected)
{
    assert(isfinite(actual));
    assert(fabs(actual - expected) < 1e-12);
}

static void all_zero(const double x[ADMITTANCE_AXES])
{
    for (size_t i = 0U; i < ADMITTANCE_AXES; ++i) close_to(x[i], 0.0);
}

static void is_cleared(const AdmittanceState *s)
{
    assert(!s->enabled);
    all_zero(s->velocity);
    all_zero(s->previous_wrench);
}


/* Fixed reference points from the MATLAB 500 Hz Tustin push/reverse/release
 * experiment with full-precision optimizer output, no active saturation.
 * Sample index k maps to t=k*0.002 seconds. */
static void verify_matlab_optimized_reference(void)
{
    static const double mass[ADMITTANCE_AXES] = {
        29.22801456612752, 26.361336639519262, 23.74469161916069,
        1.6319684618011627, 1.4732529319920249, 1.1148713535550765
    };
    static const double damping[ADMITTANCE_AXES] = {
        143.9195532171934, 132.5682958794217, 121.32087961505862,
        8.1027427354888548, 7.2923649840052081, 6.0692154632587449
    };
    static const double design_wrench[ADMITTANCE_AXES] = {
        10.0, 8.0, 6.0, 0.9, 0.7, 0.5
    };
    static const struct {
        unsigned sample;
        double velocity[ADMITTANCE_AXES];
    } reference[] = {
        {50U, {0.0003404610659743335, 0.0003019562431248535,
               0.0002514035388319595, 0.0005487566622171539,
               0.0004727987731477917, 0.0004460539780783164}},
        {250U, {0.05915630475251431, 0.05170973427483317,
                0.04257895488461211, 0.09480752581918135,
                0.08185647545287762, 0.07220605156762863}},
        {375U, {-0.03158062565537759, -0.02817110725253352,
                -0.02354749312227944, -0.05102123403706519,
                -0.04392143120605297, -0.04229888664155659}},
        {600U, {-0.003444275056460595, -0.0029308038452241,
                -0.002362626804278767, -0.005462836783766367,
                -0.004734880986731036, -0.003650976853760065}}
    };
    AdmittanceParams p = fixture();
    AdmittanceState s = {0};
    double wrench[ADMITTANCE_AXES] = {0};
    double twist[ADMITTANCE_AXES] = {0};
    size_t point = 0U;
    for (size_t i = 0U; i < ADMITTANCE_AXES; ++i) {
        p.virtual_mass[i] = mass[i];
        p.damping[i] = damping[i];
        p.speed_limit[i] = 2.0;
        p.acceleration_limit[i] = 100.0;
    }
    assert(admittance_enable(&s, &p));
    for (unsigned k = 1U; k <= 600U; ++k) {
        for (size_t i = 0U; i < ADMITTANCE_AXES; ++i) {
            wrench[i] = (k >= 50U && k < 250U) ? design_wrench[i] :
                        (k >= 250U && k < 375U) ? -design_wrench[i] : 0.0;
        }
        assert(admittance_step(&p, &s, wrench, 0.002, twist));
        if (point < sizeof(reference)/sizeof(reference[0]) &&
            k == reference[point].sample) {
            for (size_t i = 0U; i < ADMITTANCE_AXES; ++i)
                close_to(twist[i], reference[point].velocity[i]);
            ++point;
        }
    }
    assert(point == sizeof(reference)/sizeof(reference[0]));
}

int main(void)
{
    const double dt = 0.002;
    AdmittanceParams p = fixture();
    AdmittanceState s = {0};
    double wrench[ADMITTANCE_AXES] = {0};
    double out[ADMITTANCE_AXES] = {1,1,1,1,1,1};

    /* Disabled cannot generate motion. Explicit enable clears state. */
    assert(!admittance_step(&p, &s, wrench, dt, out));
    all_zero(out);
    assert(admittance_enable(&s, &p));
    all_zero(s.velocity);

    /* Translation and rotation have independent mass/inertia parameters. */
    wrench[0] = 4.0;
    wrench[3] = 0.25;
    assert(admittance_step(&p, &s, wrench, dt, out));
    const double expected_linear = (dt * 4.0) / (2.0*2.0 + dt*4.0);
    const double expected_angular = (dt * 0.25) / (2.0*0.5 + dt*1.0);
    close_to(out[0], expected_linear);
    close_to(out[3], expected_angular);
    for (size_t i = 1U; i < ADMITTANCE_AXES; ++i)
        if (i != 3U) close_to(out[i], 0.0);

    /* First release step averages the last applied wrench with zero.
     * Subsequent zero-wrench samples decay by the Tustin discrete pole. */
    wrench[0] = 0.0;
    wrench[3] = 0.0;
    for (unsigned k = 0U; k < 20U; ++k) {
        const double old_linear = s.velocity[0];
        const double old_angular = s.velocity[3];
        const double old_force = (k == 0U) ? 4.0 : 0.0;
        const double old_torque = (k == 0U) ? 0.25 : 0.0;
        assert(admittance_step(&p, &s, wrench, dt, out));
        close_to(out[0], ((2.0*2.0 - dt*4.0)*old_linear + dt*old_force) /
                         (2.0*2.0 + dt*4.0));
        close_to(out[3], ((2.0*0.5 - dt*1.0)*old_angular + dt*old_torque) /
                         (2.0*0.5 + dt*1.0));
        if (k > 0U) {
            assert(out[0] < old_linear && out[0] > 0.0);
            assert(out[3] < old_angular && out[3] > 0.0);
        }
    }

    /* Enabling anew cannot inherit an old velocity state. */
    assert(admittance_enable(&s, &p));
    all_zero(s.velocity);

    /* Both positive and negative commanded acceleration are bounded. */
    p.acceleration_limit[0] = 0.2; /* m/s^2: <= 0.0004 m/s/cycle */
    wrench[0] = 100.0;
    assert(admittance_step(&p, &s, wrench, dt, out));
    close_to(out[0], dt * 0.2);
    wrench[0] = -100.0;
    assert(admittance_step(&p, &s, wrench, dt, out));
    /* First reversal averages +100 and -100, then a second -100 step
     * reaches the negative acceleration clamp. */
    const double reversal_v = dt * 0.2 * (2.0*2.0 - dt*4.0) /
                              (2.0*2.0 + dt*4.0);
    close_to(out[0], reversal_v);
    assert(admittance_step(&p, &s, wrench, dt, out));
    close_to(out[0], reversal_v - dt * 0.2);

    /* Speed saturation and acceleration saturation must both hold. */
    assert(admittance_enable(&s, &p));
    p.acceleration_limit[0] = 100.0;
    p.speed_limit[0] = 0.001;
    wrench[0] = 100.0;
    assert(admittance_step(&p, &s, wrench, dt, out));
    close_to(out[0], 0.001);
    assert(admittance_step(&p, &s, wrench, dt, out));
    close_to(out[0], 0.001);

    /* Controller disable makes subsequent motion requests fail closed. */
    admittance_disable(&s);
    is_cleared(&s);
    assert(!admittance_step(&p, &s, wrench, dt, out));
    all_zero(out);

    /* Invalid configuration, input, timing and previous state all reset. */
    p.acceleration_limit[5] = 0.0;
    assert(!admittance_enable(&s, &p));
    is_cleared(&s);
    p = fixture();
    p.maximum_step_s = NAN;
    assert(!admittance_enable(&s, &p));
    is_cleared(&s);
    p = fixture();
    assert(admittance_enable(&s, &p));
    assert(!admittance_step(&p, &s, wrench, 0.0, out));
    is_cleared(&s);
    assert(admittance_enable(&s, &p));
    assert(!admittance_step(&p, &s, wrench, INFINITY, out));
    is_cleared(&s);
    assert(admittance_enable(&s, &p));
    assert(!admittance_step(&p, &s, wrench, 0.0041, out));
    is_cleared(&s);
    assert(admittance_enable(&s, &p));
    wrench[5] = NAN;
    assert(!admittance_step(&p, &s, wrench, dt, out));
    is_cleared(&s);
    all_zero(out);
    wrench[5] = 0.0;
    assert(admittance_enable(&s, &p));
    s.velocity[0] = INFINITY;
    assert(!admittance_step(&p, &s, wrench, dt, out));
    is_cleared(&s);
    assert(admittance_enable(&s, &p));
    s.previous_wrench[2] = INFINITY; /* corrupted input history */
    assert(!admittance_step(&p, &s, wrench, dt, out));
    is_cleared(&s);
    assert(admittance_enable(&s, &p));
    s.velocity[0] = 3.0; /* greater than speed limit */
    assert(!admittance_step(&p, &s, wrench, dt, out));
    is_cleared(&s);
    assert(admittance_enable(&s, &p));
    assert(!admittance_step(&p, &s, NULL, dt, out));
    is_cleared(&s);
    assert(admittance_enable(&s, &p));
    assert(!admittance_step(&p, &s, wrench, dt, NULL));
    is_cleared(&s);

    /* Cross-axis failure does not leave any partially committed state. */
    assert(admittance_enable(&s, &p));
    wrench[0] = 4.0;
    wrench[5] = INFINITY;
    assert(!admittance_step(&p, &s, wrench, dt, out));
    is_cleared(&s);
    all_zero(out);

    verify_matlab_optimized_reference();
    puts("PASS: Batch 4C six-axis Tustin numerical, MATLAB reference and fault tests");
    return 0;
}
