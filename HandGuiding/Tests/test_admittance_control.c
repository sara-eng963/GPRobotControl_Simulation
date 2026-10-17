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
    const double expected_linear = (dt * 4.0) / (2.0 + dt*4.0);
    const double expected_angular = (dt * 0.25) / (0.5 + dt*1.0);
    close_to(out[0], expected_linear);
    close_to(out[3], expected_angular);
    for (size_t i = 1U; i < ADMITTANCE_AXES; ++i)
        if (i != 3U) close_to(out[i], 0.0);

    /* Release decays exactly according to backward Euler, with no spring. */
    wrench[0] = 0.0;
    wrench[3] = 0.0;
    for (unsigned k = 0U; k < 20U; ++k) {
        const double old_linear = s.velocity[0];
        const double old_angular = s.velocity[3];
        assert(admittance_step(&p, &s, wrench, dt, out));
        close_to(out[0], old_linear * 2.0 / (2.0 + dt*4.0));
        close_to(out[3], old_angular * 0.5 / (0.5 + dt*1.0));
        assert(out[0] < old_linear && out[0] > 0.0);
        assert(out[3] < old_angular && out[3] > 0.0);
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
    close_to(out[0], 0.0);

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

    puts("PASS: Batch 4C six-axis admittance numerical and fault tests");
    return 0;
}
