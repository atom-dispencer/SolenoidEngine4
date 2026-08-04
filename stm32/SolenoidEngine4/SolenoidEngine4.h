#ifndef GUARD_SOLENOIDENGINE4
#define GUARD_SOLENOIDENGINE4

enum Solenoid
{
    SOLENOID_1,
    SOLENOID_2,
    SOLENOID_3,
    SOLENOID_4,
    //
    SOLENOID_COUNT
};

static const enum Solenoid SOLENOIDS[SOLENOID_COUNT] =
{
    SOLENOID_1,
    SOLENOID_2,
    SOLENOID_3,
    SOLENOID_4
};

static const bool is_valid_solenoid(enum Solenoid s)
{
    switch(s)
    {
        case SOLENOID_1:
        case SOLENOID_2:
        case SOLENOID_3:
        case SOLENOID_4:
            return true;
        default:
            return false;
    }
}

#endif