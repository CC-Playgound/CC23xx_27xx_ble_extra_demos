/*
 *  ======== LedDriverLPF3.c ========
 *  See LedDriverLPF3.h for a description of the module.
 */

#include "LedDriverLPF3.h"

/* Nanosecond pulse timing required by the external LED driver IC */
#define T1H_NS 550
#define T1L_NS 350
#define T0H_NS 230
#define T0L_NS 670

/* Convert a nanosecond duration to the nearest number of timer clock ticks,
 * rounding to the nearest tick.
 */
#define NS_TO_TICKS(ns) \
    ((uint32_t)(((uint64_t)(ns) * LEDDRIVER_TIMER_CLOCK_FREQ + 500000000ULL) / 1000000000ULL))

#define BIT1_HIGH_TICKS NS_TO_TICKS(T1H_NS)
#define BIT1_LOW_TICKS  NS_TO_TICKS(T1L_NS)
#define BIT0_HIGH_TICKS NS_TO_TICKS(T0H_NS)
#define BIT0_LOW_TICKS  NS_TO_TICKS(T0L_NS)

#define BIT1_TOTAL_TICKS (BIT1_HIGH_TICKS + BIT1_LOW_TICKS)
#define BIT0_TOTAL_TICKS (BIT0_HIGH_TICKS + BIT0_LOW_TICKS)

static LGPTimerLPF3_Handle ledDriverHandle = NULL;

/*
 *  ======== LedDriverLPF3_sendPulse ========
 *  Generates one pulse on the LED data pin: output goes high when the
 *  counter is 0 and low again when the counter reaches highTicks. The
 *  counter runs in one-shot mode up to (totalTicks - 1), so the full pulse
 *  period (high + low) is exactly totalTicks timer clock periods.
 */
static void LedDriverLPF3_sendPulse(uint32_t highTicks, uint32_t totalTicks)
{
    uint32_t targetVal = totalTicks - 1;

    /* Timer must be stopped (it is, following the previous one-shot pulse)
     * before its initial compare/target values are updated.
     */
    LGPTimerLPF3_setInitialChannelCompVal(ledDriverHandle, LEDDRIVER_LGPT_CHANNEL, highTicks, true);
    LGPTimerLPF3_setInitialCounterTarget(ledDriverHandle, targetVal, true);

    /* The counter restarts from 0 every time MODE is written, so a fresh
     * one-shot pulse is generated on every call.
     */
    LGPTimerLPF3_start(ledDriverHandle, LGPTimerLPF3_CTL_MODE_UP_ONCE);

    /* Block until the counter has reached its target, i.e. until the pulse
     * has been fully output on the pin. The hardware then automatically
     * stops counting.
     */
    while (LGPTimerLPF3_getCounter(ledDriverHandle) != targetVal) {}

    /* The hardware has already stopped counting on its own; call
     * LGPTimerLPF3_stop() to keep the driver's internal power constraint
     * ref-count balanced with the LGPTimerLPF3_start() call above.
     */
    LGPTimerLPF3_stop(ledDriverHandle);
}

/*
 *  ======== LedDriverLPF3_init ========
 */
bool LedDriverLPF3_init(void)
{
    LGPTimerLPF3_Params params;

    LGPTimerLPF3_Params_init(&params);
    params.channelProperty[LEDDRIVER_LGPT_CHANNEL].action    = LGPTimerLPF3_CH_SET_ON_0_TOGGLE_ON_CMP_PERIODIC;
    params.channelProperty[LEDDRIVER_LGPT_CHANNEL].inputEdge = LGPTimerLPF3_CH_EDGE_NONE;

    ledDriverHandle = LGPTimerLPF3_open(LEDDRIVER_LGPT_INDEX, &params);
    if (ledDriverHandle == NULL)
    {
        return false;
    }

    /* Idle the LED data line low until the first bit is sent */
    LGPTimerLPF3_setChannelOutputLevel(ledDriverHandle, LEDDRIVER_LGPT_CHANNEL, LGPTimerLPF3_CH_LEVEL_LOW);

    return true;
}

/*
 *  ======== LedDriverLPF3_close ========
 */
void LedDriverLPF3_close(void)
{
    if (ledDriverHandle != NULL)
    {
        LGPTimerLPF3_close(ledDriverHandle);
        ledDriverHandle = NULL;
    }
}

/*
 *  ======== LedDriverLPF3_setHigh ========
 */
void LedDriverLPF3_setHigh(void)
{
    LedDriverLPF3_sendPulse(BIT1_HIGH_TICKS, BIT1_TOTAL_TICKS);
}

/*
 *  ======== LedDriverLPF3_setLow ========
 */
void LedDriverLPF3_setLow(void)
{
    LedDriverLPF3_sendPulse(BIT0_HIGH_TICKS, BIT0_TOTAL_TICKS);
}
