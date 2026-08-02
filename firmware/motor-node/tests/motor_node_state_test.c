#include "motor_node_state.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

/* Host-side mock of the Phase 2.1.6 motor-output driver. */
static bool mock_force_safe_succeeds;
static bool mock_outputs_safe;
static uint32_t mock_force_safe_call_count;

void motor_outputs_force_safe(void)
{
    mock_force_safe_call_count++;

    if (mock_force_safe_succeeds)
    {
        mock_outputs_safe = true;
    }
}

bool motor_outputs_are_safe(void)
{
    return mock_outputs_safe;
}

bool motor_outputs_safe_state_was_requested(void)
{
    return (mock_force_safe_call_count != 0UL);
}

static void mock_reset(bool force_safe_succeeds)
{
    mock_force_safe_succeeds = force_safe_succeeds;
    mock_outputs_safe = false;
    mock_force_safe_call_count = 0UL;
}

static void test_initialization_enters_disarmed(void)
{
    mock_reset(true);

    assert(motor_node_state_init());
    assert(motor_node_is_disarmed());
    assert(g_motor_node_state == MOTOR_NODE_STATE_DISARMED);
    assert(g_motor_node_state_initialized == 1UL);
    assert(g_disarmed_enforcement_count == 1UL);
    assert(g_disarmed_safety_failure_count == 0UL);
    assert(mock_force_safe_call_count == 1UL);
}

static void test_disarmed_is_continuously_enforced(void)
{
    mock_reset(true);
    assert(motor_node_state_init());

    /* Simulate another code path changing a motor output. */
    mock_outputs_safe = false;

    assert(motor_node_state_process());
    assert(mock_outputs_safe);
    assert(g_disarmed_enforcement_count == 2UL);
    assert(mock_force_safe_call_count == 2UL);
}

static void test_unknown_state_returns_to_disarmed(void)
{
    mock_reset(true);
    assert(motor_node_state_init());

    g_motor_node_state = (motor_node_state_t)99;

    assert(motor_node_state_process());
    assert(g_motor_node_state == MOTOR_NODE_STATE_DISARMED);
    assert(g_unexpected_state_count == 1UL);
    assert(mock_outputs_safe);
}

static void test_safety_enforcement_failure_is_reported(void)
{
    mock_reset(false);

    assert(!motor_node_state_init());
    assert(!motor_node_is_disarmed());
    assert(g_motor_node_state == MOTOR_NODE_STATE_DISARMED);
    assert(g_motor_node_state_initialized == 0UL);
    assert(g_disarmed_safety_failure_count == 1UL);
}

int main(void)
{
    test_initialization_enters_disarmed();
    test_disarmed_is_continuously_enforced();
    test_unknown_state_returns_to_disarmed();
    test_safety_enforcement_failure_is_reported();

    return 0;
}
