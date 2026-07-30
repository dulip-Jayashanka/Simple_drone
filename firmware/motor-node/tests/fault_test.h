#ifndef FAULT_TEST_H
#define FAULT_TEST_H

/*
 * FAULT_TEST_MODE:
 *
 * 0 = normal firmware
 * 1 = Default_Handler test
 * 2 = NMI test
 * 3 = HardFault test
 */
void fault_test_run(void);

#endif /* FAULT_TEST_H */