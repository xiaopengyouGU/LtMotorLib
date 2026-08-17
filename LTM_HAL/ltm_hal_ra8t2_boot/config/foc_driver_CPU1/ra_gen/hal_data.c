/* generated HAL source file - do not edit */
#include "hal_data.h"
ipc_instance_ctrl_t g_ipc1_ctrl;

/** IPC configuration */
const ipc_cfg_t g_ipc1_cfg = { .channel = 1, .p_callback = ipc1_callback,
#if defined(NULL)
                .p_context = NULL,
#else
		.p_context = (void*) &NULL,
#endif
		.ipl = (10),
#if defined(VECTOR_NUMBER_IPC_IRQ1)
                .irq = VECTOR_NUMBER_IPC_IRQ1,
#else
		.irq = FSP_INVALID_VECTOR,
#endif
		};

/* Instance structure to use this module. */
const ipc_instance_t g_ipc1 = { .p_ctrl = &g_ipc1_ctrl, .p_cfg = &g_ipc1_cfg,
		.p_api = &g_ipc_on_ipc };
wdt_instance_ctrl_t g_wdt0_ctrl;

const wdt_cfg_t g_wdt0_cfg = { .timeout = WDT_TIMEOUT_16384, .clock_division =
		WDT_CLOCK_DIVISION_8192, .window_start = WDT_WINDOW_START_100,
		.window_end = WDT_WINDOW_END_0,
		.reset_control = WDT_RESET_CONTROL_RESET, .stop_control =
				WDT_STOP_CONTROL_ENABLE, .p_callback = NULL, };

/* Instance structure to use this module. */
const wdt_instance_t g_wdt0 = { .p_ctrl = &g_wdt0_ctrl, .p_cfg = &g_wdt0_cfg,
		.p_api = &g_wdt_on_wdt };
ipc_instance_ctrl_t g_ipc0_ctrl;

/** IPC configuration */
const ipc_cfg_t g_ipc0_cfg = { .channel = 0, .p_callback = ipc0_callback,
#if defined(NULL)
                .p_context = NULL,
#else
		.p_context = (void*) &NULL,
#endif
		.ipl = (10),
#if defined(VECTOR_NUMBER_IPC_IRQ0)
                .irq = VECTOR_NUMBER_IPC_IRQ0,
#else
		.irq = FSP_INVALID_VECTOR,
#endif
		};

/* Instance structure to use this module. */
const ipc_instance_t g_ipc0 = { .p_ctrl = &g_ipc0_ctrl, .p_cfg = &g_ipc0_cfg,
		.p_api = &g_ipc_on_ipc };
void g_hal_init(void) {
	g_common_init();
}
