/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zephyr.h>
#include <string.h>
#include <py/compile.h>
#include <py/runtime.h>
#include <py/gc.h>
#include "py/mperrno.h"
#include "py/builtin.h"
#include "py/compile.h"
#include "py/runtime.h"
#include "py/repl.h"
#include "py/gc.h"
#include "py/mphal.h"
#include "py/stackctrl.h"
#include "shared/runtime/pyexec.h"
#include "shared/readline/readline.h"
#include "extmod/modbluetooth.h"
// uint8_t program[] = {
// 	0x4D, 0x06, 0x00, 0x1F, 0x03, 0x01, 0x12, 0x68,
// 	0x65, 0x6C, 0x6C, 0x6F, 0x2E, 0x6D, 0x70, 0x79,
// 	0x00, 0x0F, 0x81, 0x77, 0x05, 0x0B, 0x68, 0x65,
// 	0x6C, 0x6C, 0x6F, 0x20, 0x77, 0x6F, 0x72, 0x6C,
// 	0x64, 0x00, 0x60, 0x08, 0x02, 0x01, 0x11, 0x02,
// 	0x23, 0x00, 0x34, 0x01, 0x59, 0x51, 0x63
// };
static char heap[MICROPY_HEAP_SIZE];

static const char *demo_single_input =
    "print('hello world!', list(x + 1 for x in range(10)), end='eol\\n')";

static const char *demo_file_input =
    "import micropython\n"
    "\n"
    "print(dir(micropython))\n"
    "\n"
    "for i in range(10):\n"
    "    print('iter {:08}'.format(i))";


static void do_str(const char *src, mp_parse_input_kind_t input_kind) {
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        // Compile, parse and execute the given string.
        mp_lexer_t *lex = mp_lexer_new_from_str_len(MP_QSTR__lt_stdin_gt_, src, strlen(src), 0);
        qstr source_name = lex->source_name;
        mp_parse_tree_t parse_tree = mp_parse(lex, input_kind);
        mp_obj_t module_fun = mp_compile(&parse_tree, source_name, true);
        mp_call_function_0(module_fun);
        nlr_pop();
    } else {
        // Uncaught exception: print it out.
        mp_obj_print_exception(&mp_plat_print, (mp_obj_t)nlr.ret_val);
    }
}


// void run(uint8_t *data, size_t size)
// {
// 		mp_obj_t module_fun;
//         mp_module_context_t *ctx = m_new_obj(mp_module_context_t);
//             ctx->module.globals = mp_globals_get();
//             #else
//             mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("script compilation not supported"));
//             #endif
//         }

//         // execute code
//         mp_hal_set_interrupt_char(CHAR_CTRL_C); // allow ctrl-C to interrupt us
//         #if MICROPY_REPL_INFO
//         start = mp_hal_ticks_ms();
//         #endif
//         mp_call_function_0(module_fun);
// }

void main(void)
{
    k_msleep(7000);
	printf("Hello World1! %s\n", CONFIG_BOARD);

    mp_stack_ctrl_init();
    // Make MicroPython's stack limit somewhat smaller than full stack available
    mp_stack_set_limit(CONFIG_MAIN_STACK_SIZE - 512);

    mp_hal_init();

    gc_init(heap, heap + sizeof(heap));
    mp_init();


    // mp_init();
    do_str(demo_single_input, MP_PARSE_SINGLE_INPUT);
    do_str(demo_file_input, MP_PARSE_FILE_INPUT);
    mp_deinit();

}

void nlr_jump_fail(void *val) {
    for (;;) {
    }
}

void gc_collect(void) {
    // WARNING: This gc_collect implementation doesn't try to get root
    // pointers from CPU registers, and thus may function incorrectly.
    void *dummy;
    gc_collect_start();
    gc_collect_root(&dummy, ((mp_uint_t)MP_STATE_THREAD(stack_top) - (mp_uint_t)&dummy) / sizeof(mp_uint_t));
    gc_collect_end();
    // gc_dump_info();
}
