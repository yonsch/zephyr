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
#include "py/persistentcode.h"
#include "py/repl.h"
#include "py/gc.h"
#include "py/mphal.h"
#include "py/stackctrl.h"
#include "shared/runtime/pyexec.h"
#include "shared/readline/readline.h"
#include "extmod/modbluetooth.h"
uint8_t program[] = {
	0x4D, 0x06, 0x00, 0x1F, 0x03, 0x01, 0x10, 0x68,
    0x65, 0x6C, 0x6C, 0x6F, 0x2E, 0x70, 0x79, 0x00,
    0x0F, 0x81, 0x77, 0x05, 0x0B, 0x68, 0x65, 0x6C,
    0x6C, 0x6F, 0x20, 0x77, 0x6F, 0x72, 0x6C, 0x64,
    0x00, 0x60, 0x08, 0x02, 0x01, 0x11, 0x02, 0x23,
    0x00, 0x34, 0x01, 0x59, 0x51, 0x63
};
static char heap[MICROPY_HEAP_SIZE];

static const char *demo_file_input =
    "import micropython\n"
    "print('Hello world!', list(x for x in range(10)))\n"
    "print(dir(micropython))\n"
    "for i in range(10):\n"
    "    print('{:02}'.format(i))\n"

#ifdef CONFIG_MICROPY_PY_BUILTINS_MIN_MAX
    "print('min(2, 3) =', min(2, 3))\n"
    "print('max(2, 3) =', max(2, 3))\n"
#endif /* CONFIG_MICROPY_PY_BUILTINS_MIN_MAX */

#ifdef CONFIG_MICROPY_PY_BUILTINS_ENUMERATE
    "for i, color in enumerate(['red', 'green', 'blue']):\n"
    "    print(i, color)\n"
#endif /* CONFIG_MICROPY_PY_BUILTINS_ENUMERATE */

#ifdef CONFIG_MICROPY_PY_BUILTINS_FILTER
    "print(list(filter(lambda x: x % 2 == 0, range(10))))\n"
#endif /* CONFIG_MICROPY_PY_BUILTINS_FILTER */

#ifdef CONFIG_MICROPY_PY_BUILTINS_REVERSED
    "print(list(reversed(range(10))))\n"
#endif /* CONFIG_MICROPY_PY_BUILTINS_REVERSED */

#ifdef CONFIG_MICROPY_PY_BUILTINS_SET
    "print(set(range(10)) - set(range(3, 7)))\n"
#endif /* CONFIG_MICROPY_PY_BUILTINS_SET */
    ;


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

static void do_bytecode(uint8_t *data, size_t size)
{
    nlr_buf_t nlr;

    if (nlr_push(&nlr) == 0) {
        mp_module_context_t *context = m_new_obj(mp_module_context_t);

        context->module.globals = mp_globals_get();
        mp_compiled_module_t module = mp_raw_code_load_mem(data, size, context);

        mp_obj_t module_fun = mp_make_function_from_raw_code(module.rc, module.context, NULL);
        mp_call_function_0(module_fun);
        nlr_pop();
    } else {
        // Uncaught exception: print it out.
        mp_obj_print_exception(&mp_plat_print, (mp_obj_t)nlr.ret_val);
    }
}

void main(void)
{
    mp_stack_ctrl_init();
    // Make MicroPython's stack limit somewhat smaller than full stack available
    mp_stack_set_limit(CONFIG_MAIN_STACK_SIZE - 512);

    mp_hal_init();

    gc_init(heap, heap + sizeof(heap));
    mp_init();

    // mp_init();
    do_str(demo_file_input, MP_PARSE_FILE_INPUT);
    do_bytecode(program, sizeof(program));
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
