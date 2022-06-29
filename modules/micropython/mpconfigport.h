// #include <alloca.h>

// Include Zephyr's autoconf.h, which should be made first by Zephyr makefiles
#include "autoconf.h"
// Included here to get basic Zephyr environment (macros, etc.)
#include <zephyr/zephyr.h>
#include <zephyr/drivers/spi.h>

#define MICROPY_HEAP_SIZE CONFIG_MICROPY_HEAP_SIZE

#ifdef CONFIG_MICROPY_ENABLE_SOURCE_LINE
#define MICROPY_ENABLE_SOURCE_LINE (1)
#else
#define MICROPY_ENABLE_SOURCE_LINE (0)
#endif

#ifdef CONFIG_MICROPY_STACK_CHECK
#define MICROPY_STACK_CHECK (1)
#else
#define MICROPY_STACK_CHECK (0)
#endif

#ifdef CONFIG_MICROPY_ENABLE_GC
#define MICROPY_ENABLE_GC (1)
#else
#define MICROPY_ENABLE_GC (0)
#endif

#ifdef CONFIG_MICROPY_HELPER_REPL
#define MICROPY_HELPER_REPL (1)
#else
#define MICROPY_HELPER_REPL (0)
#endif

#ifdef CONFIG_MICROPY_REPL_AUTO_INDENT
#define MICROPY_REPL_AUTO_INDENT (1)
#else
#define MICROPY_REPL_AUTO_INDENT (0)
#endif

#ifdef CONFIG_MICROPY_KBD_EXCEPTION
#define MICROPY_KBD_EXCEPTION (1)
#else
#define MICROPY_KBD_EXCEPTION (0)
#endif

#ifdef CONFIG_MICROPY_CPYTHON_COMPAT
#define MICROPY_CPYTHON_COMPAT (1)
#else
#define MICROPY_CPYTHON_COMPAT (0)
#endif

#ifdef CONFIG_MICROPY_PY_ASYNC_AWAIT
#define MICROPY_PY_ASYNC_AWAIT (1)
#else
#define MICROPY_PY_ASYNC_AWAIT (0)
#endif

#ifdef CONFIG_MICROPY_PY_ATTRTUPLE
#define MICROPY_PY_ATTRTUPLE (1)
#else
#define MICROPY_PY_ATTRTUPLE (0)
#endif

#ifdef CONFIG_MICROPY_PY_BUILTINS_ENUMERATE
#define MICROPY_PY_BUILTINS_ENUMERATE (1)
#else
#define MICROPY_PY_BUILTINS_ENUMERATE (0)
#endif

#ifdef CONFIG_MICROPY_PY_BUILTINS_FILTER
#define MICROPY_PY_BUILTINS_FILTER (1)
#else
#define MICROPY_PY_BUILTINS_FILTER (0)
#endif

#ifdef CONFIG_MICROPY_PY_BUILTINS_MIN_MAX
#define MICROPY_PY_BUILTINS_MIN_MAX (1)
#else
#define MICROPY_PY_BUILTINS_MIN_MAX (0)
#endif

#ifdef CONFIG_MICROPY_PY_BUILTINS_PROPERTY
#define MICROPY_PY_BUILTINS_PROPERTY (1)
#else
#define MICROPY_PY_BUILTINS_PROPERTY (0)
#endif

#ifdef CONFIG_MICROPY_PY_BUILTINS_RANGE_ATTRS
#define MICROPY_PY_BUILTINS_RANGE_ATTRS (1)
#else
#define MICROPY_PY_BUILTINS_RANGE_ATTRS (0)
#endif

#ifdef CONFIG_MICROPY_PY_BUILTINS_REVERSED
#define MICROPY_PY_BUILTINS_REVERSED (1)
#else
#define MICROPY_PY_BUILTINS_REVERSED (0)
#endif

#ifdef CONFIG_MICROPY_PY_BUILTINS_SET
#define MICROPY_PY_BUILTINS_SET (1)
#else
#define MICROPY_PY_BUILTINS_SET (0)
#endif

#ifdef CONFIG_MICROPY_PY_BUILTINS_STR_COUNT
#define MICROPY_PY_BUILTINS_STR_COUNT (1)
#else
#define MICROPY_PY_BUILTINS_STR_COUNT (0)
#endif

#ifdef CONFIG_MICROPY_PY_BUILTINS_MEMORYVIEW
#define MICROPY_PY_BUILTINS_MEMORYVIEW (1)
#else
#define MICROPY_PY_BUILTINS_MEMORYVIEW (0)
#endif

#ifdef CONFIG_MICROPY_PY_BUILTINS_HELP
#define MICROPY_PY_BUILTINS_HELP (1)
#else
#define MICROPY_PY_BUILTINS_HELP (0)
#endif

#ifdef CONFIG_MICROPY_PY_ARRAY
#define MICROPY_PY_ARRAY (1)
#else
#define MICROPY_PY_ARRAY (0)
#endif

#ifdef CONFIG_MICROPY_PY_COLLECTIONS
#define MICROPY_PY_COLLECTIONS (1)
#else
#define MICROPY_PY_COLLECTIONS (0)
#endif

#ifdef CONFIG_MICROPY_PY_CMATH
#define MICROPY_PY_CMATH (1)
#else
#define MICROPY_PY_CMATH (0)
#endif

#ifdef CONFIG_MICROPY_PY_IO
#define MICROPY_PY_IO (1)
#else
#define MICROPY_PY_IO (0)
#endif

#ifdef CONFIG_MICROPY_PY_MICROPYTHON_MEM_INFO
#define MICROPY_PY_MICROPYTHON_MEM_INFO (1)
#else
#define MICROPY_PY_MICROPYTHON_MEM_INFO (0)
#endif

#ifdef CONFIG_MICROPY_PY_MACHINE
#define MICROPY_PY_MACHINE (1)
#else
#define MICROPY_PY_MACHINE (0)
#endif

#ifdef CONFIG_MICROPY_PY_MACHINE_I2C
#define MICROPY_PY_MACHINE_I2C (1)
#else
#define MICROPY_PY_MACHINE_I2C (0)
#endif

#ifdef CONFIG_MICROPY_PY_MACHINE_SPI
#define MICROPY_PY_MACHINE_SPI (1)
#else
#define MICROPY_PY_MACHINE_SPI (0)
#endif

#ifdef CONFIG_MICROPY_MODULE_WEAK_LINKS
#define MICROPY_MODULE_WEAK_LINKS (1)
#else
#define MICROPY_MODULE_WEAK_LINKS (0)
#endif

#ifdef CONFIG_MICROPY_PY_STRUCT
#define MICROPY_PY_STRUCT (1)
#else
#define MICROPY_PY_STRUCT (0)
#endif



#define MICROPY_PY_BUILTINS_HELP_TEXT zephyr_help_text
#define MICROPY_PY_MACHINE_PIN_MAKE_NEW mp_pin_make_new
#define MICROPY_PY_MACHINE_SPI_MSB (SPI_TRANSFER_MSB)
#define MICROPY_PY_MACHINE_SPI_LSB (SPI_TRANSFER_LSB)


#ifdef CONFIG_NETWORKING
// If we have networking, we likely want errno comfort
#define MICROPY_PY_UERRNO           (1)
#define MICROPY_PY_USOCKET          (1)
#endif
#ifdef CONFIG_BT
#define MICROPY_PY_BLUETOOTH        (1)
#ifdef CONFIG_BT_CENTRAL
#define MICROPY_PY_BLUETOOTH_ENABLE_CENTRAL_MODE (1)
#endif
#define MICROPY_PY_BLUETOOTH_ENABLE_GATT_CLIENT (0)
#endif
#define MICROPY_PY_UBINASCII        (1)
#define MICROPY_PY_UHASHLIB         (1)
#define MICROPY_PY_UOS              (1)
#define MICROPY_PY_UTIME            (1)
#define MICROPY_PY_UTIME_MP_HAL     (1)
#define MICROPY_PY_ZEPHYR           (1)
#define MICROPY_PY_ZSENSOR          (1)
#define MICROPY_PY_SYS_MODULES      (0)
#define MICROPY_LONGINT_IMPL (2)
#define MICROPY_FLOAT_IMPL (MICROPY_FLOAT_IMPL_FLOAT)
#define MICROPY_PY_BUILTINS_COMPLEX (0)
#define MICROPY_ENABLE_SCHEDULER    (1)
#define MICROPY_VFS                 (1)
#define MICROPY_READER_VFS          (MICROPY_VFS)

// fatfs configuration used in ffconf.h
#define MICROPY_FATFS_ENABLE_LFN       (1)
#define MICROPY_FATFS_LFN_CODE_PAGE    437 /* 1=SFN/ANSI 437=LFN/U.S.(OEM) */
#define MICROPY_FATFS_USE_LABEL        (1)
#define MICROPY_FATFS_RPATH            (2)
#define MICROPY_FATFS_NORTC            (1)

// Saving extra crumbs to make sure binary fits in 128K
#define MICROPY_COMP_CONST_FOLDING  (0)
#define MICROPY_COMP_CONST (0)
#define MICROPY_COMP_DOUBLE_TUPLE_ASSIGN (0)

void mp_hal_signal_event(void);
#define MICROPY_SCHED_HOOK_SCHEDULED mp_hal_signal_event()

#define MICROPY_PY_SYS_PLATFORM "zephyr"

#ifdef CONFIG_BOARD
#define MICROPY_HW_BOARD_NAME "zephyr-" CONFIG_BOARD
#else
#define MICROPY_HW_BOARD_NAME "zephyr-generic"
#endif

#ifdef CONFIG_SOC
#define MICROPY_HW_MCU_NAME CONFIG_SOC
#else
#define MICROPY_HW_MCU_NAME "unknown-cpu"
#endif

// #define MICROPY_MODULE_FROZEN_MPY 1
// #define MICROPY_MODULE_FROZEN_STR   (1)
// #define MICROPY_EMIT_BYTECODE_USES_QSTR_TABLE (1)
typedef int mp_int_t; // must be pointer size
typedef unsigned mp_uint_t; // must be pointer size
typedef long mp_off_t;

#define MP_STATE_PORT MP_STATE_VM

#define MICROPY_PORT_ROOT_POINTERS \
    const char *readline_hist[8]; \
    void *machine_pin_irq_list; /* Linked list of pin irq objects */ \
    struct _mp_bluetooth_zephyr_root_pointers_t *bluetooth_zephyr_root_pointers;

// extra built in names to add to the global namespace
#define MICROPY_PORT_BUILTINS \
    { MP_ROM_QSTR(MP_QSTR_open), MP_ROM_PTR(&mp_builtin_open_obj) },

#define MICROPY_BEGIN_ATOMIC_SECTION irq_lock
#define MICROPY_END_ATOMIC_SECTION irq_unlock
