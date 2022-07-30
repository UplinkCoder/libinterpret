#define cast(T) (T)

#define ARRAY_SIZE(A) \
     ((unsigned int)(sizeof((A)) / sizeof((A)[0])))

#if !defined(_MSC_VER)
#  define noinline volatile __attribute__ ((noinline))
#else
#  define noinline
#endif

#if defined(_MSC_VER)
#  include "stdint_msvc.h"
#  include <malloc.h>
#  define alloca _alloca
#  ifndef __cplusplus
#    error "win32 compile only works in c++ mode ... use /TP"
#  endif
#else
#  include <stdint.h>
#  include <alloca.h>
#endif

#  ifdef __CC65__
#  define bool _Bool
typedef unsigned char _Bool;
#define inline

/* Standard test-results. */
#  define false 0
#  define true  1
#else
#  ifndef __cplusplus
#     include "stdbool.h"
#  endif
#endif

#if defined(__GNUC__)
#  define UNLIKELY(X) \
    __builtin_expect((X), 0)
#else
#  define UNLIKELY(X) (X)
#endif


#ifdef __cplusplus
#  define EXTERN_C extern "C"
#else
#  define EXTERN_C extern
#endif

#ifdef _MSC_VER
#  if _MSC_VER <= 1800
#    define inline
#  endif
#endif

