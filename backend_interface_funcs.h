#include <stdarg.h>
#include "bc_common.h"

typedef struct BackendInterface BackendInterface;

typedef void* CndJmpBegin;
typedef void* SourceLocation;

typedef uint32_t (*beginFunction_t) (void* ctx, uint32_t fnId, void* fd);
typedef BCValue (*genTemporary_t) (void* ctx, BCType bct);
typedef void (*destroyTemporary_t) (void* ctx, BCValue tmp);
typedef BCValue (*genLocal_t) (void* ctx, BCType bct, const char* name);
typedef void (*Initialize_t) (void* ctx, uint32_t n_args, ...);
typedef void (*InitializeV_t) (void* ctx, uint32_t n_args, va_list args);
typedef void (*Finalize_t) (void* ctx);
typedef void* (*endFunction_t) (void* ctx, uint32_t fnIdx);
typedef BCValue (*genParameter_t) (void* ctx, BCType bct, const char*);
typedef uint32_t (*beginJmp_t) (void* ctx);
typedef void (*endJmp_t) (void* ctx, BCAddr atIp, BCLabel target);
typedef BCLabel (*genLabel_t) (void* ctx);
typedef CndJmpBegin (*beginCndJmp_t) (void* ctx, BCValue cond, bool ifTrue);
typedef void (*endCndJmp_t) (void* ctx, CndJmpBegin jmp, BCLabel target);
typedef void (*Jmp_t) (void* ctx, BCLabel target);
typedef void (*emitFlg_t) (void* ctx, BCValue lhs);
typedef void (*Alloc_t) (void* ctx, BCValue heapPtr, BCValue size, SourceLocation loc);
typedef void (*Assert_t) (void* ctx, BCValue value, BCValue err, SourceLocation loc);
typedef void (*MemCpy_t) (void* ctx, BCValue dst, BCValue src, BCValue size);
typedef void (*File_t) (void* ctx, const char* filename);
typedef void (*Line_t) (void* ctx, uint line);
typedef void (*Comment_t) (void* ctx, const char* comment);
typedef void (*Prt_t) (void* ctx, BCValue value, bool isString);
typedef void (*Set_t) (void* ctx, BCValue lhs, BCValue rhs);
typedef void (*SetHigh_t) (void* ctx, BCValue lhs, BCValue rhs);
typedef void (*Ult3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*Ule3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*Lt3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*Le3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*Ugt3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*Uge3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*Gt3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*Ge3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*Eq3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*Neq3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*Add3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*Sub3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*Mul3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*Div3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*Udiv3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*And3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*Or3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*Xor3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*Lsh3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*Rsh3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*Mod3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*Umod3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*Not_t) (void* ctx, BCValue result, BCValue val);
typedef void (*Call_t) (void* ctx, BCValue result, BCValue fn, BCValue* args, uint32_t n_args);
typedef void (*Load8_t) (void* ctx, BCValue dest, BCValue from);
typedef void (*Store8_t) (void* ctx, BCValue dest, BCValue value);
typedef void (*Load16_t) (void* ctx, BCValue dest, BCValue from);
typedef void (*Store16_t) (void* ctx, BCValue dest, BCValue value);
typedef void (*Load32_t) (void* ctx, BCValue dest, BCValue from);
typedef void (*Store32_t) (void* ctx, BCValue dest, BCValue value);
typedef void (*Load64_t) (void* ctx, BCValue dest, BCValue from);
typedef void (*Store64_t) (void* ctx, BCValue dest, BCValue value);
typedef BCValue (*castTo_t) (void* ctx, BCValue rhs, BCTypeEnum targetType);

typedef void (*Throw_t) (void* ctx, BCValue e);
typedef void (*PushCatch_t) (void* ctx);
typedef void (*PopCatch_t) (void* ctx);
typedef void (*Ret_t) (void* ctx, BCValue val);
typedef void (*IToF32_t) (void* ctx, BCValue result, BCValue rhs);
typedef void (*IToF64_t) (void* ctx, BCValue result, BCValue rhs);
typedef void (*F32ToI_t) (void* ctx, BCValue result, BCValue rhs);
typedef void (*F64ToI_t) (void* ctx, BCValue result, BCValue rhs);
typedef void (*F32ToF64_t) (void* ctx, BCValue result, BCValue rhs);
typedef void (*F64ToF32_t) (void* ctx, BCValue result, BCValue rhs);
typedef void (*StrEq3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs);
typedef void (*Cat3_t) (void* ctx, BCValue result, BCValue lhs, BCValue rhs, const uint size);

typedef BCValue (*run_t) (void* ctx, uint32_t fnIdx, BCValue* args, uint32_t n_args);
typedef void (*destroy_instance_t) (void* ctx);
typedef void (*new_instance_t) (void ** result_p);

typedef struct BackendInterface
{
    const Initialize_t Initialize;
    const InitializeV_t InitializeV;
    const Finalize_t Finalize;

    const beginFunction_t beginFunction;
    const endFunction_t endFunction;

    const genTemporary_t genTemporary;
    const destroyTemporary_t destroyTemporary;

    const genLocal_t genLocal;
    const genParameter_t genParameter;
    const emitFlg_t emitFlg;

    const Alloc_t Alloc;
    const Assert_t Assert;
    const MemCpy_t MemCpy;

    const File_t File;
    const Line_t Line;
    const Comment_t Comment;
    const Prt_t Prt;

    const Set_t Set;
    const SetHigh_t SetHigh;
    const Ult3_t Ult3;
    const Ule3_t Ule3;
    const Lt3_t Lt3;
    const Le3_t Le3;
    const Ugt3_t Ugt3;
    const Uge3_t Uge3;
    const Gt3_t Gt3;
    const Ge3_t Ge3;
    const Eq3_t Eq3;
    const Neq3_t Neq3;
    const Add3_t Add3;
    const Sub3_t Sub3;
    const Mul3_t Mul3;
    const Div3_t Div3;
    const Udiv3_t Udiv3;
    const And3_t And3;
    const Or3_t Or3;
    const Xor3_t Xor3;
    const Lsh3_t Lsh3;
    const Rsh3_t Rsh3;
    const Mod3_t Mod3;
    const Umod3_t Umod3;
    const Not_t Not;

    const Call_t Call;
    const Jmp_t Jmp;
    const beginJmp_t beginJmp;
    const endJmp_t endJmp;
    const beginCndJmp_t beginCndJmp;
    const endCndJmp_t endCndJmp;

    const Load8_t Load8;
    const Store8_t Store8;
    const Load16_t Load16;
    const Store16_t Store16;
    const Load32_t Load32;
    const Store32_t Store32;
    const Load64_t Load64;
    const Store64_t Store64;
    const castTo_t castTo;

    const Throw_t Throw;
    const PushCatch_t PushCatch;
    const PopCatch_t PopCatch;
    const Ret_t Ret;

    const IToF32_t IToF32;
    const IToF64_t IToF64;
    const F32ToI_t F32ToI;
    const F64ToI_t F64ToI;
    const F32ToF64_t F32ToF64;
    const F64ToF32_t F64ToF32;

    const StrEq3_t StrEq3;
    const Cat3_t Cat3;

    const run_t run;
    const destroy_instance_t destroy_instance;
    const new_instance_t new_instance;
} BackendInterface;
