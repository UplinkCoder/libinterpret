#include <stdarg.h>
#include "bc_common.h"

typedef struct BackendInterface BackendInterface;

typedef void* BCFunction; 
typedef void* CndJmpBegin;
typedef void* SourceLocation;

typedef uint32_t (*beginFunction_t) (void* ctx, uint32_t fnId, void* fd);
typedef BCValue (*genTemporary_t) (void* ctx, BCType bct);
typedef void (*destroyTemporary_t) (void* ctx, BCValue tmp);
typedef BCValue (*genLocal_t) (void* ctx, BCType bct, const char* name);
typedef void (*Initialize_t) (void* ctx, uint32_t n_args, ...);
typedef void (*InitializeV_t) (void* ctx, uint32_t n_args, va_list args);
typedef void (*Finalize_t) (void* ctx);
typedef BCFunction (*endFunction_t) (void* ctx, uint32_t fnIdx);
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
    Finalize_t Finalize;

    beginFunction_t beginFunction;
    endFunction_t endFunction;

    genTemporary_t genTemporary;
    destroyTemporary_t destroyTemporary;

    genLocal_t genLocal;
    genParameter_t genParameter;
    emitFlg_t emitFlg;

    Alloc_t Alloc;
    Assert_t Assert;
    MemCpy_t MemCpy;

    File_t File;
    Line_t Line;
    Comment_t Comment;
    Prt_t Prt;

    Set_t Set;
    SetHigh_t SetHigh;
    Ult3_t Ult3;
    Ule3_t Ule3;
    Lt3_t Lt3;
    Le3_t Le3;
    Ugt3_t Ugt3;
    Uge3_t Uge3;
    Gt3_t Gt3;
    Ge3_t Ge3;
    Eq3_t Eq3;
    Neq3_t Neq3;
    Add3_t Add3;
    Sub3_t Sub3;
    Mul3_t Mul3;
    Div3_t Div3;
    Udiv3_t Udiv3;
    And3_t And3;
    Or3_t Or3;
    Xor3_t Xor3;
    Lsh3_t Lsh3;
    Rsh3_t Rsh3;
    Mod3_t Mod3;
    Umod3_t Umod3;
    Not_t Not;

    Call_t Call;
    Jmp_t Jmp;
    beginJmp_t beginJmp;
    endJmp_t endJmp;
    beginCndJmp_t beginCndJmp;
    endCndJmp_t endCndJmp;

    Load8_t Load8;
    Store8_t Store8;
    Load16_t Load16;
    Store16_t Store16;
    Load32_t Load32;
    Store32_t Store32;
    Load64_t Load64;
    Store64_t Store64;
    castTo_t castTo;

    Throw_t Throw;
    PushCatch_t PushCatch;
    PopCatch_t PopCatch;
    Ret_t Ret;

    IToF32_t IToF32;
    IToF64_t IToF64;
    F32ToI_t F32ToI;
    F64ToI_t F64ToI;
    F32ToF64_t F32ToF64;
    F64ToF32_t F64ToF32;

    StrEq3_t StrEq3;
    Cat3_t Cat3;

    run_t run;
    destroy_instance_t destroy_instance;
    new_instance_t new_instance;
} BackendInterface;
