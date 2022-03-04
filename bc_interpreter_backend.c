/**
 * Written By Stefan Koch in 2016 - 2022
*/

#include "bc_common.h"
#include "backend_interface_funcs.h"
#include <math.h>
#include "int_iter.c"

#define cast(T) (T)

#ifndef NDEBUG
#  define DEBUG(...) __VA_ARGS__
#else
#  define DEBUG(...)
#endif

typedef struct RetainedCall
{
    BCValue fn;
    BCValue* args;
    uint32_t n_args;

    uint callerId;
    BCAddr callerIp;
    StackAddr callerSp;
} RetainedCall;


typedef enum LongInst
{
    LongInst_PrintValue,
    LongInst_RelJmp,
    LongInst_Ret32,
    LongInst_Ret64,
    LongInst_RetS32,
    LongInst_RetS64,
    LongInst_Not,

    LongInst_Flg, // writes the conditionFlag into [lw >> 16]
    //End Former ShortInst

    LongInst_Jmp,
    LongInst_JmpFalse,
    LongInst_JmpTrue,
    LongInst_JmpZ,
    LongInst_JmpNZ,

    LongInst_PushCatch,
    LongInst_PopCatch,
    LongInst_Throw,

    // 2 StackOperands
    LongInst_Add,
    LongInst_Sub,
    LongInst_Div,
    LongInst_Mul,
    LongInst_Mod,
    LongInst_Eq, //sets condflags
    LongInst_Neq, //sets condflag
    LongInst_Lt, //sets condflags
    LongInst_Le,
    LongInst_Gt, //sets condflags
    LongInst_Ge,
    LongInst_Ult,
    LongInst_Ule,
    LongInst_Ugt,
    LongInst_Uge,
    LongInst_Udiv,
    LongInst_Umod,
    LongInst_And,
    LongInst_And32,
    LongInst_Or,
    LongInst_Xor,
    LongInst_Xor32,
    LongInst_Lsh,
    LongInst_Rsh,
    LongInst_Set,

    LongInst_StrEq,
    LongInst_Assert,

    // Immedate operand
    LongInst_ImmAdd,
    LongInst_ImmSub,
    LongInst_ImmDiv,
    LongInst_ImmMul,
    LongInst_ImmMod,
    LongInst_ImmEq,
    LongInst_ImmNeq,
    LongInst_ImmLt,
    LongInst_ImmLe,
    LongInst_ImmGt,
    LongInst_ImmGe,
    LongInst_ImmUlt,
    LongInst_ImmUle,
    LongInst_ImmUgt,
    LongInst_ImmUge,
    LongInst_ImmUdiv,
    LongInst_ImmUmod,
    LongInst_ImmAnd,
    LongInst_ImmAnd32,
    LongInst_ImmOr,
    LongInst_ImmXor,
    LongInst_ImmXor32,
    LongInst_ImmLsh,
    LongInst_ImmRsh,

#define FLT32_BEGIN LongInst_FAdd32
    LongInst_FAdd32,
    LongInst_FSub32,
    LongInst_FDiv32,
    LongInst_FMul32,
    LongInst_FMod32,
    LongInst_FEq32,
    LongInst_FNeq32,
    LongInst_FLt32,
    LongInst_FLe32,
    LongInst_FGt32,
    LongInst_FGe32,
#define FLT32_END LongInst_FGe32

    LongInst_F32ToF64,
    LongInst_F32ToI,
    LongInst_IToF32,

#define FLT64_BEGIN LongInst_FAdd32
    LongInst_FAdd64,
    LongInst_FSub64,
    LongInst_FDiv64,
    LongInst_FMul64,
    LongInst_FMod64,
    LongInst_FEq64,
    LongInst_FNeq64,
    LongInst_FLt64,
    LongInst_FLe64,
    LongInst_FGt64,
    LongInst_FGe64,
#define FLT64_END LongInst_FGe64

    LongInst_F64ToF32,
    LongInst_F64ToI,
    LongInst_IToF64,

    LongInst_SetHighImm32,
    LongInst_SetImm32,
    LongInst_SetImm8,

    LongInst_Call,
    LongInst_HeapLoad8,
    LongInst_HeapStore8,
    LongInst_HeapLoad16,
    LongInst_HeapStore16,
    LongInst_HeapLoad32, ///SP[hi & 0xFFFF] = Heap[align4(SP[hi >> 16])]
    LongInst_HeapStore32, ///Heap[align4(SP[hi & 0xFFFF)] = SP[hi >> 16]]
    LongInst_HeapLoad64,
    LongInst_HeapStore64,
    LongInst_Alloc, /// SP[hi & 0xFFFF] = heapSize; heapSize += SP[hi >> 16]
    LongInst_MemCpy,

    LongInst_BuiltinCall, // call a builtin.
    LongInst_Cat,
    LongInst_Comment,
    LongInst_Line,
    LongInst_File,

    LongInst_max
} LongInst;

// mask for bit 0-6
#define INSTMASK 0x7F

/** 2StackInst Layout :
* [0-6] Instruction
* [6-7] Unused
* -----------------
* [8-31] Unused
* [32-48] Register (lhs)
* [48-64] Register (rhs)
* *************************
* ImmInstructions Layout :
* [0-6] Instruction
* [6-7] Unused
* ------------------------
* [8-16] Unused
* [16-32] Register (lhs)
* [32-64] Imm32 (rhs)
****************************
* 3 OperandInstuctions // memcpy
* [0-6] Instruction
* [6-7] Unused
* -----------------
* [16-32] Register (extra_data)
* [32-48] Register (lhs)
* [48-64] Register (rhs) 

*/

// static_assert(LongInst_max < INSTMASK);

static int16_t BCGen_isShortJump(const int offset)
{
    assert(offset != 0);//, "A Jump to the Jump itself is invalid");

    const bool wasNegative = (offset < 0);
    int abs_offset = wasNegative ? offset * -1 : offset;

    if (abs_offset < (1 << 15))
    {
        return (cast(ushort)(wasNegative ? abs_offset *= -1 : abs_offset));
    }
    else
    {
        return 0;
    }
}

static inline uint32_t BCGen_ShortInst16(const LongInst i, const int _imm)
{
    int16_t imm = (int16_t) _imm;
    return (i | imm << 16);
}

static inline uint32_t BCGen_ShortInst16Ex(const LongInst i, uint8_t ex, uint16_t imm)
{
    return i | ex << 8 | imm << 16;
}

typedef enum BCFunctionTypeEnum
{
    BCFunctionTypeEnum_Undef,
    BCFunctionTypeEnum_Builtin,
    BCFunctionTypeEnum_Bytecode,
    BCFunctionTypeEnum_Compiled,
} BCFunctionTypeEnum;

typedef struct BCFunction
{
    void* funcDecl;
    uint32_t fn;

    BCFunctionTypeEnum type;
    uint16_t nArgs;
    uint16_t maxStackUsed;

    uint32_t bytecode_start; // should be const but currently we need to assign to this;
    uint32_t bytecode_end;
} BCFunction;

typedef struct BCGen
{
    uint32_t byteCodeArray[8192];
    uint32_t* byteCodeArrayExtra;
    uint32_t byteCodeCount;
    uint32_t byteCodeExtraCapacity;

    /// ip starts at 4 because 0 should be an invalid address;
    uint32_t ip;
    uint32_t sp;

    uint8_t parameterCount;
    uint16_t temporaryCount;

    BCFunction* functions;
    uint32_t functionCount;
    uint32_t functionCapacity;

    uint32_t functionIdx;
    void* fd;
    bool insideFunction;

    BCLocal* locals;
    uint32_t localCount;
    uint32_t localCapacity;

    RetainedCall* calls;
    uint32_t callCount;
    uint32_t callCapacity;

    bool finalized;
} BCGen;

static const int max_call_depth = 2000;

#define INITIAL_LOCALS_CAPACITY 2048
#define INITIAL_CALLS_CAPACITY 2048

static inline void BCGen_Init(BCGen* self)
{
    self->byteCodeArrayExtra = 0;
    self->byteCodeCount = 0;
    self->byteCodeExtraCapacity = 0;

    self->locals = (BCLocal*) malloc(sizeof(BCLocal) * INITIAL_LOCALS_CAPACITY);
    self->localCount = 0;
    self->localCapacity = INITIAL_LOCALS_CAPACITY;

    self->parameterCount = 0;
    self->temporaryCount = 0;
    self->temporaryCount = 0;

    self->ip = 4;
    self->sp = 4;

    self->insideFunction = 0;
    self->functionIdx = 0;

    self->calls = (RetainedCall*) malloc(sizeof(RetainedCall) * INITIAL_CALLS_CAPACITY);
    self->callCount = 0;
    self->callCapacity = INITIAL_CALLS_CAPACITY;

    self->functions = (BCFunction*)malloc(sizeof(BCFunction) * INITIAL_LOCALS_CAPACITY);
    self->functionCount = 0;
    self->functionCapacity = INITIAL_LOCALS_CAPACITY;

    self->finalized = false;
}


void BCGen_new_instance(void** pResult)
{
    BCGen* gen = (BCGen*) malloc(sizeof(BCGen));
    BCGen_Init(gen);
    *pResult = (void*)gen;

    return ;
}

void BCGen_destroy_instance(void* p)
{
    free(p);
}

typedef struct Catch
{
    uint32_t ip;
    uint32_t stackDepth;
} Catch;

typedef struct ReturnAddr
{
    uint32_t ip;
    uint32_t fnId;
    uint32_t stackSize;
    int64_t* retval;
} ReturnAddr;

#define MAX_CALL_DEPTH 2000
#define LOCAL_STACK_SIZE 2048

typedef struct BCInterpreter {
    uint32_t ip;
    
    uint32_t n_return_addrs;
    
    uint32_t callDepth;
    uint32_t fnIdx;
    
    uint32_t stackTop;
    
    uint32_t* stackExtra;
    uint32_t stackExtraCapacity;
    
    uint32_t lastLine;
    
    int64_t stack[LOCAL_STACK_SIZE];
    ReturnAddr returnAddrs[MAX_CALL_DEPTH];

    BCValue cRetval;
} BCInterpreter;

bool BCInterpreter_Return(BCInterpreter* self)
{
    uint32_t ip = 0;

    if (self->n_return_addrs)
    {
        ReturnAddr returnAddr = self->returnAddrs[--self->n_return_addrs];
        self->fnIdx = returnAddr.fnId;
        self->ip = returnAddr.ip;

        self->stackTop -= (returnAddr.stackSize / 4);
        self->callDepth--;
        BCValue cRetval = self->cRetval;

        if (cRetval.vType == BCValueType_Exception)
        {
            assert(!"Execptions are currently unhandeld");
            //return HandleExp();
        }
        if (cRetval.vType == BCValueType_Error || cRetval.vType == BCValueType_Bailout)
        {
            return true;
        }
        if (cRetval.type.type == BCTypeEnum_i64 || cRetval.type.type == BCTypeEnum_u64 || cRetval.type.type == BCTypeEnum_f52)
        {
            (*returnAddr.retval) = cRetval.imm64.imm64;
        }
        else
        {
            (*returnAddr.retval) = cRetval.imm32.imm32;
        }
        return false;
    }
    else
    {
        return true;
    }

    assert(0);
    return true;
}

void PrintCode(IntIter* iter)
{
    uint32_t ip = 0;

    uint32_t lw;
    while (IntIter_NextInt(iter, &lw))
    {
        printf("%d: ", ip);
        uint32_t hi;
        bool worked = IntIter_NextInt(iter, &hi);
        assert(worked);

        const int32_t imm32c = (int32_t) hi;
        ip += 2;

        // consider splitting the stackPointer in stackHigh and stackLow

        const uint32_t opRefOffset = (lw >> 16) & 0xFFFF;
        const uint32_t lhsOffset   = hi & 0xFFFF;
        const uint32_t rhsOffset   = (hi >> 16) & 0xFFFF;
/*
        int64_t* lhsRef = (&stackP[(lhsOffset / 4)]);
        int64_t* rhs = (&stackP[(rhsOffset / 4)]);
        int64_t* lhsStackRef = (&stackP[(opRefOffset / 4)]);
        int64_t* opRef = &stackP[(opRefOffset / 4)];
*/
        bool cond;

        if (!lw)
        {
            printf("NOP NOP\n");
            continue;
        }

        switch (cast(LongInst)(lw & INSTMASK))
        {
        case LongInst_ImmAdd:
            {
                printf("LongInst_ImmAdd R[%d] += %d\n", opRefOffset / 4, imm32c);
            }
            break;

        case LongInst_ImmSub:
            {
                printf("LongInst_ImmSub R[%d] -= %d\n", opRefOffset / 4, imm32c);
            }
            break;

        case LongInst_ImmMul:
            {
                printf("LongInst_ImmMul R[%d] *= %d\n", opRefOffset / 4, imm32c);
            }
            break;

        case LongInst_ImmDiv:
            {
                printf("LongInst_ImmDiv R[%d] /= %d\n", opRefOffset / 4, imm32c);
            }
            break;

        case LongInst_ImmUdiv:
            {
                printf("LongInst_ImmUdiv R[%d] /= %d\n", opRefOffset / 4, imm32c);
            }
            break;

        case LongInst_ImmAnd:
            {
                printf("LongInst_ImmAnd R[%d] &= %u\n", opRefOffset / 4, hi);
            }
            break;
        case LongInst_ImmAnd32:
            {
                printf("LongInst_ImmAnd32 R[%d] &= %u\n", opRefOffset / 4, hi);
            }
            break;
        case LongInst_ImmOr:
            {
                printf("LongInst_ImmOr R[%d] |= %u\n", opRefOffset / 4, hi);
            }
            break;
        case LongInst_ImmXor:
            {
                printf("LongInst_ImmXor R[%d] ^= %u\n", opRefOffset / 4, hi);
            }
            break;
        case LongInst_ImmXor32:
            {
                printf("LongInst_ImmXor32 R[%d] ^= %u\n", opRefOffset / 4, hi);
            }
            break;

        case LongInst_ImmLsh:
            {
                printf("LongInst_ImmLsh R[%d] <<= %u\n", opRefOffset / 4, hi);
            }
            break;
        case LongInst_ImmRsh:
            {
                printf("LongInst_ImmRsh R[%d] >>= %u\n", opRefOffset / 4, hi);
            }
            break;

        case LongInst_ImmMod:
            {
                printf("LongInst_ImmMod R[%d] %%= %d\n", opRefOffset / 4, imm32c);
            }
            break;
        case LongInst_ImmUmod:
            {
                printf("LongInst_ImmUmod R[%d] %%= %u\n", opRefOffset / 4, hi);
            }
            break;

        case LongInst_SetImm8:
            {
                printf("LongInst_SetImm8 R[%d] = %u\n", opRefOffset / 4, hi);
                assert(hi <= UINT8_MAX);
            }
            break;
        case LongInst_SetImm32:
            {
                printf("LongInst_SetImm32 R[%d] = %u\n", opRefOffset / 4, hi);
            }
            break;
        case LongInst_SetHighImm32:
            {
                printf("LongInst_SetImm32High R[%d] |= (%u << 32)\n", opRefOffset / 4, hi);
            }
            break;
        case LongInst_ImmEq:
            {
                printf("LongInst_ImmEq R[%d] == %d", opRefOffset / 4, imm32c);
            }
            break;
        case LongInst_ImmNeq:
            {
                printf("LongInst_ImmNeq R[%d] != %d", opRefOffset / 4, imm32c);
            }
            break;

        case LongInst_ImmUlt:
            {
                printf("LongInst_ImmUlt R[%d] < %u\n", opRefOffset / 4, hi);
            }
            break;
        case LongInst_ImmUgt:
            {
                printf("LongInst_ImmUlt R[%d] > %u\n", opRefOffset / 4, hi);
            }
            break;
        case LongInst_ImmUle:
            {
                printf("LongInst_ImmUle R[%d] <= %u\n", opRefOffset / 4, hi);
            }
            break;
        case LongInst_ImmUge:
            {
                printf("LongInst_ImmUge R[%d] >= %u\n", opRefOffset / 4, hi);
            }
            break;

        case LongInst_ImmLt:
            {
                printf("LongInst_ImmLt R[%d] < %d\n", opRefOffset / 4, imm32c);
            }
            break;
        case LongInst_ImmGt:
            {
                printf("LongInst_ImmGt R[%d] > %d\n", opRefOffset / 4, imm32c);
            }
            break;
        case LongInst_ImmLe:
            {
                printf("LongInst_ImmLe R[%d] <= %d\n", opRefOffset / 4, imm32c);
            }
            break;
        case LongInst_ImmGe:
            {
                printf("LongInst_ImmGe R[%d] >= %d\n", opRefOffset / 4, imm32c);
            }
            break;

        case LongInst_Add:
            {
                printf("LongInst_Add R[%d] += R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_Sub:
            {
                printf("LongInst_Sub R[%d] -= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_Mul:
            {
                printf("LongInst_Mul R[%d] *= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_Div:
            {
                printf("LongInst_Div R[%d] /= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_Udiv:
            {
                printf("LongInst_Udiv R[%d] /= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_And:
            {
                printf("LongInst_And R[%d] &= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_And32:
            {
                printf("LongInst_And32 R[%d] &= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_Or:
            {
                printf("LongInst_Or R[%d] |= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_Xor32:
            {
                printf("LongInst_Xor32 R[%d] ^= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_Xor:
            {
                printf("LongInst_Xor R[%d] ^= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;

        case LongInst_Lsh:
            {
                printf("LongInst_Lsh R[%d] <<= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_Rsh:
            {
                printf("LongInst_Rsh R[%d] >>= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_Mod:
            {
                printf("LongInst_Mod R[%d] %%= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_Umod:
            {
                printf("LongInst_Umod R[%d] %%= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_FGt32 :
            {
                printf("LongInst_FGt32 R[%d] > R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_FGe32 :
            {
                printf("LongInst_FGe32 R[%d] >= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_FEq32 :
            {
                printf("LongInst_FEq32 R[%d] == R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_FNeq32 :
            {
                printf("LongInst_FNeq32 R[%d] != R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_FLt32 :
            {
                printf("LongInst_FLt32 R[%d] < R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_FLe32 :
            {
                printf("LongInst_FLe32 R[%d] <= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_F32ToF64 :
            {
                printf("LongInst_F64To32 R[%d] = cast(double) R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_F32ToI :
            {
                printf("LongInst_FToI32 R[%d] = cast(int) R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_IToF32 :
            {
                printf("LongInst_IToF32 R[%d] = cast(float) R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;

        case LongInst_FAdd32:
            {
                printf("LongInst_FAdd32 R[%d] += R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_FSub32:
            {
                printf("LongInst_FSub32 R[%d] -= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_FMul32:
            {
                printf("LongInst_FMul32 R[%d] *= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_FDiv32:
            {
                printf("LongInst_FDiv32 R[%d] /= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_FMod32:
            {
                printf("LongInst_FMod32 R[%d] %%= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_FEq64 :
            {
                printf("LongInst_FEq64 R[%d] == R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_FNeq64 :
            {
                printf("LongInst_FNeq64 R[%d] != R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_FLt64 :
            {
                printf("LongInst_FLt64 R[%d] < R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_FLe64 :
            {
                printf("LongInst_FLe64 R[%d] <= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_FGt64 :
            {
                printf("LongInst_FGt64 R[%d] > R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_FGe64 :
            {
                printf("LongInst_FGe64 R[%d] >= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;

        case LongInst_F64ToF32 :
            {
                printf("LongInst_F64ToF32 R[%d] = cast(float) R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_F64ToI :
            {
                printf("LongInst_F64ToI R[%d] = cast(int64_t) R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_IToF64 :
            {
                printf("LongInst_IToF64 R[%d] = cast(double) R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;

        case LongInst_FAdd64:
            {
                printf("LongInst_FAdd64 R[%d] == R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_FSub64:
            {
                printf("LongInst_FSub64 R[%d] -= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_FMul64:
            {
                printf("LongInst_FMul64 R[%d] *= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_FDiv64:
            {
                printf("LongInst_FDiv64 R[%d] /= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_FMod64:
            {
                printf("LongInst_FMod64 R[%d] %%= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;

        case LongInst_Assert:
            {
                printf("LongInst_Assert(R[%d])\n", opRefOffset / 4);
            }
            break;
        case LongInst_Eq:
            {
                printf("LongInst_Eq R[%d] == R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;

        case LongInst_Neq:
            {
                printf("LongInst_Neq R[%d] == R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;

        case LongInst_Set:
            {
                printf("LongInst_Set R[%d] = R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;

        case LongInst_Ult:
            {
                printf("LongInst_Ult R[%d] < R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_Ugt:
            {
                printf("LongInst_Ngt R[%d] > R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_Ule:
            {
                printf("LongInst_Ule R[%d] <= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_Uge:
            {
                printf("LongInst_Uge R[%d] >= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;

        case LongInst_Lt:
            {
                printf("LongInst_Neq R[%d] < R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_Gt:
            {
                printf("LongInst_Gt R[%d] > R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_Le:
            {
                printf("LongInst_Le R[%d] <= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_Ge:
            {
                printf("LongInst_Ge R[%d] >= R[%d]\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
#if 0
        case LongInst_PushCatch:
            {
                printf("LongInst_PushCatch\n");
                debug
                {
                    printf("PushCatch is executing\n");
                }
                Catch catch_ = Catch(ip, callDepth);
                catches ~= catch_;
            }
            break;

            case LongInst_PopCatch:
            {
                printf("LongInst_PopCatch\n");
                debug { if (!__ctfe) writeln("Poping a Catch"); }
                catches = catches[0 .. $-1];
            }
            break;

            case LongInst_Throw:
            {
                printf("LongInst_Throw\n");
                uint expP = ((*opRef) & UINT32_MAX);
                auto expTypeIdx = heapPtr->heapData[expP + ClassMetaData.TypeIdIdxOffset];
                auto expValue = BCValue(HeapAddr(expP), BCType(BCTypeEnum_Class, expTypeIdx));
                expValue.vType = BCValueType.Exception;

                cRetval = expValue;
                if (HandleExp())
                    return cRetval;
            }
            break;
#endif
        case LongInst_Jmp:
            {
                printf("LongInst_Jmp :%u\n", hi);
            }
            break;
        case LongInst_JmpNZ:
            {
                printf("LongInst_JmpNZ :%u\n", hi);
            }
            break;
        case LongInst_JmpZ:
            {
                printf("LongInst_JmpZ :%u\n", hi);
            }
            break;
        case LongInst_JmpFalse:
            {
                printf("LongInst_JmpFalse :%u\n", hi);
            }
            break;
        case LongInst_JmpTrue:
            {
                printf("LongInst_JmpTrue\n :%u\n", hi);
            }
            break;

        case LongInst_HeapLoad8:
            {
                printf("LongInst_HeapLoad8 R[%d] = HEAP[%d]\n", lhsOffset / 4, rhsOffset);
            }
            break;
        case LongInst_HeapStore8:
            {
                printf("LongInst_HeapStore8 HEAP[%d] = R[%d]\n", lhsOffset, rhsOffset / 4);
            }
            break;

            case LongInst_HeapLoad16:
            {
                printf("LongInst_HeapLoad16 R[%d] = HEAP[%d]\n", lhsOffset / 4, rhsOffset);

            }
            break;
            case LongInst_HeapStore16:
            {
                printf("LongInst_HeapStore16 HEAP[%d] = R[%d]\n", lhsOffset, rhsOffset / 4);
            }
            break;

            case LongInst_HeapLoad32:
            {
                printf("LongInst_HeapLoad32 R[%d] = HEAP[%d]\n", lhsOffset / 4, rhsOffset);
            }
            break;
        case LongInst_HeapStore32:
            {
                printf("LongInst_HeapStore32 HEAP[%d] = R[%d]\n", lhsOffset, rhsOffset / 4);
            }
            break;

        case LongInst_HeapLoad64:
            {
                 printf("LongInst_HeapLoad64 R[%d] = HEAP[%d]\n", lhsOffset / 4, rhsOffset);
           }
            break;

        case LongInst_HeapStore64:
            {
                printf("LongInst_HeapStore64 HEAP[%d] = R[%d]\n", lhsOffset, rhsOffset / 4);
            }
            break;

        case LongInst_Ret32:
            {
                printf("LongInst_Ret32 R[%d]\n", opRefOffset / 4);
            }
            break;
        case LongInst_RetS32:
            {
                printf("LongInst_RetS32 R[%d]\n", opRefOffset / 4);
            }
            break;
        case LongInst_RetS64:
            {
                printf("LongInst_RetS64 R[%d]\n", opRefOffset / 4);
            }
            break;

        case LongInst_Ret64:
            {
                printf("LongInst_Ret64 R[%d]\n", opRefOffset / 4);
            }
            break;
        case LongInst_RelJmp:
            {
                printf("LongInst_RelJmp : %d\n", (int16_t)(lw >> 16) - 2);
            }
            break;
        case LongInst_PrintValue:
            {
                bool isString = ((lw & UINT16_MAX) >> 8) != 0;
                printf("LongInst_Print%s\n", (isString ? "String" : "Value"));
/*
                if (isString)
                {
                    long offset = *opRef;
                    uint8_t length = heapPtr->heapData[offset];
                    char* string_start = cast(char*)&heapPtr->heapData[offset + 1];
                    printf("Printing string: '%.*s'\n", length, string_start);
                }
                else
                {
                    printf("Addr: %lu, Value %lx\n", (opRef - stackP) * 4, *opRef);
                }
*/
            }
            break;
        case LongInst_Not:
            {
                printf("LongInst_Not R[%d]\n", opRefOffset / 4);
            }
            break;
        case LongInst_Flg:
            {
                printf("LongInst_Flg R[%d]\n", opRefOffset / 4);
            }
            break;

        case LongInst_BuiltinCall:
            {
                printf("LongInst_BuiltinCall\n");
                assert(0);//, "Unsupported right now: BCBuiltin");
            }
#if 0
        case LongInst_Cat:
            {
                printf("LongInst_Cat\n");
                if (*rhs == 0 && *lhsRef == 0)
                {
                    *lhsStackRef = 0;
                }
                else
                {
                    const elemSize = (lw >> 8) & 255;
                    const uint _lhs =  *lhsRef & UINT32_MAX;
                    const uint _rhs =  *rhs & UINT32_MAX;

                    const llbasep = &heapPtr->heapData[_lhs + SliceDescriptor.LengthOffset];
                    const rlbasep = &heapPtr->heapData[_rhs + SliceDescriptor.LengthOffset];

                    const lhs_length = _lhs ? loadu32(llbasep) : 0;
                    const rhs_length = _rhs ? loadu32(rlbasep) : 0;

                    if (const newLength = lhs_length + rhs_length)
                    {
                        // TODO if lhs.capacity bla bla
                        const lhsBase = loadu32(&heapPtr->heapData[_lhs + SliceDescriptor.BaseOffset]);
                        const rhsBase = loadu32(&heapPtr->heapData[_rhs + SliceDescriptor.BaseOffset]);

                        const resultPtr = heapPtr.heapSize;

                        const resultLengthP = resultPtr + SliceDescriptor.LengthOffset;
                        const resultBaseP   = resultPtr + SliceDescriptor.BaseOffset;
                        const resultBase    = resultPtr + SliceDescriptor.Size;

                        const allocSize = (newLength * elemSize) + SliceDescriptor.Size;
                        const heapSize  = heapPtr.heapSize;

                        if(heapSize + allocSize  >= heapPtr.heapMax)
                        {
                            if (heapPtr.heapMax >= 2 ^^ 31)
                                assert(0, "!!! HEAP OVERFLOW !!!");
                            else
                            {
                                // we will now resize the heap to 8 times its former size
                                const newHeapSize =
                                    ((allocSize < heapPtr.heapMax * 4) ?
                                    heapPtr.heapMax * 8 :
                                    align4(cast(uint)(heapPtr.heapMax + allocSize)) * 4);

                                auto newHeap = new ubyte[](newHeapSize);
                                newHeap[0 .. heapSize] = heapPtr->heapData[0 .. heapSize];
                                if (!__ctfe) heapPtr->heapData.destroy();

                                heapPtr->heapData = newHeap;
                                heapPtr.heapMax = newHeapSize;
                            }
                        }

                        heapPtr.heapSize += allocSize;

                        const scaled_lhs_length = (lhs_length * elemSize);
                        const scaled_rhs_length = (rhs_length * elemSize);
                        const result_lhs_end    = resultBase + scaled_lhs_length;

                        storeu32(&heapPtr->heapData[resultBaseP],  resultBase);
                        storeu32(&heapPtr->heapData[resultLengthP], newLength);

                        heapPtr->heapData[resultBase .. result_lhs_end] =
                            heapPtr->heapData[lhsBase .. lhsBase + scaled_lhs_length];

                        heapPtr->heapData[result_lhs_end ..  result_lhs_end + scaled_rhs_length] =
                            heapPtr->heapData[rhsBase .. rhsBase + scaled_rhs_length];

                        *lhsStackRef = resultPtr;
                    }
                }
            }
            break;
#endif
        case LongInst_Call:
            {
                printf("LongInst_Call R[%d] = ?\n", (opRefOffset / 4));
            }
            break;

        case LongInst_Alloc:
            {
                printf("LongInst_Alloc R[%d] = ALLOC(R[%d])\n", lhsOffset / 4, rhsOffset / 4);
            }
            break;
        case LongInst_MemCpy:
            {
                //TODO verify this printout
                printf("LongInst_MemCpy(R[%d], R[%d], R[%d])\n", lhsOffset / 4 , rhsOffset / 4 , opRefOffset / 4);
            }
            break;

        case LongInst_Comment:
            {
                printf("LongInst_Comment [length:%d]\n", align4(hi) / 4);
                int k = 12;
L_LongInst_CommentP:
                k = 2;
            }
            break;
#if 0
        case LongInst_StrEq:
            {
                printf("LongInst_Comment\n");
                cond = false;

                uint32_t _lhs = cast(uint)*lhsRef;
                uint32_t _rhs = cast(uint)*rhs;

                assert(_lhs && _rhs, "trying to deref nullPointers");
                if (_lhs == _rhs)
                {
                    cond = true;
                }
                else
                {
                    immutable lhUlength = heapPtr->heapData[_lhs + SliceDescriptor.LengthOffset];
                    immutable rhUlength = heapPtr->heapData[_rhs + SliceDescriptor.LengthOffset];
                    if (lhUlength == rhUlength)
                    {
                        immutable lhsBase = heapPtr->heapData[_lhs + SliceDescriptor.BaseOffset];
                        immutable rhsBase = heapPtr->heapData[_rhs + SliceDescriptor.BaseOffset];
                        cond = true;
                        foreach (i; 0 .. lhUlength)
                        {
                            if (heapPtr->heapData[rhsBase + i] != heapPtr->heapData[lhsBase + i])
                            {
                                cond = false;
                                break;
                            }
                        }
                    }
                }
            }
            break;
#endif
        case LongInst_File :
            {
                printf("LongInst_File \n");
                goto L_LongInst_CommentP;
            }
        case LongInst_Line :
            {
                printf("LongInst_Line \n");
                uint32_t breakingOn;
                uint32_t line = hi;
#if 0
                foreach(bl;breakLines)
                {
                    if (line == bl)
                    {
                        debug
                        if (!__ctfe)
                        {
                            import std.stdio;
                            writeln("breaking at: ", ip-2);

                        }
                        paused = true;
                    }
                    break;
                }
#endif
            }
            break;
        }
    }
}


BCValue BCGen_interpret(BCGen* self, uint32_t fnIdx, BCValue* args, uint32_t n_args, BCHeap* heapPtr)
{
    assert(self->finalized);

    BCInterpreter state = {0};
    state.ip = 4;
    int64_t* stackP = state.stack;

    uint* codeP = self->byteCodeArray;
    if (self->byteCodeCount > ARRAY_SIZE(self->byteCodeArray))
            codeP = self->byteCodeArrayExtra;

    int argOffset = 1;
    for(int i = 0; i < n_args;i++)
    {
        BCValue* arg = args + i;
        assert(arg->vType == BCValueType_Immediate);

        switch (arg->type.type)
        {
            case BCTypeEnum_i32:
            case BCTypeEnum_i16:
            case BCTypeEnum_i8:
            {
                stackP[argOffset++] = cast(int32_t)arg->imm32.imm32;
            }
            break;

            case BCTypeEnum_u32:
            case BCTypeEnum_f23:
            case BCTypeEnum_c8:
            case BCTypeEnum_u16:
            case BCTypeEnum_u8:
            {
                stackP[argOffset++] = cast(uint32_t)arg->imm32.imm32;
            }
            break;

        case BCTypeEnum_i64:
            {
                stackP[argOffset++] = arg->imm64.imm64;
            }
            break;

        case BCTypeEnum_u64:
        case BCTypeEnum_f52:
        {
            stackP[argOffset++] = arg->imm64.imm64;
        }
        break;

        case BCTypeEnum_Struct:
        case BCTypeEnum_Class:
        case BCTypeEnum_string8:
        case BCTypeEnum_Array:
            {
                // This might need to be removed again?
                stackP[argOffset++] = arg->heapAddr.addr;
            }
            break;
        default:
            //return -1;
                   assert(0);//, "unsupported Type " ~ enumToString(arg.type.type));
        }
    }

    while (true)
    {
        const uint32_t lw = (codeP)[state.ip];
        const uint32_t hi = (codeP)[state.ip + 1];
        const int32_t imm32c = *(cast(int32_t*)&((codeP)[state.ip + 1]));
        state.ip += 2;

        // consider splitting the stackPointer in stackHigh and stackLow

        const uint32_t opRefOffset = (lw >> 16) & 0xFFFF;
        const uint32_t lhsOffset   = hi & 0xFFFF;
        const uint32_t rhsOffset   = (hi >> 16) & 0xFFFF;

        int64_t* lhsRef = (&stackP[(lhsOffset / 4)]);
        int64_t* rhs = (&stackP[(rhsOffset / 4)]);
        int64_t* lhsStackRef = (&stackP[(opRefOffset / 4)]);
        int64_t* opRef = &stackP[(opRefOffset / 4)];

        float flhs;
        float frhs;

        double drhs;
        double dlhs;

        if ((lw & INSTMASK) >= FLT32_BEGIN && (lw & INSTMASK) <= FLT32_END)
        {
            uint32_t _lhs = *lhsRef & UINT32_MAX;
            flhs = *(float*)&_lhs;
            uint32_t _rhs = *rhs & UINT32_MAX;
            frhs = *(float*)&_rhs;
        }
        else if ((lw & INSTMASK) >= FLT64_BEGIN && (lw & INSTMASK) <= FLT64_END)
        {
            dlhs = *(double*)lhsRef;
            drhs = *(double*)rhs;
        }

        bool cond;

        if (!lw)
        { // Skip NOPS
            continue;
        }

        switch (cast(LongInst)(lw & INSTMASK))
        {
        case LongInst_ImmAdd:
            {
                (*lhsStackRef) += imm32c;
            }
            break;

        case LongInst_ImmSub:
            {
                (*lhsStackRef) -= imm32c;
            }
            break;

        case LongInst_ImmMul:
            {
                (*lhsStackRef) *= imm32c;
            }
            break;

        case LongInst_ImmDiv:
            {
                printf("LongInst_ImmDiv\n");
                (*lhsStackRef) /= imm32c;
            }
            break;

        case LongInst_ImmUdiv:
            {
                (*cast(uint64_t*)lhsStackRef) /= imm32c;
            }
            break;

        case LongInst_ImmAnd:
            {
                (*lhsStackRef) &= hi;
            }
            break;
        case LongInst_ImmAnd32:
            {
                *lhsStackRef = (cast(uint32_t)*lhsStackRef) & hi;
            }
            break;
        case LongInst_ImmOr:
            {
                (*lhsStackRef) |= hi;
            }
            break;
        case LongInst_ImmXor:
            {
                (*lhsStackRef) ^= hi;
            }
            break;
        case LongInst_ImmXor32:
            {
                *lhsStackRef = (cast(uint32_t)*lhsStackRef) ^ hi;
            }
            break;

        case LongInst_ImmLsh:
            {
                (*lhsStackRef) <<= hi;
            }
            break;
        case LongInst_ImmRsh:
            {
                (*lhsStackRef) >>= hi;
            }
            break;

        case LongInst_ImmMod:
            {
                (*lhsStackRef) %= imm32c;
            }
            break;
        case LongInst_ImmUmod:
            {
                (*cast(uint64_t*)lhsStackRef) %= imm32c;
            }
            break;

        case LongInst_SetImm8:
            {
                (*lhsStackRef) = hi;
                assert(hi <= UINT8_MAX);
            }
            break;
        case LongInst_SetImm32:
            {
                (*lhsStackRef) = hi;
            }
            break;
        case LongInst_SetHighImm32:
            {
                uint64_t hi64 = hi;
                *lhsStackRef = (*lhsStackRef & 0x00000000FFFFFFFF) | ((hi64) << 32UL);
            }
            break;
        case LongInst_ImmEq:
            {
                if ((*lhsStackRef) == imm32c)
                {
                    cond = true;
                }
                else
                {
                    cond = false;
                }
            }
            break;
        case LongInst_ImmNeq:
            {
                if ((*lhsStackRef) != imm32c)
                {
                    cond = true;
                }
                else
                {
                    cond = false;
                }
            }
            break;

        case LongInst_ImmUlt:
            {
                if (((int64_t)(*lhsStackRef)) < cast(uint)hi)
                {
                    cond = true;
                }
                else
                {
                    cond = false;
                }
            }
            break;
        case LongInst_ImmUgt:
            {
                if (((uint64_t)(*lhsStackRef)) > cast(uint)hi)
                {
                    cond = true;
                }
                else
                {
                    cond = false;
                }
            }
            break;
        case LongInst_ImmUle:
            {
                if (((uint64_t)(*lhsStackRef)) <= cast(uint)hi)
                {
                    cond = true;
                }
                else
                {
                    cond = false;
                }
            }
            break;
        case LongInst_ImmUge:
            {
                if (((uint64_t)(*lhsStackRef)) >= cast(uint)hi)
                {
                    cond = true;
                }
                else
                {
                    cond = false;
                }
            }
            break;

        case LongInst_ImmLt:
            {
                if ((*lhsStackRef) < imm32c)
                {
                    cond = true;
                }
                else
                {
                    cond = false;
                }
            }
            break;
        case LongInst_ImmGt:
            {
                if ((*lhsStackRef) > imm32c)
                {
                    cond = true;
                }
                else
                {
                    cond = false;
                }
            }
            break;
        case LongInst_ImmLe:
            {
                if ((*lhsStackRef) <= imm32c)
                {
                    cond = true;
                }
                else
                {
                    cond = false;
                }
            }
            break;
        case LongInst_ImmGe:
            {
                if ((*lhsStackRef) >= imm32c)
                {
                    cond = true;
                }
                else
                {
                    cond = false;
                }
            }
            break;

        case LongInst_Add:
            {
                (*lhsRef) += *rhs;
            }
            break;
        case LongInst_Sub:
            {
                (*lhsRef) -= *rhs;
            }
            break;
        case LongInst_Mul:
            {
                (*lhsRef) *= *rhs;
            }
            break;
        case LongInst_Div:
            {
                (*lhsRef) /= *rhs;
            }
            break;
        case LongInst_Udiv:
            {
                (*cast(uint64_t*)lhsRef) /= (*cast(uint64_t*)rhs);
            }
            break;
        case LongInst_And:
            {
                (*lhsRef) &= *rhs;
            }
            break;
        case LongInst_And32:
            {
               (*lhsRef) = (cast(uint) *lhsRef) & (cast(uint)*rhs);
            }
            break;
        case LongInst_Or:
            {
                (*lhsRef) |= *rhs;
            }
            break;
        case LongInst_Xor32:
            {
                (*lhsRef) = (cast(uint) *lhsRef) ^ (cast(uint)*rhs);
            }
            break;
        case LongInst_Xor:
            {
                (*lhsRef) ^= *rhs;
            }
            break;

        case LongInst_Lsh:
            {
                (*lhsRef) <<= *rhs;
            }
            break;
        case LongInst_Rsh:
            {
                (*lhsRef) >>= *rhs;
            }
            break;
        case LongInst_Mod:
            {
                (*lhsRef) %= *rhs;
            }
            break;
        case LongInst_Umod:
            {
                (*cast(uint64_t*)lhsRef) %= (*cast(uint64_t*)rhs);
            }
            break;
        case LongInst_FGt32 :
            {
                cond = flhs > frhs;
            }
            break;
        case LongInst_FGe32 :
            {
                cond = flhs >= frhs;
            }
            break;
        case LongInst_FEq32 :
            {
                cond = flhs == frhs;
            }
            break;
        case LongInst_FNeq32 :
            {
                cond = flhs != frhs;
            }
            break;
        case LongInst_FLt32 :
            {
                cond = flhs < frhs;
            }
            break;
        case LongInst_FLe32 :
            {
                cond = flhs <= frhs;
            }
            break;
        case LongInst_F32ToF64 :
            {
                uint rhs32 = (*rhs & UINT32_MAX);
                float frhs = *cast(float*)&rhs32;
                double flhs = frhs;
                *lhsRef = *cast(long*)&flhs;
            }
            break;
        case LongInst_F32ToI :
            {
                uint32_t rhs32 = (*rhs & UINT32_MAX);
                float frhs = *cast(float*)&rhs32;
                *lhsRef = cast(int32_t)frhs;
            }
            break;
        case LongInst_IToF32 :
            {
                float frhs = *rhs;
                uint _lhs = *cast(uint*)&frhs;
                *lhsRef = _lhs;
            }
            break;

        case LongInst_FAdd32:
            {
                flhs += frhs;

                *lhsRef = *(uint32_t*)&flhs;
            }
            break;
        case LongInst_FSub32:
            {
                flhs -= frhs;
                *lhsRef = *(uint32_t*)&flhs;
            }
            break;
        case LongInst_FMul32:
            {
                flhs *= frhs;
                *lhsRef = *(uint32_t*)&flhs;
            }
            break;
        case LongInst_FDiv32:
            {
                flhs /= frhs;
                *lhsRef = *(uint32_t*)&flhs;
            }
            break;
        case LongInst_FMod32:
            {
                flhs = fmodf(flhs, frhs);
                *lhsRef = *(uint32_t*)&flhs;
            }
            break;
        case LongInst_FEq64 :
            {
                cond = dlhs == drhs;
            }
            break;
        case LongInst_FNeq64 :
            {
                cond = dlhs < drhs;
            }
            break;
        case LongInst_FLt64 :
            {
                cond = dlhs < drhs;
            }
            break;
        case LongInst_FLe64 :
            {
                cond = dlhs <= drhs;
            }
            break;
        case LongInst_FGt64 :
            {
                cond = dlhs > drhs;
            }
            break;
        case LongInst_FGe64 :
            {
                cond = dlhs >= drhs;
            }
            break;

        case LongInst_F64ToF32 :
            {
                double drhs_ = *cast(double*)rhs;
                float flhs_ = drhs_;
                *lhsRef = *(uint32_t*)&flhs_;
            }
            break;
        case LongInst_F64ToI :
            {
                float drhs_ = *(double*)rhs;
                *lhsRef = (int64_t)drhs_;
            }
            break;
        case LongInst_IToF64 :
            {
                double drhs_ = (double)*rhs;
                *lhsRef = *(int64_t*)&drhs_;
            }
            break;

        case LongInst_FAdd64:
            {
                dlhs += drhs;
                *lhsRef = *(uint64_t*)&dlhs;
            }
            break;
        case LongInst_FSub64:
            {
                dlhs -= drhs;
                *lhsRef = *(uint64_t*)&dlhs;
            }
            break;
        case LongInst_FMul64:
            {
                dlhs *= drhs;
                *lhsRef = *(uint64_t*)&dlhs;
            }
            break;
        case LongInst_FDiv64:
            {
                dlhs /= drhs;

                *(cast(uint64_t*)lhsRef) = *cast(uint64_t*)&dlhs;
            }
            break;
        case LongInst_FMod64:
            {
                dlhs = fmod(dlhs, drhs);

                *(cast(uint64_t*)lhsRef) = *cast(uint64_t*)&dlhs;
            }
            break;

        case LongInst_Assert:
            {
                if (*opRef == 0)
                {
                    BCValue retval = imm32(hi);
                    retval.vType = BCValueType_Error;
                    return retval;
                }
            }
            break;
        case LongInst_Eq:
            {
                if ((*lhsRef) == *rhs)
                {
                    cond = true;
                }
                else
                {
                    cond = false;
                }

            }
            break;

        case LongInst_Neq:
            {
                if ((*lhsRef) != *rhs)
                {
                    cond = true;
                }
                else
                {
                    cond = false;
                }
            }
            break;

        case LongInst_Set:
            {
                (*lhsRef) = *rhs;
            }
            break;

        case LongInst_Ult:
            {
                if (((uint64_t)(*lhsRef)) < ((uint64_t)*rhs))
                {
                    cond = true;
                }
                else
                {
                    cond = false;
                }
            }
            break;
        case LongInst_Ugt:
            {
                if ((uint64_t)(*lhsRef) > (uint64_t)*rhs)
                {
                    cond = true;
                }
                else
                {
                    cond = false;
                }
            }
            break;
        case LongInst_Ule:
            {
                if (((uint64_t)(*lhsRef)) <= ((uint64_t)*rhs))
                {
                    cond = true;
                }
                else
                {
                    cond = false;
                }
            }
            break;
        case LongInst_Uge:
            {
                if (((uint64_t)(*lhsRef)) >= ((uint64_t)*rhs))
                {
                    cond = true;
                }
                else
                {
                    cond = false;
                }

            }
            break;

        case LongInst_Lt:
            {
                if ((*lhsRef) < *rhs)
                {
                    cond = true;
                }
                else
                {
                    cond = false;
                }
            }
            break;
        case LongInst_Gt:
            {
                if ((*lhsRef) > *rhs)
                {
                    cond = true;
                }
                else
                {
                    cond = false;
                }
            }
            break;
        case LongInst_Le:
            {
                if ((*lhsRef) <= *rhs)
                {
                    cond = true;
                }
                else
                {
                    cond = false;
                }

            }
            break;
        case LongInst_Ge:
            {
                if ((*lhsRef) >= *rhs)
                {
                    cond = true;
                }
                else
                {
                    cond = false;
                }

            }
            break;
#if 0
        case LongInst_PushCatch:
            {
                debug
                {
                    printf("PushCatch is executing\n");
                }
                Catch catch_ = Catch(ip, callDepth);
                catches ~= catch_;
            }
            break;

            case LongInst_PopCatch:
            {
                debug { if (!__ctfe) writeln("Poping a Catch"); }
                catches = catches[0 .. $-1];
            }
            break;

            case LongInst_Throw:
            {
                uint expP = ((*opRef) & UINT32_MAX);
                auto expTypeIdx = heapPtr->heapData[expP + ClassMetaData.TypeIdIdxOffset];
                auto expValue = BCValue(HeapAddr(expP), BCType(BCTypeEnum_Class, expTypeIdx));
                expValue.vType = BCValueType.Exception;

                cRetval = expValue;
                if (HandleExp())
                    return cRetval;
            }
            break;
#endif
        case LongInst_Jmp:
            {
                state.ip = hi;
            }
            break;
        case LongInst_JmpNZ:
            {
                if ((*lhsStackRef) != 0)
                {
                    state.ip = hi;
                }
            }
            break;
        case LongInst_JmpZ:
            {
                if ((*lhsStackRef) == 0)
                {
                    state.ip = hi;
                }
            }
            break;
        case LongInst_JmpFalse:
            {
                if (!cond)
                {
                    state.ip = hi;
                }
            }
            break;
        case LongInst_JmpTrue:
            {
                if (cond)
                {
                    state.ip = hi;
                }
            }
            break;

        case LongInst_HeapLoad8:
            {
                assert(*rhs);//, "trying to deref null pointer inLine: " ~ itos(lastLine));
                (*lhsRef) = heapPtr->heapData[*rhs];
            }
            break;
        case LongInst_HeapStore8:
            {
                assert(*lhsRef);//, "trying to deref null pointer SP[" ~ itos(cast(int)((lhsRef - &stackP[0])*4)) ~ "] at : &" ~ itos (ip - 2));
                heapPtr->heapData[*lhsRef] = ((*rhs) & 0xFF);
            }
            break;

            case LongInst_HeapLoad16:
            {
                assert(*rhs);//, "trying to deref null pointer inLine: " ~ itos(lastLine));
                const long addr = *lhsRef;
                (*lhsRef) =  heapPtr->heapData[addr]
                          | (heapPtr->heapData[addr + 1] << 8);

            }
            break;
            case LongInst_HeapStore16:
            {
                assert(*lhsRef);//, "trying to deref null pointer SP[" ~ itos(cast(int)((lhsRef - &stackP[0])*4)) ~ "] at : &" ~ itos (ip - 2));
                const uint32_t addr = *lhsRef;
                heapPtr->heapData[addr    ] = ((*rhs     ) & 0xFF);
                heapPtr->heapData[addr + 1] = ((*rhs >> 8) & 0xFF);
            }
            break;

            case LongInst_HeapLoad32:
            {
                assert(*rhs); //, "trying to deref null pointer inLine: " ~ itos(lastLine));
                (*lhsRef) = loadu32(heapPtr->heapData + *rhs);
            }
            break;
        case LongInst_HeapStore32:
            {
                assert(*lhsRef);//, "trying to deref null pointer SP[" ~ itos(cast(int)((lhsRef - &stackP[0])*4)) ~ "] at : &" ~ itos (ip - 2));
                //(*(heapPtr->heapData.ptr + *lhsRef)) = (*rhs) & 0xFF_FF_FF_FF;
                storeu32((&heapPtr->heapData[*lhsRef]),  (*rhs) & UINT32_MAX);
            }
            break;

        case LongInst_HeapLoad64:
            {
                assert(*rhs);//, "trying to deref null pointer ");
                const uint32_t addr = *rhs;
                uint64_t value = loadu32(&heapPtr->heapData[addr + 4]);
                value <<= 32UL;
                value |= loadu32(&heapPtr->heapData[addr]);
                
                (*lhsRef) = value;
            }
            break;

        case LongInst_HeapStore64:
            {
                assert(*lhsRef);//, "trying to deref null pointer SP[" ~ itos(cast(int)(lhsRef - &stackP[0])*4) ~ "] at : &" ~ itos (ip - 2));
                const int64_t heapOffset = *lhsRef;
                assert(heapOffset < heapPtr->heapSize);//, "Store out of range at ip: &" ~ itos(ip - 2) ~ " atLine: " ~ itos(lastLine));
                const uint8_t* basePtr = (heapPtr->heapData + *lhsRef);
                const long addr = *lhsRef;
                const long value = *rhs;

                storeu32(&heapPtr->heapData[addr],     value & UINT32_MAX);
                storeu32(&heapPtr->heapData[addr + 4], cast(uint32_t)(value >> 32));
            }
            break;

        case LongInst_Ret32:
            {
                state.cRetval = imm32(*opRef & UINT32_MAX);
                if (BCInterpreter_Return(&state)) return state.cRetval;
            }
            break;
        case LongInst_RetS32:
            {
                state.cRetval = imm32_(*opRef & UINT32_MAX, true);
                if (BCInterpreter_Return(&state)) return state.cRetval;
            }
            break;
        case LongInst_RetS64:
            {
                state.cRetval = imm64_(*opRef, true);
                if (BCInterpreter_Return(&state)) return state.cRetval;
            }
            break;

        case LongInst_Ret64:
            {
                state.cRetval = imm64_(*opRef, false);
                if (BCInterpreter_Return(&state)) return state.cRetval;
            }
            break;
        case LongInst_RelJmp:
            {
                state.ip += (cast(short)(lw >> 16)) - 2;
            }
            break;
        case LongInst_PrintValue:
            {
                if ((lw & UINT16_MAX) >> 8)
                {
                    long offset = *opRef;
                    uint8_t length = heapPtr->heapData[offset];
                    char* string_start = cast(char*)&heapPtr->heapData[offset + 1];
                }
                else
                {
                    //TODO readd print!
                }
            }
            break;
        case LongInst_Not:
            {
                (*opRef) = ~(*opRef);
            }
            break;
        case LongInst_Flg:
            {
                (*opRef) = cond;
            }
            break;

        case LongInst_BuiltinCall:
            {
                assert(0);//, "Unsupported right now: BCBuiltin");
            }
#if 0
        case LongInst_Cat:
            {
                if (*rhs == 0 && *lhsRef == 0)
                {
                    *lhsStackRef = 0;
                }
                else
                {
                    const elemSize = (lw >> 8) & 255;
                    const uint _lhs =  *lhsRef & UINT32_MAX;
                    const uint _rhs =  *rhs & UINT32_MAX;

                    const llbasep = &heapPtr->heapData[_lhs + SliceDescriptor.LengthOffset];
                    const rlbasep = &heapPtr->heapData[_rhs + SliceDescriptor.LengthOffset];

                    const lhs_length = _lhs ? loadu32(llbasep) : 0;
                    const rhs_length = _rhs ? loadu32(rlbasep) : 0;

                    if (const newLength = lhs_length + rhs_length)
                    {
                        // TODO if lhs.capacity bla bla
                        const lhsBase = loadu32(&heapPtr->heapData[_lhs + SliceDescriptor.BaseOffset]);
                        const rhsBase = loadu32(&heapPtr->heapData[_rhs + SliceDescriptor.BaseOffset]);

                        const resultPtr = heapPtr.heapSize;

                        const resultLengthP = resultPtr + SliceDescriptor.LengthOffset;
                        const resultBaseP   = resultPtr + SliceDescriptor.BaseOffset;
                        const resultBase    = resultPtr + SliceDescriptor.Size;

                        const allocSize = (newLength * elemSize) + SliceDescriptor.Size;
                        const heapSize  = heapPtr.heapSize;

                        if(heapSize + allocSize  >= heapPtr.heapMax)
                        {
                            if (heapPtr.heapMax >= 2 ^^ 31)
                                assert(0, "!!! HEAP OVERFLOW !!!");
                            else
                            {
                                // we will now resize the heap to 8 times its former size
                                const newHeapSize =
                                    ((allocSize < heapPtr.heapMax * 4) ?
                                    heapPtr.heapMax * 8 :
                                    align4(cast(uint)(heapPtr.heapMax + allocSize)) * 4);

                                auto newHeap = new ubyte[](newHeapSize);
                                newHeap[0 .. heapSize] = heapPtr->heapData[0 .. heapSize];
                                if (!__ctfe) heapPtr->heapData.destroy();

                                heapPtr->heapData = newHeap;
                                heapPtr.heapMax = newHeapSize;
                            }
                        }

                        heapPtr.heapSize += allocSize;

                        const scaled_lhs_length = (lhs_length * elemSize);
                        const scaled_rhs_length = (rhs_length * elemSize);
                        const result_lhs_end    = resultBase + scaled_lhs_length;

                        storeu32(&heapPtr->heapData[resultBaseP],  resultBase);
                        storeu32(&heapPtr->heapData[resultLengthP], newLength);

                        heapPtr->heapData[resultBase .. result_lhs_end] =
                            heapPtr->heapData[lhsBase .. lhsBase + scaled_lhs_length];

                        heapPtr->heapData[result_lhs_end ..  result_lhs_end + scaled_rhs_length] =
                            heapPtr->heapData[rhsBase .. rhsBase + scaled_rhs_length];

                        *lhsStackRef = resultPtr;
                    }
                }
            }
            break;
#endif
        case LongInst_Call:
            {
                assert(self->functions);//, "When calling functions you need functions to call");
                RetainedCall call = self->calls[(*rhs & UINT32_MAX) - 1];
                ReturnAddr returnAddr = {state.ip, state.fnIdx, call.callerSp.addr, lhsRef};

                uint32_t fn = ((call.fn.vType == BCValueType_Immediate) ?
                    call.fn.imm32.imm32 :
                    stackP[call.fn.stackAddr.addr / 4]
                ) & UINT32_MAX;

                state.fnIdx = fn - 1;

                uint32_t stackOffsetCall = state.stackTop + call.callerSp.addr;
                long* newStack = stackP + (call.callerSp.addr / 4);

                if (fn == skipFn)
                    continue;

                //foreach(size_t i,ref arg;call.args)
                for(int i = 0; i < call.n_args; i++)
                {
                    const BCValue* arg = call.args + i;
                    const int argOffset_ = (i * 1) + 1;
                    if(BCValue_isStackValueOrParameter(arg))
                    {
                        newStack[argOffset_] = stackP[arg->stackAddr.addr / 4];
                    }
                    else if (arg->vType == BCValueType_Immediate)
                    {
                        newStack[argOffset_] = arg->imm64.imm64;
                    }
                    else
                    {
                        assert(0);//, "Argument " ~ itos(cast(int)i) ~" ValueType unhandeled: " ~ enumToString(arg.vType));
                    }
                }

                if (state.callDepth++ == max_call_depth)
                {
                    BCValue bailoutValue;
                    bailoutValue.vType = BCValueType_Bailout;
                    bailoutValue.imm32.imm32 = 2000;
                    return bailoutValue;
                }

                {
                    state.returnAddrs[state.n_return_addrs++] = returnAddr;
                    state.stackTop = state.stackTop + (call.callerSp.addr / 4);
                    BCFunction* f =  self->functions + state.fnIdx;
                    state.ip = f->bytecode_start;
                }
            }
            break;

        case LongInst_Alloc:
            {
                const uint32_t allocSize = *rhs;
                const uint32_t heapSize = heapPtr->heapSize;

                if(heapSize + allocSize  >= heapPtr->heapMax)
                {
                    if (heapPtr->heapMax >= (1 << 31))
                        assert(0);//, "!!! HEAP OVERFLOW !!!");
                    else
                    {
                        // we will now resize the heap to 4 times its former size
                        const uint32_t newHeapSize =
                            ((allocSize < heapPtr->heapMax * 2) ?
                            heapPtr->heapMax * 4 :
                            align4(cast(uint32_t)(heapPtr->heapMax + allocSize)) * 2);

                        heapPtr->heapData = realloc(heapPtr->heapData, newHeapSize);
                        heapPtr->heapMax = newHeapSize;
                    }
                }

                *lhsRef = heapSize;
                heapPtr->heapSize += allocSize;
            }
            break;
        case LongInst_MemCpy:
            {
                uint32_t cpySize = cast(uint32_t) *opRef;
                uint32_t cpySrc = cast(uint32_t) *rhs;
                uint32_t cpyDst = cast(uint32_t) *lhsRef;
                
                if (cpySrc != cpyDst && cpySize != 0)
                {
                    // assert(cpySize, "cpySize == 0");
                    assert(cpySrc);//, "cpySrc == 0" ~ " inLine: " ~ itos(lastLine));

                    assert(cpyDst);//, "cpyDst == 0" ~ " inLine: " ~ itos(lastLine));

                    assert(cpyDst >= cpySrc + cpySize || cpyDst + cpySize <= cpySrc);
                    //, "Overlapping MemCpy is not supported --- src: " ~ itos(cpySrc)
                    //    ~ " dst: " ~ itos(cpyDst) ~ " size: " ~ itos(cpySize));
                    
                    uint8_t* heapData = heapPtr->heapData;
                    
                    uint8_t* cpyDstP = heapPtr->heapData + cpyDst;
                    uint8_t* cpySrcP = heapPtr->heapData + cpySrc;

                    memcpy(cpyDstP, cpySrcP, cpySize * sizeof(*heapPtr->heapData));
                    //heapPtr->heapData[cpyDst .. cpyDst + cpySize] = heapPtr->heapData[cpySrc .. cpySrc + cpySize];
                }
            }
            break;

        case LongInst_Comment:
            {
L_LongInst_Comment:
                state.ip += align4(hi) / 4;
            }
            break;
#if 0
        case LongInst_StrEq:
            {
                cond = false;

                uint32_t _lhs = cast(uint)*lhsRef;
                uint32_t _rhs = cast(uint)*rhs;

                assert(_lhs && _rhs, "trying to deref nullPointers");
                if (_lhs == _rhs)
                {
                    cond = true;
                }
                else
                {
                    immutable lhUlength = heapPtr->heapData[_lhs + SliceDescriptor.LengthOffset];
                    immutable rhUlength = heapPtr->heapData[_rhs + SliceDescriptor.LengthOffset];
                    if (lhUlength == rhUlength)
                    {
                        immutable lhsBase = heapPtr->heapData[_lhs + SliceDescriptor.BaseOffset];
                        immutable rhsBase = heapPtr->heapData[_rhs + SliceDescriptor.BaseOffset];
                        cond = true;
                        foreach (i; 0 .. lhUlength)
                        {
                            if (heapPtr->heapData[rhsBase + i] != heapPtr->heapData[lhsBase + i])
                            {
                                cond = false;
                                break;
                            }
                        }
                    }
                }
            }
            break;
#endif
        case LongInst_File :
            {
                goto L_LongInst_Comment;
            }
        case LongInst_Line :
            {
                uint32_t breakingOn;
                uint32_t line = hi;
                state.lastLine = line;
#if 0
                foreach(bl;breakLines)
                {
                    if (line == bl)
                    {
                        debug
                        if (!__ctfe)
                        {
                            import std.stdio;
                            writeln("breaking at: ", ip-2);

                        }
                        paused = true;
                    }
                    break;
                }
#endif
            }
            break;
        }
    }
    BCValue bailoutValue;
Lbailout :
    bailoutValue.vType = BCValueType_Bailout;
    
    return bailoutValue;
}

static void inline BCGen_emit2_at(BCGen* self, uint32_t low, uint32_t high, uint32_t atIp)
{
    uint* codeP;
    if (atIp < ARRAY_SIZE(self->byteCodeArray))
    {
        codeP = self->byteCodeArray + atIp;
    }
    else
    {
        atIp -= ARRAY_SIZE(self->byteCodeArray);
        if (atIp < self->byteCodeExtraCapacity)
        {
            codeP = self->byteCodeArrayExtra + atIp;
        }
        else
        {
            // Code area should be expanded
            assert (0);
        }
    }
    
    codeP[0] = low;
    codeP[1] = high;
}

static void inline BCGen_emit2(BCGen* self, uint32_t low, uint32_t high)
{
    uint32_t atIp = self->ip;
    self->ip += 2;

    BCGen_emit2_at(self, low, high, atIp);
}

static void inline BCGen_emitLongInstSA(BCGen* self, const LongInst i, const StackAddr stackAddrLhs, const BCAddr targetAddr)
{
    BCGen_emit2(self
              , i | stackAddrLhs.addr << 16
              , targetAddr.addr);
    
}

static inline void BCGen_emitLongInstSS(BCGen* self, const LongInst i, const StackAddr stackAddrLhs,
    const StackAddr stackAddrRhs)
{
    BCGen_emit2(self
              , i
              , stackAddrLhs.addr | stackAddrRhs.addr << 16);
}

static inline void BCGen_emitLongInstSI(BCGen* self, const LongInst i, const StackAddr stackAddrLhs, uint32_t rhs)
{
    BCGen_emit2(self
              , i | stackAddrLhs.addr << 16
              , rhs);
}

static inline void BCGen_emitLongInstSSS(BCGen* self, const LongInst i, const StackAddr stackAddrOp,
    const StackAddr stackAddrLhs, const StackAddr stackAddrRhs)
{
    BCGen_emit2(self
              , i | stackAddrOp.addr << 16
              , stackAddrLhs.addr | stackAddrRhs.addr << 16);
}

static inline void BCGen_emitLongInstA(BCGen* self, const LongInst i, const BCAddr targetAddr)
{
    BCGen_emit2(self
              , i
              , targetAddr.addr);
}

static inline uint32_t BCGen_currSp(BCGen* self)
{
    return self->sp;
}

/// semi-public functions for the vtbl start here

BCValue BCGen_genTemporary(BCGen* self, BCType bct)
{
    BCValue result;

    result.type = bct;
    result.temporaryIndex = ++self->temporaryCount;
    result.stackAddr.addr = self->sp;

    if (BCType_isBasicBCType(bct))
    {
        self->sp += align4(BCTypeEnum_basicTypeSize(bct.type));
    }
    else
    {
        self->sp += 4;
    }

    return result;
}

void BCGen_destroyTemporary(BCGen* self, BCValue tmp)
{
    assert(BCValue_isStackValueOrParameter(&tmp));//, "tmporary has to be stack-value");
    uint32_t sz;
    if (BCType_isBasicBCType(tmp.type))
    {
        sz = align4(BCTypeEnum_basicTypeSize(tmp.type.type));
    }
    else
    {
        sz = 4;
    }
    if (self->sp - sz == tmp.stackAddr.addr)
    {
        // this is the last thing we pushed on
        // free the stack space immediately.
        self->sp -= sz;
    }
}

static inline void BCGen_Initialize(BCGen* self)
{
    self->callCount = 0;
    self->parameterCount = 0;
    self->temporaryCount = 0;
    self->localCount = 0;
    
    self->byteCodeArray[0] = 0;
    self->byteCodeArray[1] = 0;
    self->byteCodeArray[2] = 0;
    self->byteCodeArray[3] = 0;

    self->ip = 4;
    self->sp = 4;

    self->insideFunction = false;
    self->fd = 0;
}

static inline void BCGen_Finalize(BCGen* self)
{
    assert(self->ip < ARRAY_SIZE(self->byteCodeArray));
    self->finalized = true;
    // TODO write some kind of end marker into the bytecode
}


static inline uint32_t BCGen_beginFunction(BCGen* self, uint32_t fnIdx, void* fd)
{
    assert(self->fd == 0);

    if (!fnIdx)
        fnIdx = ++self->functionCount;

    // emit four zeros as start marker
    BCGen_emit2(self, 0, 0);
    BCGen_emit2(self, 0, 0);

    BCFunction* f = self->functions + (fnIdx - 1);
    f->bytecode_start = self->ip;

    self->insideFunction = true;
    self->functionIdx = fnIdx;
    self->fd = fd;

    return fnIdx;
}

static inline void BCGen_endFunction(BCGen* self, uint32_t fIdx)
{
    assert(self->insideFunction);
    assert(self->functionIdx == fIdx);
    self->insideFunction = false;

    self->localCount = 0;

    BCFunction *f = self->functions + (fIdx - 1);

    f->type = BCFunctionTypeEnum_Bytecode;
    f->maxStackUsed = self->sp;
    f->fn = self->functionIdx;
    f->nArgs = self->parameterCount;
    f->bytecode_end = self->ip;

    self->sp = 4;
    self->fd = 0;
    self->parameterCount = 0;
}

static inline BCValue BCGen_genLocal(BCGen* self, BCType bct, const char* name)
{
    uint32_t sp = self->sp;
    uint16_t localAddr = (uint16_t)sp;
    uint16_t localIdx = ++self->localCount;

    BCValue result = {0};
    result.type = bct;
    result.stackAddr.addr = localAddr;
    result.localIndex  = localIdx;
    result.vType = BCValueType_StackValue;

    if (BCType_isBasicBCType(bct))
    {
        sp += align4(BCTypeEnum_basicTypeSize(bct.type));
    }
    else
    {
        sp += 4;
    }
    BCLocal local = {localIdx, bct, {localAddr}, name};

    self->locals[localIdx - 1] = local;

    self->sp = sp;
    return result;
}

static inline BCValue BCGen_genParameter(BCGen* self, BCType bct, const char* name)
{
    BCValue p;

    p.type =  bct;
    p.vType = BCValueType_Parameter;
    p.parameterIndex = ++self->parameterCount;
    p.stackAddr.addr = self->sp;

    self->sp += 4;
    p.name = name;
    
    return p;
}

static inline uint32_t BCGen_beginJmp(BCGen* self)
{
    uint32_t atIp = self->ip;
    self->ip += 2;

    return atIp;
}
static inline void BCGen_emitArithInstruction(BCGen* self
                                            , LongInst inst
                                            , BCValue lhs
                                            , BCValue rhs
                                            , BCTypeEnum* resultTypeEnum);

static inline BCValue BCGen_castTo(BCGen* self, BCValue rhs, BCTypeEnum targetType)
{
    BCTypeEnum sourceType = rhs.type.type;

    if (sourceType == targetType)
        return rhs;

    BCType type = {targetType};
    BCValue lhs = BCGen_genTemporary(self, type);

    assert(BCValue_isStackValueOrParameter(&rhs));

    switch(targetType)
    {
        case BCTypeEnum_f52 :
            if (sourceType == BCTypeEnum_f23)
                BCGen_emitLongInstSS(self, LongInst_F32ToF64, lhs.stackAddr, rhs.stackAddr);
            else
                BCGen_emitLongInstSS(self, LongInst_IToF64, lhs.stackAddr, rhs.stackAddr);
        break;
        case BCTypeEnum_f23 :
            if (sourceType == BCTypeEnum_f52)
                BCGen_emitLongInstSS(self, LongInst_F64ToF32, lhs.stackAddr, rhs.stackAddr);
            else
                BCGen_emitLongInstSS(self, LongInst_IToF32, lhs.stackAddr, rhs.stackAddr);
        break;
        case BCTypeEnum_i32 :
        case BCTypeEnum_i64 :
            if (sourceType == BCTypeEnum_f23)
                BCGen_emitLongInstSS(self, LongInst_F32ToI, lhs.stackAddr, rhs.stackAddr);
            else if (sourceType == BCTypeEnum_f52)
                BCGen_emitLongInstSS(self, LongInst_F64ToI, lhs.stackAddr, rhs.stackAddr);
        break;
        default :
            assert(0); //, "me no have no cast for targetType " ~ enumToString(targetType));}
        //break;
    }

    return lhs;
}

void BCGen_Set(BCGen* self, BCValue lhs, BCValue rhs)
{
    assert(BCValue_isStackValueOrParameter(&lhs));//, "Set lhs is has to be a StackValue. Not: " ~ enumToString(lhs.vType));
    assert(rhs.vType == BCValueType_Immediate || BCValue_isStackValueOrParameter(&rhs));//, "Set rhs is has to be a StackValue or Imm not: " ~ rhs.vType.enumToString);

    if (rhs.vType == BCValueType_Immediate && (rhs.type.type == BCTypeEnum_i64 || rhs.type.type == BCTypeEnum_u64 || rhs.type.type == BCTypeEnum_f52))
    {
        BCGen_emitLongInstSI(self, LongInst_SetImm32, lhs.stackAddr, (rhs.imm64.imm64 & UINT32_MAX));
        if ((((rhs.type.type == BCTypeEnum_u64 || rhs.type.type == BCTypeEnum_i64)) && rhs.imm64.imm64 > UINT32_MAX) || rhs.type.type == BCTypeEnum_f52) // if there are high bits
            BCGen_emitLongInstSI(self, LongInst_SetHighImm32, lhs.stackAddr, (rhs.imm64.imm64 >> 32));
    }

    else if (!BCValue_eq(&lhs, &rhs)) // do not emit self assignments;
    {
        BCGen_emitArithInstruction(self, LongInst_Set, lhs, rhs, 0);
    }
}

static inline BCValue BCGen_pushOntoStack(BCGen* self, BCValue val)
{
    if (!BCValue_isStackValueOrParameter(&val))
    {
        BCValue stackref;
        stackref.type = val.type;
        stackref.vType = BCValueType_StackValue;
        stackref.stackAddr.addr = self->sp;
        stackref.temporaryIndex = ++self->temporaryCount;

        BCGen_Set(self, u32(stackref), val);

        self->sp += align4(BCTypeEnum_basicTypeSize(val.type.type));
        return stackref;
    }
    else
    {
        return val;
    }
}


static inline void BCGen_emitArithInstruction(BCGen* self
                                            , LongInst inst
                                            , BCValue lhs
                                            , BCValue rhs
                                            , BCTypeEnum* resultTypeEnum)
{
    assert(inst >= LongInst_Add && inst < LongInst_ImmAdd); //,
//        "Instruction is not in Range for Arith Instructions");

    BCTypeEnum commonType = BCTypeEnum_commonTypeEnum(lhs.type.type, rhs.type.type);

    // FIXME Implement utf8 <-> utf32 conversion
    assert(commonType == BCTypeEnum_i32 || commonType == BCTypeEnum_i64
        || commonType == BCTypeEnum_u32 || commonType == BCTypeEnum_u64
        || commonType == BCTypeEnum_f23 || commonType == BCTypeEnum_c32
        || commonType == BCTypeEnum_c8  || commonType == BCTypeEnum_f52);//,
    //    "only i32, i64, f23, f52, is supported for now not: " ~ enumToString(commonType));
    //assert(lhs.type.type == rhs.type.type, enumToString(lhs.type.type) ~ " != " ~ enumToString(rhs.type.type));

    if (lhs.vType == BCValueType_Immediate)
    {
        lhs = BCGen_pushOntoStack(self, lhs);
    }

    if (resultTypeEnum)
        *resultTypeEnum = commonType;

    if (lhs.type.type == BCTypeEnum_f23)
    {
        if(rhs.type.type == BCTypeEnum_i32 || rhs.type.type == BCTypeEnum_u32)
        {
            if (rhs.vType == BCValueType_Immediate)
            {
                float frhs = (float) rhs.imm32.imm32;
                rhs = imm32(*(int32_t*)&frhs);
            }
            else
                rhs = BCGen_castTo(self, rhs, BCTypeEnum_f23);
        }
        else if (rhs.type.type == BCTypeEnum_f23)
        {
            rhs = BCGen_pushOntoStack(self, rhs);
        }
        else if (rhs.type.type == BCTypeEnum_f52)
        {
            rhs = BCGen_castTo(self, rhs, lhs.type.type);
        }
        else
            assert(0);//, "did not expect type " ~ enumToString(rhs.type.type) ~ "to be used in a float expression");
        if (inst != LongInst_Set)
        {
            inst += (LongInst_FAdd32 - LongInst_Add);
        }
    }
    else if (lhs.type.type == BCTypeEnum_f52)
    {
        if(rhs.type.type != BCTypeEnum_f52)
        {
            // TOOD there was
            // assert (rhs.type.type == BCTypeEnum_f52)
            // here before .... check if this is an invariant
            rhs = BCGen_castTo(self, rhs, BCTypeEnum_f52);
        }

        rhs = BCGen_pushOntoStack(self, rhs);
        if (inst != LongInst_Set)
        {
            inst += (LongInst_FAdd64 - LongInst_Add);
        }
    }
    else if (rhs.vType == BCValueType_Immediate)
    {
        const int64_t imm64s = (BCTypeEnum_basicTypeSize(rhs.type.type) == 8 ? cast(int64_t)rhs.imm64.imm64 : 0);
        if  (BCTypeEnum_basicTypeSize(rhs.type.type) <= 4 || (imm64s <= INT32_MAX && imm64s > -INT32_MAX))
        {
            //Change the instruction into the corresponding Imm Instruction;
            if (inst != LongInst_Set)
            {
                inst += (LongInst_ImmAdd - LongInst_Add);
            }
            else
            {
                inst = LongInst_SetImm32;
            }
            BCGen_emitLongInstSI(self, inst, lhs.stackAddr, rhs.imm32.imm32);
            return ;
        }
        else
        {
            rhs = BCGen_pushOntoStack(self, rhs);
        }
    }

    if (BCValue_isStackValueOrParameter(&rhs))
    {
        BCGen_emitLongInstSS(self, inst, lhs.stackAddr, rhs.stackAddr);
    }
    else
    {
        assert(0);//, "Cannot handle: " ~ enumToString(rhs.vType));
    }
}

static inline void BCGen_Add3(BCGen* self, BCValue result, BCValue lhs, BCValue rhs)
{
    assert(result.vType != BCValueType_Immediate); //, "Cannot add to Immediate");

    result = (result.vType != BCValueType_Unknown ? result : lhs);
    if (!BCValue_eq(&lhs, &result))
    {
        BCGen_Set(self, result, lhs);
    }

    BCGen_emitArithInstruction(self, LongInst_Add, result, rhs, &result.type.type);
}

static inline void BCGen_Ret(BCGen*self, BCValue val)
{
    LongInst inst = ((BCTypeEnum_basicTypeSize(val.type.type) == 8) ? LongInst_Ret64 : LongInst_Ret32);
    val = BCGen_pushOntoStack(self, val);

    if (BCValue_isStackValueOrParameter(&val))
    {
        BCGen_emit2(self, BCGen_ShortInst16(inst, val.stackAddr.addr), 0);
    }
    else
    {
        assert(0);//, "I cannot deal with this type of return" ~ enumToString(val.vType));
    }
}

static inline BCValue BCGen_run(BCGen* self, uint32_t fnIdx, BCValue* args, uint32_t n_args)
{
    BCValue result;

    assert(self->finalized);

    BCHeap newHeap = {0};
    newHeap.heapMax = 1 << 14;
    newHeap.heapData = malloc(newHeap.heapMax);

    result = BCGen_interpret(self, fnIdx, args, n_args, &newHeap);

    return result;
}

EXTERN_C BackendInterface BCGen_newInterface(void)
{
    BackendInterface result = {
        .Initialize = cast(Initialize_t) BCGen_Initialize,
        .Finalize = cast(Finalize_t) BCGen_Finalize,
        .beginFunction = cast(beginFunction_t) BCGen_beginFunction,
        .endFunction = cast(endFunction_t) BCGen_endFunction,
        .genParameter = cast(genLocal_t) BCGen_genParameter,
        .genLocal = cast(genLocal_t) BCGen_genLocal,

        .Add3 = cast(Add3_t) BCGen_Add3,
        .Ret = cast(Ret_t) BCGen_Ret,

        .run = cast(run_t) BCGen_run,

        .new_instance = (new_instance_t) BCGen_new_instance,
        .destroy_instance = (destroy_instance_t) BCGen_destroy_instance
    };
    return result;
}

void endJmp(BCGen* self, BCAddr atIp, BCLabel target)
{
    uint16_t offset = BCGen_isShortJump(target.addr.addr - atIp.addr);
    if (offset)
    {
        BCGen_emit2_at(self, BCGen_ShortInst16(LongInst_RelJmp, offset), 0, atIp.addr);
    }
    else
    {
        BCGen_emit2_at(self, LongInst_Jmp, target.addr.addr, atIp.addr);
    }
}

#undef FLT32_BEGIN
#undef FLT32_END

#undef FLT64_BEGIN
#undef FLT64_END

#if 0
{

    BCLabel genLabel()
    {
        return BCLabel(ip);
    }

    CndJmpBegin beginCndJmp(BCValue cond = BCValue.init, bool ifTrue = false)
    {
        if (cond.vType == BCValueType.Immediate)
        {
            cond = pushOntoStack(cond);
        }

        auto result = CndJmpBegin(ip, cond, ifTrue);
        ip += 2;
        return result;
    }

    void endCndJmp(CndJmpBegin jmp, BCLabel target)
    {
        auto atIp = jmp.at;
        auto cond = jmp.cond;
        auto ifTrue = jmp.ifTrue;

        LongInst64 lj;

        if (BCValue_isStackValueOrParameter(&cond))
        {
            lj = (ifTrue ? LongInst64(LongInst_JmpNZ, cond.stackAddr,
                target.addr) : LongInst64(LongInst_JmpZ, cond.stackAddr, target.addr));
        }
        else // if (cond == bcLastCond)
        {
            lj = (ifTrue ? LongInst64(LongInst_JmpTrue,
                target.addr) : LongInst64(LongInst_JmpFalse, target.addr));
        }

        byteCodeArray[atIp] = lj.lw;
        byteCodeArray[atIp + 1] = lj.hi;
    }

    void Jmp(BCLabel target)
    {
        assert(target.addr);
        if (ip != target.addr)
        {
            auto at = beginJmp();
            endJmp(at, target);
        }
    }

    void emitFlg(BCValue lhs)
    {
        assert(BCValue_isStackValueOrParameter(&lhs))//, "Can only store flags in Stack Values");
        byteCodeArray[ip] = ShortInst16(LongInst_Flg, lhs.stackAddr.addr);
        byteCodeArray[ip + 1] = 0;
        ip += 2;
    }

    void Alloc(BCValue heapPtr, BCValue size, uint line = __LINE__)
    {
        assert(size.type.type == BCTypeEnum_u32)//, "Size for alloc needs to be an u32" ~ " called by:" ~ itos(line));
        if (size.vType == BCValueType.Immediate)
        {
            size = pushOntoStack(size);
        }
        assert(BCValue_isStackValueOrParameter(&size));
        assert(BCValue_isStackValueOrParameter(&heapPtr));

        BCGen_emitLongInstSS(self, LongInst_Alloc, heapPtr.stackAddr, size.stackAddr);
    }

    void Assert(BCValue value, BCValue err, uint l = __LINE__)
    {
        BCValue _msg;
        if (BCValue_isStackValueOrParameter(&err))
        {
            assert(0);//, "err.vType is not Error but: " ~ enumToString(err.vType));
        }

        if (value)
        {
            emitLongInst(LongInst_Assert, pushOntoStack(value).stackAddr, imm32(err));
        }
        else
        {
            assert(0, "BCValue.init is no longer a valid value for assert -- fromLine: " ~ itos(l));
        }

    }

    void MemCpy(BCValue dst, BCValue src, BCValue size)
    {
        size = pushOntoStack(size);
        src = pushOntoStack(src);
        dst = pushOntoStack(dst);

        BCGen_emitLongInstSS(self, LongInst_MemCpy, size.stackAddr, dst.stackAddr, src.stackAddr);
    }


    void outputBytes(const (char)[] s)
    {
        outputBytes(cast(const ubyte[]) s);
    }

    void outputBytes (const ubyte[] bytes)
    {
        auto len = bytes.length;
        size_t idx = 0;

        while (len >= 4)
        {
            byteCodeArray[ip++] =
                bytes[idx+0] << 0 |
                bytes[idx+1] << 8 |
                bytes[idx+2] << 16 |
                bytes[idx+3] << 24;

            idx += 4;
            len -= 4;
        }

        uint lastField;

        final switch(len)
        {
            case 3 :
                lastField |= bytes[idx+2] << 16;
                goto case;
            case 2 :
                lastField |= bytes[idx+1] << 8;
                goto case;
            case 1 :
                lastField |= bytes[idx+0] << 0;
                byteCodeArray[ip++] = lastField;
                goto case;
            case 0 :
                break;
        }
    }

    void File(string filename)
    {
        auto filenameLength = cast(uint) filename.length;

        emitLongInst(LongInst_File, StackAddr.init, Imm32(filenameLength));

        outputBytes(filename);
    }

    void Line(uint line)
    {
         emitLongInst(LongInst_Line, StackAddr(0), Imm32(line));
    }

    void Comment(lazy const (char)[] comment)
    {
        debug
        {
            uint commentLength = cast(uint) comment.length;

            emitLongInstSI(LongInst_Comment, StackAddr.init, commentLength);

            outputBytes(comment);
        }
    }

    void Prt(BCValue value, bool isString = false)
    {
        if (value.vType == BCValueType.Immediate)
            value = pushOntoStack(value);

        byteCodeArray[ip] = ShortInst16Ex(LongInst_PrintValue, isString, value.stackAddr);
        byteCodeArray[ip + 1] = 0;
        ip += 2;
    }

    void Not(BCValue result, BCValue val)
    {
        if (result != val)
        {
            Set(result, val);
            val = result;
        }
        if (val.vType == BCValueType.Immediate)
            val = pushOntoStack(val);

        byteCodeArray[ip] = ShortInst16(LongInst_Not, val.stackAddr);
        byteCodeArray[ip + 1] = 0;
        ip += 2;
    }

    
    void SetHigh(BCValue lhs, BCValue rhs)
    {
        assert(BCValue_isStackValueOrParameter(&lhs), "SetHigh lhs is has to be a StackValue");
        assert(rhs.vType == BCValueType.Immediate || BCValue_isStackValueOrParameter(&rhs), "SetHigh rhs is has to be a StackValue or Imm");
        assert(0, "SetHigh is not implemented");
        //two cases :
        //    lhs.type.size == 4 && rhs.type.size == 8
        // OR
        //    lhs.type.size == 8 && rhs.type.size == 4

    }

    void Ult3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(result.vType == BCValueType.Unknown
            || BCValue_isStackValueOrParameter(&result),
            "The result for this must be Empty or a StackValue");
        emitArithInstruction(LongInst_Ult, lhs, rhs, null);

        if (BCValue_isStackValueOrParameter(&result))
        {
            emitFlg(result);
        }
    }

    void Ule3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(result.vType == BCValueType.Unknown
            || BCValue_isStackValueOrParameter(&result),
            "The result for this must be Empty or a StackValue");
        emitArithInstruction(LongInst_Ule, lhs, rhs);

        if (BCValue_isStackValueOrParameter(&result))
        {
            emitFlg(result);
        }
    }

    void Lt3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(result.vType == BCValueType.Unknown
            || BCValue_isStackValueOrParameter(&result),
            "The result for this must be Empty or a StackValue");
        emitArithInstruction(LongInst_Lt, lhs, rhs);

        if (BCValue_isStackValueOrParameter(&result))
        {
            emitFlg(result);
        }
    }

    void Le3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(result.vType == BCValueType.Unknown
            || BCValue_isStackValueOrParameter(&result),
            "The result for this must be Empty or a StackValue");
        emitArithInstruction(LongInst_Le, lhs, rhs);

        if (BCValue_isStackValueOrParameter(&result))
        {
            emitFlg(result);
        }
    }

    void Ugt3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(result.vType == BCValueType.Unknown
            || BCValue_isStackValueOrParameter(&result),
            "The result for this must be Empty or a StackValue");
        emitArithInstruction(LongInst_Ugt, lhs, rhs);
        if (BCValue_isStackValueOrParameter(&result))
        {
            emitFlg(result);
        }
    }

    void Uge3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(result.vType == BCValueType.Unknown
            || BCValue_isStackValueOrParameter(&result),
            "The result for this must be Empty or a StackValue");
        emitArithInstruction(LongInst_Uge, lhs, rhs);
        if (BCValue_isStackValueOrParameter(&result))
        {
            emitFlg(result);
        }
    }

    void Gt3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(result.vType == BCValueType.Unknown
            || BCValue_isStackValueOrParameter(&result),
            "The result for this must be Empty or a StackValue");
        emitArithInstruction(LongInst_Gt, lhs, rhs);
        if (BCValue_isStackValueOrParameter(&result))
        {
            emitFlg(result);
        }
    }

    void Ge3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(result.vType == BCValueType.Unknown
            || BCValue_isStackValueOrParameter(&result),
            "The result for this must be Empty or a StackValue");
        emitArithInstruction(LongInst_Ge, lhs, rhs);
        if (BCValue_isStackValueOrParameter(&result))
        {
            emitFlg(result);
        }
    }

    void Eq3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(result.vType == BCValueType.Unknown
            || BCValue_isStackValueOrParameter(&result),
            "The result for this must be Empty or a StackValue not " ~ enumToString(result.vType) );
        emitArithInstruction(LongInst_Eq, lhs, rhs);

        if (BCValue_isStackValueOrParameter(&result))
        {
            emitFlg(result);
        }
    }

    void Neq3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(result.vType == BCValueType.Unknown
            || BCValue_isStackValueOrParameter(&result),
            "The result for this must be Empty or a StackValue");
        emitArithInstruction(LongInst_Neq, lhs, rhs);

        if (BCValue_isStackValueOrParameter(&result))
        {
            emitFlg(result);
        }
    }

    void Sub3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(result.vType != BCValueType.Immediate, "Cannot sub to Immediate");

        result = (result ? result : lhs);
        if (lhs != result)
        {
            Set(result, lhs);
        }

        emitArithInstruction(LongInst_Sub, result, rhs, &result.type.type);
    }

    void Mul3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(result.vType != BCValueType.Immediate, "Cannot mul to Immediate");

        result = (result ? result : lhs);
        if (lhs != result)
        {
            Set(result, lhs);
        }

        // Prt(result); Prt(lhs); Prt(rhs);

        emitArithInstruction(LongInst_Mul, result, rhs, &result.type.type);
    }

    void Div3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(result.vType != BCValueType.Immediate, "Cannot div to Immediate");

        result = (result ? result : lhs);
        if (lhs != result)
        {
            Set(result, lhs);
        }
        emitArithInstruction(LongInst_Div, result, rhs, &result.type.type);
    }

    void Udiv3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(result.vType != BCValueType.Immediate, "Cannot div to Immediate");

        result = (result ? result : lhs);
        if (lhs != result)
        {
            Set(result, lhs);
        }
        emitArithInstruction(LongInst_Udiv, result, rhs, &result.type.type);
    }

    void And3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(result.vType != BCValueType.Immediate, "Cannot and to Immediate");

        result = (result ? result : lhs);
        if (lhs != result)
        {
            Set(result, lhs);
        }
        if (lhs.type.type == BCTypeEnum_i32 && rhs.type.type == BCTypeEnum_i32)
            emitArithInstruction(LongInst_And32, result, rhs);
        else
            emitArithInstruction(LongInst_And, result, rhs);

    }

    void Or3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(result.vType != BCValueType.Immediate, "Cannot or to Immediate");

        result = (result ? result : lhs);
        if (lhs != result)
        {
            Set(result, lhs);
        }
        emitArithInstruction(LongInst_Or, result, rhs);

    }

    void Xor3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(result.vType != BCValueType.Immediate, "Cannot xor to Immediate");

        result = (result ? result : lhs);
        if (lhs != result)
        {
            Set(result, lhs);
        }
        if (lhs.type.type == BCTypeEnum_i32 && rhs.type.type == BCTypeEnum_i32)
            emitArithInstruction(LongInst_Xor32, result, rhs);
        else
            emitArithInstruction(LongInst_Xor, result, rhs);
    }

    void Lsh3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(result.vType != BCValueType.Immediate, "Cannot lsh to Immediate");

        result = (result ? result : lhs);
        if (lhs != result)
        {
            Set(result, lhs);
        }
        emitArithInstruction(LongInst_Lsh, result, rhs);
    }

    void Rsh3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(result.vType != BCValueType.Immediate, "Cannot rsh to Immediate");

        result = (result ? result : lhs);
        if (lhs != result)
        {
            Set(result, lhs);
        }
        emitArithInstruction(LongInst_Rsh, result, rhs);
    }

    void Mod3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(result.vType != BCValueType.Immediate, "Cannot mod to Immediate");

        result = (result ? result : lhs);
        if (lhs != result)
        {
            Set(result, lhs);
        }
        emitArithInstruction(LongInst_Mod, result, rhs, &result.type.type);
    }

    void Umod3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(result.vType != BCValueType.Immediate, "Cannot mod to Immediate");

        result = (result ? result : lhs);
        if (lhs != result)
        {
            Set(result, lhs);
        }
        emitArithInstruction(LongInst_Umod, result, rhs, &result.type.type);
    }

    void Call(BCValue result, BCValue fn, BCValue[] args)
    {
        auto call_id = pushOntoStack(imm32(callCount + 1)).stackAddr;
        calls[callCount++] = RetainedCall(fn, args, functionIdx, ip, sp);
        emitLongInst(LongInst_Call, result.stackAddr, call_id);
    }

    void Load8(BCValue _to, BCValue from)
    {
        if (!BCValue_isStackValueOrParameter(&from))
        {
            from = pushOntoStack(from);
        }
        if (!BCValue_isStackValueOrParameter(&_to))
        {
            _to = pushOntoStack(_to);
        }
        assert(BCValue_isStackValueOrParameter(&_to), "to has the vType " ~ enumToString(_to.vType));
        assert(BCValue_isStackValueOrParameter(&from), "from has the vType " ~ enumToString(from.vType));
        
        emitLongInst(LongInst_HeapLoad8, _to.stackAddr, from.stackAddr);
    }

    void Store8(BCValue _to, BCValue value)
    {
        if (!BCValue_isStackValueOrParameter(&value))
        {
            value = pushOntoStack(value);
        }

        if (!BCValue_isStackValueOrParameter(&_to))
        {
            _to = pushOntoStack(_to);
        }

        assert(BCValue_isStackValueOrParameter(&_to), "to has the vType " ~ enumToString(_to.vType));
        assert(BCValue_isStackValueOrParameter(&value), "value has the vType " ~ enumToString(value.vType));

        emitLongInst(LongInst_HeapStore8, _to.stackAddr, value.stackAddr);
    }

    void Load16(BCValue _to, BCValue from)
    {
        if (!BCValue_isStackValueOrParameter(&from))
        {
            from = pushOntoStack(from);
        }
        if (!BCValue_isStackValueOrParameter(&_to))
        {
            _to = pushOntoStack(_to);
        }
        assert(BCValue_isStackValueOrParameter(&_to), "to has the vType " ~ enumToString(_to.vType));
        assert(BCValue_isStackValueOrParameter(&from), "from has the vType " ~ enumToString(from.vType));
        
        emitLongInst(LongInst_HeapLoad16, _to.stackAddr, from.stackAddr);
    }
    
    void Store16(BCValue _to, BCValue value)
    {
        if (!BCValue_isStackValueOrParameter(&value))
        {
            value = pushOntoStack(value);
        }
        
        if (!BCValue_isStackValueOrParameter(&_to))
        {
            _to = pushOntoStack(_to);
        }
        
        assert(BCValue_isStackValueOrParameter(&_to), "to has the vType " ~ enumToString(_to.vType));
        assert(BCValue_isStackValueOrParameter(&value), "value has the vType " ~ enumToString(value.vType));
        
        emitLongInst(LongInst_HeapStore16, _to.stackAddr, value.stackAddr);
    }

    void Load32(BCValue _to, BCValue from)
    {
        if (!BCValue_isStackValueOrParameter(&from))
        {
            from = pushOntoStack(from);
        }
        
        if (!BCValue_isStackValueOrParameter(&_to))
        {
            _to = pushOntoStack(_to);
        }
        
        assert(BCValue_isStackValueOrParameter(&_to), "to has the vType " ~ enumToString(_to.vType));
        assert(BCValue_isStackValueOrParameter(&from), "from has the vType " ~ enumToString(from.vType));
        
        emitLongInst(LongInst_HeapLoad32, _to.stackAddr, from.stackAddr);
    }

    void Store32(BCValue _to, BCValue value)
    {
        if (!BCValue_isStackValueOrParameter(&value))
        {
            value = pushOntoStack(value);
        }

        if (!BCValue_isStackValueOrParameter(&_to))
        {
            _to = pushOntoStack(_to);
        }

        assert(BCValue_isStackValueOrParameter(&_to), "to has the vType " ~ enumToString(_to.vType));
        assert(BCValue_isStackValueOrParameter(&value), "value has the vType " ~ enumToString(value.vType));

        emitLongInst(LongInst_HeapStore32, _to.stackAddr, value.stackAddr);
    }

    void Load64(BCValue _to, BCValue from)
    {
        if (!BCValue_isStackValueOrParameter(&from))
        {
            from = pushOntoStack(from);
        }
        if (!BCValue_isStackValueOrParameter(&_to))
        {
            _to = pushOntoStack(_to);
        }
        assert(BCValue_isStackValueOrParameter(&_to), "to has the vType " ~ enumToString(_to.vType));
        assert(BCValue_isStackValueOrParameter(&from), "from has the vType " ~ enumToString(from.vType));

        emitLongInst(LongInst_HeapLoad64, _to.stackAddr, from.stackAddr);
    }

    void Store64(BCValue _to, BCValue value)
    {
        if (!BCValue_isStackValueOrParameter(&value))
        {
            value = pushOntoStack(value);
        }
        if (!BCValue_isStackValueOrParameter(&_to))

        {
            _to = pushOntoStack(_to);
        }

        assert(BCValue_isStackValueOrParameter(&_to), "to has the vType " ~ enumToString(_to.vType));
        assert(BCValue_isStackValueOrParameter(&value), "value has the vType " ~ enumToString(value.vType));

        emitLongInst(LongInst_HeapStore64, _to.stackAddr, value.stackAddr);
    }

    void Throw(BCValue e)
    {
        assert(BCValue_isStackValueOrParameter(&e));
        byteCodeArray[ip] = ShortInst16(LongInst_Throw, e.stackAddr);
        byteCodeArray[ip + 1] = 0;
        ip += 2;
    }

    void PushCatch()
    {
        byteCodeArray[ip] = ShortInst16(LongInst_PushCatch, 0);
        byteCodeArray[ip + 1] = 0;
        ip += 2;
    }

    void PopCatch()
    {
        byteCodeArray[ip] = ShortInst16(LongInst_PopCatch, 0);
        byteCodeArray[ip + 1] = 0;
        ip += 2;
    }

/+
    void Push(BCValue v)
    {
        const sz = BCTypeEnum_basicTypeSize(v.typ.type);
        assert(sz >= 1 && sz <= 4);
        if (v.vType == BCValueType.Immediate)
        {
            byteCodeArray[ip] = LongInst_PushImm32;
            byteCodeArray[ip + 1] = v.imm32.imm32;
        }
        else
        {
            byteCodeArray[ip] = ShortInst16(LongInst_Push32, v.stackAddr);
            byteCodeArray[ip + 1] = 0;
        }
        ip += 2;
    }
+/
    void IToF32(BCValue result, BCValue rhs)
    {
        assert(BCValue_isStackValueOrParameter(&result));
        assert(BCValue_isStackValueOrParameter(&rhs));

        emitLongInst(LongInst_IToF32, result.stackAddr, rhs.stackAddr);
    }

    void IToF64(BCValue result, BCValue rhs)
    {
        assert(BCValue_isStackValueOrParameter(&result));
        assert(BCValue_isStackValueOrParameter(&rhs));

        emitLongInst(LongInst_IToF64, result.stackAddr, rhs.stackAddr);
    }

    void F32ToI(BCValue result, BCValue rhs)
    {
        assert(BCValue_isStackValueOrParameter(&result));
        assert(BCValue_isStackValueOrParameter(&rhs));

        emitLongInst(LongInst_F32ToI, result.stackAddr, rhs.stackAddr);
    }

    void F64ToI(BCValue result, BCValue rhs)
    {
        assert(BCValue_isStackValueOrParameter(&result));
        assert(BCValue_isStackValueOrParameter(&rhs));

        emitLongInst(LongInst_F64ToI, result.stackAddr, rhs.stackAddr);
    }

    void F32ToF64(BCValue result, BCValue rhs)
    {
        assert(BCValue_isStackValueOrParameter(&result));
        assert(BCValue_isStackValueOrParameter(&rhs));

        emitLongInst(LongInst_F32ToF64, result.stackAddr, rhs.stackAddr);

    }

    void F64ToF32(BCValue result, BCValue rhs)
    {
        assert(BCValue_isStackValueOrParameter(&result));
        assert(BCValue_isStackValueOrParameter(&rhs));

        emitLongInst(LongInst_F64ToF32, result.stackAddr, rhs.stackAddr);
    }


    void StrEq3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(result.vType == BCValueType.Unknown
            || BCValue_isStackValueOrParameter(&result),
            "The result for this must be Empty or a StackValue not: " ~ enumToString(result.vType));
        if (lhs.vType == BCValueType.Immediate)
        {
            lhs = pushOntoStack(lhs);
        }
        if (rhs.vType == BCValueType.Immediate)
        {
            rhs = pushOntoStack(rhs);
        }
        assert(BCValue_isStackValueOrParameter(&lhs),
            "The lhs of StrEq3 is not a StackValue " ~ enumToString(rhs.vType));
        assert(BCValue_isStackValueOrParameter(&rhs),
            "The rhs of StrEq3 not a StackValue" ~ enumToString(rhs.vType));

        emitLongInst(LongInst_StrEq, lhs.stackAddr, rhs.stackAddr);

        if (BCValue_isStackValueOrParameter(&result))
        {
            emitFlg(result);
        }
    }

    void Cat3(BCValue result, BCValue lhs, BCValue rhs, const uint size)
    {
        assert(size <= 255);

        assert(BCValue_isStackValueOrParameter(&result));

        lhs = pushOntoStack(lhs);
        rhs = pushOntoStack(rhs);
        emitLongInst(LongInst_Cat, result.stackAddr, lhs.stackAddr, rhs.stackAddr);
        // Hack! we have no overload to store additional information in the 8 bit
        // after the inst so just dump it in there let's hope we don't overwrite
        // anything important
        byteCodeArray[ip-2] |= (size & 255) << 8;

    }

}
#endif
