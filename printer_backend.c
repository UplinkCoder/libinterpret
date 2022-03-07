#include "backend_interface_funcs.h"
#include "bc_common.h"

#define cast(T) (T)

typedef struct ErrorInfo
{
    const char* msg;
    BCValue values[4];
    uint32_t valueCount;
} ErrorInfo;

typedef struct Printer
{
    char* Buffer;
    uint32_t BufferSize;
    uint32_t BufferCapacity;
    
    uint32_t CurrentIndent;
    
    ErrorInfo* ErrorInfos;
    uint32_t ErrorInfoCount;
    uint32_t ErrorInfoCapacity;
    
    char* functionSuffix;
} Printer;



#include "int_to_str.c"
#include "fpconv/fpconv.c"

static inline void Printer_PutNl(Printer* self)
{
    assert(self->BufferCapacity >= 1);
    
    *self->Buffer++ = '\n';
    self->BufferSize++;
    self->BufferCapacity--;
}

static inline void Printer_PutStr(Printer* self, const char* str)
{
    uint32_t length = 0;
    char c;
    while((c = *str++))
    {
        length++;
        *self->Buffer++ = c;
    }

    self->BufferSize += length;
    self->BufferCapacity -= length;
}

static inline void Printer_PutU64(Printer* self, const uint64_t v)
{
    char buffer[21];

    char* begin_number = u64tostr(v, buffer);

    Printer_PutStr(self, begin_number);
}


static inline void Printer_PutI64(Printer* self, const int64_t v)
{
    char buffer[21];

    char* begin_number = i64tostr(v, buffer);

    Printer_PutStr(self, begin_number);
}

#define Printer_PutU32 Printer_PutU32
#define Printer_PutU32 Printer_PutI64

static inline void Printer_PrintType(Printer* self, const BCType* type)
{
    
}

static inline void Printer_PrintBCValue(Printer* self, const BCValue* val)
{
    BCValueType vType = val->vType;
    const BCType type = val->type;
    const char* name = val->name;
    uint32_t val_imm32 = (vType  == BCValueType_Immediate ? val->imm32.imm32 : 0);
    
    switch (vType)
    {
    case BCValueType_Immediate:
        {
            switch(type.type)
            {
            case BCTypeEnum_c8:
                {
                    Printer_PutStr(self, "Imm32('");
                    Printer_PutChar(self, val_imm32 & 0xFF);
                    Printer_putStr(self, "')");
                } break;
            case BCTypeEnum_i8:
                {
                    Printer_PutStr(self, "Imm32("); 
                    Printer_PutU32(self, val_imm32);
                    Printer_PutStr(self, " ) // i8");
                    Printer_PutNl(self);
                } break;
            case BCTypeEnum_u32:
                {
                    Printer_PutStr(self, "Imm32(");
                    Printer_PutU32(self, val_imm32);
                    Printer_PutStr(self, " )");
                } break;
            case BCTypeEnum_i32:
                {
                    Printer_PutStr(self, "Imm32(");
                    Printer_PutI32(self, val_imm32);
                    Printer_PutStr(self, ", true)");
                } break;
            case BCTypeEnum_i64:
                {
                    Printer_PutStr(self, "Imm64("); 
                    Printer_PutI64(self, val->imm64.imm64);
                    Printer_PutStr(self, "LL, true)");
                } break;
            case BCTypeEnum_u64:
                {
                    Printer_PutStr(self, "Imm64("); 
                    Printer_PutU64(self, val->imm64.imm64);
                    Printer_PutStr(self, "ULL << 32 , true)");
                } break;
            case BCTypeEnum_f23:
                {
                    Printer_PutStr(self, "Imm23f("); 
                    Printer_PrintFloat(self, *cast(float*)&val_imm32);
                    Printer_PutStr(self, ")");
                } break;
            case BCTypeEnum_f52:
                {
                    Printer_PutStr(self, "Imm52f(");
                    Printer_PrintDouble(self, *cast(double*)&val->imm64.imm64);
                    Printer_PutStr(self, ")");
                } break;
            case BCTypeEnum_Null:
                {
                    Printer_PutStr(self, "Imm32(0/*null*/)");
                } break;
            case BCTypeEnum_Array:
                {
                    Printer_PutStr(self, "Imm32("); 
                    Printer_PutU32(self, val_imm32);
                    Printer_PutStr(self, " /*Array*/)");
                } break;
            default :
                {
#define CatStr(VAR, STR) \
    for(char c, *p = STR; (c = *p); p++) \
        *(VAR)++ = c;

#define LogError(STR)

                    //assert(0, "Unexpected Immediate of Type");
                    char errorBuffer[256];
                    char* msg = errorBuffer;
     
                    CatStr(msg, "Unexpected Immediate of Type");
                    CatStr(msg, BCTypeEnum_toChars(&type.type));
                    *msg = '\0';
                    LogError(errorBuffer);
                }
            }
        } break;

    case BCValueType_StackValue:
        {
            if (val->temporaryIndex)
            {
                Printer_PutStr(self,"tmp"); 
                Printer_PutU32(self, val->temporaryIndex);
                Printer_PutStr(self, self->functionSuffix);
            }
            else
            {
                Printer_PutStr(self, "StackAddr(");
                Printer_PutU32(self, val->stackAddr.addr);
                Printer_PutStr(self, "), ");
                Printer_PrintType(self, &type);
            }
        } break;

    case BCValueType_Local :
        {
            if (name)
            {
                Printer_PutStr(self, name);
            }
            else
            {
                Printer_PutStr(self, "local");
                Printer_PutU32(self, val->localIndex);
            }
        } break;
    case BCValueType_Temporary:
        {
            Printer_PutStr(self, "tmp");
            Printer_PutU32(self, val->temporaryIndex);
        } break;
    case BCValueType_Parameter:
        {
            if (name)
            {
                Printer_PutStr(self, name);
            }
            else
            {
                Printer_PutStr(self, "p");
                Printer_PutU32(self, val->parameterIndex);
            }
        } break;
    case BCValueType_Error:
    case BCValueType_ErrorWithMessage:
        {
            Printer_PutStr(self, "imm32("); 
            Printer_PutU32(self, val_imm32);
            Printer_PutStr(self, ")");
            
            if (val_imm32)
            {
                int closeMultilineCommment = 0;
                assert(self->ErrorInfoCount <= val_imm32);
                ErrorInfo* eInfo = self->ErrorInfos + (val_imm32 - 1);
                
                
                if (eInfo->msg && eInfo->msg[0] != '\0')
                {
                    closeMultilineCommment = 1;
                    Printer_PutStr(self, "/*");
                    Printer_PutStr(self, " \"");
                    Printer_PutStr(self, eInfo->msg);
                    Printer_PutStr(self, "\" ");
                }
                else
                {
                    Printer_PutStr(self, "// ");
                }
                

                for(int i = 0; i < eInfo->valueCount; i++)
                {
                    Printer_PrintBCValue(self, eInfo->values + i);
                    if (i != (eInfo->valueCount - 1))
                        Printer_PutStr(self, ", ");
                }
                if (closeMultilineCommment)
                    Printer_PutStr(self, "*/");
            }
        } break;
    case BCValueType_Unknown:
        {
            return "BCValue.init";
        } break;
    default:
        {
            char errorBuffer[256];
            char* msg = errorBuffer;
            
            CatStr(msg, "Printing for ");
            CatStr(msg, BCValueType_toChars(&val->vType));
            CatStr(msg, " unimplemented ");
            *msg = '\0';
            
            LogError(errorBuffer);
        }
    }

    Printer_PutStr(self, ")");
}
static inline void Printer_StreamToFile()
{

}

static inline void Printer_Op3(Printer* self,
    const BCValue *result, const BCValue *lhs, const BCValue *rhs
  , const char* inst)
{

}

static inline void Printer_Op2(Printer* self,
    const BCValue *lhs, const BCValue *rhs
  , const char* inst)
{

}

static inline void Printer_Op1(Printer* self,
    const BCValue *lhs
  , const char* inst)
{

}

static inline void Printer_Op0(Printer* self
  , const char* inst)
{

}

#define PR_OP3(OP) \
    static inline void Printer_##OP(Printer* self, BCValue *result, const BCValue* lhs, const BCValue* rhs) \
    { Printer_Op3(self, result, lhs, rhs, #OP); }

PR_OP3(Gt3)
PR_OP3(Ugt3)
PR_OP3(Ge3)
PR_OP3(Uge3)
PR_OP3(Lt3)
PR_OP3(Ult3)
PR_OP3(Le3)
PR_OP3(Ule3)
PR_OP3(Eq3)
PR_OP3(Neq3)

PR_OP3(Add3)
PR_OP3(Sub3)
PR_OP3(Mul3)
PR_OP3(Div3)
PR_OP3(Udiv3)
PR_OP3(Mod3)
PR_OP3(Umod3)
PR_OP3(Or3)
PR_OP3(Xor3)
PR_OP3(Rsh3)
PR_OP3(Lsh3)
PR_OP3(And3)

#define PR_OP2(OP) \
    static inline void Printer_##OP(Printer* self, \
        const BCValue* lhs, const BCValue* rhs) \
    { Printer_Op2(self, lhs, rhs, #OP); }

#define PR_OP1(OP) \
    static inline void Printer_##OP(Printer* self, \
        const BCValue* lhs) \
    { Printer_Op1(self, lhs, #OP); }

#define PR_OP0(OP) \
    static inline void Printer_##OP(Printer* self) \
    { Printer_Op0(self, #OP); }

PR_OP2(Store8)
PR_OP2(Store16)
PR_OP2(Store32)
PR_OP2(Store64)

PR_OP2(Load8)
PR_OP2(Load16)
PR_OP2(Load32)
PR_OP2(Load64)

static inline void Printer_Initialize(Printer* self, uint32_t n_args, ...)
{
}

static inline void Printer_InitializeV(Printer* self, uint32_t n_args, va_list args)
{
}

PR_OP0(Finalize)

static inline uint32_t Printer_beginFunction(Printer* self, uint32_t fnId, const void* fd)
{
}

static inline void* Printer_endFunction(Printer* self, uint32_t fnIdx)
{
}

static inline BCValue Printer_genTemporary(Printer* self, BCType bct)
{
}

PR_OP1(destroyTemporary)

static inline BCValue Printer_genLocal(Printer* self, BCType bct, const char* name)
{
}

static inline BCValue Printer_genParameter(Printer* self, BCType bct, const char* name)
{
}

PR_OP1(emitFlag)

PR_OP2(Alloc);
PR_OP2(Assert);
PR_OP3(MemCpy);

static inline void Printer_File(Printer* self, const char* filename)
{
}

static inline void Printer_Line(Printer* self, uint32_t line)
{
}

static inline void Printer_Comment(Printer* self, const char* comment)
{
}

static inline void Printer_Prt(Printer* self, const BCValue* value, bool isString)
{
}

PR_OP2(Set);
PR_OP2(Not);

static inline void Printer_LoadFramePointer(Printer* self, BCValue *result, const int32_t offset)
{
}

static inline void Printer_Call(Printer* self, BCValue *result, const BCValue* fn, const BCValue* args, uint32_t n_args)
{
}

PR_OP1(genLabel)

static inline void Printer_Jmp(Printer* self, BCLabel target)
{
}

PR_OP1(beginJmp)

static inline void Printer_endJmp(Printer* self, BCAddr atIp, BCLabel target)
{
}

static inline CndJmpBegin Printer_beginCndJmp(Printer* self, const BCValue* cond, _Bool ifTrue)
{
}

static inline void Printer_endCndJmp(Printer* self, CndJmpBegin jmp, BCLabel target)
{
}

PR_OP1(Throw)
PR_OP1(PushCatch)
PR_OP1(PopCatch)
PR_OP1(Ret)

PR_OP2(IToF32)
PR_OP2(IToF64)
PR_OP2(F32ToI)
PR_OP2(F64ToI)
PR_OP2(F32ToF64)
PR_OP2(F64ToF32)

PR_OP3(Memcmp)
PR_OP3(Realloc)

static inline BCValue Printer_run(Printer* self, uint32_t fnIdx, const BCValue* args, uint32_t n_args)
{
}

static inline void Printer_destroy_instance(Printer* self)
{
    free(self);
}

static inline void Printer_new_instance(Printer** resultP)
{
    Printer* result =  malloc(sizeof(Printer));

    const uint32_t initialSize = 8192 * 8;

    result->Buffer = malloc(initialSize);
    result->BufferSize = 0;
    result->BufferCapacity = initialSize;

    *resultP = result;
}


const BackendInterface Printer_interface = {
    .name = "Printer",

    .Initialize = (Initialize_t) Printer_Initialize,
    .InitializeV = (InitializeV_t) Printer_InitializeV,
    .Finalize = (Finalize_t) Printer_Finalize,
    .beginFunction = (beginFunction_t) Printer_beginFunction,
    .endFunction = (endFunction_t) Printer_endFunction,
    .genTemporary = (genTemporary_t) Printer_genTemporary,
    .destroyTemporary = (destroyTemporary_t) Printer_destroyTemporary,
    .genLocal = (genLocal_t) Printer_genLocal,
    .genParameter = (genParameter_t) Printer_genParameter,
    .emitFlag = (emitFlag_t) Printer_emitFlag,
    .Alloc = (Alloc_t) Printer_Alloc,
    .Assert = (Assert_t) Printer_Assert,
    .MemCpy = (MemCpy_t) Printer_MemCpy,
    .File = (File_t) Printer_File,
    .Line = (Line_t) Printer_Line,
    .Comment = (Comment_t) Printer_Comment,
    .Prt = (Prt_t) Printer_Prt,
    .Set = (Set_t) Printer_Set,
    .Ult3 = (Ult3_t) Printer_Ult3,
    .Ule3 = (Ule3_t) Printer_Ule3,
    .Lt3 = (Lt3_t) Printer_Lt3,
    .Le3 = (Le3_t) Printer_Le3,
    .Ugt3 = (Ugt3_t) Printer_Ugt3,
    .Uge3 = (Uge3_t) Printer_Uge3,
    .Gt3 = (Gt3_t) Printer_Gt3,
    .Ge3 = (Ge3_t) Printer_Ge3,
    .Eq3 = (Eq3_t) Printer_Eq3,
    .Neq3 = (Neq3_t) Printer_Neq3,
    .Add3 = (Add3_t) Printer_Add3,
    .Sub3 = (Sub3_t) Printer_Sub3,
    .Mul3 = (Mul3_t) Printer_Mul3,
    .Div3 = (Div3_t) Printer_Div3,
    .Udiv3 = (Udiv3_t) Printer_Udiv3,
    .And3 = (And3_t) Printer_And3,
    .Or3 = (Or3_t) Printer_Or3,
    .Xor3 = (Xor3_t) Printer_Xor3,
    .Lsh3 = (Lsh3_t) Printer_Lsh3,
    .Rsh3 = (Rsh3_t) Printer_Rsh3,
    .Mod3 = (Mod3_t) Printer_Mod3,
    .Umod3 = (Umod3_t) Printer_Umod3,
    .Not = (Not_t) Printer_Not,
    .LoadFramePointer = (LoadFramePointer_t) Printer_LoadFramePointer,
    .Call = (Call_t) Printer_Call,
    .genLabel = (genLabel_t) Printer_genLabel,
    .Jmp = (Jmp_t) Printer_Jmp,
    .beginJmp = (beginJmp_t) Printer_beginJmp,
    .endJmp = (endJmp_t) Printer_endJmp,
    .beginCndJmp = (beginCndJmp_t) Printer_beginCndJmp,
    .endCndJmp = (endCndJmp_t) Printer_endCndJmp,
    .Load8 = (Load8_t) Printer_Load8,
    .Store8 = (Store8_t) Printer_Store8,
    .Load16 = (Load16_t) Printer_Load16,
    .Store16 = (Store16_t) Printer_Store16,
    .Load32 = (Load32_t) Printer_Load32,
    .Store32 = (Store32_t) Printer_Store32,
    .Load64 = (Load64_t) Printer_Load64,
    .Store64 = (Store64_t) Printer_Store64,
    .Throw = (Throw_t) Printer_Throw,
    .PushCatch = (PushCatch_t) Printer_PushCatch,
    .PopCatch = (PopCatch_t) Printer_PopCatch,
    .Ret = (Ret_t) Printer_Ret,
    .IToF32 = (IToF32_t) Printer_IToF32,
    .IToF64 = (IToF64_t) Printer_IToF64,
    .F32ToI = (F32ToI_t) Printer_F32ToI,
    .F64ToI = (F64ToI_t) Printer_F64ToI,
    .F32ToF64 = (F32ToF64_t) Printer_F32ToF64,
    .F64ToF32 = (F64ToF32_t) Printer_F64ToF32,
    .Memcmp = (Memcmp_t) Printer_Memcmp,
    .Realloc = (Realloc_t) Printer_Realloc,
    .run = (run_t) Printer_run,
    .destroy_instance = (destroy_instance_t) Printer_destroy_instance,
    .new_instance = (new_instance_t) Printer_new_instance
};
