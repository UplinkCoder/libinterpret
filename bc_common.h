#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#undef offsetof

#define offsetof(st, m) \
    ((size_t)((char *)&((st *)0)->m - (char *)0))

/// functions with index skipFn will be skipped
/// calling them is equivlent to an expensive nop
/// this is true for direct and indirect calls
const uint32_t skipFn = UINT32_MAX;

const uint32_t nodeFromName = UINT32_MAX - 1;
const uint32_t currentScope = UINT32_MAX - 2;

#define CONSTEXPR

typedef struct HeapAddr
{
    uint32_t addr;
} HeapAddr;

typedef struct StackAddr
{
    uint16_t addr;
} StackAddr;

typedef struct Imm32
{
    uint32_t imm32;
    bool signed_;
} Imm32;

typedef struct Imm64
{
    uint64_t imm64;
    bool signed_;
} Imm64;

typedef enum BCTypeEnum
{
    BCTypeEnum_Undef,

    BCTypeEnum_Null,
    BCTypeEnum_Void,

    BCTypeEnum_c8,
    BCTypeEnum_c16,
    BCTypeEnum_c32,

    /// signed by default
    BCTypeEnum_i8,
    /// DITTO
    BCTypeEnum_i16,
    /// DITTO
    BCTypeEnum_i32,
    /// DITTO
    BCTypeEnum_i64,

    BCTypeEnum_u8,
    BCTypeEnum_u16,
    BCTypeEnum_u32,
    BCTypeEnum_u64,

    BCTypeEnum_f23, /// 32  bit float mantissa has 23 bit
    BCTypeEnum_f52, /// 64  bit float mantissa has 52 bit
    BCTypeEnum_f106, /// 128 bit float mantissa has 106 bit (52+52)

    BCTypeEnum_string8,
    BCTypeEnum_string16,
    BCTypeEnum_string32,

    BCTypeEnum_Function, // synonymous to i32
    BCTypeEnum_Delegate, // synonymous to {i32, i32}

    //  everything below here is not used by the bc layer.
    BCTypeEnum_Array,
    BCTypeEnum_AArray,
    BCTypeEnum_Struct,
    BCTypeEnum_Class,
    BCTypeEnum_Ptr,
    BCTypeEnum_Slice,
} BCTypeEnum;

const char* BCTypeEnum_toChars(BCTypeEnum* self)
{
    switch(*self)
    {
    case BCTypeEnum_Undef:
        return "BCTypeEnum_Undef";

    case BCTypeEnum_Null:
        return "BCTypeEnum_Null";

    case BCTypeEnum_Void:
        return "BCTypeEnum_Void";


    case BCTypeEnum_c8:
        return "BCTypeEnum_c8";

    case BCTypeEnum_c16:
        return "BCTypeEnum_c16";

    case BCTypeEnum_c32:
        return "BCTypeEnum_c32";


    case BCTypeEnum_i8:
        return "BCTypeEnum_i8";

    case BCTypeEnum_i16:
        return "BCTypeEnum_i16";

    case BCTypeEnum_i32:
        return "BCTypeEnum_i32";

    case BCTypeEnum_i64:
        return "BCTypeEnum_i64";


    case BCTypeEnum_u8:
        return "BCTypeEnum_u8";

    case BCTypeEnum_u16:
        return "BCTypeEnum_u16";

    case BCTypeEnum_u32:
        return "BCTypeEnum_u32";

    case BCTypeEnum_u64:
        return "BCTypeEnum_u64";


    case BCTypeEnum_f23:
        return "BCTypeEnum_f23";

    case BCTypeEnum_f52:
        return "BCTypeEnum_f52";


    case BCTypeEnum_f106 :
        return "BCTypeEnum_f106";

    case BCTypeEnum_string8:
        return "BCTypeEnum_string8";

    case BCTypeEnum_string16:
        return "BCTypeEnum_string16";

    case BCTypeEnum_string32:
        return "BCTypeEnum_string32";

    case BCTypeEnum_Function:
        return "BCTypeEnum_Function";

    case BCTypeEnum_Delegate:
        return "BCTypeEnum_Delegate";

    case BCTypeEnum_Array:
        return "BCTypeEnum_Array";

    case BCTypeEnum_AArray:
        return "BCTypeEnum_AArray";

    case BCTypeEnum_Struct:
        return "BCTypeEnum_Struct";

    case BCTypeEnum_Class:
        return "BCTypeEnum_Class";

    case BCTypeEnum_Ptr:
        return "BCTypeEnum_Ptr";

    case BCTypeEnum_Slice:
        return "BCTypeEnum_Slice";

    }
}

typedef struct BCAddr
{
    uint32_t addr;

    //    T opCast(T : bool)()
    //    {
    //        return addr != 0;
    //    }
} BCAddr;

typedef enum BCTypeFlags
{
    BCTypeFlags_None = 0,
    BCTypeFlags_Const = 1 << 0,
} BCTypeFlags;

const char* BCTypeFlags_toChars(BCTypeFlags* self)
{
    if (*self == 0)
        return "None";

    if ((*self & (1 << 0)) != 0)
        return "Const";
}

#define STRUCT_NAME BCType
typedef struct BCType
{
    BCTypeEnum type;
    uint32_t typeIndex;

    // additional information
    BCTypeFlags flags;
} STRUCT_NAME;

const char* BCType_toChars(BCType* self)
{
    char format_buffer[1024];

    int sz = sprintf(format_buffer, "BCType\n\t{type: %s, typeIndex: %u, flags: %s}\n"
            , BCTypeEnum_toChars(&self->type)
            , self->typeIndex
            , BCTypeFlags_toChars(&self->flags));

    assert(sz < 1024);

    char* result = (char*) malloc (sz);
    memcpy(result, format_buffer, sz);

    return result;
}
#undef STRUCT_NAME

typedef enum BCValueType
{
    BCValueType_Unknown = 0,

    BCValueType_Temporary = 1,
    BCValueType_Parameter = 2,
    BCValueType_Local = 3,

    BCValueType_StackValue = 1 << 3,
    BCValueType_Immediate = 2 << 3,
    BCValueType_HeapValue = 3 << 3,

    BCValueType_LastCond = 0xFB,
    BCValueType_Bailout = 0xFC,
    BCValueType_Exception = 0xFD,
    BCValueType_ErrorWithMessage = 0xFE,
    BCValueType_Error = 0xFF, //Pinned = 0x80,
    /// Pinned values can be returned
    /// And should be kept in the compacted heap

} BCValueType;

const char* BCValueType_toChars(const BCValueType* vTypePtr)
{
    const BCValueType vType = *vTypePtr;

    switch(vType)
    {
        case BCValueType_Unknown:
            return "BCValueType_Unknown";

        case BCValueType_Temporary:
            return "BCValueType_Temporary";

        case BCValueType_Parameter:
            return "BCValueType_Parameter";

        case BCValueType_Local:
            return "BCValueType_Local";


        case BCValueType_StackValue:
            return "BCValueType_StackValue";

        case BCValueType_Immediate:
            return "BCValueType_Immediate";

        case BCValueType_HeapValue:
            return "BCValueType_HeapValue";


        case BCValueType_LastCond:
            return "BCValueType_LastCond";

        case BCValueType_Bailout:
            return "BCValueType_Bailout";

        case BCValueType_Exception:
            return "BCValueType_Exception";

        case BCValueType_ErrorWithMessage:
            return "BCValueType_ErrorWithMessage";

        case BCValueType_Error:
            return "BCValueType_Error";
    }

    assert(0);
}

typedef struct BCHeapRef
{
#define STRUCT_NAME BCHeapRef
    BCValueType vType;
    union
    {
        uint16_t tmpIndex;
        uint16_t localIndex;
    };

    union
    {
        HeapAddr heapAddr;
        StackAddr stackAddr;
        Imm32 imm32;
    };

    const char* name;
#ifdef __cplusplus
    bool operator bool() const pure
    {
        // the check for Undef is a workaround
        // consider removing it when everything works correctly.

        return this.vType != vType.Unknown;
    }

    STRUCT_NAME(const BCValue that)
    {
        switch (that.vType)
        {
        case BCValueType.StackValue, BCValueType.Parameter:
        case BCValueType.Temporary:
            stackAddr = that.stackAddr;
            tmpIndex = that.tmpIndex;
            break;

        case BCValueType.Local:
            stackAddr = that.stackAddr;
            localIndex = that.localIndex;
            this.name = that.name;
            break;

        case BCValueType.HeapValue:
            heapAddr = that.heapAddr;
            break;

        case BCValueType.Immediate:
            imm32 = that.imm32;
            break;

        default:
            printf("vType unsupported: %s\n", BCValueType_toChars(that.vType));
            assert(0);
        }
        vType = that.vType;
    }
#endif
} STRUCT_NAME;
#undef STRUCT_NAME

typedef struct BCValue
{
#define STRUCT_NAME BCValue
    BCType type;
    BCValueType vType;
    bool couldBeVoid; // = false

    BCHeapRef heapRef;
    const char* name;

    union
    {
        int8_t paramIndex;
        uint16_t tmpIndex;
        uint16_t localIndex;
    };

    union
    {
        StackAddr stackAddr;
        HeapAddr heapAddr;
        Imm32 imm32;
        Imm64 imm64;
/* for now we represent floats in imm32 or imm64 respectively
        Imm23f imm23f;
        Imm52f imm52f;
*/
        // instead of void*
        void* voidStar;
    };
    //TODO PERF minor: use a 32bit value for heapRef;
    
#ifdef _cplusplus
    uint toUint() const pure
    {
        switch (this.vType)
        {
        case BCValueType.Parameter, BCValueType.Temporary,
                BCValueType.StackValue:
                return stackAddr;
        case BCValueType.HeapValue:
            return heapAddr;
        case BCValueType.Immediate:
            return imm32;
        case BCValueType.Unknown:
            return this.imm32;
        default:
            {
                printf("toUint not implemented for %s\n", BCValueType_toChars(vType))
                assert(0);
            }
        }

    }

    const char* toChars() const
    {
        const char* result = "vType: ";
/*
        result ~= enumToString(vType);
        result ~= "\tType: "; 
        result ~= type.toString;
        result ~= "\n\tValue: ";
        result ~= valueToString;
        result ~= "\n";
        if (name)
            result ~= "\tname: " ~ name ~ "\n";
*/
        return result;
    }

    string valueToString()
    {
        switch (vType)
        {
        case BCValueType.Local : goto case;
        case BCValueType.Parameter, BCValueType.Temporary,
                BCValueType.StackValue:
                return "stackAddr: " ~ itos(stackAddr);
        case BCValueType.HeapValue:
            return "heapAddr: " ~ itos(heapAddr);
        case BCValueType.Immediate:
            return "imm: " ~ (type.type == BCTypeEnum.i64 || type.type == BCTypeEnum.f52
                    ? itos64(imm64) : itos(imm32));
        default:
            return "unknown value format";
        }
    }

    bool operator bool()
    {
        // the check for Undef is a workaround
        // consider removing it when everything works correctly.

        return this.vType != vType.Unknown && this.type.type != BCTypeEnum.Undef;
    }

    bool operator == (const BCValue* rhs) const
    {
        BCTypeEnum commonType = commonTypeEnum(this.type.type, rhs.type.type);

        if (this->vType == rhs->vType)
        {
            final switch (this->vType)
            {
            case BCValueType.StackValue,
                    BCValueType.Parameter, BCValueType.Local:
                    return this.stackAddr == rhs.stackAddr;
            case BCValueType.Temporary:
                return tmpIndex == rhs.tmpIndex;
            case BCValueType.Immediate:
                switch (commonType)
                {
                case BCTypeEnum.i32, BCTypeEnum.u32:
                    {
                        return imm32.imm32 == rhs.imm32.imm32;
                    }
                case BCTypeEnum.i64, BCTypeEnum.u64:
                    {
                        return imm64.imm64 == rhs.imm64.imm64;
                    }

                default:
                    assert(0, "No comperasion for immediate");
                }
            case BCValueType.HeapValue:
                return this.heapAddr == rhs.heapAddr;

            case BCValueType.Unknown, BCValueType.Bailout:
                return false;
            case BCValueType.Error, BCValueType.ErrorWithMessage,
                        BCValueType.Exception:
                return false;
            case BCValueType.LastCond:
                return true;
            }

        }

        return false;
    }

    STRUCT_NAME(const Imm32 imm32) pure
    {
        this.type.type = imm32.signed ? BCTypeEnum.i32 : BCTypeEnum.u32;
        this.vType = BCValueType.Immediate;
        this.imm32.imm32 = imm32.imm32;
    }

    STRUCT_NAME(const Imm64 imm64) pure
    {
        this.type.type = imm64.signed ? BCTypeEnum.i64 : BCTypeEnum.u64;
        this.vType = BCValueType.Immediate;
        this.imm64 = imm64;
    }

    STRUCT_NAME(const Imm23f imm23f) pure @trusted
    {
        this.type.type = BCTypeEnum.f23;
        this.vType = BCValueType.Immediate;
        this.imm32.imm32 = *cast(uint*)&imm23f;
    }

    STRUCT_NAME(const Imm52f imm52f) pure @trusted
    {
        this.type.type = BCTypeEnum.f52;
        this.vType = BCValueType.Immediate;
        this.imm64.imm64 = *cast(uint64_t*)&imm52f;
    }

    STRUCT_NAME(const BCParameter param) pure
    {
        this.vType = BCValueType.Parameter;
        this.type = param.type;
        this.paramIndex = param.idx;
        this.stackAddr = param.pOffset;
        this.name = param.name;
    }

    STRUCT_NAME(const StackAddr sp, const BCType type, const ushort tmpIndex = 0) pure
    {
        this.vType = BCValueType.StackValue;
        this.stackAddr = sp;
        this.type = type;
        this.tmpIndex = tmpIndex;
    }

    STRUCT_NAME(const StackAddr sp, const BCType type, const ushort localIndex, const char* name) pure
    {
        this.vType = BCValueType.Local;
        this.stackAddr = sp;
        this.type = type;
        this.localIndex = localIndex;
        this.name = name;
    }

    STRUCT_NAME(const void* base, const short addr, const BCType type) pure
    {
        this.vType = BCValueType.StackValue;
        this.stackAddr = StackAddr(addr);
        this.type = type;
    }

    STRUCT_NAME(const HeapAddr addr, const BCType type = i32Type) pure
    {
        this.vType = BCValueType.HeapValue;
        this.type = type;
        this.heapAddr = addr;
    }

    STRUCT_NAME(const BCHeapRef heapRef) pure
    {
        this.vType = heapRef.vType;
        switch (vType)
        {
        case BCValueType.StackValue, BCValueType.Parameter:
            stackAddr = heapRef.stackAddr;
            tmpIndex = heapRef.tmpIndex;
            break;
        case BCValueType.Local:
            stackAddr = heapRef.stackAddr;
            tmpIndex = heapRef.localIndex;
            name = heapRef.name;
            break;

        case BCValueType.Temporary:
            stackAddr = heapRef.stackAddr;
            tmpIndex = heapRef.tmpIndex;
            break;

        case BCValueType.HeapValue:
            heapAddr = heapRef.heapAddr;
            break;

        case BCValueType.Immediate:
            imm32 = heapRef.imm32;
            break;

        default:
            assert(0, "vType unsupported: " ~ enumToString(vType));
        }
    }
#endif
} STRUCT_NAME;

void BCValue_Init(BCValue* self)
{
    self->couldBeVoid = false;
}

#undef STRUCT_NAME

struct CndJmpBegin
{
    BCAddr at;
    BCValue cond;
    bool ifTrue;
};

CONSTEXPR const uint32_t align4(const uint32_t val)
{
    return ((val + 3) & ~3);
}

static inline void storeu32(uint8_t* ptr, const uint32_t v32)
{
    ptr[0] = (v32 >> 0)  & 0xFF;
    ptr[1] = (v32 >> 8)  & 0xFF;
    ptr[2] = (v32 >> 16) & 0xFF;
    ptr[3] = (v32 >> 24) & 0xFF;
}

static inline uint32_t loadu32(const uint8_t* ptr)
{
    uint32_t v32 = (ptr[0] << 0)
             | (ptr[1] << 8)
             | (ptr[2] << 16)
             | (ptr[3] << 24);
    return v32;
}


CONSTEXPR static inline uint32_t align16(const uint32_t val)
{
    return ((val + 15) & ~15);
}

const uint32_t basicTypeSize(const BCTypeEnum bct)
{
    switch (bct)
    {
    case BCTypeEnum_Undef:
        {
            assert(!"This is not supposed to happen");
            return 0;
        }
    case BCTypeEnum_c8:
    case BCTypeEnum_i8:
    case BCTypeEnum_u8:
        {
            return 1;
        }
    case BCTypeEnum_c16:
    case BCTypeEnum_i16:
    case BCTypeEnum_u16:
        {
            return 2;
        }
    case BCTypeEnum_c32:
    case BCTypeEnum_i32:
    case BCTypeEnum_u32:
    case BCTypeEnum_f23:
        {
            return 4;
        }
    case BCTypeEnum_i64:
    case BCTypeEnum_u64:
    case BCTypeEnum_f52:
        {
            return 8;
        }
    case BCTypeEnum_f106:
        {
            return 16;
        }

    case BCTypeEnum_Function:
    case BCTypeEnum_Null:
        {
            return 4;
        }
    case BCTypeEnum_Delegate:
        {
            return 8;
        }
    case BCTypeEnum_Ptr:
    {
            assert(!"Ptr is not supposed to be a basicType anymore");
            return 0;
    }

    case BCTypeEnum_string8:
    case BCTypeEnum_string16:
    case BCTypeEnum_string32:
    {
        //FIXME actually strings don't have a basicTypeSize as is
        return 16;
    }


    case BCTypeEnum_Void:
    case BCTypeEnum_Array:
    case BCTypeEnum_Slice:
    case BCTypeEnum_Struct:
    case BCTypeEnum_Class:
    case BCTypeEnum_AArray:
    {
        return 0;
    }

    default : assert(0);
    }
}


const uint32_t adjustmentMask(BCTypeEnum t)
{
    uint32_t mask = 0;
    int typeSize = basicTypeSize(t);

    if (typeSize == 1)
        mask = 0xFF;
    else if (typeSize == 2)
        mask = 0xFFFF;

    return mask;
}


const uint32_t fastLog10(const uint32_t val)
{
    return (val < 10) ? 0 : (val < 100) ? 1 : (val < 1000) ? 2 : (val < 10000) ? 3
        : (val < 100000) ? 4 : (val < 1000000) ? 5 : (val < 10000000) ? 6
        : (val < 100000000) ? 7 : (val < 1000000000) ? 8 : 9;
}

/*@unique*/
/*
static immutable fastPow10tbl = [
    1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000,
];

const char* itos(const uint val) pure @trusted nothrow
{
    char* result = new char[](length);
    immutable length = fastLog10(val) + 1;

    foreach (i; 0 .. length)
    {
        immutable _val = val / fastPow10tbl[i];
        result[length - i - 1] = cast(char)((_val % 10) + '0');
    }

    return result;
}

string itos64(const uint64_t val)
{
    if (val <= UINT32_MAX)
        return itos(val & UINT32_MAX);

    uint lw = val & UINT32_MAX;
    uint hi = val >> 32;

    auto lwString = itos(lw);
    auto hiString = itos(hi);

    return cast(string) "((" ~ hiString ~ "UL << 32)" ~ "|" ~ lwString ~ ")";
}

string sitos(const int val) pure @trusted nothrow
{
    int sign = (val < 0) ? 1 : 0;
    uint abs_val = (val < 0) ? -val : val;

    immutable length = fastLog10(abs_val) + 1;
    char[] result;
    result.length = length + sign;

    foreach (i; 0 .. length)
    {
        immutable _val = abs_val / fastPow10tbl[i];
        result[length - i - !sign] = cast(char)((_val % 10) + '0');
    }

    if (sign)
    {
        result[0] = '-';
    }

    return cast(string) result;
}

string floatToString(float f)
{
    return fpconv_dtoa(f) ~ "f";
}

string doubleToString(double d)
{
   return fpconv_dtoa(d);
}
*/

bool anyOf(BCTypeEnum type, const BCTypeEnum acceptedTypes[], uint32_t n_types)
{
    bool result = false;

    for(int i = 0; i < n_types; i++)
    {
        if (type == acceptedTypes[i])
        {
            result = true;
            break;
        }
    }

    return result;
}

bool isFloat(BCType bct)
{
    return bct.type == BCTypeEnum_f23 || bct.type == BCTypeEnum_f52;
}

bool isBasicBCType(BCType bct)
{
    return !(bct.type == BCTypeEnum_Struct || bct.type == BCTypeEnum_Array || bct.type == BCTypeEnum_Class
            || bct.type == BCTypeEnum_Slice || bct.type == BCTypeEnum_Undef || bct.type == BCTypeEnum_Ptr
            || bct.type == BCTypeEnum_AArray);
}

bool isStackValueOrParameter(BCValue val)
{
    return (val.vType == BCValueType_StackValue || val.vType == BCValueType_Parameter || val.vType == BCValueType_Local);
}

/*
string typeFlagsToString(BCTypeFlags flags) pure @safe
{
    string result;

    if (!flags)
    {
        result = "None";
        goto Lret;
    }

    if (flags & BCTypeFlags_Const)
    {
        result ~= "Const|";
    }

    // trim last |
    result = result[0 .. $-1];

Lret:
    return result;
}
*/

struct RegStatusList
{
#define STRUCT_NAME RegStatusList
    const int NREGS;
    uint32_t freeBitfield;
    uint32_t unusedBitfield;
    uint32_t dirtyBitfield;

#ifdef __cplusplus
    STRUCT_NAME(int NREGS) pure
    {
        assert(NREGS < 32  /*"extending freeBitField is not yet done"*/);
        this.NREGS = NREGS;

        unusedBitfield = 0;
        dirtyBitfield = 0;
        freeBitfield = ((1 << NREGS) - 1);
    }
    
    inline uint32_t nextFree()
    {
        uint result = 0;
        if (freeBitfield != 0)
            result = bsf(freeBitfield) + 1;
        return result;
    }
    
    inline uint32_t nextUnused()
    {
        uint result = 0;
        if (unusedBitfield)
            result = bsf(unusedBitfield) + 1;
        return result;
    }
    
    inline uint32_t nextDirty()
    {
        uint result = 0;
        if (dirtyBitfield)
            result = bsf(dirtyBitfield) + 1;
        return result;
    }
    
    inline uint32_t n_free()
    {
        assert(popcnt(freeBitfield) <= NREGS);
        return popcnt(freeBitfield);
    }
    
    /// mark register as unoccupied
    inline void markFree(int regIdx)
    {
        assert(regIdx && regIdx <= NREGS);
        freeBitfield |= (1 << (regIdx - 1));
    }
    
    /// mark register as eviction canidate
    inline void markUnused(int regIdx)
    {
        assert(regIdx && regIdx <= NREGS);
        unusedBitfield |= (1 << (regIdx - 1));
    }
    
    /// mark register as used
    inline void markUsed(int regIdx)
    {
        assert(regIdx && regIdx <= NREGS);
        freeBitfield &= ~(1 << (regIdx - 1));
        unusedBitfield &= ~(1 << (regIdx - 1));
    }
    
    inline void markClean(int regIdx)
    {
        assert(regIdx && regIdx <= NREGS);
        dirtyBitfield &= ~(1 << (regIdx - 1));
    }
    
    inline void markDirty(int regIdx)
    {
        assert(regIdx && regIdx <= NREGS);
        dirtyBitfield |= (1 << (regIdx - 1));
    }

    inline bool isDirty(int regIdx)
    {
        assert(regIdx && regIdx <= NREGS);
        return (dirtyBitfield & (1 << (regIdx - 1))) != 0;
    }

    inline bool isUnused(int regIdx)
    {
        assert(regIdx && regIdx <= NREGS);
        return (unusedBitfield & (1 << (regIdx - 1))) != 0;
    }

    inline bool isFree(int regIdx)
    {
        assert(regIdx && regIdx <= NREGS);
        return (freeBitfield & (1 << (regIdx - 1))) != 0;
    }
#endif
} STRUCT_NAME;
#undef STRUCT_NAME
/*
static assert(()
    {
        RegStatusList!16 f;

        assert(f.n_free == 16);
        assert(f.nextDirty() == 0);
        assert(f.nextUnused() == 0);
        auto nextReg = f.nextFree();
        f.markUsed(nextReg);
        assert(f.n_free == 15);
        f.markDirty(nextReg);
        assert(f.nextDirty() == nextReg);
        f.markClean(nextReg);
        assert(f.nextDirty() == 0);
        foreach(r; 1 .. 17)
            f.markUnused(r);
        foreach(r; 0 .. 16)
        {
            auto nextUnused = f.nextUnused();
            f.markUsed(nextUnused);
        }
        assert(f.nextUnused() == 0);

        RegStatusList!0 d = RegStatusList!0(2);
        assert(d.n_free() == 2);

        return true;
    }
());
*/

const uint8_t toParamCode(const BCValue val)
{
    if (val.type.type == BCTypeEnum_i32)
        return 0x0;
    /*else if (val.type.type)
        return 0b0001;*/
    else if (val.type.type == BCTypeEnum_Struct)
        return 0x2;
    else if (val.type.type == BCTypeEnum_Slice
            || val.type.type == BCTypeEnum_Array || val.type.type == BCTypeEnum_string8)
        return 0x3;
    else
        assert(!"ParameterType unsupported");
}

const int BCHeap_initHeapMax = (1 << 15);

typedef struct BCHeap
{
    uint32_t heapMax;// = initHeapMax;
    uint32_t heapSize;// = 4;
    uint8_t* heapData;
} BCHeap;

const int heapSizeOffset = offsetof(BCHeap, heapSize);
const int heapMaxOffset = offsetof(BCHeap, heapMax);
const int heapDataOffset = offsetof(BCHeap, heapData);

const int heapDataPtrOffset    = offsetof(BCHeap, heapData) + sizeof(uint8_t*);
const int heapDataLengthOffset = offsetof(BCHeap, heapData) + sizeof(uint8_t*) + sizeof(void*);

typedef struct BCLabel
{
    BCAddr addr;
} BCLabel;


typedef struct BCLocal
{
    uint16_t idx;
    BCType type;
    StackAddr addr;
    const char* name;
} BCLocal;

typedef struct BCParameter
{
    uint8_t idx;
    BCType type;
    StackAddr pOffset;
    const char* name;
} BCParameter;


#define imm32(VALUE) imm32_((VALUE), false);

BCValue imm32_(uint32_t value, bool signed_)
{
    BCValue ret;

    ret.vType = BCValueType_Immediate;
    ret.type.type = signed_ ? BCTypeEnum_i32 : BCTypeEnum_u32;
    ret.type.typeIndex = 0;
    ret.type.flags = BCTypeFlags_None;
    ret.imm32.imm32 = value;

    ret.imm64.imm64 &= UINT32_MAX;
    return ret;
}

#define imm64(VALUE) imm64_((VALUE), false);

BCValue imm64_(uint64_t value, bool signed_)
{
    BCValue ret;

    ret.vType = BCValueType_Immediate;
    ret.type.type = signed_ ? BCTypeEnum_i64 : BCTypeEnum_u64;
    ret.type.typeIndex = 0;
    ret.type.flags = BCTypeFlags_None;
    ret.imm64.imm64 = value;
    return ret;
}

BCValue i32(BCValue val)
{
    val.type.type = BCTypeEnum_i32;
    return val;
}

BCValue u32(BCValue val)
{
    val.type.type = BCTypeEnum_u32;
    return val;
}

typedef struct Imm23f
{
    float imm23f;
} Imm23f;

typedef struct Imm52f
{
    double imm52f;
} Imm52f;

typedef struct BCBlock
{

    BCLabel begin;
    BCLabel end;
} BCBlock;

typedef struct BCBranch
{
    BCLabel ifTrue;
    BCLabel ifFalse;
} BCBranch;

/*
template BCGenFunction(T, alias fn)
{
    static assert(ensureIsBCGen!T && is(typeof(fn()) == T));
    BCValue[] params;

    static if (is(typeof(T.init.functionalize()) == string))
    {
        static immutable BCGenFunction = mixin(fn().functionalize);
    }
    else static if (is(typeof(T.init.interpret(typeof(T.init.byteCode), typeof(params).init)()) : int))
    {
        static immutable BCGenFunction = ((BCValue[] args,
                BCHeap* heapPtr) => fn().interpret(args, heapPtr));
    }
}
*/
/*
template ensureIsBCGen(BCGenT)
{
    static assert(is(typeof(BCGenT.beginFunction(uint.init)) == void),
            BCGenT.stringof ~ " is missing void beginFunction(uint)");
    static assert(is(typeof(BCGenT.endFunction())), BCGenT.stringof ~ " is missing endFunction()");
    static assert(is(typeof(BCGenT.Initialize()) == void),
            BCGenT.stringof ~ " is missing void Initialize()");
    static assert(is(typeof(BCGenT.Finalize()) == void),
            BCGenT.stringof ~ " is missing void Finalize()");
    static assert(is(typeof(BCGenT.genTemporary(BCType.init)) == BCValue),
            BCGenT.stringof ~ " is missing BCValue genTemporary(BCType bct)");
    static assert(is(typeof(BCGenT.genParameter(BCType.init, string.init)) == BCValue),
            BCGenT.stringof ~ " is missing BCValue genParameter(BCType bct, string name)");
    static assert(is(typeof(BCGenT.beginJmp()) == BCAddr),
            BCGenT.stringof ~ " is missing BCAddr beginJmp()");
    static assert(is(typeof(BCGenT.endJmp(BCAddr.init, BCLabel.init)) == void),
            BCGenT.stringof ~ " is missing void endJmp(BCAddr atIp, BCLabel target)");
    static assert(is(typeof(BCGenT.incSp()) == void), BCGenT.stringof ~ " is missing void incSp()");
    static assert(is(typeof(BCGenT.currSp()) == StackAddr),
            BCGenT.stringof ~ " is missing StackAddr currSp()");
    static assert(is(typeof(BCGenT.genLabel()) == BCLabel),
            BCGenT.stringof ~ " is missing BCLabel genLabel()");
    static assert(is(typeof(BCGenT.beginCndJmp(BCValue.init, bool.init)) == CndJmpBegin),
            BCGenT.stringof
            ~ " is missing CndJmpBegin beginCndJmp(BCValue cond = BCValue.init, bool ifTrue = false)");
    static assert(is(typeof(BCGenT.endCndJmp(CndJmpBegin.init, BCLabel.init)) == void),
            BCGenT.stringof ~ " is missing void endCndJmp(CndJmpBegin jmp, BCLabel target)");
    static assert(is(typeof(BCGenT.Jmp(BCLabel.init)) == void),
            BCGenT.stringof ~ " is missing void Jmp(BCLabel target)");
    static assert(is(typeof(BCGenT.emitFlg(BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void emitFlg(BCValue lhs)");
    static assert(is(typeof(BCGenT.Alloc(BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Alloc(BCValue heapPtr, BCValue size)");
    static assert(is(typeof(BCGenT.Assert(BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Assert(BCValue value, BCValue message)");
    static assert(is(typeof(BCGenT.Not(BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Not(BCValue result, BCValue val)");
    static assert(is(typeof(BCGenT.Set(BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Set(BCValue lhs, BCValue rhs)");

    static assert(is(typeof(BCGenT.Ult3(BCValue.init, BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Ult3(BCValue result, BCValue lhs, BCValue rhs)");
    static assert(is(typeof(BCGenT.Ugt3(BCValue.init, BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Ugt3(BCValue result, BCValue lhs, BCValue rhs)");

    static assert(is(typeof(BCGenT.Lt3(BCValue.init, BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Lt3(BCValue result, BCValue lhs, BCValue rhs)");
    static assert(is(typeof(BCGenT.Gt3(BCValue.init, BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Gt3(BCValue result, BCValue lhs, BCValue rhs)");

    static assert(is(typeof(BCGenT.Le3(BCValue.init, BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Le3(BCValue result, BCValue lhs, BCValue rhs)");
    static assert(is(typeof(BCGenT.Ge3(BCValue.init, BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Ge3(BCValue result, BCValue lhs, BCValue rhs)");

    static assert(is(typeof(BCGenT.Ule3(BCValue.init, BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Ule3(BCValue result, BCValue lhs, BCValue rhs)");
    static assert(is(typeof(BCGenT.Uge3(BCValue.init, BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Uge3(BCValue result, BCValue lhs, BCValue rhs)");

    static assert(is(typeof(BCGenT.Neq3(BCValue.init, BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Neq3(BCValue result, BCValue lhs, BCValue rhs)");
    static assert(is(typeof(BCGenT.Add3(BCValue.init, BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Add3(BCValue result, BCValue lhs, BCValue rhs)");
    static assert(is(typeof(BCGenT.Sub3(BCValue.init, BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Sub3(BCValue result, BCValue lhs, BCValue rhs)");
    static assert(is(typeof(BCGenT.Mul3(BCValue.init, BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Mul3(BCValue result, BCValue lhs, BCValue rhs)");
    static assert(is(typeof(BCGenT.Div3(BCValue.init, BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Div3(BCValue result, BCValue lhs, BCValue rhs)");
    static assert(is(typeof(BCGenT.Udiv3(BCValue.init, BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Udiv3(BCValue result, BCValue lhs, BCValue rhs)");
    static assert(is(typeof(BCGenT.And3(BCValue.init, BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void And3(BCValue result, BCValue lhs, BCValue rhs)");
    static assert(is(typeof(BCGenT.Or3(BCValue.init, BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Or3(BCValue result, BCValue lhs, BCValue rhs)");
    static assert(is(typeof(BCGenT.Xor3(BCValue.init, BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Xor3(BCValue result, BCValue lhs, BCValue rhs)");
    static assert(is(typeof(BCGenT.Lsh3(BCValue.init, BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Lsh3(BCValue result, BCValue lhs, BCValue rhs)");
    static assert(is(typeof(BCGenT.Rsh3(BCValue.init, BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Rsh3(BCValue result, BCValue lhs, BCValue rhs)");
    static assert(is(typeof(BCGenT.Mod3(BCValue.init, BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Mod3(BCValue result, BCValue lhs, BCValue rhs)");
    static assert(is(typeof(BCGenT.Umod3(BCValue.init, BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Umod3(BCValue result, BCValue lhs, BCValue rhs)");
    static assert(is(typeof(BCGenT.Call(BCValue.init, BCValue.init,
            BCValue[].init)) == void),
            BCGenT.stringof ~ " is missing void Call(BCValue result, BCValue fn, BCValue[] args)");

    static assert(is(typeof(BCGenT.Load8(BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Load8(BCValue _to, BCValue from)");
    static assert(is(typeof(BCGenT.Store8(BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Store8(BCValue _to, BCValue value)");

    static assert(is(typeof(BCGenT.Load16(BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Load162(BCValue _to, BCValue from)");
    static assert(is(typeof(BCGenT.Store16(BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Store16(BCValue _to, BCValue value)");

    static assert(is(typeof(BCGenT.Load32(BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Load32(BCValue _to, BCValue from)");
    static assert(is(typeof(BCGenT.Store32(BCValue.init, BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Store32(BCValue _to, BCValue value)");

    static assert(is(typeof(BCGenT.Load64(BCValue.init, BCValue.init)) == void),
        BCGenT.stringof ~ " is missing void Load64(BCValue _to, BCValue from)");
    static assert(is(typeof(BCGenT.Store64(BCValue.init, BCValue.init)) == void),
        BCGenT.stringof ~ " is missing void Store64(BCValue _to, BCValue value)");

    static assert(is(typeof(BCGenT.Ret(BCValue.init)) == void),
            BCGenT.stringof ~ " is missing void Ret(BCValue val)");
    static assert(is(typeof(BCGenT.insideFunction) == bool),
        BCGenT.stringof ~ " is missing bool insideFunction");

    enum ensureIsBCGen = true;
}
*/
/// commonType enum used for implicit conversion
static const BCTypeEnum smallIntegerTypes[] = {BCTypeEnum_u16, BCTypeEnum_u8,
                                      BCTypeEnum_i16, BCTypeEnum_i8,
                                      BCTypeEnum_c32, BCTypeEnum_c16, BCTypeEnum_c8};
#define ARRAY_SIZE(A) (sizeof(A) / sizeof(A[0]))

BCTypeEnum commonTypeEnum(BCTypeEnum lhs, BCTypeEnum rhs)
{
    // HACK

    BCTypeEnum commonType = BCTypeEnum_Undef;

    if (lhs == BCTypeEnum_f52 || rhs == BCTypeEnum_f52)
    {
        commonType = BCTypeEnum_f52;
    }
    else if (lhs == BCTypeEnum_f23 || rhs == BCTypeEnum_f23)
    {
        commonType = BCTypeEnum_f23;
    }
    else if (lhs == BCTypeEnum_u64 || rhs == BCTypeEnum_u64)
    {
        commonType = BCTypeEnum_u64;
    }
    else if (lhs == BCTypeEnum_i64 || rhs == BCTypeEnum_i64)
    {
        commonType = BCTypeEnum_i64;
    }
    else if (lhs == BCTypeEnum_u32 || rhs == BCTypeEnum_u32)
    {
        commonType = BCTypeEnum_u32;
    }
    else if (lhs == BCTypeEnum_i32 || rhs == BCTypeEnum_i32)
    {
        commonType = BCTypeEnum_i32;
    }
    else if (anyOf(lhs, smallIntegerTypes, ARRAY_SIZE(smallIntegerTypes)) || anyOf(rhs, smallIntegerTypes, ARRAY_SIZE(smallIntegerTypes)))
    {
        commonType = BCTypeEnum_i32;
    }

    if (commonType == BCTypeEnum_Undef)
    {
        // debug { if (!__ctfe) writeln("could not find common type for lhs: ", lhs, " and rhs: ", rhs); }
    }

    return commonType;
}

#undef offsetof