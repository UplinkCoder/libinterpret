module dmd.ctfe.bc_lightning_backend;

enum withStrEq3 = 1;
enum withMemCpy = 1;

import dmd.ctfe.bc_common;
import dmd.ctfe.bc_abi;
import dmd.ctfe.bc_limits;

import lightning;
version(AArch64)
{
    import jit_aarch64;
    version = _64bit;
}
else version (X86_64)
{
    import jit_x86_64;
    version = _64bit;
}
else version (X86)
{
    import jit_x86;
}
else static assert(0, "Architecture unsupported");

struct LightningFunction
{
    int fIdx;
    void* fnAddr;

}

/// used during the execution of the generated function.
struct RuntimeContext
{
    size_t[RegisterState.nTempRegs] rSpill;
    union
    {
        struct
        {
            uint rtArg0;
            uint rtArg1;
        }
        long rtLongArg0;
    }

    union
    {
        struct
        {
            uint rtArg2;
            uint rtArg3;
        }
        long rtLongArg1;
    }

    uint currentLine;
    const(char)* currentFile;


    long* stackDataBegin;
    ubyte* heapDataBegin;
    uint* heapDataLength;
    void* functions;

    //
    uint* heapSizeP;
    uint* stackSizeP;

    enum huge_stack = false;

    static if (huge_stack)
    {
        // 1 GB Stack 3 GB Heap
        enum stackAddrMask = ((1 << 31) | (1 << 30));
    } else {
        // 128 MB Stack 3.87 GB Heap
        enum stackAddrMask = ((1 << 31) | 
                              (1 << 30) | 
                              (1 << 29) | 
                              (1 << 28) | 
                              (1 << 27));
    }
    /// if the two most significant bits are both one it's on the stack
    static bool isPointerToStack(uint unrealPointer)
    {
        pragma(inline, true);
        return (unrealPointer & stackAddrMask) == stackAddrMask;
    }

    void* toRealPointer(uint unrealPointer)
    {
        void* realPointer = heapDataBegin + unrealPointer;

        if (isPointerToStack(unrealPointer))
        {
            uint stackOffset = (unrealPointer & ~stackAddrMask);
            realPointer = stackDataBegin + stackOffset;
        }

        return realPointer;
    }
}

alias register_index = uint;
enum register_index INVALID_IDX = 0;

enum ContextOffset
{
    rSpill = RuntimeContext.init.rSpill.offsetof,
    rtArg0 = RuntimeContext.init.rtArg0.offsetof,
    rtLongArg0 = RuntimeContext.init.rtLongArg0.offsetof,
    rtArg1 = RuntimeContext.init.rtArg1.offsetof,
    rtArg2 = RuntimeContext.init.rtArg2.offsetof,
    rtLongArg1 = RuntimeContext.init.rtLongArg1.offsetof,

    currentLine = RuntimeContext.init.currentLine.offsetof,
    currentFile = RuntimeContext.init.currentFile.offsetof,

    stackDataBegin = RuntimeContext.init.stackDataBegin.offsetof,
    heapDataBegin = RuntimeContext.init.heapDataBegin.offsetof,
    heapDataLength = RuntimeContext.init.functions.offsetof,
    functions = RuntimeContext.init.functions.offsetof,
    heapSizeP = RuntimeContext.init.heapSizeP.offsetof,
}

pure nothrow @safe @nogc
int min(int x, int y) { pragma(inline, true); return (((x) < (y)) ? (x) : (y)); }
pure nothrow @safe @nogc
int max(int x, int y) { pragma(inline, true); return (((x) > (y)) ? (x) : (y)); }

bool isRegisterPair(register_index regIdx)
{
    return (regIdx & 0xffff) != 0;
}

bool NoRegister(register_index regIdx)
{
    return regIdx == 0;
}

static register_index reg2idx(jit_reg_t reg)
{
    assert(reg <= jit_v(0) && reg >= jit_v(jit_v_num()),
        "register is not a valid v-reg: " ~ enumToString(reg));
    return (reg  - jit_v(0)) + 1;
}


import std.file : append;
struct RegisterState
{
    enum nRegs = jit_v_num();
    enum nTempRegs = jit_r_num();

    BCValue[nRegs] valueInReg;

    register_index[nRegs] pairedWith = 0;

    register_index getRegisterIndex(BCValue v)
    {
        foreach(int i, valR;valueInReg)
        {
            if (valR == v)
            {
                assert(pairedWith[i] == INVALID_IDX);
                regStatus.markUsed(i);
                return i;
            }
        }
        return allocReg(v);
    }
    /// get a register for a stack value
    /// either we already have it in a register
    /// or we allocate a register for it
    /// if needed we evict a value
    register_index getSingleRegisterIndex(BCValue v)
    {
        append("reg_alloc.log", cast(void[])("\nrequesting reg for: " ~ v.toString()));
        jit_reg_t result = JIT_NOREG;

        foreach(int i, valR;valueInReg)
        {
            append("reg_alloc.log", cast(void[])("... found in regIdx: " ~ itos(i)));
            if (valR == v)
            {
                assert(!pairedWith[i]);
                regStatus.markUsed(i);
                return i;
            }
        }

        return allocReg(v, true);
    }

    register_index aquireTempReg(bool paired)
    {
        assert(0, "TODO implement me.");
    }

    void releaseTempReg(register_index tmpRegIdx)
    {
        assert(0, "TODO implement me.");
    }

    register_index allocReg(BCValue v, bool notPaired = false)
    {
        register_index result = INVALID_IDX;
        with(regStatus)
        {
            // first let's look for a free register.
            register_index nextReg = nextFree();
            if (nextReg != INVALID_IDX)
            {
                assert(pairedWith[nextReg] == INVALID_IDX, "free register cannot have pairing relationships!");
                // we have a free register let's use it
                markUsed(nextReg);
                valueInReg[nextReg] = v;
                result = jit_v(nextReg);
            }
            // no free register now we need to evict ... sigh
            // first let's look for someome marked for eviction
            register_index unusedReg = nextUnused();
            if (result == INVALID_IDX && unusedReg != INVALID_IDX)
            {
                // depending on wheter we need a register pair or not we have to do
                // diffrent things here.

                // if the unused reg was part of a register pair
                // let's see if we can find a single one
                // for easier searching lets start with a while loop right away
                typeof(unusedBitfield) masked = unusedBitfield;

                // we keep searching for a long as there are unused regs
                // for one which is a single pair
                auto pair_idx = pairedWith[unusedReg];
                while (pair_idx != INVALID_IDX)
                {
                    masked &= (~(1 << unusedReg));
                    if (masked != 0)
                    {
                        import core.bitop : bsf;
                        unusedReg = bsf(masked);
                        pair_idx = pairedWith[unusedReg];
                    }
                }
                // if we don't find one we split the last unused paired reg
                if (pair_idx != INVALID_IDX)
                {
                    // if our last canidate is paired we need to unpair it
                    // of course it's partner should also be unused.
                    const idx1 = (pair_idx & 0xFFFF) - 1;
                    const idx2 = (pair_idx >> 16) - 1;
                    assert(isUnused(idx1) && isUnused(idx2));
                    unpair(pair_idx);
                }
                markUsed(unusedReg);
            } else
            // we got no one marked for evication
            // well then we get a 'random' canidate and force evict
            {
                auto nextEvict = nextEvictim();
                const pair_idx = pairedWith[nextEvict];
                result = jit_v(nextEvict);
                // we sync on eviction
                // now we need to unpair if needed
                if (pair_idx != INVALID_IDX)
                {
                    unpair(pair_idx);
                }
                else
                {

                }
            }
        }

        assert(!regStatus.isDirty(result));
        /// 
        const low_idx = result & 0xFFFF;
        valueInReg[low_idx] = v;
        assert(result != INVALID_IDX);
        read_stack(v, result);
        return result;
    }


    // calls get register index and verfies it's not a register pair
    jit_reg_t getSingleRegister(BCValue v)
    {
        if (basicTypeSize(v.type.type) <= 4)
        {
            const iReg =
                cast(int)(getSingleRegisterIndex(v));
            assert(iReg < jit_v_num() && iReg >= 0);
            return jit_v(iReg);
        }
        else
        {
            // return JIT_NOREG;
            assert(0);
        }
    }

    /// write the value in the register to the stack location it mirrors
    void write_stack_pair(BCValue v, register_index reg)
    {
        assert(isRegisterPair(reg));
        assert(0, "Not Implemented");
    }

    /// write the value in the register to the stack location it mirrors
    void write_stack(BCValue v, register_index reg)
    {
        if (isRegisterPair(reg))
        {
            write_stack_pair(v, reg);
        }
        else
        {
            const idx = reg - jit_v(0);
            regStatus.markClean(reg);
        }
    }

    /// value in register is unlikely to be used again
    /// mark this register as a canidate for eviction
    void markUnused(jit_reg_t reg)
    {
        const idx = idxFromReg(reg);
        const pair_idx = pairedWith[idx - 1];
        if (pair_idx != 0)
        {
            const lw_idx = pair_idx & 0xFFFF;
            const hi_idx = pair_idx >> 16;

            regStatus.markUsed(lw_idx);
            regStatus.markUsed(hi_idx);
        }
        else
        {
            regStatus.markUnused(idx);
        }
    }

    /// evict the value from the register
    void freeReg(jit_reg_t reg)
    {
        const idx = reg - jit_v(0);
        if (regStatus.isDirty(idx))
        {
            const v = valueInReg[idx];
            write_stack(v, reg);
        }
        regStatus.markFree(idx);
        valueInReg[idx] = BCValue.init;
    }
    /// set two registers as paried
    void set_paired(jit_reg_t reg1, jit_reg_t reg2)
    {
        assert(regStatus.isUnused(reg1) && regStatus.isUnused(reg2));
        assert(pairedWith[reg1] == INVALID_IDX &&
               pairedWith[reg2] == INVALID_IDX);
        const idx1 = reg1 - jit_v(0);
        const idx2 = reg2 - jit_v(0);

        const pair_idx = (min(idx1, idx2) | (max(idx1, idx2) << 16));

        pairedWith[idx1] = pair_idx;
        pairedWith[idx2] = pair_idx;
    }

    /// writes the pair to stack and unpairs and frees the regs
    void unpair(register_index idx)
    {
        assert(isRegisterPair(idx));
        //before we unpair we need to sync.
        const lw_idx = idx & 0xFFFF;
        // the lower of the pair always has the value

        auto v = valueInReg[lw_idx];
        write_stack_pair(v, idx);
        const hi_idx = idx >> 16;

        pairedWith[lw_idx] = INVALID_IDX;
        pairedWith[hi_idx] = INVALID_IDX;

        regStatus.markFree(lw_idx);
        regStatus.markFree(hi_idx);
    }

    register_index get_index(jit_reg_t reg)
    {
        assert(reg >= jit_v(0) && reg < jit_v(jit_v_num()));
        const idx = reg - jit_v(0);
        const pair_idx = pairedWith[idx];
        if (pair_idx != INVALID_IDX)
        {
            return pair_idx;
        }
        else
        {
            return idx;
        }
    }

    RegStatusList!(nRegs) regStatus;
    RegStatusList!(nTempRegs) tmpStatus;
    uint evictionCounter;

    // it's a pun ... I could not resist.
    uint nextEvictim()
    {
        return evictionCounter++ % nRegs;
    }
    bool[nTempRegs] spilled;
    bool r0HasCond;
}

struct LightningGen
{
    jit_state* _jit;
    RegisterState regs;
    LightningFunction *functions;
    uint functionCount;


    BCParameter[64] parameters;
    byte parameterCount;
    BCValue[bc_max_locals] temporaries;
    ushort temporaryCount;
    BCLocal[bc_max_locals] locals;
    ushort localCount;
    jit_node*[short.max] locations;
    uint locationCount;

    alias LightningLabel = typeof(_jit_label(cast(jit_state_t*)null));

    LightningLabel[] labels;
    uint labelCount;
    uint labelMaxCount;

    uint lastLine;
    const(char*) lastFile;

    StackAddr currentFrameOffset;
    StackAddr maxFameOffset;

    struct ReturnValue
    {
        align(1) union
        {
        align(1):
            align(1) struct
            {
            align(1):
                uint lw;
                uint hi;
            }
            long simm32;
            ulong imm32;
        }

        BCValueType type;
    }

    struct BCStack
    {
        void* start;
        uint maxStackSize;
    }

    alias jit_fn = extern (C) int function (
        BCHeap * heap,
        BCStack* stack,
        ReturnValue* returnValue,
        void** functions
    );

    void Initialize()
    {
        // make sure jit state is initalized;
        if (_jit) _jit_destroy_state(_jit);
        else init_jit(null);
        _jit = jit_new_state();
        labelMaxCount = 4096;
        import core.stdc.stdlib;
        labels = (cast(LightningLabel*)malloc(
            (labels[0]).sizeof * labelMaxCount
        ))[0 .. labelMaxCount];
        assert(labels !is null);
    }

    void Finalize()
    {
        _jit_clear_state(_jit);

        finish_jit();
    }
    typeof(_jit_arg(cast(jit_state_t*)null)) context_arg;

    bool insideFunction = false;

    extern (C) void _jit_getarg_ptrint(jit_reg_t r, jit_node_t* arg)
    {
        version(_64bit)
            _jit_getarg_l(_jit, r, arg); // git it as 64bit
        else
            _jit_getarg_i(_jit, r, arg); // get it as 32bit int
    }

    void beginFunction(uint f = 0, void* fnDecl = null)
    {
        functionCount++;
        auto fIdx = f;
        assert(currentFrameOffset == 0, "by the time we have call beginFunction either we are freshly initalized or have called endFunction before");
        assert(0, "Not properly implemented yet");
        _jit_prolog(_jit);

        context_arg = _jit_arg(_jit); // RuntimeContext*
    }

    void endFunction()
    {
        _jit_epilog(_jit);
        currentFrameOffset = 0;
        locationCount = 0;
        parameterCount = 0;

        assert(0, "Not properly implemented yet");
    }

    BCLabel genLabel()
    {
        locations[locationCount++] = _jit_label(_jit);
        return BCLabel(BCAddr(locationCount));
    }

    BCValue genParameter(BCType bct, string name, uint userSize = 0)
    {
        assert(!needsUserSize(bct.type) || userSize > 0);
        const stackAddr = currentFrameOffset;


        ++parameterCount;
        auto p = BCParameter(parameterCount, bct, currentFrameOffset);
        parameters[parameterCount] = p;

        auto stackSize = (typeIsPointerOnStack(bct.type) ? PtrSize :  basicTypeSize(bct.type));
        if (needsUserSize(bct.type))
        {
            stackSize = userSize;
        }
        currentFrameOffset += stackSize;

        return BCValue(p);
    }

    BCValue genTemporary(BCType bct, uint userSize = 0)
    {
        assert(!needsUserSize(bct.type) || userSize > 0);
        ++temporaryCount;
        auto tmp =  BCValue(currentFrameOffset, bct, temporaryCount);
        temporaries[temporaryCount - 1] = tmp;
        auto stackSize = (typeIsPointerOnStack(bct.type) ? PtrSize :  basicTypeSize(bct.type));
        if (needsUserSize(bct.type))
        {
            stackSize = userSize;
        }
        currentFrameOffset += stackSize;

        if (currentFrameOffset > maxFameOffset)
            maxFameOffset = currentFrameOffset;

        return tmp;
    }

    void destroyTemporary(BCType bct, uint userSize = 0)
    {
        assert(!needsUserSize(bct.type) || userSize > 0);
        auto stackSize = (typeIsPointerOnStack(bct.type) ? PtrSize :  basicTypeSize(bct.type));

        if (needsUserSize(bct.type))
        {
            stackSize = userSize;
        }

        currentFrameOffset -= stackSize;
    }

    BCValue genLocal(BCType bct, string name, uint userSize = 0)
    {
        assert(!needsUserSize(bct.type) || userSize > 0);
        ++localCount;
        auto localIdx = localCount;
        auto localAddr = currentFrameOffset;
        locals[localCount - 1] = BCLocal(localIdx, bct, localAddr, name);
        auto stackSize = (typeIsPointerOnStack(bct.type) ? PtrSize :  basicTypeSize(bct.type));
        if (needsUserSize(bct.type))
        {
            stackSize = userSize;
        }
        currentFrameOffset += stackSize;
        // this(const StackAddr sp, const BCType type, const ushort localIndex, string name)
        return BCValue(localAddr, bct, localIdx, name);
    }

    BCAddr beginJmp()
    {
        locations[locationCount++] =
            _jit_new_node_p(_jit, jit_code_t.jit_code_jmpi, null);
        return BCAddr(locationCount);

    }

    void endJmp(BCAddr atIp, BCLabel target)
    {
        _jit_patch_at(_jit, locations[atIp.addr], locations[target.addr.addr]);
    }

    void Jmp(BCLabel target)
    {
        auto jmp = _jit_new_node_p(_jit, jit_code_t.jit_code_jmpi, null);
        _jit_patch_at(_jit, jmp, locations[target.addr.addr]);
    }

    /// peform a conditional jmp based on the value of the parameter cond
    /// BCValue.init is special and means jmp based upon the last condition flag
    CndJmpBegin beginCndJmp(BCValue cond = BCValue.init, bool ifTrue = false)
    {
        auto flagReg = JIT_R0;
        if (cond == BCValue.init)
        {
            assert(regs.r0HasCond);
        }
        else
        {
            flagReg = regs.getSingleRegister(cond);
        }

        jit_node* at;

        if (ifTrue)
        {
            at = _jit_new_node_pww(_jit, jit_code_t.jit_code_bnei, null, flagReg, 0);
        }
        else
        {
            at = _jit_new_node_pww(_jit, jit_code_t.jit_code_beqi, null, flagReg, 0);
        }

        locations[locationCount++] = at;

        return CndJmpBegin(BCAddr(locationCount), cond, ifTrue);
    }

    void endCndJmp(CndJmpBegin jmp, BCLabel target)
    {
        assert(jmp.at);
        assert(target.addr);

        auto jmp_loc = locations[jmp.at - 1];
        auto target_loc = locations[target.addr - 1];

        _jit_patch_at(_jit, jmp_loc, target_loc);
    }

    void emitFlg(BCValue lhs) { assert(0, "Not Implemented yet"); }
    void Throw(BCValue e) { assert(0, "Not Implemented yet"); }
    void PushCatch() { assert(0, "Not Implemented yet"); }
    void PopCatch() { assert(0, "Not Implemented yet"); }

    void loadImm32(jit_reg_t r, uint imm32)
    {
        _jit_new_node_ww(_jit, jit_code_t.jit_code_movi, r, imm32);
    }

    void Set(BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = regs.getSingleRegister(lhs);
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_ww(_jit, jit_code_t.jit_code_movi, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = regs.getSingleRegister(rhs);
            _jit_new_node_ww(_jit, jit_code_t.jit_code_movr, lhs_r, rhs_r);
            regs.markUnused(rhs_r);
        }
    }

    void Ult3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = regs.getSingleRegister(lhs);
        auto result_r = JIT_R0;
        if (result == BCValue.init)
        {
            assert(!regs.r0HasCond, "We would override our flag");
            regs.r0HasCond = true;
        }
        else
        {
            result_r = regs.getSingleRegister(result);
        }

        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_lti_u, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = regs.getSingleRegister(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_ltr_u, result_r, lhs_r, rhs_r);
            regs.markUnused(rhs_r);
        }
    }

    void Ugt3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = regs.getSingleRegister(lhs);
        auto result_r = JIT_R0;
        if (result == BCValue.init)
        {
            assert(!regs.r0HasCond, "We would override our flag");
            regs.r0HasCond = true;
        }
        else
        {
            result_r = regs.getSingleRegister(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_gti_u, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = regs.getSingleRegister(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_gtr_u, result_r, lhs_r, rhs_r);
            regs.markUnused(rhs_r);
        }
    }

    void Ule3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = regs.getSingleRegister(lhs);
        auto result_r = JIT_R0;
        if (result == BCValue.init)
        {
            assert(!regs.r0HasCond, "We would override our flag");
            regs.r0HasCond = true;
        }
        else
        {
            result_r = regs.getSingleRegister(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_lei_u, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = regs.getSingleRegister(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_ler_u, result_r, lhs_r, rhs_r);
            regs.markUnused(rhs_r);
        }
    }

    void Uge3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = regs.getSingleRegister(lhs);
        auto result_r = JIT_R0;
        if (result == BCValue.init)
        {
            assert(!regs.r0HasCond, "We would override our flag");
            regs.r0HasCond = true;
        }
        else
        {
            result_r = regs.getSingleRegister(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_gei_u, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = regs.getSingleRegister(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_ger_u, result_r, lhs_r, rhs_r);
        }
    }

    void Lt3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = regs.getSingleRegister(lhs);
        auto result_r = JIT_R0;
        if (result == BCValue.init)
        {
            assert(!regs.r0HasCond, "We would override our flag");
            regs.r0HasCond = true;
        }
        else
        {
            result_r = regs.getSingleRegister(result);
        }

        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_lti, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = regs.getSingleRegister(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_ltr, result_r, lhs_r, rhs_r);
        }
    }

    void Gt3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = regs.getSingleRegister(lhs);
        auto result_r = JIT_R0;
        if (result == BCValue.init)
        {
            assert(!regs.r0HasCond, "We would override our flag");
            regs.r0HasCond = true;
        }
        else
        {
            result_r = regs.getSingleRegister(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_gti, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = regs.getSingleRegister(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_gtr, result_r, lhs_r, rhs_r);
        }
    }

    void Le3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = regs.getSingleRegister(lhs);
        auto result_r = JIT_R0;
        if (result == BCValue.init)
        {
            assert(!regs.r0HasCond, "We would override our flag");
            regs.r0HasCond = true;
        }
        else
        {
            result_r = regs.getSingleRegister(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_lei, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = regs.getSingleRegister(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_ler, result_r, lhs_r, rhs_r);
        }
    }

    void Ge3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = regs.getSingleRegister(lhs);
        auto result_r = JIT_R0;
        if (result == BCValue.init)
        {
            assert(!regs.r0HasCond, "We would override our flag");
            regs.r0HasCond = true;
        }
        else
        {
            result_r = regs.getSingleRegister(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_gei, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = regs.getSingleRegister(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_ger, result_r, lhs_r, rhs_r);
        }
    }

    void Eq3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = regs.getSingleRegister(lhs);
        auto result_r = JIT_R0;
        if (result == BCValue.init)
        {
            assert(!regs.r0HasCond, "We would override our flag");
            regs.r0HasCond = true;
        }
        else
        {
            result_r = regs.getSingleRegister(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_eqi, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = regs.getSingleRegister(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_eqr, result_r, lhs_r, rhs_r);
        }
    }

    void Neq3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = regs.getSingleRegister(lhs);
        auto result_r = JIT_R0;
        if (result == BCValue.init)
        {
            assert(!regs.r0HasCond, "We would override our flag");
            regs.r0HasCond = true;
        }
        else
        {
            result_r = regs.getSingleRegister(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_nei, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = regs.getSingleRegister(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_ner, result_r, lhs_r, rhs_r);
        }
    }
    void Add3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = regs.getSingleRegister(lhs);
        regs.read_stack(lhs, lhs_r);

        auto res_r = regs.getSingleRegister(result);
        if (commonTypeEnum(lhs.type.type, rhs.type.type).anyOf([BCTypeEnum.f23, BCTypeEnum.f52]))
        {
            if (rhs.vType == BCValueType.Immediate)
            {
                _jit_new_node_www(_jit, jit_code_t.jit_code_addi, res_r, lhs_r, rhs.imm32.imm32);
            }
            else
            {
                auto rhs_r = regs.getSingleRegister(rhs);
                _jit_new_node_www(_jit, jit_code_t.jit_code_addr, res_r, lhs_r, rhs_r);
                regs.markUnused(rhs_r);
            }
        }
        else
            assert (0, "only integers are supported right now");
        regs.markUnused(lhs_r);
    }

    void Sub3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = regs.getSingleRegister(lhs);
        regs.read_stack(lhs, lhs_r);
        auto res_r = regs.getSingleRegister(result);
        if (commonTypeEnum(lhs.type.type, rhs.type.type).anyOf([BCTypeEnum.f23, BCTypeEnum.f52]))
        {
            if (rhs.vType == BCValueType.Immediate)
            {
                _jit_new_node_www(_jit, jit_code_t.jit_code_subi, res_r, lhs_r, rhs.imm32.imm32);
            }
            else
            {
                auto rhs_r = regs.getSingleRegister(rhs);
                _jit_new_node_www(_jit, jit_code_t.jit_code_subr, res_r, lhs_r, rhs_r);
                regs.markUnused(rhs_r);
            }
        }
        else
            assert (0, "only integers are supported right now");
        regs.markUnused(lhs_r);
    }

    void Mul3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = regs.getSingleRegister(lhs);
        auto res_r = regs.getSingleRegister(result);
        if (commonTypeEnum(lhs.type.type, rhs.type.type).anyOf([BCTypeEnum.f23, BCTypeEnum.f52]))
        {
            if (rhs.vType == BCValueType.Immediate)
            {
                _jit_new_node_www(_jit, jit_code_t.jit_code_muli, res_r, lhs_r, rhs.imm32.imm32);
            }
            else
            {
                auto rhs_r = regs.getSingleRegister(rhs);
                _jit_new_node_www(_jit, jit_code_t.jit_code_mulr, res_r, lhs_r, rhs_r);
                regs.markUnused(rhs_r);
            }
        }
        else
            assert (0, "only integers are supported right now");
        regs.markUnused(lhs_r);
    }

    void Div3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = regs.getSingleRegister(lhs);
        auto res_r = regs.getSingleRegister(result);
        if (commonTypeEnum(lhs.type.type, rhs.type.type).anyOf([BCTypeEnum.f23, BCTypeEnum.f52]))
        {
            if (rhs.vType == BCValueType.Immediate)
            {
                _jit_new_node_www(_jit, jit_code_t.jit_code_divi, res_r, lhs_r, rhs.imm32.imm32);
            }
            else
            {
                auto rhs_r = regs.getSingleRegister(rhs);
                _jit_new_node_www(_jit, jit_code_t.jit_code_divr, res_r, lhs_r, rhs_r);
                regs.markUnused(rhs_r);
            }
        }
        else
            assert (0, "only integers are supported right now");
        regs.markUnused(lhs_r);
     }

    void Udiv3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = regs.getSingleRegister(lhs);
        auto res_r = regs.getSingleRegister(result);
        if (commonTypeEnum(lhs.type.type, rhs.type.type).anyOf([BCTypeEnum.f23, BCTypeEnum.f52]))
        {
            if (rhs.vType == BCValueType.Immediate)
            {
                _jit_new_node_www(_jit, jit_code_t.jit_code_divi_u, res_r, lhs_r, rhs.imm32.imm32);
            }
            else
            {
                auto rhs_r = regs.getSingleRegister(rhs);
                _jit_new_node_www(_jit, jit_code_t.jit_code_divr_u, res_r, lhs_r, rhs_r);
                regs.markUnused(rhs_r);
            }
        }
        else
            assert (0, "only integers are supported right now");
        regs.markUnused(lhs_r);
    }

    void And3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = regs.getSingleRegister(lhs);
        auto res_r = regs.getSingleRegister(result);
        if (commonTypeEnum(lhs.type.type, rhs.type.type) == BCTypeEnum.i32)
        {
            if (rhs.vType == BCValueType.Immediate)
            {
                _jit_new_node_www(_jit, jit_code_t.jit_code_andi, res_r, lhs_r, rhs.imm32.imm32);
            }
            else
            {
                auto rhs_r = regs.getSingleRegister(rhs);
                _jit_new_node_www(_jit, jit_code_t.jit_code_andr, res_r, lhs_r, rhs_r);
                regs.markUnused(rhs_r);
            }
        }
        else
            assert (0, "only 32 bit integers are supported right now");
        regs.markUnused(lhs_r);
    }
    void Or3(BCValue result, BCValue lhs, BCValue rhs) { assert(0, "Not Implemented yet"); }
    void Xor3(BCValue result, BCValue lhs, BCValue rhs) { assert(0, "Not Implemented yet"); }
    void Lsh3(BCValue result, BCValue lhs, BCValue rhs) { assert(0, "Not Implemented yet"); }
    void Rsh3(BCValue result, BCValue lhs, BCValue rhs) { assert(0, "Not Implemented yet"); }
    void Mod3(BCValue result, BCValue lhs, BCValue rhs) { assert(0, "Not Implemented yet"); }
    void Umod3(BCValue result, BCValue lhs, BCValue rhs) { assert(0, "Not Implemented yet"); }
    void Byte3(BCValue result, BCValue word, BCValue idx) { assert(0, "Not Implemented yet"); }

    void Call(BCValue result, BCValue fn, BCValue[] args)
    {
        if (fn.vType == BCValueType.Immediate)
        {
            _jit_new_node_ww(_jit, jit_code_t.jit_code_movi, JIT_R1, fn.imm32.imm32);
            _jit_finishr(_jit, JIT_R1);
        }
    }
    void Load8(BCValue to, BCValue from) { assert(0, "Not Implemented yet"); }
    void Store8(BCValue to, BCValue from) { assert(0, "Not Implemented yet"); }
    void Load16(BCValue to, BCValue from) { assert(0, "Not Implemented yet"); }
    void Store16(BCValue to, BCValue from) { assert(0, "Not Implemented yet"); }

    void spill_to_context(jit_reg_t r)
    {
        const idx = r - jit_r(0);
        assert(regs.spilled[idx] == false);
        assert(idx < RegisterState.nTempRegs);
        assert(idx == 1 || idx == 2); // might be relaxed later
        _jit_getarg_ptrint(jit_r(0), context_arg);
        auto offset = (ContextOffset.rSpill + (size_t.sizeof * idx));
        storeToContextPtrSize(r, cast(ContextOffset)offset, jit_r(0));
        regs.spilled[idx] = true;
    }

    void storeToContext32(jit_reg_t r, ContextOffset offset, jit_reg_t ctxPtrReg)
    {
        _jit_new_node_www(_jit, jit_code_t.jit_code_stxi_i, offset, ctxPtrReg, r);
    }

    void storeToContextPtrSize(jit_reg_t r, ContextOffset offset, jit_reg_t ctxPtrReg)
    {
        version(_64bit)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_stxi_l, offset, ctxPtrReg, r);
        }
        else
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_stxi_i, offset, ctxPtrReg, r);
        }
    }

    jit_node_t* load_size_t_immoffset(jit_reg_t to_r, jit_reg_t from_r, int offset)
    {
        version(_64bit)
        {
            return _jit_new_node_www(_jit, jit_code_t.jit_code_ldxr_l, to_r, from_r, offset);
        }
        else
        {
            return _jit_new_node_www(_jit, jit_code_t.jit_code_ldxr_i, to_r, from_r, offset);
        }
    }

    extern (C) void RT_BoundsCheck(RuntimeContext* context)
    {
        auto SliceDescPtr = context.rtArg0;
        auto Index = context.rtArg1;
        SliceDescriptor desc = *(cast(SliceDescriptor*)context.toRealPointer(SliceDescPtr));

    }

    extern (C) uint RT_NullPtrCheck(RuntimeContext* context)
    {
        if (!context.rtArg0)
        {
            // context.currentLine
        }

        return 0;
    }

    /// returns whether it overflowed
    extern (C) uint RT_Mul64(RuntimeContext* context)
    {
        long lhs = context.rtLongArg0;
        long rhs = context.rtLongArg1;

        long result = lhs * rhs;
        context.rtLongArg0 = lhs;

        return 0;
    }

    /// returns whether it overflowed
    extern (C) uint RT_Div64(RuntimeContext* context)
    {
        long lhs = context.rtLongArg0;
        long rhs = context.rtLongArg1;

        long result = lhs / rhs;
        context.rtLongArg0 = lhs;

        return 0;
    }

    void nullptr_check(register_index idx)
    {
        assert(!isRegisterPair(idx));
        assert(idx != jit_r(0), "r0 must not be used for nullptr_checks");
        //_jit_new_nod
    }

    void heapoverflow_check(register_index idx, jit_reg_t tmpReg, jit_reg_t contextPtrReg)
    {
        static bool isTempReg(jit_reg_t r)
        {
            return r > jit_r(0) && r < jit_r(0) + jit_r_num();
        }

        assert(isTempReg(tmpReg));
        assert(contextPtrReg == jit_r(0));
        assert(!isRegisterPair(idx));

    }

    void Load32(BCValue to, BCValue from)
    {
        auto from_r = regs.getSingleRegister(from);
        // this is the address we want to load from
        auto to_r = regs.getSingleRegister(to);
        // this is the stack address we want to load into
        assert(!regs.r0HasCond);

        _jit_getarg_ptrint(JIT_R0, context_arg);
        static if (1)
        {
            nullptr_check(from_r);
            load_size_t_immoffset(JIT_R2, JIT_R0, heapDataLengthOffset);
            heapoverflow_check(from_r, JIT_R2, JIT_R0);
        }
        load_size_t_immoffset(JIT_R1, JIT_R0, heapDataPtrOffset);

        _jit_new_node_www(_jit, jit_code_t.jit_code_ldxr_i, to_r, from_r, JIT_R2);
    }
    void Store32(BCValue to, BCValue from) { assert(0, "Not Implemented yet"); }
    void Load64(BCValue to, BCValue from) { assert(0, "Not Implemented yet"); }
    void Store64(BCValue to, BCValue from) { assert(0, "Not Implemented yet"); }
    void Alloc(BCValue heapPtr, BCValue size) { assert(0, "Not Implemented yet"); }
    void Not(BCValue result, BCValue val) { assert(0, "Not Implemented yet"); }
    void Ret(BCValue val)
    {
        _jit_getarg_ptrint(JIT_R0, context_arg);
//        load_size_t_immoffset(JIT_R0, JIT_R0, offset);

        if (val.type.type == BCTypeEnum.i64 || val.type.type == BCTypeEnum.u64)
        {
            // 8 byte return uses R1 for the lo word and R2 for the high word
            // runtime return type information (is it an error or execption is passed in R0)


        }
        else
        {
            // regular 4 byte return uses R1 for the value and R0 for the runtime return type

        }
        assert(0, "TODO implement this");
        _jit_ret(_jit);
    }
    void Cat(BCValue result, BCValue lhs, BCValue rhs, const uint elmSize) { assert(0, "Not Implemented yet"); }
    void Assert(BCValue value, BCValue err) { assert(0, "Not Implemented yet"); }
    static if (withMemCpy)
    {
        void MemCpy(BCValue dst, BCValue src, BCValue size) { assert(0, "Not Implemented yet"); }
    }
    static if (withStrEq3)
    {
        void StrEq3(BCValue result, BCValue lhs, BCValue rhs) { assert(0, "Not Implemented yet"); }
    }
    void Comment(const(char)[] comment) { assert(0, "Not Implemented yet"); }
    void Line(uint line)
    {
        if (line != lastLine)
        {
            const rIdx = regs.aquireTempReg(false);
            assert(rIdx != regs.INVALID_IDX, "Couldn't aquire TempReg");
            assert((rIdx >> 16) == 0, "I asked for an unpaired reg but got a paired one");
            assert(rIdx != 0, "r0 must not be used as TempReg");

            const r = jit_r(rIdx);

            loadImm32(r, line);
            storeToContext32(r, ContextOffset.currentLine, jit_r(0));
            regs.releaseTempReg(r);
        }
    }
    void File(string filename) { assert(0, "Not Implemented yet"); }
    void IToF32(BCValue result, BCValue value) { assert(0, "Not Implemented yet"); }
    void IToF64(BCValue result, BCValue value) { assert(0, "Not Implemented yet"); }
    void F32ToI(BCValue result, BCValue value) { assert(0, "Not Implemented yet"); }
    void F64ToI(BCValue result, BCValue value) { assert(0, "Not Implemented yet"); }
    void F64ToF32(BCValue result, BCValue value) { assert(0, "Not Implemented yet"); }
    void F32ToF64(BCValue result, BCValue value) { assert(0, "Not Implemented yet"); }


    BCValue run(int fnIdx, BCValue[] args, BCHeap* heapPtr)
    {
        assert(0, "Not Implemented yet");
    }
}
