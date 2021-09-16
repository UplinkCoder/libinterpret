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


    long* stackP;
    ubyte* heapDataBegin;
    uint* heapDataLength;
    void* functions;

    //
    uint* heapSizeP;
    uint* stackSizeP;

}

alias register_index = uint;

enum ContextOffsets
{
    rSpill = RuntimeContext.init.rSpill.offsetof,
    rtArg0 = RuntimeContext.init.rtArg0.offsetof,
    rtArg1 = RuntimeContext.init.rtArg1.offsetof,
    rtArg2 = RuntimeContext.init.rtArg2.offsetof,

    currentLine = RuntimeContext.init.currentLine.offsetof,
    currentFile = RuntimeContext.init.currentFile.offsetof,

    stackP = RuntimeContext.init.stackP.offsetof,
    heapDataBegin = RuntimeContext.init.heapDataBegin.offsetof,
    heapDataLength = RuntimeContext.init.functions.offsetof,
    functions = RuntimeContext.init.functions.offsetof,
    heapSizePtr = RuntimeContext.init.heapSizePtr.offsetof,
}
struct RegisterState
{
    enum nRegs = jit_v_num();
    enum nTempRegs = jit_r_num();

    BCValue[nRegs] valueInReg;
    enum register_index invalid_idx = (0xFFFF | (0xFFFF << 16));

    register_index[nRegs] pariedWith = invalid_idx;

    void set_paired(jit_reg_t reg1, jit_reg_t reg2)
    {
        assert(regStatus.isUnused(reg1) && regStatus.isUnused(reg2));
        assert(pariedWith[reg1] == invalid_idx && 
               pariedWith[reg2] == invalid_idx);
        const idx1 = reg1 - jit_v(0);
        const idx2 = reg2 - jit_v(0);

        const pair_idx = (min(idx1, idx2) | (max(idx1, idx2) << 16));

        pariedWith[idx1] = pair_idx;
        pariedWith[idx2] = pair_idx;
    }

    register_index get_index(jit_reg_t reg)
    {
        assert(reg >= jit_v(0) && reg < jit_v(jit_v_num()));
        const idx = reg - reg_v(0);
        const pair_idx = pariedWith[idx];
        if (pair_idx != invalid_idx)
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
    bool[nTempReg] spilled;
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
    typeof(_jit_getarg(cast(jit_state_t)null)) heap_arg, stack_arg, ret_arg;

    /// get a register for a stack value
    /// either we already have it in a register
    /// or we allocate a register for it
    /// if needed we evict a value
    register_index getRegisterIndex(BCValue v)
    {
        jit_reg_t result = JIT_NOREG;
        
        foreach(int i, valR;regs.valueInReg)
        {
            if (valR == v)
            {
                regs.regStatus.markUsed(i);
                return jit_v(i);
            }
        }
        
        // we could not find the value
        with (regs) with(regs.regStatus)
        {
            auto nextReg = nextFree();
            if (nextReg != INVALID_IDX)
            {
                // we have a free register let's use it
                markUsed(nextReg);
                valueInReg[nextReg] = v;
                result = jit_v(nextReg);
                sync_reg(v, result);
                return result;
            }
            // no free register now we need to evict ... sigh
            // first let's look for someome marked for eviction
            auto unusedReg = nextUnused();
            if (nextUnused != INVALID_IDX)
            {
                markUsed(nextUnused);
                valueInReg[nextUnused] = v;
                result = jit_v(nextUnused);
                sync_reg(v, result);
                return result;
            }
            // we got no one marked for evication
            // well the we get a 'random' canidate and force evict
            {
                auto nextEvict = nextEvictim();
                result = jit_v(nextEvict);
                sync_reg(valueInReg[nextEvict], result);
                // we sync on eviction
                valueInReg[nextEvict] = v;
                sync_reg(v, result);
                return result;
            }
        }
    }


    // calls get register index and verfies it's not a register pair
    jit_reg_t getReg(BCValue v)
    {
 
    }

    /// write the value in the register to the stack location it mirrors
    void sync_reg(BCValue v, register_index reg)
    {
        const idx = reg - jit_v(0);
        regs.regStatus.markClean(reg);
        assert(0, "Not implemented");
    }

    /// value in register is unlikely to be used again
    /// mark this register as a canidate for eviction
    void markUnused(jit_reg_t reg)
    {
        const idx = reg - jit_v(0);
        regs.regStatus.markUnused(idx);
    }

    /// evict the value from the register
    void freeReg(jit_reg_t reg)
    {
        const idx = reg - jit_v(0);
        if (regs.regStatus.isDirty(idx))
        {
            const v = regs.valueInReg[idx];
            sync_reg(v, reg);
        }
        regs.regStatus.markFree(idx);
        regs.valueInReg[idx] = BCValue.init;
    }

    bool insideFunction = false;

    extern (C) _jit_getarg_ptrint(jit_reg_t r, jit_node_t* arg)
    {
        version(_64bit)
            _jit_getarg_l(_jit, r, arg); // git it as 64bit
        else
            _jit_getarg_i(_jit, r, arg); // get it as 32bit int
    }

    void beginFunction(uint f = 0, void* fnDecl = null)
    {
        functionCount++;
        fIdx = f;
        assert(currentFrameOffset == 0, "by the time we have call beginFunction either we are freshly initalized or have called endFunction before");
        assert(0, "Not properly implemented yet");
        _jit_prolog();

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

    bool isRegisterPair(register_index regIdx)
    {
        return (regIndex & 0xffff) != 0;
    }

    bool NoRegister(register_index regIdx)
    {
        return regIdx == uint.max;
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

    BCAddr beginJmp() { assert(0, "Not Implemented yet"); }
    void endJmp(BCAddr atIp, BCLabel target) { assert(0, "Not Implemented yet"); }
    void Jmp(BCLabel target) { assert(0, "Not Implemented yet"); }

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
            flagReg = getReg(cond);
        }

        jit_node* at;

        if (ifTrue)
        {
            at = jit_new_node_pww(_jit, jit_code_t.jit_code_bnei, null, flagReg, 0); 
        }
        else
        {
            at = jit_new_node_pww(_jit, jit_code_t.jit_code_beqi, null, flagReg, 0);
        }

        locations[locationCount++] = at;

        return CndJmpBegin(cond, BCAddr(locationCount), ifTrue);
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
        auto lhs_r = getReg(lhs);
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_ww(_jit, jit_code_t.jit_code_movi, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = getReg(rhs);
            _jit_new_node_ww(_jit, jit_code_t.jit_code_movr, lhs_r, rhs_r);
            markUnused(rhs_r);
        }
    }

    void Ult3(BCValue result, BCValue lhs, BCValue rhs)
    {
        auto lhs_r = getReg(lhs);
        auto result_r = JIT_R0;
        if (result == BCValue.init)
        {
            assert(!regs.r0HasCond, "We would override our flag");

            regs.r0HasCond = true;
        }
        else
        {
            result_r = getReg(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_lti_u, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = getReg(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_ltr_u, result_r, lhs_r, rhs_r);
        }
    }

    void Ugt3(BCValue result, BCValue lhs, BCValue rhs)
    {
        auto lhs_r = getReg(lhs);
        auto result_r = JIT_R0;
        if (result == BCValue.init)
        {
            assert(!regs.r0HasCond, "We would override our flag");
            regs.r0HasCond = true;
        }
        else
        {
            result_r = getReg(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_gti_u, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = getReg(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_gtr_u, result_r, lhs_r, rhs_r);
        }
    }
    void Ule3(BCValue result, BCValue lhs, BCValue rhs)
    {
        auto lhs_r = getReg(lhs);
        auto result_r = JIT_R0;
        if (result == BCValue.init)
        {
            assert(!regs.r0HasCond, "We would override our flag");
            regs.r0HasCond = true;
        }
        else
        {
            result_r = getReg(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_lei_u, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = getReg(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_ler_u, result_r, lhs_r, rhs_r);
        }
    }

    void Uge3(BCValue result, BCValue lhs, BCValue rhs)
    {
        auto lhs_r = getReg(lhs);
        auto result_r = JIT_R0;
        if (result == BCValue.init)
        {
            assert(!regs.r0HasCond, "We would override our flag");
            regs.r0HasCond = true;
        }
        else
        {
            result_r = getReg(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_gei_u, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = getReg(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_ger_u, result_r, lhs_r, rhs_r);
        }
    }

    void Lt3(BCValue result, BCValue lhs, BCValue rhs)
    {
        auto lhs_r = getReg(lhs);
        auto result_r = JIT_R0;
        if (result == BCValue.init)
        {
            assert(!regs.r0HasCond, "We would override our flag");
            regs.r0HasCond = true;
        }
        else
        {
            result_r = getReg(result);
        }

        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_lti, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = getReg(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_ltr, result_r, lhs_r, rhs_r);
        }
    }

    void Gt3(BCValue result, BCValue lhs, BCValue rhs)
    {
        auto lhs_r = getReg(lhs);
        auto result_r = JIT_R0;
        if (result == BCValue.init)
        {
            assert(!regs.r0HasCond, "We would override our flag");
            regs.r0HasCond = true;
        }
        else
        {
            result_r = getReg(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_gti, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = getReg(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_gtr, result_r, lhs_r, rhs_r);
        }
    }

    void Le3(BCValue result, BCValue lhs, BCValue rhs)
    {
        auto lhs_r = getReg(lhs);
        auto result_r = JIT_R0;
        if (result == BCValue.init)
        {
            assert(!regs.r0HasCond, "We would override our flag");
            regs.r0HasCond = true;
        }
        else
        {
            result_r = getReg(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_lei, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = getReg(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_ler, result_r, lhs_r, rhs_r);
        }
    }

    void Ge3(BCValue result, BCValue lhs, BCValue rhs)
    {
        auto lhs_r = getReg(lhs);
        auto result_r = JIT_R0;
        if (result == BCValue.init)
        {
            assert(!regs.r0HasCond, "We would override our flag");
            regs.r0HasCond = true;
        }
        else
        {
            result_r = getReg(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_gei, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = getReg(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_ger, result_r, lhs_r, rhs_r);
        }
    }

    void Eq3(BCValue result, BCValue lhs, BCValue rhs) { assert(0, "Not Implemented yet"); }
    void Neq3(BCValue result, BCValue lhs, BCValue rhs) { assert(0, "Not Implemented yet"); }
    void Add3(BCValue result, BCValue lhs, BCValue rhs)
    {
        auto lhs_r = getReg(lhs);
        sync_reg(lhs, lhs_r);

        auto res_r = getReg(result);
        if (commonTypeEnum(lhs.type.type, rhs.type.type).anyOf([BCTypeEnum.f23, BCTypeEnum.f52]))
        {
            if (rhs.vType == BCValueType.Immediate)
            {
                _jit_new_node_www(_jit, jit_code_t.jit_code_addi, res_r, lhs_r, rhs.imm32.imm32);
            }
            else
            {
                auto rhs_r = getReg(rhs);
                _jit_new_node_www(_jit, jit_code_t.jit_code_addr, res_r, lhs_r, rhs_r);
                markUnused(rhs_r);
            }
        }
        else
            assert (0, "only integers are supported right now");
        markUnused(lhs_r);
    }

    void Sub3(BCValue result, BCValue lhs, BCValue rhs)
    {
        auto lhs_r = getReg(lhs);
        sync_reg(lhs, lhs_r);
        auto res_r = getReg(result);
        if (commonTypeEnum(lhs.type.type, rhs.type.type).anyOf([BCTypeEnum.f23, BCTypeEnum.f52]))
        {
            if (rhs.vType == BCValueType.Immediate)
            {
                _jit_new_node_www(_jit, jit_code_t.jit_code_subi, res_r, lhs_r, rhs.imm32.imm32);
            }
            else
            {
                auto rhs_r = getReg(rhs);
                _jit_new_node_www(_jit, jit_code_t.jit_code_subr, res_r, lhs_r, rhs_r);
                markUnused(rhs_r);
            }
        }
        else
            assert (0, "only integers are supported right now");
        markUnused(lhs_r);
        sync_reg(result, res_r);
    }

    void Mul3(BCValue result, BCValue lhs, BCValue rhs)
    {
        auto lhs_r = getReg(lhs);
        sync_reg(lhs, lhs_r);
        auto res_r = getReg(result);
        if (commonTypeEnum(lhs.type.type, rhs.type.type).anyOf([BCTypeEnum.f23, BCTypeEnum.f52]))
        {
            if (rhs.vType == BCValueType.Immediate)
            {
                _jit_new_node_www(_jit, jit_code_t.jit_code_muli, res_r, lhs_r, rhs.imm32.imm32);
            }
            else
            {
                auto rhs_r = getReg(rhs);
                _jit_new_node_www(_jit, jit_code_t.jit_code_mulr, res_r, lhs_r, rhs_r);
                markUnused(rhs_r);
            }
        }
        else
            assert (0, "only integers are supported right now");
        markUnused(lhs_r);
        sync_reg(result, res_r);
    }

    void Div3(BCValue result, BCValue lhs, BCValue rhs)
    {
        auto lhs_r = getReg(lhs);
        sync_reg(lhs, lhs_r);
        auto res_r = getReg(result);
        if (commonTypeEnum(lhs.type.type, rhs.type.type).anyOf([BCTypeEnum.f23, BCTypeEnum.f52]))
        {
            if (rhs.vType == BCValueType.Immediate)
            {
                _jit_new_node_www(_jit, jit_code_t.jit_code_divi, res_r, lhs_r, rhs.imm32.imm32);
            }
            else
            {
                auto rhs_r = getReg(rhs);
                _jit_new_node_www(_jit, jit_code_t.jit_code_divr, res_r, lhs_r, rhs_r);
                markUnused(rhs_r);
            }
        }
        else
            assert (0, "only integers are supported right now");
        markUnused(lhs_r);
        sync_reg(result, res_r);
    }

    void Udiv3(BCValue result, BCValue lhs, BCValue rhs)
    {
        auto lhs_r = getReg(lhs);
        sync_reg(lhs, lhs_r);
        auto res_r = getReg(result);
        if (commonTypeEnum(lhs.type.type, rhs.type.type).anyOf([BCTypeEnum.f23, BCTypeEnum.f52]))
        {
            if (rhs.vType == BCValueType.Immediate)
            {
                _jit_new_node_www(_jit, jit_code_t.jit_code_divi_u, res_r, lhs_r, rhs.imm32.imm32);
            }
            else
            {
                auto rhs_r = getReg(rhs);
                _jit_new_node_www(_jit, jit_code_t.jit_code_divr_u, res_r, lhs_r, rhs_r);
                markUnused(rhs_r);
            }
        }
        else
            assert (0, "only integers are supported right now");
        markUnused(lhs_r);
        sync_reg(result, res_r);
    }

    void And3(BCValue result, BCValue lhs, BCValue rhs)
    {
        auto lhs_r = getReg(lhs);
        sync_reg(lhs, lhs_r);
        auto res_r = getReg(result);
        if (commonTypeEnum(lhs.type.type, rhs.type.type) == BCTypeEnum.i32)
        {
            if (rhs.vType == BCValueType.Immediate)
            {
                _jit_new_node_www(_jit, jit_code_t.jit_code_andi, res_r, lhs_r, rhs.imm32.imm32);
            }
            else
            {
                auto rhs_r = getReg(rhs);
                _jit_new_node_www(_jit, jit_code_t.jit_code_andr, res_r, lhs_r, rhs_r);
                markUnused(rhs_r);
            }
        }
        else
            assert (0, "only 32 bit integers are supported right now");
        markUnused(lhs_r);
        sync_reg(result, res_r);
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
        assert(reg.spilled[idx] == false);
        const idx = r - jit_r(0);
        assert(idx < RegisterState.nTempRegs);
        assert(idx == 1 || idx == 2); // might be relaxed later
        _jit_getarg_ptrint(jit_r(0), context);
        auto offset = (ContextOffsets.rSpill + (size_t.sizeof * idx));
        version(_64bit)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_stxi_l, offset, jit_r(0), r);
        }
        else
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_stxi_i, offset, jit_r(0), r);
        }
        regs.spilled[idx] = true;
    }

    void storeToContext(jit_reg_t r, ContextOffset offset, Spillidx = 0)
    {

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
        SliceDescriptor desc = *(cast(SliceDescriptor*)toRealPointer(SliceDescPtr));

    }

    extern (C) void RT_NullPtrCheck(RuntimeContext* context)
    {
        if (context.rtArg0 == 0)
        {
         //   context.assertion = 
        }
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

        assert(isTmepReg(tmpReg));
        assert(contextPtrReg == jit_r(0));
        assert(!isRegisterPair(idx));

    }

    void Load32(BCValue to, BCValue from)
    {
        auto from_r = getReg(from);
        // this is the address we want to load from
        auto to_r = getReg(to);
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
        sync_reg(to, to_r);
    }
    void Store32(BCValue to, BCValue from) { assert(0, "Not Implemented yet"); }
    void Load64(BCValue to, BCValue from) { assert(0, "Not Implemented yet"); }
    void Store64(BCValue to, BCValue from) { assert(0, "Not Implemented yet"); }
    void Alloc(BCValue heapPtr, BCValue size) { assert(0, "Not Implemented yet"); }
    void Not(BCValue result, BCValue val) { assert(0, "Not Implemented yet"); }
    void Ret(BCValue val)
    {
        _jit_getarg_ptrint(JIT_R0, context_arg);
        load_size_t_immoffset(JIT_R0, JIT_R0, offset

        if (val.type == BCTypeEnum.i64 || val.type == BCTypeEnum.u64)
        {
            // 8 byte return uses R1 for the lo word and R2 for the high word
            // runtime return type information (is it an error or execption is passed in R0)


        }
        else
        {
            // regular 4 byte return uses R1 for the value and R0 for the runtime return type

        }
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
            auto r = regs.aquireTmpReg();
            assert(r != jit_reg_t._NOREG);
            loadImm32(r, line);
            storeToContext(r, ContextLineOffset);
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
