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

    ubyte* framePointer;
    ubyte* stackPointer;

    ubyte* stackDataBegin;
    uint* stackDataLength;

    ubyte* heapDataBegin;
    uint* heapDataLength;
    // bc function to be determined.
    void* functions;

    //
    uint* heapSizeP;
    uint* stackSizeP;

    BCValue returnValue;

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

enum ContextOffset
{
    rSpill = RuntimeContext.init.rSpill.offsetof,

    rtArg0 = RuntimeContext.init.rtArg0.offsetof,
    rtArg1 = RuntimeContext.init.rtArg1.offsetof,
    rtLongArg0 = RuntimeContext.init.rtLongArg0.offsetof,

    rtArg2 = RuntimeContext.init.rtArg2.offsetof,
    rtArg3 = RuntimeContext.init.rtArg3.offsetof,
    rtLongArg1 = RuntimeContext.init.rtLongArg1.offsetof,

    currentLine = RuntimeContext.init.currentLine.offsetof,
    currentFile = RuntimeContext.init.currentFile.offsetof,

    framePointer = RuntimeContext.init.framePointer.offsetof,
    stackPointer = RuntimeContext.init.stackPointer.offsetof,

    stackDataBegin = RuntimeContext.init.stackDataBegin.offsetof,
    heapDataBegin = RuntimeContext.init.heapDataBegin.offsetof,
    heapDataLength = RuntimeContext.init.functions.offsetof,
    functions = RuntimeContext.init.functions.offsetof,

    heapSizeP = RuntimeContext.init.heapSizeP.offsetof,
    stackSizeP = RuntimeContext.init.stackSizeP.offsetof,

    returnValue = RuntimeContext.init.returnValue.offsetof,
}

pragma(msg, "framePtrOffset: ", ContextOffset.framePointer);
pragma(msg, "stackPtrOffset: ", ContextOffset.stackPointer);

pure nothrow @safe @nogc
int min(int x, int y) { pragma(inline, true); return (((x) < (y)) ? (x) : (y)); }
pure nothrow @safe @nogc
int max(int x, int y) { pragma(inline, true); return (((x) > (y)) ? (x) : (y)); }

bool isRegisterPair(register_index regIdx)
{
    return (regIdx & ~0xffff) != 0;
}

bool NoRegister(register_index regIdx)
{
    return regIdx == 0;
}

static register_index reg2idx(jit_reg_t reg)
{
    assert(reg >= jit_v(0) && reg < jit_v(jit_v_num()),
        "register is not a valid v-reg: " ~ enumToString(reg));
    return (reg  - jit_v(0)) + 1;
}

static jit_reg_t idx2reg(register_index idx)
{
    assert(idx >= 1 && idx <= jit_v_num(),
        "register index not in range");
    return  cast(jit_reg_t) (jit_v(0) + (idx - 1));
}


import std.file : append;
struct RegisterState
{
    enum nRegs = jit_v_num() - 1;
    enum nTempRegs = jit_r_num();


    /// frame of the value offset which is currently held by this register
    ushort[nRegs] frameOffsetInReg = 0;

    register_index[nRegs] pairedWith = 0;

    register_index aquireTempReg(bool paired)
    {
        assert(0, "TODO implement me.");
    }

    void releaseTempReg(register_index tmpRegIdx)
    {
        assert(0, "TODO implement me.");
    }


    /// set two registers as paried
    void set_paired(jit_reg_t reg1, jit_reg_t reg2)
    {
        assert(
            (
                regStatus.isUnused(reg1) && regStatus.isUnused(reg2) 
                && !regStatus.isDirty(reg1) && !regStatus.isDirty(reg2)
            )
            || (regStatus.isFree(reg1) && regStatus.isFree(reg2))

        );
        assert(pairedWith[reg1] == 0 &&
               pairedWith[reg2] == 0);
        const idx1 = reg2idx(reg1);
        const idx2 = reg2idx(reg2);

        const pair_idx = (min(idx1, idx2) | (max(idx1, idx2) << 16));

        pairedWith[idx1] = pair_idx;
        pairedWith[idx2] = pair_idx;
    }

    /// value in register is unlikely to be used again
    /// mark this register as a canidate for eviction
    void markUnused(jit_reg_t reg)
    {
        const idx = reg2idx(reg);
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

    register_index get_index(jit_reg_t reg)
    {
        assert(reg >= jit_v(0) && reg < jit_v(jit_v_num()));
        const idx = reg - jit_v(0);
        const pair_idx = pairedWith[idx];
        if (pair_idx != 0)
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
        return (evictionCounter++ % nRegs) + 1;
    }
    bool[nTempRegs] spilled;
    bool r2HasCond;
}

bool isInRegisterRange(jit_reg_t r, jit_reg_t beg, jit_reg_t end)
{
    bool result = true;
    if (r < beg || r >= end)
        return result = false;
    
    return result;
}

jit_reg_t LowReg(register_index Idx)
{
    assert(isRegisterPair(Idx));
    return jit_v((Idx & 0xFFFF) - 1);
}

jit_reg_t HighReg(register_index Idx)
{
    assert(isRegisterPair(Idx));
    return jit_v((Idx >> 16) - 1);
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


    // --------- Register Management Helpers ---------------


    /// get a register for a stack value
    /// either we already have it in a register
    /// or we allocate a register for it
    /// if needed we evict a value
    register_index getRegisterIndex(BCValue v, bool pair = false)
    {
        assert(v);
        assert(v.vType != BCValueType.Immediate);

        append("reg_alloc.log", cast(void[])("\nrequesting reg for: " ~ v.toString()));

        foreach(int i, frameOffset;regs.frameOffsetInReg)
        {
            if (frameOffset == v.stackAddr)
            {
                append("reg_alloc.log", cast(void[])("... found in regIdx: " ~ itos(i + 1)) ~"\n");
                
                assert(!regs.pairedWith[i]);
                regs.regStatus.markUsed(i + 1);
                return i + 1;
            }
        }
        append("reg_alloc.log", cast(void[])("... not found in register allocation table .. allocating ... \n"));
        return allocReg(v.stackAddr, pair);
    }


    // calls get register index and verfies it's not a register pair
    jit_reg_t getSingleRegister(BCValue v)
    {
        if (basicTypeSize(v.type.type) <= 4)
        {
            const iReg =
                cast(int)(getRegisterIndex(v, false));
            assert(iReg <= jit_v_num() && iReg > 0);
            return jit_v(iReg - 1);
        }
        else
        {
            // return JIT_NOREG;
            assert(0, "we should never have gotten here, value doesn't fit into a single reg");
        }
    }

    register_index getPairedRegisterIndex(BCValue v)
    {
        if (basicTypeSize(v.type.type) == 8)
        {
            const RegIdx =
                cast(int)(getRegisterIndex(v, true));
            assert (isRegisterPair(RegIdx));
            assert(isInRegisterRange(LowReg(RegIdx), jit_v(0), jit_v(jit_v_num() - 1)));
            assert(isInRegisterRange(HighReg(RegIdx), jit_v(1), jit_v(jit_v_num())));
            return RegIdx;
            // assert(iReg <= jit_v_num() && iReg > 0);
            // return jit_v(iReg - 1);
        }
        else
        {
            // return JIT_NOREG;
            assert(0, "we should never have gotten here, value doesn't fit into a single reg");
        }
    }


    /// must not be called by the backend proper!
    /// writes the value of a register back into the stack location it mirrors
    /// only happens when stack values are evicted or on calls when we have to flush the registers
    void write_stack(ushort fOffset, register_index reg)
    {

        append("reg_alloc.log", "Writing  regIdx: " ~ itos(reg) ~ " into offset " ~ itos(fOffset) ~ "\n");
        assert(reg);

        const low_idx = (reg & 0xffff);
        assert(low_idx);
        assert(regs.regStatus.isDirty(low_idx), "syncing a non dirtied register");
        _jit_getarg_ptrint(jit_r(0), context_arg);
        const framePointerReg = jit_r(1);
        // load fp into framePointer reg
        load_size_t_immoffset(framePointerReg, jit_r(0), ContextOffset.framePointer);
        
        // low part of the register index is always valid

        assert(regs.regStatus.isDirty(low_idx));
        
        jit_reg_t r1 = jit_v(low_idx - 1);
        // load the frameOffset from the frame pointer into r1
        _jit_name(_jit, "write_stack low Idx");
        //printReg(r1, "value to be written to stack");
        _jit_new_node_www(_jit, jit_code_t.jit_code_stxi_i, fOffset, framePointerReg, r1);
        regs.regStatus.markClean(low_idx);
        
        if (const high_idx = (reg >> 16))
        {
            assert(regs.regStatus.isDirty(high_idx));
            jit_reg_t r2 = jit_v(high_idx - 1);
            _jit_new_node_www(_jit, jit_code_t.jit_code_stxi_i, fOffset + 4, framePointerReg, r2);
            regs.regStatus.markClean(high_idx);
        }
    }

    void clearR0()
    {
        _jit_name(_jit, "clear R0".ptr);
        _jit_new_node_ww(_jit, jit_code_t.jit_code_xorr, jit_r(0), jit_r(0));
    }

    void printReg(jit_reg_t r, const(char)* msg)
    {
        bool isTempReg = isInRegisterRange(r, jit_r(0), jit_r(jit_r_num()));
        assert(!isTempReg, "The temp registers cannot be printed: " ~ "r0:" ~ itos(jit_r(0)) ~ " r:"  ~itos(r));

        spill_to_context();
        scope(exit) unspill_from_context();

        // now we need to clear our r0 reg otherwise someone might write into your ctx ... how dare they!
        clearR0();

        import core.stdc.stdio;
        _jit_prepare(_jit);
        _jit_pushargi(_jit, cast(jit_word_t)(cast(void*)"%s reg: %p\n".ptr));
        _jit_ellipsis(_jit);
        _jit_pushargi(_jit, cast(jit_word_t) msg);
        _jit_pushargr(_jit, r);
        _jit_finishi(_jit, cast(void*)&printf);

    }

    void jit_halt()
    {
        static hlt()
        {
            version(X86Asm)
            {
                asm { int 3; }
            }
        }
        _jit_new_node_w(_jit, jit_code_t.jit_code_calli, cast(jit_word_t)&hlt);
    }

    /// must not be called by the backend proper!
    /// reads a stack value into a register ... this only happens on allocation
    void read_stack(ushort fOffset, register_index reg)
    {
        append("reg_alloc.log", "Reading offset" ~ itos(fOffset) ~ "into regIdx: " ~ itos(reg) ~ "\n");
        assert(reg);
        assert(!regs.regStatus.isFree(reg & ushort.max));
        _jit_getarg_ptrint(jit_r(0), context_arg);
        const framePointerReg = jit_r(1);

        // load fp into framePointer reg
        _jit_name(_jit, "load context pointer for read_stack");
        load_size_t_immoffset(framePointerReg, jit_r(0), ContextOffset.framePointer);

        // low part of the register index is always valid
        const low_idx = (reg & 0xffff);
        assert(low_idx && low_idx <= jit_v_num());
        assert(!regs.regStatus.isDirty(low_idx));

        jit_reg_t r1 = jit_v(low_idx - 1);
        // load the frameOffset from the frame pointer into r1
        _jit_name(_jit, ("read stack low idx for offset: " ~ itos(fOffset) ~ "\0").ptr);
        _jit_new_node_www(_jit, jit_code_t.jit_code_ldxi_i, r1, framePointerReg, fOffset);
        _jit_name(_jit, "read stack low idx done");

        if (const high_idx = (reg >> 16))
        {
            assert(!regs.regStatus.isDirty(high_idx));
            jit_reg_t r2 = jit_v(high_idx - 1);
            _jit_new_node_www(_jit, jit_code_t.jit_code_ldxi_i, r2, framePointerReg, fOffset + 4);
        }


    }

    register_index allocReg(ushort fOffset, bool wantPair = false)
    {
        register_index result = 0;
        with(regs) with(regStatus)
        {
            // first let's look for a free register.
            register_index freeReg = nextFree();
            if (freeReg != 0)
            {
                markUsed(freeReg); // we need to mark it used immediately otherwise we get it again when asking for a pair
                assert(pairedWith[freeReg - 1] == 0, "free register cannot have pairing relationships!");
                if (wantPair)
                {
                    // when we want a pair another register has to be allocated.
                    register_index freeReg2 = nextFree();
                    if (freeReg2)
                    {
                        markUsed(freeReg2);
                        auto lw_reg = min(freeReg, freeReg2);
                        auto hi_reg = max(freeReg, freeReg2);

                        set_paired(idx2reg(freeReg), idx2reg(freeReg2));
                        result = lw_reg | hi_reg << 16;
                        regs.regStatus.markUsed(freeReg);
                        append("reg_alloc.log", cast(void[])("... Found free Regs: " ~ 
                        enumToString(jit_v(freeReg - 1)) ~ " : " ~
                        enumToString(jit_v(freeReg2 - 1))
                            ~ "\n\n"
                    ));

                    }
                }
                else
                {
                    // we have a free register let's use it
                    regs.regStatus.markUsed(freeReg);
                    frameOffsetInReg[freeReg - 1] = fOffset;
                    result = freeReg;
                    append("reg_alloc.log", cast(void[])("... Found free Reg: " ~ enumToString(jit_v(result - 1))) ~ "\n\n");
                }
            }
            // no free register now we need to evict ... sigh
            // first let's look for someome marked for eviction
            register_index unusedReg = nextUnused();
            if (result == 0 && unusedReg != 0)
            {
                // depending on wheter we need a register pair or not we have to do
                // diffrent things here.
                
                // if the unused reg was part of a register pair
                // let's see if we can find a single one
                // for easier searching lets start with a while loop right away
                typeof(unusedBitfield) masked = unusedBitfield;

                // we keep searching for a long as there are unused regs
                // for one which is a single pair
                auto pair_idx = pairedWith[unusedReg - 1];
                while ((!!pair_idx) != wantPair)
                {
                    masked &= (~(1 << (unusedReg - 1)));
                    if (masked != 0)
                    {
                        import core.bitop : bsf;
                        unusedReg = bsf(masked);
                        pair_idx = pairedWith[unusedReg - 1];
                    }
                }
                // if we don't find one we split the last unused paired reg
                if (pair_idx)
                {
                    // if our last canidate is paired we need to unpair it
                    // of course it's partner should also be unused.
                    const idx1 = (pair_idx & 0xFFFF);
                    const idx2 = (pair_idx >> 16);
                    assert(isUnused(idx1) && isUnused(idx2));
                    unpair(pair_idx);
                }
                else
                {
                    if (regs.regStatus.isDirty(unusedReg))
                    {
                        const syncWithOffset = frameOffsetInReg[result - 1];
                        write_stack(syncWithOffset, unusedReg);
                    }
                }

                result = unusedReg;
            } else if (!result)
                // we got no one marked for evication
                // well then we get a 'random' canidate and force evict
            {
                auto nextEvict = nextEvictim();
                const pair_idx = pairedWith[nextEvict - 1];
                result = nextEvict;
                // we sync on eviction
                // now we need to unpair if needed
                if (pair_idx != 0)
                {
                    unpair(pair_idx);
                    import core.stdc.stdio; printf("unpairing\n");
                }
                else
                {
                    if (regs.regStatus.isDirty(result))
                    {
                        const syncWithOffset = frameOffsetInReg[result - 1];
                        write_stack(syncWithOffset, result);
                    }
                }
            }

            /// 
            const low_idx = (result & 0xFFFF);
            assert(!regs.regStatus.isDirty(low_idx));
            frameOffsetInReg[low_idx - 1] = fOffset;
            assert(result != 0);
            read_stack(fOffset, result);
        }
        
        return result;
    }

    /// writes the pair to stack and unpairs and frees the regs
    void unpair(register_index idx)
    {
        assert(isRegisterPair(idx));
        //before we unpair we need to sync.
        const lw_idx = idx & 0xFFFF;
        // the lower of the pair always has the value
        
        auto fOffset = regs.frameOffsetInReg[lw_idx];
        write_stack(fOffset, idx);
        const hi_idx = idx >> 16;
        
        regs.pairedWith[lw_idx] = 0;
        regs.pairedWith[hi_idx] = 0;
        
        regs.regStatus.markFree(lw_idx);
        regs.regStatus.markFree(hi_idx);
        regs.regStatus.markClean(hi_idx);
    }

    void sync_stack()
    {
        append("reg_alloc.log", "Froce syncing all dirty registers");
        auto dirty = regs.regStatus.nextDirty();
        while(dirty)
        {
            auto offset = regs.frameOffsetInReg[dirty - 1];
            write_stack(offset, dirty);
            dirty = regs.regStatus.nextDirty();
        }
    }

    /// evict the value from the register
    void freeReg(jit_reg_t reg)
    {
        const idx = reg - jit_v(0);
        if (regs.regStatus.isDirty(idx))
        {
            const v = regs.frameOffsetInReg[idx];
            write_stack(v, reg);
        }
        regs.regStatus.markFree(idx);
        regs.frameOffsetInReg[idx] = 0;
    }


    // ---------- backend proper ----------

    extern (C) void _jit_getarg_ptrint(jit_reg_t r, jit_node_t* arg, int line = __LINE__)
    {
        version(_64bit)
            _jit_getarg_l(_jit, r, arg); // git it as 64bit
        else
            _jit_getarg_i(_jit, r, arg); // get it as 32bit int
        // _jit_note(_jit, "getarg".ptr, line);
    }

    void Initialize()
    {
        // make sure jit state is initalized;
        //if (_jit) _jit_destroy_state(_jit);
        if(!_jit) init_jit(null);
        _jit = jit_new_state();
        currentFrameOffset = 4; // we don't start at zero because 0 is a special error value
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

    void beginFunction(uint f = 0, void* fnDecl = null)
    {
        functionCount++;
        auto fIdx = f;
        // assert(currentFrameOffset == 0, "by the time we have call beginFunction either we are freshly initalized or have called endFunction before");
        // assert(0, "Not properly implemented yet");
        _jit_prolog(_jit);

        _jit_name(_jit, "context_arg".ptr);
        context_arg = _jit_arg(_jit); // RuntimeContext*

        _jit_getarg_ptrint(JIT_R0, context_arg);
        // load context pointer into R0
        load_size_t_immoffset(JIT_R2, JIT_R0, ContextOffset.stackPointer);
        // load store stackPtr in FramePtr
        store_size_t_immoffset(JIT_R0, ContextOffset.framePointer, JIT_R2);
        // asm { int 3; }
    }

    void endFunction()
    {
        _jit_epilog(_jit);
        currentFrameOffset = 4;
        locationCount = 0;
        parameterCount = 0;

        //assert(0, "Not properly implemented yet");
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
        auto p = BCParameter(parameterCount, bct, currentFrameOffset, name);
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

    void destroyTemporary(BCValue v, uint userSize = 0)
    {
        assert(!needsUserSize(v.type.type) || userSize > 0);
        auto stackSize = (typeIsPointerOnStack(v.type.type) ? PtrSize :  basicTypeSize(v.type.type));

        if (needsUserSize(v.type.type))
        {
            stackSize = userSize;
        }

        currentFrameOffset -= stackSize;
    }

    BCValue genLocal(BCType bct, string name, uint userSize = 0)
    {
        _jit_note(_jit, __FILE__.ptr, __LINE__);
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
        _jit_patch_at(_jit, locations[atIp.addr], locations[target.addr.addr - 1]);
    }

    void Jmp(BCLabel target)
    {
        auto jmp = _jit_new_node_p(_jit, jit_code_t.jit_code_jmpi, null);
        _jit_patch_at(_jit, jmp, locations[target.addr.addr - 1]);
    }

    /// peform a conditional jmp based on the value of the parameter cond
    /// BCValue.init is special and means jmp based upon the last condition flag
    CndJmpBegin beginCndJmp(BCValue cond = BCValue.init, bool ifTrue = false)
    {
        auto flagReg = JIT_R2;
        if (!cond)
        {
            assert(regs.r2HasCond);
        }
        else
        {
            flagReg = getSingleRegister(cond);
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
        auto lhs_r = getSingleRegister(lhs);
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_ww(_jit, jit_code_t.jit_code_movi, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = getSingleRegister(rhs);
            _jit_new_node_ww(_jit, jit_code_t.jit_code_movr, lhs_r, rhs_r);
            regs.markUnused(rhs_r);
        }
        regs.regStatus.markDirty(reg2idx(lhs_r));
    }

    void Ult3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = getSingleRegister(lhs);
        auto result_r = JIT_R2;
        if (!result)
        {
            assert(!regs.r2HasCond, "We would override our flag");
            regs.r2HasCond = true;
        }
        else
        {
            result_r = getSingleRegister(result);
        }

        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_lti_u, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = getSingleRegister(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_ltr_u, result_r, lhs_r, rhs_r);
            regs.markUnused(rhs_r);
        }
    }

    void Ugt3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = getSingleRegister(lhs);
        auto result_r = JIT_R2;
        if (!result)
        {
            assert(!regs.r2HasCond, "We would override our flag");
            regs.r2HasCond = true;
        }
        else
        {
            result_r = getSingleRegister(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_gti_u, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = getSingleRegister(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_gtr_u, result_r, lhs_r, rhs_r);
            regs.markUnused(rhs_r);
        }
    }

    void Ule3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = getSingleRegister(lhs);
        auto result_r = JIT_R2;
        if (!result)
        {
            assert(!regs.r2HasCond, "We would override our flag");
            regs.r2HasCond = true;
        }
        else
        {
            result_r = getSingleRegister(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_lei_u, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = getSingleRegister(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_ler_u, result_r, lhs_r, rhs_r);
            regs.markUnused(rhs_r);
        }
    }

    void Uge3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = getSingleRegister(lhs);
        auto result_r = JIT_R2;
        if (!result)
        {
            assert(!regs.r2HasCond, "We would override our flag");
            regs.r2HasCond = true;
        }
        else
        {
            result_r = getSingleRegister(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_gei_u, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = getSingleRegister(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_ger_u, result_r, lhs_r, rhs_r);
        }
    }

    void Lt3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = getSingleRegister(lhs);
        auto result_r = JIT_R2;
        if (!result)
        {
            assert(!regs.r2HasCond, "We would override our flag");
            regs.r2HasCond = true;
        }
        else
        {
            result_r = getSingleRegister(result);
        }

        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_lti, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = getSingleRegister(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_ltr, result_r, lhs_r, rhs_r);
        }
    }

    void Gt3(BCValue result, BCValue lhs, BCValue rhs)
    {
        _jit_note(_jit, __FILE__.ptr, __LINE__);
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = getSingleRegister(lhs);
        auto result_r = JIT_R2;
        if (!result)
        {
            assert(!regs.r2HasCond, "We would override our flag");
            regs.r2HasCond = true;
        }
        else
        {
            result_r = getSingleRegister(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_gti, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = getSingleRegister(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_gtr, result_r, lhs_r, rhs_r);
        }
        _jit_note(_jit, __FILE__.ptr, __LINE__);
    }

    void Le3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = getSingleRegister(lhs);
        auto result_r = JIT_R2;
        if (!result)
        {
            assert(!regs.r2HasCond, "We would override our flag");
            regs.r2HasCond = true;
        }
        else
        {
            result_r = getSingleRegister(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_lei, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = getSingleRegister(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_ler, result_r, lhs_r, rhs_r);
        }
    }

    void Ge3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = getSingleRegister(lhs);
        auto result_r = JIT_R2;
        if (!result)
        {
            assert(!regs.r2HasCond, "We would override our flag");
            regs.r2HasCond = true;
        }
        else
        {
            result_r = getSingleRegister(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_gei, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = getSingleRegister(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_ger, result_r, lhs_r, rhs_r);
        }
    }

    void Eq3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = getSingleRegister(lhs);
        auto result_r = JIT_R2;
        if (!result)
        {
            assert(!regs.r2HasCond, "We would override our flag");
            regs.r2HasCond = true;
        }
        else
        {
            result_r = getSingleRegister(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_eqi, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = getSingleRegister(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_eqr, result_r, lhs_r, rhs_r);
        }
    }

    void Neq3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = getSingleRegister(lhs);
        auto result_r = JIT_R2;
        if (!result)
        {
            assert(!regs.r2HasCond, "We would override our flag");
            regs.r2HasCond = true;
        }
        else
        {
            result_r = getSingleRegister(result);
        }
        
        if (rhs.vType == BCValueType.Immediate)
        {
            _jit_new_node_www(_jit, jit_code_t.jit_code_nei, result_r, lhs_r, rhs.imm32.imm32);
        }
        else
        {
            auto rhs_r = getSingleRegister(rhs);
            _jit_new_node_www(_jit, jit_code_t.jit_code_ner, result_r, lhs_r, rhs_r);
        }
    }
    void Add3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = getSingleRegister(lhs);

        auto res_r = getSingleRegister(result);
        auto commonType = commonTypeEnum(lhs.type.type, rhs.type.type);
        if (!commonType.anyOf([BCTypeEnum.f23, BCTypeEnum.f52]))
        {
            if (rhs.vType == BCValueType.Immediate)
            {
                _jit_new_node_www(_jit, jit_code_t.jit_code_addi, res_r, lhs_r, rhs.imm32.imm32);
            }
            else
            {
                auto rhs_r = getSingleRegister(rhs);
                _jit_new_node_www(_jit, jit_code_t.jit_code_addr, res_r, lhs_r, rhs_r);
                regs.markUnused(rhs_r);
            }
        }
        else
            assert (0, "only integers are supported right now ... not: " ~ commonType.enumToString());
        regs.markUnused(lhs_r);
    }

    void Sub3(BCValue result, BCValue lhs, BCValue rhs)
    {
        assert(basicTypeSize(lhs.type.type) <= 4);
        auto lhs_r = getSingleRegister(lhs);

        auto res_r = getSingleRegister(result);
        if (!commonTypeEnum(lhs.type.type, rhs.type.type).anyOf([BCTypeEnum.f23, BCTypeEnum.f52]))
        {
            if (rhs.vType == BCValueType.Immediate)
            {
                _jit_new_node_www(_jit, jit_code_t.jit_code_subi, res_r, lhs_r, rhs.imm32.imm32);
            }
            else
            {
                auto rhs_r = getSingleRegister(rhs);
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
        auto lhs_r = getSingleRegister(lhs);
        auto res_r = getSingleRegister(result);
        if (!commonTypeEnum(lhs.type.type, rhs.type.type).anyOf([BCTypeEnum.f23, BCTypeEnum.f52]))
        {
            if (rhs.vType == BCValueType.Immediate)
            {
                _jit_new_node_www(_jit, jit_code_t.jit_code_muli, res_r, lhs_r, rhs.imm32.imm32);
            }
            else
            {
                auto rhs_r = getSingleRegister(rhs);
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
        auto lhs_r = getSingleRegister(lhs);
        auto res_r = getSingleRegister(result);
        if (!commonTypeEnum(lhs.type.type, rhs.type.type).anyOf([BCTypeEnum.f23, BCTypeEnum.f52]))
        {
            if (rhs.vType == BCValueType.Immediate)
            {
                _jit_new_node_www(_jit, jit_code_t.jit_code_divi, res_r, lhs_r, rhs.imm32.imm32);
            }
            else
            {
                auto rhs_r = getSingleRegister(rhs);
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
        auto lhs_r = getSingleRegister(lhs);
        auto res_r = getSingleRegister(result);
        if (!commonTypeEnum(lhs.type.type, rhs.type.type).anyOf([BCTypeEnum.f23, BCTypeEnum.f52]))
        {
            if (rhs.vType == BCValueType.Immediate)
            {
                _jit_new_node_www(_jit, jit_code_t.jit_code_divi_u, res_r, lhs_r, rhs.imm32.imm32);
            }
            else
            {
                auto rhs_r = getSingleRegister(rhs);
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
        auto lhs_r = getSingleRegister(lhs);
        auto res_r = getSingleRegister(result);
        if (commonTypeEnum(lhs.type.type, rhs.type.type) == BCTypeEnum.i32)
        {
            if (rhs.vType == BCValueType.Immediate)
            {
                _jit_new_node_www(_jit, jit_code_t.jit_code_andi, res_r, lhs_r, rhs.imm32.imm32);
            }
            else
            {
                auto rhs_r = getSingleRegister(rhs);
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
            _jit_prepare(_jit);
            _jit_finishr(_jit, JIT_R1);
        }
    }
    void Load8(BCValue to, BCValue from) { assert(0, "Not Implemented yet"); }
    void Store8(BCValue to, BCValue from) { assert(0, "Not Implemented yet"); }
    void Load16(BCValue to, BCValue from) { assert(0, "Not Implemented yet"); }
    void Store16(BCValue to, BCValue from) { assert(0, "Not Implemented yet"); }

    void unspill_from_context()
    {
        foreach(r; jit_r(1) .. jit_r(3))
        {
            const idx = r - jit_r(0);
            assert(regs.spilled[idx] == true);
            assert(idx < RegisterState.nTempRegs);
            assert(idx == 1 || idx == 2); // might be relaxed later
            _jit_getarg_ptrint(jit_r(0), context_arg);
            auto offset = (ContextOffset.rSpill + (size_t.sizeof * idx));
            load_size_t_immoffset(r, jit_r(0), cast(int)offset);
            regs.spilled[idx] = false;
        }
    }

    void spill_to_context()
    {
        foreach(r; jit_r(1) .. jit_r(3))
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
        _jit_note(_jit, __FILE__.ptr, __LINE__);
        version(_64bit)
        {
            return _jit_new_node_www(_jit, jit_code_t.jit_code_ldxi_l, to_r, from_r, offset);
        }
        else
        {
            return _jit_new_node_www(_jit, jit_code_t.jit_code_ldxi_i, to_r, from_r, offset);
        }

    }

    jit_node_t* store_size_t_immoffset(jit_reg_t to_r, int offset, jit_reg_t from_r)
    {
        version(_64bit)
        {
            return _jit_new_node_www(_jit, jit_code_t.jit_code_stxi_l, offset, to_r, from_r);
        }
        else
        {
            return _jit_new_node_www(_jit, jit_code_t.jit_code_stxi_i, offset, to_r, from_r);
        }
    }

    extern (C) uint RT_BoundsCheck(RuntimeContext* context)
    {
        auto SliceDescPtr = context.rtArg0;
        auto Index = context.rtArg1;
        SliceDescriptor desc = *(cast(SliceDescriptor*)context.toRealPointer(SliceDescPtr));
        import core.stdc.stdio;
        printf("doing a bounds check for idx %d\n", Index);

        return 0;
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
        auto from_r = getSingleRegister(from);
        // this is the address we want to load from
        auto to_r = getSingleRegister(to);
        // this is the stack address we want to load into
        assert(!regs.r2HasCond);

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
    void Store32(BCValue to, BCValue from)
    {
        BCValue tmp;
        BCValue tmp2;
        if (from.vType == BCValueType.Immediate)
        {
            tmp = genTemporary(i32Type);
            Set(tmp, from);
            from = tmp;
        }

        if (to.vType == BCValueType.Immediate)
        {
            tmp2 = genTemporary(i32Type);
            Set(tmp2, to);
            to = tmp2;
        }

        auto to_r = getSingleRegister(to);
        // this is the address we want to store to
        auto from_r = getSingleRegister(from);
        // this is the stack address we want to store into it
        _jit_getarg_ptrint(JIT_R0, context_arg);
        // get the ctx to load the heap ptr from
        load_size_t_immoffset(JIT_R1, JIT_R0, ContextOffset.heapDataBegin);
        // R1 is now the heapPtr

        _jit_new_node_www(_jit, jit_code_t.jit_code_stxr_i, JIT_R1, to_r, from_r);

        // now do the store
    }
    void Load64(BCValue to, BCValue from)
    {
        auto from_r = getSingleRegister(from);
        // this is the address we want to load from
        auto to_idx = getPairedRegisterIndex(to);
        // this is the stack address we want to load into
        assert(!regs.r2HasCond);

        _jit_getarg_ptrint(JIT_R0, context_arg);
        static if (1)
        {
            nullptr_check(from_r);
            load_size_t_immoffset(JIT_R2, JIT_R0, heapDataLengthOffset);
            heapoverflow_check(from_r, JIT_R2, JIT_R0);
        }
        load_size_t_immoffset(JIT_R1, JIT_R0, heapDataPtrOffset);

        _jit_new_node_www(_jit, jit_code_t.jit_code_ldxr_i, LowReg(to_idx), from_r, JIT_R2);

        // load high part.
        _jit_new_node_www(_jit, jit_code_t.jit_code_addi, jit_r(2), jit_r(2), 4);

        _jit_new_node_www(_jit, jit_code_t.jit_code_ldxr_i, HighReg(to_idx), from_r, JIT_R2);
    }
    void Store64(BCValue to, BCValue from) { assert(0, "Not Implemented yet"); }
    void Alloc(BCValue heapPtr, BCValue size) { assert(0, "Not Implemented yet"); }
    void Not(BCValue result, BCValue val) { assert(0, "Not Implemented yet"); }
    void Ret(BCValue val)
    {
        sync_stack();
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
        //assert(0, "TODO implement this");
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
    void Comment(const(char)[] comment) { _jit_name(_jit, comment.ptr); }
    void Line(uint line)
    {
        if (line != lastLine && context_arg !is null)
        { 
            //const rIdx = regs.aquireTempReg(false);
            auto rIdx = 1;
            // hard_coded to r1 for now
            assert(rIdx != 0, "Couldn't aquire TempReg");
            assert((rIdx >> 16) == 0, "I asked for an unpaired reg but got a paired one");
            assert(rIdx != 0, "r0 must not be used as TempReg");

            const r = jit_r(rIdx);

            loadImm32(r, line);
            _jit_getarg_ptrint(jit_r(0), context_arg);
            storeToContext32(r, ContextOffset.currentLine, jit_r(0));
            //regs.releaseTempReg(r);
        }
    }
    void File(string filename)
    {

    }
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
