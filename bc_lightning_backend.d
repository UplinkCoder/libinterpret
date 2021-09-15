﻿module dmd.ctfe.bc_lightning_backend;

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



struct RegisterState
{
    enum nRegs = jit_v_num();
    BCValue[nRegs] valueInReg;
    RegStatusList!(nRegs) regStatus;
    uint evictionCounter;

    // it's a pun ... I could not resist.
    uint nextEvictim()
    {
        return evictionCounter++ % nRegs;
    }
}

struct LightingGen
{
    jit_state* _jit;
    RegisterState regs;

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

    alias jit_fn = extern (C) int function (BCHeap * heap, int nArgs, ReturnValue* returnValue);

    void Initialize()
    {
        // make sure jit state is initalized;
        if (_jit) _jit_destroy_state(_jit);
        else init_jit(null);
        _jit = jit_new_state();
    }

    void Finalize()
    {
        _jit_clear_state(_jit);

        finish_jit();
    }

    /// get a register for a stack value
    /// either we already have it in a register
    /// or we allocate a register for it
    /// if needed we evict a value
    jit_reg_t getReg(BCValue v)
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

    /// write the value in the register to the stack location it mirrors
    void sync_reg(BCValue v, jit_reg_t reg)
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

    void beginFunction(uint f = 0, void* fnDecl = null)  { assert(0, "Not Implemented yet"); }
    void endFunction() { assert(0, "Not Implemented yet"); }
    BCLabel genLabel() { assert(0, "Not Implemented yet"); }
    BCValue genParameter(BCType bct, string name = null) { assert(0, "Not Implemented yet"); }
    BCValue genTemporary(BCType bct) { assert(0, "Not Implemented yet"); }
    void destroyTemporary(BCValue tmp) { assert(0, "Not Implemented yet"); }
    BCValue genLocal(BCType bct, string name) { assert(0, "Not Implemented yet"); }
    BCAddr beginJmp() { assert(0, "Not Implemented yet"); }
    void endJmp(BCAddr atIp, BCLabel target) { assert(0, "Not Implemented yet"); }
    void Jmp(BCLabel target) { assert(0, "Not Implemented yet"); }
    CndJmpBegin beginCndJmp(BCValue cond = BCValue.init, bool ifTrue = false) { assert(0, "Not Implemented yet"); }
    void endCndJmp(CndJmpBegin jmp, BCLabel target) { assert(0, "Not Implemented yet"); }
    void emitFlg(BCValue lhs) { assert(0, "Not Implemented yet"); }
    void Throw(BCValue e) { assert(0, "Not Implemented yet"); }
    void PushCatch() { assert(0, "Not Implemented yet"); }
    void PopCatch() { assert(0, "Not Implemented yet"); }
    void Set(BCValue lhs, BCValue rhs) { assert(0, "Not Implemented yet"); }
    void Ult3(BCValue result, BCValue lhs, BCValue rhs) { assert(0, "Not Implemented yet"); }
    void Ugt3(BCValue result, BCValue lhs, BCValue rhs) { assert(0, "Not Implemented yet"); }
    void Ule3(BCValue result, BCValue lhs, BCValue rhs) { assert(0, "Not Implemented yet"); }
    void Uge3(BCValue result, BCValue lhs, BCValue rhs) { assert(0, "Not Implemented yet"); }
    void Lt3(BCValue result, BCValue lhs, BCValue rhs) { assert(0, "Not Implemented yet"); }
    void Gt3(BCValue result, BCValue lhs, BCValue rhs) { assert(0, "Not Implemented yet"); }
    void Le3(BCValue result, BCValue lhs, BCValue rhs) { assert(0, "Not Implemented yet"); }
    void Ge3(BCValue result, BCValue lhs, BCValue rhs) { assert(0, "Not Implemented yet"); }
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
    void Sub3(BCValue result, BCValue lhs, BCValue rhs) { assert(0, "Not Implemented yet"); }
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

    void Call(BCValue result, BCValue fn, BCValue[] args) { assert(0, "Not Implemented yet"); }
    void Load8(BCValue to, BCValue from) { assert(0, "Not Implemented yet"); }
    void Store8(BCValue to, BCValue from) { assert(0, "Not Implemented yet"); }
    void Load16(BCValue to, BCValue from) { assert(0, "Not Implemented yet"); }
    void Store16(BCValue to, BCValue from) { assert(0, "Not Implemented yet"); }
    void Load32(BCValue to, BCValue from)
    {
        auto from_r = getReg(from);
        // this is the address we want to load from
        auto to_r = getReg(to);
        // this is the stack address we want to load into
/+
            _jit_getarg_i(_jit, JIT_R1, arg2); // get it as 32bit int
        version(_64bit)
            _jit_getarg_l(_jit, JIT_R2, arg2); // get it as 64bit long
        else
            _jit_getarg_i(_jit, JIT_R2, arg2); // git it as 32bit int
+/
        _jit_new_node_www(_jit, jit_code_t.jit_code_ldxr_i, to_r, from_r, JIT_R2);
        sync_reg(to, to_r);

    }
    void Store32(BCValue to, BCValue from) { assert(0, "Not Implemented yet"); }
    void Load64(BCValue to, BCValue from) { assert(0, "Not Implemented yet"); }
    void Store64(BCValue to, BCValue from) { assert(0, "Not Implemented yet"); }
    void Alloc(BCValue heapPtr, BCValue size) { assert(0, "Not Implemented yet"); }
    void Not(BCValue result, BCValue val) { assert(0, "Not Implemented yet"); }
    void Ret(BCValue val) { assert(0, "Not Implemented yet"); }
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
    void Line(uint line) { assert(0, "Not Implemented yet"); }
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
