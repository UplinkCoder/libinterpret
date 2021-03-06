module dmd.ctfe.bc_printer_backend;
//version = std_algo;
import dmd.ctfe.bc_common;

string fromStringz(const(char)* cstring)
{
    int length = 0;
    auto ptr = cstring;
    if (ptr)
    {
        while(*(ptr++)) length++;
    }
    return cast(string) cstring[0 .. length];
}

enum BCFunctionTypeEnum : byte
{
    undef,
    Builtin,
    Bytecode,
    Compiled,
}
enum withMemCpy = 1;
enum withStrEq3 = 1;

static if (is(typeof(() { import dmd.declaration : FuncDeclaration; })))
{
    import dmd.declaration : FuncDeclaration;
    alias FT = FuncDeclaration;
}
else
{
   alias FT = void*;
}

struct BCFunction
{
    void* funcDecl;
}

struct ErrorInfo
{
    string msg;
    BCValue[4] values;
    uint valueCount;
}

struct Print_BCGen
{
    struct FunctionState
    {
        uint cndJumpCount;
        uint jmpCount;
        ubyte parameterCount;
        ushort localCount;
        ushort temporaryCount;
        uint labelCount;
        bool sameLabel;
        StackAddr sp = StackAddr(4);
    }

    bool insideFunction = false;
    ErrorInfo[102_000/4] errorInfos;
    uint errorInfoCount;
    FunctionState[ubyte.max * 8] functionStates;
    uint functionStateCount;
    uint currentFunctionStateNumber;
    uint indentLevel;
    string indent = "    ";

    void incIndent()
    {
        indent ~= "    ";
    }

    void decIndent()
    {
        indent = indent[0 .. $-4];
    }

    @property FunctionState* currentFunctionState() return
    {
        return &functionStates[currentFunctionStateNumber];
    }

    @property string functionSuffix()
    {
        return currentFunctionStateNumber > 0 ? "_fn_" ~ itos(currentFunctionStateNumber) : "";
    }

    alias currentFunctionState this;

    string result = "\n";

    uint addErrorMessage(const(char)[] msg)
    {
        if (errorInfoCount < errorInfos.length)
        {
            errorInfos[errorInfoCount++].msg = cast(string)msg;
            return errorInfoCount;
        }

        return 0;
    }

    uint addErrorValue(BCValue v)
    {
        // FIXME HACK renable this!
/+ 
        if (errorInfoCount < errorInfos.length)
        {
            assert(errorInfoCount);
            auto eInfo = &errorInfos[errorInfoCount - 1];

            eInfo.values[eInfo.valueCount++] = v;
            return eInfo.valueCount;
        }
+/
        return 0;
    }

    string print(BCLabel label)
    {
        return ("label" ~ itos(label.addr.addr) ~ functionSuffix);
    }

    string print(BCType type)
    {
        return "BCType(BCTypeEnum." ~ enumToString(type.type) ~ (
            isBasicBCType(type) ? ")" : (", " ~ itos(type.typeIndex) ~ ")"));
    }

    string print(BCValue val)
    {
        string result = "BCValue(";

        switch (val.vType)
        {
        case BCValueType.Immediate:
            {
                if (val.type.type == BCTypeEnum.c8)
                {
                    result ~= "Imm32('" ~ [cast(char)(val.imm32 & 0xFF)] ~ "')";
                }
                else if (val.type.type == BCTypeEnum.i8)
                {
                    result ~= "Imm32(" ~ itos(val.imm32.imm32) ~ ") // i8";
                }
                else if (val.type.type == BCTypeEnum.u32)
                {
                    result ~= "Imm32(" ~ itos(val.imm32.imm32) ~ ")";
                }
                else if (val.type.type == BCTypeEnum.i32)
                {
                    result ~= "Imm32(" ~ sitos(val.imm32.imm32) ~ ", true)";
                }
                else if (val.type.type == BCTypeEnum.i64)
                {
                    result ~= "Imm64(" ~ 
                        ((cast(ulong)val.imm64.imm64 <= uint.max) 
                            ? itos(val.imm64.imm64 & uint.max) 
                            : itos(val.imm64.imm64 & uint.max) ~ " | (" ~
                                itos(val.imm64.imm64 >> 32) ~"UL << 32)" 
                        )
                    ~ ", true)";
		}
                else if (val.type.type == BCTypeEnum.u64)
                {
                    result ~= "Imm64(" ~ 
                        ((cast(ulong)val.imm64.imm64 <= uint.max) 
                            ? itos(val.imm64.imm64 & uint.max) 
                            : itos(val.imm64.imm64 & uint.max) ~ " | (" ~
                                itos(val.imm64.imm64 >> 32) ~"UL << 32)" 
                        )
                    ~ ")";
                }
                else if (val.type.type == BCTypeEnum.f23)
                {
                    result ~= "Imm23f(" ~ floatToString(*cast(float*)&val.imm32.imm32) ~ ")";
                }
                else if (val.type.type == BCTypeEnum.f52)
                {
                    result ~= "Imm52f(" ~ doubleToString(*cast(double*)&val.imm64.imm64) ~ ")";
                }
                else if (val.type.type == BCTypeEnum.Null)
                {
                    result ~= "Imm32(0/*null*/)";
                }
                else if (val.type.type == BCTypeEnum.Array)
                {
                    result ~= "Imm32(" ~ itos(val.imm32.imm32) ~ "/*Array*/)";
                }
                else
                {
                    assert(0, "Unexpected Immediate of Type" ~ enumToString(val.type.type));
                }
            }
            break;

        case BCValueType.StackValue:
            {
                if (val.tmpIndex)
                {
                    return "tmp" ~ itos(val.tmpIndex) ~ functionSuffix;
                }
                result ~= "StackAddr(" ~ itos(val.stackAddr.addr) ~ "), " ~ print(val.type);
            }
            break;

        case BCValueType.Local :
        {
            auto name = val.name ? val.name ~ "_" ~ itos(val.localIndex) : "local" ~ itos(val.localIndex);
            return name ~ functionSuffix;
        }

        case BCValueType.Temporary:
            {
                return "tmp" ~ itos(val.tmpIndex) ~ functionSuffix;
            }
        case BCValueType.Parameter:
            {
                return (val.name ? val.name : "p") ~ "_" ~ itos(val.paramIndex) ~ functionSuffix;
            }
        case BCValueType.Error, BCValueType.ErrorWithMessage:
            {
                char[] _result = cast(char[])"imm32(" ~ itos(val.imm32) ~ ") /*";
                if (val.imm32)
                {
                    auto eInfo = errorInfos[val.imm32 - 1];
                    _result ~= `"` ~ eInfo.msg ~ `", `;

                    foreach(i;0 .. eInfo.valueCount)
                        _result ~= print(eInfo.values[i]) ~ ", ";

                    _result[$-2 .. $] = "*/";
                }
                return cast(string)_result;
            }
        case BCValueType.Unknown:
            {
                return "BCValue.init";
            }
        default:
            assert(0, "printing for " ~ enumToString(val.vType) ~ " unimplemented ");
        }

        result ~= ")";

        return result;
    }

    BCLabel genLabel()
    {
        if (!sameLabel)
        {
            ++labelCount;
            result ~= indent;
            sameLabel = true;
        }
        else
        {
            result ~= indent ~ "//";
        }
        result ~= "auto label" ~ itos(labelCount) ~ functionSuffix ~ " = genLabel();\n";
        return BCLabel(BCAddr(labelCount));
    }

    void incSp()
    {
        sameLabel = false;
        sp += 4;
        result ~= indent ~ "incSp();\n";
    }

    StackAddr currSp()
    {
        result ~= indent ~ "//currSp();//SP[" ~ itos(sp.addr) ~ "]\n";
        return sp;
    }

    void Initialize()
    {
        result = result[0 .. 0];
        result ~= indent ~ "Initialize(" ~ ");\n";
        incIndent();
    }

    void Finalize()
    {
        decIndent();
        result ~= indent ~ "Finalize(" ~ ");\n";
    }

    void PrintString (const(char)[] message)
    {
        result ~= indent ~ `PrintString("` ~ message ~ "\");\n";
    }

    void beginFunction(uint f = 0, void* fnDecl = null)
    {
        sameLabel = false;
        import dmd.func : FuncDeclaration;
//        assert(!insideFunction);
        insideFunction = true;
        auto fd = *(cast(FuncDeclaration*) &fnDecl);

        {
        }

        result ~= indent ~ "beginFunction(" ~ itos(f) ~ ");//" ~ (fd && fd.ident ? fromStringz(fd.toChars) : "(nameless)") ~ "\n";
        incIndent();
    }

    void endFunction()
    {
        decIndent();
        currentFunctionStateNumber++;
        assert(insideFunction);
        insideFunction = false;
        result ~= indent ~ "endFunction(" ~ ");\n\n";
    }

    BCValue genParameter(BCType bct, string name = null)
    {
        //currentFunctionStateNumber++;
        if (!parameterCount)
        {
            //write a newline when we effectively begin a new function;
            result ~= "\n";
        }
        name =  name ? name : "p";
        result ~= indent ~ "auto " ~ name ~ "_" ~ itos(++parameterCount) ~ functionSuffix ~ " = genParameter(" ~ print(
            bct) ~ ");//SP[" ~ itos(sp) ~ "]\n";
        //currentFunctionStateNumber--;
        sp += 4;
        auto p = BCValue(BCParameter(parameterCount, bct));
        p.name = name;
        return p;
    }

    BCValue genTemporary(BCType bct)
    {
        sameLabel = false;
        auto tmpAddr = sp.addr;
        sp += isBasicBCType(bct) ? align4(basicTypeSize(bct.type)) : 4;

        result ~= indent ~ "auto tmp" ~ itos(++temporaryCount) ~ functionSuffix ~ " = genTemporary(" ~ print(
            bct) ~ ");//SP[" ~ itos(tmpAddr) ~ "]\n";
        return BCValue(StackAddr(tmpAddr), bct, temporaryCount);
    }

    void destroyTemporary(BCValue tmp)
    {
        sameLabel = false;

        uint sz;
        if (isBasicBCType(tmp.type))
        {
            sz = align4(basicTypeSize(tmp.type.type));
        }
        else
        {
            sz = 4;
        }
        if (sp - sz == tmp.stackAddr)
        {
            // this is the last thing we pushed on
            // free the stack space immediately.
            sp -= sz;
        }

        result ~= indent ~ "destroyTemporary(" ~ print(tmp) ~ ")\n";
    }

    BCValue genLocal(BCType bct, string name)
    {
        sameLabel = false;
        auto localAddr = sp.addr;
        sp += isBasicBCType(bct) ? align4(basicTypeSize(bct.type)) : 4;

        auto localName = name ? name ~ "_" ~ itos(++localCount) : "local" ~ itos(++localCount);

        result ~= indent ~ "auto " ~ localName ~ functionSuffix ~ " = genLocal(" ~ print(
            bct) ~ ", \"" ~ (name ? name : "") ~ "\");//SP[" ~ itos(localAddr) ~ "]\n";
        return BCValue(StackAddr(localAddr), bct, localCount, name);
    }


    BCAddr beginJmp()
    {
        sameLabel = false;
        result ~= indent ~ "auto jmp" ~ itos(++jmpCount) ~ functionSuffix ~ " = beginJmp();\n";
        incIndent();
        return BCAddr(jmpCount);
    }

    void endJmp(BCAddr atIp, BCLabel target)
    {
        sameLabel = false;
        decIndent();
        result ~= indent ~ "endJmp(jmp" ~ itos(atIp.addr) ~ functionSuffix ~ ", " ~ print(target) ~ ");\n";
    }

    void Jmp(BCLabel target)
    {
        sameLabel = false;
        result ~= indent ~ "Jmp(" ~ print(target) ~ ");\n";
    }

    CndJmpBegin beginCndJmp(BCValue cond = BCValue.init, bool ifTrue = false)
    {
        sameLabel = false;
        result ~= indent ~ "auto cndJmp" ~ itos(++cndJumpCount) ~ functionSuffix ~ " = beginCndJmp(" ~ (
            cond ? (print(cond) ~ (ifTrue ? ", true" : "")) : "") ~ ");\n";
        incIndent();
        result ~= "\n";
        return CndJmpBegin(BCAddr(cndJumpCount), cond, ifTrue);
    }

    void endCndJmp(CndJmpBegin jmp, BCLabel target)
    {
        sameLabel = false;
        decIndent();
        result ~= "\n";
        result ~= indent ~ "endCndJmp(cndJmp" ~ itos(jmp.at.addr) ~ functionSuffix ~ ", " ~ print(target) ~ ");\n";
    }

    void emitFlg(BCValue lhs)
    {
        sameLabel = false;
        result ~= indent ~ "emitFlg(" ~ print(lhs) ~ ");\n";
    }

    void Throw(BCValue e)
    {
        result ~= indent ~ "Throw (" ~ print(e) ~ ");\n";
    }

    void PushCatch()
    {
        result ~= indent ~ "PushCatch ();\n";
    }

    void PopCatch()
    {
        result ~= indent ~ "PopCatch ();\n";
    }

    void Set(BCValue lhs, BCValue rhs)
    {
        if (lhs == rhs)
            return;
        sameLabel = false;
        result ~= indent ~ "Set(" ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Ult3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "Ult3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Ugt3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "Ugt3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Ule3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "Ule3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Uge3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "Uge3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Lt3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "Lt3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Gt3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "Gt3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Le3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "Le3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Ge3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "Ge3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Eq3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "Eq3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Neq3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "Neq3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Add3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "Add3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Sub3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "Sub3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Mul3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "Mul3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Div3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "Div3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Udiv3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "Udiv3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void And3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "And3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Or3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "Or3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Xor3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "Xor3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Lsh3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "Lsh3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Rsh3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "Rsh3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Mod3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "Mod3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Umod3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "Umod3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Byte3(BCValue _result, BCValue word, BCValue idx)
    {
        sameLabel = false;
        result ~= indent ~ "Byte3(" ~ print(_result) ~ ", " ~ print(word) ~ ", " ~ print(idx) ~ ");\n";
    }

    import dmd.globals : Loc;
    void Call(BCValue _result, BCValue fn, BCValue[] args, Loc l = Loc.init)
    {
        version(std_algo)
        {
            import std.algorithm : map;
            import std.range : join;

            sameLabel = false;

            result ~= indent ~ "Call(" ~ print(_result) ~ ", " ~ print(fn) ~ ", [" ~ args.map!(
                a => print(a)).join(", ") ~ "]);\n";
        }
        else
        {
            sameLabel = false;
            result ~= indent ~ "Call(" ~ print(_result) ~ ", " ~ print(fn) ~ ", [";
            foreach(arg;args)
            {
                result ~= print(arg) ~ ", ";
            }
            result = result[0 .. $-2] ~ "]);\n";
        }
    }

    void Load8(BCValue to, BCValue from)
    {
        sameLabel = false;
        result ~= indent ~ "Load8(" ~ print(to) ~ ", " ~ print(from) ~ ");\n";
    }

    void Store8(BCValue to, BCValue from)
    {
        sameLabel = false;
        result ~= indent ~ "Store8(" ~ print(to) ~ ", " ~ print(from) ~ ");\n";
    }

    void Load16(BCValue to, BCValue from)
    {
        sameLabel = false;
        result ~= indent ~ "Load16(" ~ print(to) ~ ", " ~ print(from) ~ ");\n";
    }

    void Store16(BCValue to, BCValue from)
    {
        sameLabel = false;
        result ~= indent ~ "Store16(" ~ print(to) ~ ", " ~ print(from) ~ ");\n";
    }

    void Load32(BCValue to, BCValue from)
    {
        sameLabel = false;
        result ~= indent ~ "Load32(" ~ print(to) ~ ", " ~ print(from) ~ ");\n";
    }

    void Store32(BCValue to, BCValue from)
    {
        sameLabel = false;
        result ~= indent ~ "Store32(" ~ print(to) ~ ", " ~ print(from) ~ ");\n";
    }

    void Load64(BCValue to, BCValue from)
    {
        sameLabel = false;
        result ~= indent ~ "Load64(" ~ print(to) ~ ", " ~ print(from) ~ ");\n";
    }

    void Store64(BCValue to, BCValue from)
    {
        sameLabel = false;
        result ~= indent ~ "Store64(" ~ print(to) ~ ", " ~ print(from) ~ ");\n";
    }

    void Alloc(BCValue heapPtr, BCValue size)
    {
        sameLabel = false;
        result ~= indent ~ "Alloc(" ~ print(heapPtr) ~ ", " ~ print(size) ~ ");\n";
    }

    void Not(BCValue _result, BCValue val)
    {
        sameLabel = false;
        result ~= indent ~ "Not(" ~ print(_result) ~ ", " ~ print(val) ~ ");\n";
    }

    void Ret(BCValue val)
    {
        sameLabel = false;
        result ~= indent ~ "Ret(" ~ print(val) ~ ");\n";
    }

    void Cat(BCValue _result, BCValue lhs, BCValue rhs, const uint elmSize)
    {
        sameLabel = false;
        result ~= indent ~ "Cat(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ", " ~ itos(
            elmSize) ~ ");\n";
    }

    void Assert(BCValue value, BCValue err)
    {
        sameLabel = false;
        result ~= indent ~ "Assert(" ~ print(value) ~ ", " ~ print(err) ~ ");\n";
    }

    static if (withMemCpy)
        void MemCpy(BCValue dst, BCValue src, BCValue size)
    {
        sameLabel = false;
        result ~= indent ~ "MemCpy(" ~ print(dst) ~ ", " ~ print(src) ~ ", " ~ print(size) ~ ");\n";
    }

    static if (withStrEq3)
        void StrEq3(BCValue _result, BCValue lhs, BCValue rhs)
    {
        sameLabel = false;
        result ~= indent ~ "StrEq3(" ~ print(_result) ~ ", " ~ print(lhs) ~ ", " ~ print(rhs) ~ ");\n";
    }

    void Comment(const(char)[] comment)
    {
        result ~= "\n" ~ indent ~ "Comment(\"" ~ comment ~ "\");\n";
    }

    void Line(uint line)
    {
        result ~= indent ~ "Line(" ~ itos(line) ~ ");\n";
    }

    void File(string filename)
    {
        result ~= indent ~ "File(\"" ~ filename ~ "\");\n";
    }

    void IToF32(BCValue _result, BCValue value)
    {
        result ~= indent ~ "IToF32(" ~ print(_result) ~ ", " ~ print(value) ~ ");\n";
    }

    void IToF64(BCValue _result, BCValue value)
    {
        result ~= indent ~ "IToF64(" ~ print(_result) ~ ", " ~ print(value) ~ ");\n";
    }

    void F32ToI(BCValue _result, BCValue value)
    {
        result ~= indent ~ "F32ToI(" ~ print(_result) ~ ", " ~ print(value) ~ ");\n";
    }

    void F64ToI(BCValue _result, BCValue value)
    {
        result ~= indent ~ "F64ToI(" ~ print(_result) ~ ", " ~ print(value) ~ ");\n";
    }

    void F64ToF32(BCValue _result, BCValue value)
    {
        result ~= indent ~ "F64ToF32(" ~ print(_result) ~ ", " ~ print(value) ~ ");\n";
    }

    void F32ToF64(BCValue _result, BCValue value)
    {
        result ~= indent ~ "F32ToF64(" ~ print(_result) ~ ", " ~ print(value) ~ ");\n";
    }
}

enum genString = q{
    auto tmp1 = genTemporary(BCType(BCTypeEnum.i32));//SP[4]
    Mul3(tmp1, BCValue(Imm32(2)), BCValue(Imm32(16)));
    Div3(tmp1, tmp1, BCValue(Imm32(4)));
    Sub3(tmp1, tmp1, BCValue(Imm32(1)));
    Ret(tmp1);
};

static assert(ensureIsBCGen!Print_BCGen);
