import dmd.ctfe.bc_common;
import dmd.ctfe.bc;
// Fact(n) = (n > 1 ? ( n*(n-1) * Fact(n-2) ) : 1);

import std.datetime.stopwatch;
bool inited = false;


/+
auto testFact(GenT)(auto ref GenT gen)
{
    with (gen)
    {
        Initialize();
        {
            auto n = genParameter(i32Type, "n");
            beginFunction(0);
            {
                auto result = genLocal(i32Type, "result");
                Gt3(BCValue.init, n, imm32(1, true));
                auto j_n_lt_1 = beginCndJmp();
                {
                    auto n_sub_1 = genTemporary(i32Type);
                    Sub3(n_sub_1, n, imm32(1));
                    auto n_mul_n_sub_1 = genTemporary(i32Type);
                    Mul3(n_mul_n_sub_1, n, n_sub_1);
                    Sub3(n, n, imm32(2));

                    auto result_fact = genTemporary(i32Type);
                    Call(result_fact, imm32(1), [n]);

                    Mul3(result, n_mul_n_sub_1, result_fact);
                    Ret(result);
                }
                auto l_n_lt_1 = genLabel();
                {
                    Set(result, imm32(1));
                    Ret(result);
                }
                endCndJmp(j_n_lt_1, l_n_lt_1);
            }
            endFunction();
        }
        //Finalize();
        return gen;
    }
}
// static assert(testFact(BCGen.init).interpret([imm32(5)]) == imm32(120));
+/

auto countUp(GenT)(auto ref GenT gen)
{
    with(gen)
    {
    Initialize();
        File("bug6500.d");
        Line(1);

        auto a_1 = genParameter(BCType(BCTypeEnum.i32), "a");//SP[4]
        beginFunction(0);//f
            Line(2);
            Line(3);
            auto x_1 = genLocal(BCType(BCTypeEnum.i32), "x");//SP[8]
            Set(a_1, imm32(ushort.max * 255));
            Set(x_1, BCValue(Imm32(0, true)));
            Line(4);
            auto label1 = genLabel();

            Comment("genTemporary from:4789");
            auto tmp1 = genTemporary(BCType(BCTypeEnum.i32));//SP[12]
            Set(tmp1, a_1);
            Sub3(a_1, a_1, BCValue(Imm32(1)));
            auto cndJmp1 = beginCndJmp(tmp1);

                auto label2 = genLabel();

                Comment("genTemporary from:4789");
                auto tmp2 = genTemporary(BCType(BCTypeEnum.i32));//SP[16]
                Set(tmp2, x_1);
                Add3(x_1, x_1, BCValue(Imm32(1)));
                auto label3 = genLabel();
                Jmp(label1);
                auto label4 = genLabel();

            endCndJmp(cndJmp1, label4);
            Line(5);
            Ret(x_1);
            Line(6);
        endFunction();

    //Finalize();
    return gen;
    }
}

import dmd.ctfe.bc_lightning_backend;
void main(string[] args)
{
    import core.memory : GC;
    GC.disable();
    alias fType = extern (C) int function (RuntimeContext* ctx);

    long[] stack = new long[](4096);
    uint[] heap = new uint[](4096);
    uint stackDataLength = 4096;
    uint heapDataLength = 4096;

    RuntimeContext ctx;
    ctx.stackDataBegin = cast(ubyte*)stack.ptr;
    ctx.stackPointer = cast(ubyte*)stack.ptr;
    ctx.heapDataBegin = cast(ubyte*)heap.ptr;
    import core.stdc.stdio;
    printf("MY heapPtr: %p\n", heap.ptr);
    import core.time;
    import core.stdc.stdio;
    StopWatch sw;
        sw.reset();
        sw.start();
        //BCGen gen;
        import lightning;
        LightningGen gen;
        init_jit(cast(const(char)*)args[0].ptr);
        gen._jit = jit_new_state();

        // testFact(gen);
        countUp(gen);
        auto func = cast(fType)  _jit_emit(gen._jit);
        sw.stop();
        printf("compile time usecs: %d\n", sw.peek.total!"usecs");
        // _jit_print(gen._jit);
        //_jit_disassemble(gen._jit);
    printf("before execution stack[0 .. 4] == [%d, %d, %d, %d]\n", stack[0], stack[1], stack[2], stack[3]);
    StopWatch swr;
        swr.reset();
        swr.start();
    func(&ctx);
        swr.stop();
        printf("runtime usecs: %d\n", swr.peek.total!"usecs");
    printf("%p == %p\n", ctx.framePointer, ctx.stackPointer);
    printf("heap[0 .. 4] == [%d, %d, %d, %d]\n", heap[0], heap[1], heap[2], heap[3]);
    printf("stack[0 .. 4] == [%d, %d, %d, %d]\n", stack[0], stack[1], stack[2], stack[3]);
}
