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
import dmd.ctfe.bc_lightning_backend;
void main()
{
    alias fType = extern (C) int function (RuntimeContext* ctx);

    long[] stack = new long[](4096);
    uint[] heap = new uint[](4096);
    uint stackDataLength = 4096;
    uint heapDataLength = 4096;

    RuntimeContext ctx;
    ctx.stackDataBegin = cast(int*)stack.ptr;
    ctx.heapDataBegin = cast(ubyte*)heap.ptr;
    import core.stdc.stdio;
    printf("MY heapPtr: %p\n", heap.ptr);
    import core.time;
    import core.stdc.stdio;
    StopWatch sw;
        sw.reset();
        sw.start();
        //BCGen gen;
        LightningGen gen;
        gen.Initialize();
        auto p1 = gen.genParameter(i32Type, "p1");
        gen.beginFunction(0);
        auto tmp1 = gen.genLocal(i32Type, "result");
        //gen.Add3(tmp1, imm32(0x64), imm32(0x32));
        //gen.Add3(tmp1, tmp1, imm32(0x48));
        gen.Store32(imm32(4), imm32(4));
        gen.Store32(imm32(8), imm32(8));
        gen.Store32(imm32(12), imm32(12));
        gen.Ret(tmp1);
        gen.endFunction();

        // testFact(gen);
        import lightning;
        auto func = cast(fType)  _jit_emit(gen._jit);
        sw.stop();
        printf("usecs: %d\n", sw.peek.total!"usecs");
        _jit_print(gen._jit);
    func(&ctx);
    printf("func(%d) == [%d, %d, %d, %d]\n", 5, heap[0], heap[1], heap[2], heap[3]);
}
