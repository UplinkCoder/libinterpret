#include <stdio.h>
#include "backend_interface_funcs.h"

extern BackendInterface BCGen_interface;

#if DIS
#  include <unistd.h>
#  include "int_iter.h"
  void PrintCode(IntIter*);
#endif

int main(int argc, char* argv[])
{
#if DIS
    if (argc != 1)
    {
        printf("0 Arguments expected\n");
        return -1;
    }
    IntIter iter = {0};
    IntIter_Init(&iter, STDIN_FILENO);
    PrintCode(&iter);

    return 0;
#else
    if (argc != 3)
    {
        printf("2 Numeric Arguments expected\n");
        return -1;
    }

    const BackendInterface i = BCGen_interface;

    void* c;
    i.new_instance(&c);

    uint32_t fIdx;
    uint32_t add_idx;

    i.Initialize(c, 0); // zero extra arguments
    {
        fIdx = i.beginFunction(c, 0, "add");
        {
            // TODO i.returnTypeAnnotation(c, BCType_i32);

            BCValue a = i.genParameter(c, (BCType){BCTypeEnum_i32}, "a");
            BCValue b = i.genParameter(c, BCType_i32, "b");

            BCValue res = i.genLocal(c, (BCType){BCTypeEnum_i32}, "result");
            BCValue res2 = i.genLocal(c, (BCType){BCTypeEnum_i32}, "res2");

            i.Add3(c, &res, &a, &b);
            i.Eq3(c, &res2, &a, &b);
            i.Eq3(c, 0, &a, &res);
            i.Ret(c, &res);

            i.endFunction(c, fIdx);
        }

        add_idx = i.beginFunction(c, 1, "add");
        {
            BCValue a = i.genParameter(c, (BCType){BCTypeEnum_i32, 0}, "a");
            BCValue b = i.genParameter(c, (BCType){BCTypeEnum_i32, 0}, "b");

            BCValue result = i.genLocal(c, (BCType){BCTypeEnum_i32, 0}, "result");
            BCValue l1 = imm32(64);

            i.Eq3(c, 0, &a, &l1);
            CndJmpBegin cndJmp2 = i.beginCndJmp(c, 0, false);
            i.Add3(c, &result, &a, &b);
            BCLabel label6 = i.genLabel(c);
            i.endCndJmp(c, &cndJmp2, label6);
            i.Ret(c, &result);
        }
        i.endFunction(c, add_idx);

    }

    i.Finalize(c);

    {
        int a = atoi(argv[1]);
        int b = atoi(argv[2]);

        BCValue arguments[2];

        arguments[0] = imm32(a);
        arguments[1] = imm32(b);

        BCValue res = i.run(c, add_idx, arguments, 2);

        printf("%d + %d = %d\n", a, b, res.imm32.imm32);
    }

    BCGen* g = c;
    char textBuffer[8192];
    char* p = textBuffer;
    // yeah I know it's stupid to first convert to text and then disassemble the text
    // but meh ...
    for(int i = 0; i < g->byteCodeCount; i++)
    {
        p += sprintf(p, "%d ", g->byteCodeArray[i]);
    }

    IntIter it;
    IntIter_FromBuffer(&it, textBuffer, p - textBuffer);
    PrintCode(it);    

    i.destroy_instance(c);

    return 0;
#endif
}
