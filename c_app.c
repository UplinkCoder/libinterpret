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

    i.Initialize(c, 0); // zero extra arguments
    {
        fIdx = i.beginFunction(c, 0, "add");
        {
            // TODO i.returnTypeAnnotation(c, BCType_i32);

            BCValue a = i.genParameter(c, (BCType){BCTypeEnum_i32}, "a");
            BCValue b = i.genParameter(c, BCType_i32, "b");

            BCValue res = i.genLocal(c, (BCType){BCTypeEnum_i32}, "result");

            i.Add3(c, &res, &a, &b);
            i.Ret(c, &res);

            i.endFunction(c, fIdx);
        }
    }
    i.Finalize(c);

    {
        int a = atoi(argv[1]);
        int b = atoi(argv[2]);

        BCValue arguments[2];

        arguments[0] = imm32(a);
        arguments[1] = imm32(b);

        BCValue res = i.run(c, fIdx, arguments, 2);

        printf("%d + %d = %d\n", a, b, res.imm32.imm32);
    }

    i.destroy_instance(c);

    return 0;
#endif
}
