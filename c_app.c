#include <stdio.h>
#include "backend_interface_funcs.h"
extern BackendInterface BCGen_interface;
extern BackendInterface Printer_interface;

#include "bc_interpreter_backend.h"

int main(int argc, char* argv[])
{
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
        add_idx = i.BeginFunction(c, 0, "add");
        {
            BCValue a = i.GenParameter(c, (BCType){BCTypeEnum_i32, 0}, "a");
            BCValue b = i.GenParameter(c, (BCType){BCTypeEnum_i32, 0}, "b");

            BCValue result = i.GenLocal(c, (BCType){BCTypeEnum_i32, 0}, "result");

            i.Add3(c, &result, &a, &b);
            i.Ret(c, &result);
        }
        i.EndFunction(c, add_idx);

    }

    i.Finalize(c);

    {
        int a = atoi(argv[1]);
        int b = atoi(argv[2]);

        BCValue arguments[2];

        arguments[0] = imm32(a);
        arguments[1] = imm32(b);

        BCValue res = i.Run(c, add_idx, arguments, 2);

        printf("%d + %d = %d\n", a, b, res.imm32.imm32);
    }

    i.destroy_instance(c);

    return 0;
}
