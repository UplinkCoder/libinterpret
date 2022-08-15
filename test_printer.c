#include "stdio.h"
#include "bc_common.h"
#include "backend_interface_funcs.h"

typedef struct Printer Printer;
extern void Printer_StreamToFile(Printer* self, FILE* fd);

extern BackendInterface Printer_interface;

int main(int argc, char* argv[])
{
    const BackendInterface i = Printer_interface;

    void* c;
    i.new_instance(&c);

    uint32_t fIdx;

    i.Initialize(c, 0); // zero extra arguments
    {
        fIdx = i.BeginFunction(c, 0, "add");
        {
            // TODO i.returnTypeAnnotation(c, BCType_i32);

            BCValue a = i.GenParameter(c, BCType_i32, "a");
            BCValue b = i.GenParameter(c, BCType_i32, "b");

            BCValue res = i.GenLocal(c, BCType_i32, "result");
            BCValue imm__64 = imm32(64);
            i.Eq3(c, 0, &a, &imm__64);
            
            CndJmpBegin cj = i.BeginCndJmp(c, 0, 0);
            i.Add3(c, &res, &a, &b);
            i.EndCndJmp(c, &cj, i.GenLabel(c));
            i.Ret(c, &res);

            i.EndFunction(c, fIdx);
        }
    }
    i.Finalize(c);
    Printer_StreamToFile((Printer*)c, stdout);
}
