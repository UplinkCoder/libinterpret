#include "stdio.h"
#include "bc_common.h"
#include "backend_interface_funcs.h"

typedef struct Printer Printer;
extern void Printer_StreamToFile(Printer* self, FILE* fd);
void test_printer_putstr_batching();

extern BackendInterface Printer_interface;

int main(int argc, char* argv[])
{
    const BackendInterface i = Printer_interface;

    void* c = malloc(i.sizeof_instance());
    i.init_instance(c);

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

    test_printer_putstr_batching();
}


void test_printer_putstr_batching()
{
    printf("Testing Printer_PutStr batching logic...\n");

    // Setup
    Printer printer = {0};
    Printer_init_instance(&printer);

    // Test 1: Short string (< 128 bytes)
    {
        const char* short_str = "Hello World!";
        uint32_t initial_cap = printer.BufferCapacity;

        Printer_PutStr(&printer, short_str);

        uint32_t expected_used = strlen(short_str);
        uint32_t actual_used = initial_cap - printer.BufferCapacity;

        assert(actual_used == expected_used);
        printf("✓ Short string test passed (used %u bytes)\n", actual_used);
    }

    // Reset for next test
    printer.Buffer = (char*)printer.BufferStart;
    printer.BufferCapacity = 8192 * 8;

    // Test 2: Exactly 128 bytes
    {
        char str_128[129];
        memset(str_128, 'A', 128);
        str_128[128] = '\0';

        uint32_t initial_cap = printer.BufferCapacity;

        Printer_PutStr(&printer, str_128);

        uint32_t expected_used = 128;
        uint32_t actual_used = initial_cap - printer.BufferCapacity;

        assert(actual_used == expected_used);
        printf("✓ 128-byte boundary test passed (used %u bytes)\n", actual_used);
    }

    // Reset
    printer.Buffer = (char*)printer.BufferStart;
    printer.BufferCapacity = 8192 * 8;

    // Test 3: Just over 128 bytes (129)
    {
        char str_129[130];
        memset(str_129, 'B', 129);
        str_129[129] = '\0';

        uint32_t initial_cap = printer.BufferCapacity;

        Printer_PutStr(&printer, str_129);

        uint32_t expected_used = 129;
        uint32_t actual_used = initial_cap - printer.BufferCapacity;

        assert(actual_used == expected_used);
        printf("✓ 129-byte test passed (used %u bytes)\n", actual_used);
    }

    // Reset
    printer.Buffer = (char*)printer.BufferStart;
    printer.BufferCapacity = 8192 * 8;

    // Test 4: Multiple batches (200 bytes)
    {
        char str_200[201];
        memset(str_200, 'C', 200);
        str_200[200] = '\0';

        uint32_t initial_cap = printer.BufferCapacity;

        Printer_PutStr(&printer, str_200);

        uint32_t expected_used = 200;
        uint32_t actual_used = initial_cap - printer.BufferCapacity;

        assert(actual_used == expected_used);
        printf("✓ 200-byte multi-batch test passed (used %u bytes)\n", actual_used);
    }

    // Reset
    printer.Buffer = (char*)printer.BufferStart;
    printer.BufferCapacity = 8192 * 8;

    // Test 5: Exactly 256 bytes (2 batches)
    {
        char str_256[257];
        memset(str_256, 'D', 256);
        str_256[256] = '\0';

        uint32_t initial_cap = printer.BufferCapacity;

        Printer_PutStr(&printer, str_256);

        uint32_t expected_used = 256;
        uint32_t actual_used = initial_cap - printer.BufferCapacity;

        assert(actual_used == expected_used);
        printf("✓ 256-byte (2 batches) test passed (used %u bytes)\n", actual_used);
    }

    // Test 6: Large string (500 bytes)
    {
        printer.Buffer = (char*)printer.BufferStart;
        printer.BufferCapacity = 8192 * 8;

        char str_500[501];
        memset(str_500, 'E', 500);
        str_500[500] = '\0';

        uint32_t initial_cap = printer.BufferCapacity;

        Printer_PutStr(&printer, str_500);

        uint32_t expected_used = 500;
        uint32_t actual_used = initial_cap - printer.BufferCapacity;

        assert(actual_used == expected_used);
        printf("✓ 500-byte large string test passed (used %u bytes)\n", actual_used);
    }

    // Verify buffer contents for one test
    {
        printer.Buffer = (char*)printer.BufferStart;
        printer.BufferCapacity = 8192 * 8;
        printer.LineIndented = true; // Skip indent

        const char* test_str = "Test123";
        Printer_PutStr(&printer, test_str);

        assert(strncmp(printer.BufferStart, test_str, strlen(test_str)) == 0);
        printf("✓ Buffer contents verification passed\n");
    }

    Printer_fini_instance(&printer);

    printf("\n✅ All Printer_PutStr tests passed!\n");
}
