#include <stdio.h>
#include <stdlib.h> // For atoi
#include "backend_interface_funcs.h"
#include "bc_interpreter_backend.h"

// Declare external backend interfaces
extern BackendInterface BCGen_interface;
extern BackendInterface Printer_interface;

int main(int argc, char* argv[])
{
    // Check if the number of command-line arguments is correct
    if (argc != 3)
    {
        printf("2 Numeric Arguments expected\n");
        return -1;
    }

    // Select the backend interface for bytecode generation
    const BackendInterface i = BCGen_interface;

    // Allocate memory for the backend interface instance
    void* c = malloc(i.sizeof_instance());

    // Initialize the backend interface instance
    i.init_instance(c);

    // Variables to store function indices
    uint32_t fIdx;
    uint32_t add_idx;

    // Initialize the code generation context with zero extra arguments
    i.Initialize(c, 0);
    {
        // Start generating a new function named "add"
        add_idx = i.BeginFunction(c, 0, "add");
        {
            // Generate function parameters a and b of type BCTypeEnum_i32
            BCValue a = i.GenParameter(c, (BCType){BCTypeEnum_i32, 0}, "a");
            BCValue b = i.GenParameter(c, (BCType){BCTypeEnum_i32, 0}, "b");

            // Generate a local variable named result of type BCTypeEnum_i32
            BCValue result = i.GenLocal(c, (BCType){BCTypeEnum_i32, 0}, "result");

            // Perform addition: result = a + b
            i.Add3(c, &result, &a, &b);

            // Return the value of result as the function's return value
            i.Ret(c, &result);
        }
        // End the function code generation
        i.EndFunction(c, add_idx);

    }

    // Finalize the code generation
    i.Finalize(c);

    // Perform bytecode execution
    {
        // Convert command-line arguments to integers
        int a = atoi(argv[1]);
        int b = atoi(argv[2]);

        // Create an array of BCValue to hold the function arguments
        BCValue arguments[2];

        // Assign the converted arguments to the BCValue array
        arguments[0] = imm32(a);
        arguments[1] = imm32(b);

        // Execute the "add" function with the provided arguments
        BCValue res = i.Run(c, add_idx, arguments, 2, 0);

        // Print the result of the addition
        printf("%d + %d = %d\n", a, b, res.imm32.imm32);
    }

    // Clean up the backend interface instance
    i.fini_instance(c);

    return 0;
}
