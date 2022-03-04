#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

int main(int argc, char* argv[])
{
    char out_filename[1024];
    // First Check for invalid arguments

    {
        if (argc < 2 || argc > 3)
        {
            fprintf(stderr, "Usage: make_backend_boilerplate <backend_name> [exisiting implementation.c]\n");
            return -1;
        }
        else if (argc == 2)
        {
            if (strlen(argv[1]) > 64)
            {
                fprintf(stderr, "string '%s' is too long. Keep it under 64 chars\n", argv[1]);
                return -1;
            }

            strcpy(out_filename, argv[1]);
            strcat(out_filename, "_backend.c");
        }
        else if (argc == 3)
        {
            if (strlen(argv[2]) > 1000)
            {
                fprintf(stderr, "string '%s' is too long. Keep it under 1000 chars\n", argv[2]);
                return -1;
            }
            strcpy(out_filename, argv[2]);
            strcat(out_filename, ".new");
        }
    }

    char* backend_funcs_h;
    int f_length;
    {
        const char* f_name = "backend_interface_funcs.h";
        FILE* fd = fopen(f_name, "r");
        {
            fseek(fd, 0, SEEK_END);
            f_length = ftell(fd);
            fseek(fd, 0, SEEK_SET);
            backend_funcs_h = (char*)malloc(f_length + 1);
            int read_length =
                fread(backend_funcs_h, 1, f_length, fd);
            if (read_length != f_length)
            {
                fprintf(stderr, "File '%s' could not be read properly", f_name);
                return -1;
            }
            backend_funcs_h[f_length] = '\0';
            fclose(fd);
        }
    }

    /// how many funcion pointer types are defined 
    int n_functions = 0;
    {
        char *op = backend_funcs_h, *cp;
        int remaining_length = f_length;
        // first let's count the occourcenes of '(*'
        for(;;)
        {
            cp = memchr(op, '(', remaining_length);
            if (!cp)
                break;
            ++cp;
            if (*cp == '*')
            {
                n_functions++;
            }
            remaining_length -= (cp - op);
            op = cp;
        }
    }

    char** function_names = (char**)malloc(sizeof(char*) * n_functions);
    char** return_types = (char**)malloc(sizeof(char*) * n_functions);
    char** parameter_lists = (char**)malloc(sizeof(char*) * n_functions);

    // now we can start the actual parsing
    int n_parsed = 0;
    
    for(char *cp = backend_funcs_h, c = *cp++; c; c = *cp++)
    {
        if (c == 't' && n_parsed < n_functions)
        {
            if (0 == memcmp("ypedef", cp, sizeof("ypedef") - 1))
            {
                // line: typedef uint32_t (*beginFunction_t) (void* ctx, uint32_t fnId, void* fd);
                cp += (sizeof("typedef") - 1);
                assert(*(cp - 1) == ' ');
                // cp should be the start of the return type
                {
                    const char* begin_return_type = cp;
                    while(*++cp != ' ') {}
                    // cp should now be one past the end of the return type
                    int len = cp - begin_return_type;
                    return_types[n_parsed] = malloc(len + 1);
                    memcpy(return_types[n_parsed], begin_return_type, len);
                    return_types[n_parsed][len] = '\0';
                }
                cp += 3;
                assert(*(cp - 1) == '*');
                assert(*(cp - 2) == '(');
                {
                    const char* begin_function_type_name = cp;
                    while(*++cp != ')') {}
                    // cp should now be one past the of the function type
                    int len = cp - begin_function_type_name - 2;
                    function_names[n_parsed] = malloc(len + 1);
                    memcpy(function_names[n_parsed], begin_function_type_name, len);
                    function_names[n_parsed][len] = '\0';
                }
                cp += 2;
                assert(*(cp - 1) == ' ');
                assert(*(cp) == '(');
                // now for the argument list we store it inclusive of the ( and )
                {
                    const char* begin_argument_list = cp;
                    while(*++cp != ';') {}
                    // cp should now be one past the argument list
                    int len = cp - begin_argument_list;
                    parameter_lists[n_parsed] = malloc(len + 1);
                    memcpy(parameter_lists[n_parsed], begin_argument_list, len);
                    parameter_lists[n_parsed][len] = '\0';
                }
                n_parsed++;
            }
        }
    }

    // now we scan the exisiting implementation if there is one

    for(int i = 0; i < n_parsed; i++)
    {
        if (argc == 3)
        {
            // whatevs
        }
        if (0 != strcmp(function_names[i], "new_instance"))
        {
            printf("static inline %s %s_%s(%s* self%s\n{\n}\n\n"
                , return_types[i]
                , argv[1]
                , function_names[i]
                , argv[1]
                , parameter_lists[i] + sizeof("void* ctx")
            );
        }
        else
        {
            printf("static inline %s %s_%s(%s** resultP%s\n{\n}\n\n"
                , return_types[i]
                , argv[1]
                , function_names[i]
                , argv[1]
                , ")"
            );
        }
    }
    printf("\nconst BackendInterface %s_interface = {\n", argv[1]);
    // .Initialize = (Initialize_t) BCGen_Initialize,
    for(int i = 0; i < n_parsed - 1; i++)
    {
        printf("    .%s = (%s_t) %s_%s,\n"
            , function_names[i]
            , function_names[i]
            , argv[1]
            , function_names[i]
        );
    }
    printf("    .%s = (%s_t) %s_%s\n};\n"
        , function_names[n_parsed -1]
        , function_names[n_parsed -1]
        , argv[1]
        , function_names[n_parsed -1]
    );

    return 0;
}
