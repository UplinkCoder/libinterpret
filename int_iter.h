#ifndef _INT_ITER_H_
#define _INT_ITER_H_

#ifndef NDEBUG
#  define DEBUG(...) __VA_ARGS__
#else
#  define DEBUG(...)
#endif

typedef struct IntIter {
    int fd;
    int remaining_chars;
    DEBUG(int lastRead;)
    char* currentReadPosition;
    _Bool readAgain;
    char char_buf[1024];
} IntIter;

void IntIter_Init(IntIter* self, int fd);
void IntIter_FromBuffer(IntIter* self, void* buffer, uint32_t sz);
bool IntIter_NextInt(IntIter* self, int* value);

#undef DEBUG
#endif
