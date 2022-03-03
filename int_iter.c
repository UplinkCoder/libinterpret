#include <string.h>
#include <assert.h>
#include <stdbool.h>
#include <unistd.h>

#include "int_iter.h"

#ifndef NDEBUG
#  define DEBUG(...) __VA_ARGS__
#else
#  define DEBUG(...)
#endif

#define IntIter_Check(SELF) { \
    assert(SELF->fd ? (SELF->lastRead - SELF->remaining_chars == \
        SELF->currentReadPosition - SELF->char_buf) : 1); \
}

void IntIter_RefillBuffer(IntIter* self)
{
    if (!self->fd) assert(!"InitIter should never be called if using a memory buffer");

    IntIter_Check(self);
    int oldCharsInBuffer = self->remaining_chars;
    // move remianing content to the start of the buffer
    memmove(self->char_buf
            , self->currentReadPosition
            , oldCharsInBuffer
    );

    self->remaining_chars =
        read(self->fd
        , self->char_buf + oldCharsInBuffer
        , (sizeof(self->char_buf) - oldCharsInBuffer)
    );

    self->remaining_chars += oldCharsInBuffer;
    DEBUG(self->lastRead = self->remaining_chars);
    self->readAgain = (self->remaining_chars == sizeof(self->char_buf));
    self->currentReadPosition = self->char_buf;
}

void IntIter_Init(IntIter* self, int fd)
{
    self->remaining_chars = 0;
    DEBUG(self->lastRead = 0;)
    self->fd = fd;
    self->currentReadPosition = self->char_buf;

    IntIter_RefillBuffer(self);
    IntIter_Check(self);
}

void IntIter_FromBuffer(IntIter* self, void* buffer, uint32_t sz)
{
    self->fd = 0;
    self->readAgain = 0;

    self->remaining_chars = sz;
    DEBUG(self->lastRead = sz;)
    self->currentReadPosition = buffer;
}

_Bool IntIter_RefillWhenEmpty(IntIter *self, int chars_in_buffer)
{
    if (chars_in_buffer == 0)
    {
        if (self->readAgain)
        {
            self->remaining_chars = 0;
            IntIter_RefillBuffer(self);
            return true;
        }
        else
            return false;
    }
    // assert(!"Unreachable this function must only be called when chars_in_buffer == 0");
    return true;
}

bool IntIter_NextInt(IntIter* self, int* value)
{
    int result = 0;
    _Bool isNegative = false;

    char c;

    if (self->fd && (self->remaining_chars < 10))
        IntIter_RefillBuffer(self);

    IntIter_Check(self);
    int chars_in_buffer = self->remaining_chars;
    if (chars_in_buffer == 0)
        return false;

    // skip whitespace

    while((c = *(self->currentReadPosition++)), (c == ' ' || c == '\n' || c == '\t')) {
        chars_in_buffer--;
        if (!IntIter_RefillWhenEmpty(self, chars_in_buffer))
            return false;
    }

    chars_in_buffer--;
    self->remaining_chars = chars_in_buffer;
    IntIter_Check(self);

    // c is now the first non-whitespace character
    isNegative = (c == '-');
    // if it's negative we need to skip the next char
    if (isNegative) {
        c = *(self->currentReadPosition++);
        chars_in_buffer--;
        if (!IntIter_RefillWhenEmpty(self, chars_in_buffer))
            return false;
    }

    self->remaining_chars = chars_in_buffer;
    IntIter_Check(self);

    // now the char has to be in range of '0' - '9' or it's invalid

    if (c > '9' || c < '0')
    {
        // printf("NonNumberChar: '%c' == %d\n", c, c);
        return false;
    }
    while(c <= '9' && c >= '0')
    {
        result *= 10;
        result += (c - '0');

        if (!IntIter_RefillWhenEmpty(self, chars_in_buffer))
            return false;

        c = *(self->currentReadPosition++);
        chars_in_buffer--;
    }

    if (isNegative)
        result *= -1;

    self->remaining_chars = chars_in_buffer;

    IntIter_Check(self);
    *value = result;
    return true;
}
#undef DEBUG
