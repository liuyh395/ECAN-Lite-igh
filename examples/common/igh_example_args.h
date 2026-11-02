/* SPDX-License-Identifier: MIT */
#ifndef IGH_EXAMPLE_ARGS_H
#define IGH_EXAMPLE_ARGS_H

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* 仅用于示例启动阶段。非法文本必须在申请 master 前退出，不能截断后成为另一节点/通道。 */
static inline void igh_example_bad_number(const char *text)
{
    fprintf(stderr, "Invalid numeric argument: %s\n", text ? text : "(null)");
    exit(2);
}

static inline uint64_t igh_example_unsigned(const char *text, uint64_t maximum)
{
    char *end;
    unsigned long long value;
    if (text == NULL || !isdigit((unsigned char)text[0]))
        igh_example_bad_number(text);
    errno = 0;
    value = strtoull(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || value > maximum)
        igh_example_bad_number(text);
    return (uint64_t)value;
}

static inline int64_t igh_example_signed(const char *text, int64_t minimum, int64_t maximum)
{
    char *end;
    long long value;
    if (text == NULL || text[0] == '\0' || isspace((unsigned char)text[0]))
        igh_example_bad_number(text);
    errno = 0;
    value = strtoll(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || value < minimum || value > maximum)
        igh_example_bad_number(text);
    return (int64_t)value;
}

#define IGH_EXAMPLE_UNSIGNED_READER(name, type, maximum) \
    static inline type name(const char *text) { return (type)igh_example_unsigned(text, maximum); }
IGH_EXAMPLE_UNSIGNED_READER(igh_example_u8, uint8_t, UINT8_MAX)
IGH_EXAMPLE_UNSIGNED_READER(igh_example_u16, uint16_t, UINT16_MAX)
IGH_EXAMPLE_UNSIGNED_READER(igh_example_u32, uint32_t, UINT32_MAX)
IGH_EXAMPLE_UNSIGNED_READER(igh_example_uint, unsigned, UINT_MAX)
IGH_EXAMPLE_UNSIGNED_READER(igh_example_size, size_t, SIZE_MAX)
IGH_EXAMPLE_UNSIGNED_READER(igh_example_ulong, unsigned long, ULONG_MAX)
#undef IGH_EXAMPLE_UNSIGNED_READER

static inline int igh_example_int(const char *text)
{ return (int)igh_example_signed(text, INT_MIN, INT_MAX); }
static inline int32_t igh_example_i32(const char *text)
{ return (int32_t)igh_example_signed(text, INT32_MIN, INT32_MAX); }
static inline long igh_example_long(const char *text)
{ return (long)igh_example_signed(text, LONG_MIN, LONG_MAX); }

static inline float igh_example_float(const char *text)
{
    char *end;
    float value;
    if (text == NULL || text[0] == '\0' || isspace((unsigned char)text[0]))
        igh_example_bad_number(text);
    errno = 0;
    value = strtof(text, &end);
    if (errno != 0 || end == text || *end != '\0' || !isfinite(value))
        igh_example_bad_number(text);
    return value;
}
#endif
