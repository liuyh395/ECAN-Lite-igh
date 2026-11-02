#include "../examples/common/igh_example_args.h"
#include <assert.h>
#include <string.h>

int main(int argc, char **argv)
{
    if (argc == 3) {
        if (strcmp(argv[1], "u8") == 0) (void)igh_example_u8(argv[2]);
        else if (strcmp(argv[1], "u32") == 0) (void)igh_example_u32(argv[2]);
        else if (strcmp(argv[1], "i32") == 0) (void)igh_example_i32(argv[2]);
        else if (strcmp(argv[1], "float") == 0) (void)igh_example_float(argv[2]);
        else return 1;
        return 0;
    }
    assert(igh_example_u8("0xff") == 255);
    assert(igh_example_u16("65535") == 65535);
    assert(igh_example_u32("4294967295") == UINT32_MAX);
    assert(igh_example_i32("-2147483648") == INT32_MIN);
    assert(igh_example_int("-1") == -1);
    assert(igh_example_float("0.125") == 0.125F);
    return 0;
}
