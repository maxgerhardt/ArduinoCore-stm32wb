#include <stdio.h>

int main()
{
    int i, c;

    i = 0;

    do
    {
	c = getchar();

	if (c >= 0) {
	    printf("0x%02x, ", c ^ 0xff);
	}

	if ((i & 15) == 15) {
	    printf("\n    ");
	}

	i++;
    }
    while (c >= 0);

    if (i & 15) {
        while (i & 15) {
            printf("0x%02x, ", 0);
            i++;
        }

        printf("\n    ");
    }
}
