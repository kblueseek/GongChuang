/* Host-only test driver: feed a raw byte stream through the parser. */
#include <stdio.h>
#include "picomm.h"

int main(int argc, char **argv)
{
    FILE *f;
    pi_parser_t p;
    pi_frame_t fr;
    int c;
    int i;

    if (argc < 2) { printf("usage: hosttest <file>\n"); return 2; }
    f = fopen(argv[1], "rb");
    if (f == NULL) { printf("E: cannot open %s\n", argv[1]); return 2; }

    pi_parser_reset(&p);
    p.frames_ok = 0;
    p.frames_bad = 0;

    while ((c = fgetc(f)) != EOF) {
        if (pi_parser_feed(&p, (uint8_t)c, &fr)) {
            printf("FRAME ok=%u cmd=0x%02X seq=%u len=%u n=%u",
                   (unsigned)fr.crc_ok, (unsigned)fr.cmd, (unsigned)fr.seq,
                   (unsigned)fr.len, (unsigned)fr.data_len);
            for (i = 0; i < (int)fr.data_len; i++) {
                printf(" %02X", (unsigned)fr.data[i]);
            }
            printf("\n");
        }
    }
    fclose(f);
    printf("STAT ok=%lu bad=%lu\n",
           (unsigned long)p.frames_ok, (unsigned long)p.frames_bad);
    return 0;
}
