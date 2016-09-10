/*
 * pwm-profiler: capture profile of lighting fixtures at PWM level
 * Copyright (C) 2016  Daniel Gröber <dxld@darkboxed.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <termios.h>
#include "term.h"
#include "concat.h"

#include "../comm.h"

#define asizeof(xs) (sizeof(xs) / sizeof(*(xs)))

const char stopword = '\x13';
const char contword = '\x11';
const char syncword = '\r';

struct dma_entry {
        uint16_t per;
        uint16_t duty;
};

struct dma_packet {
        struct dma_header header;
        struct dma_entry data[dma_chunk_size/4];
};

void terror(enum term_errno_e e) {
        const char* err = term_strerror(term_errno, errno);
        fprintf(stderr, "Error, %s", err);
        exit(1);
}

void perror_(const char* str) {
	perror(str);
	exit(1);
}

int full_write(int fd, const char *buf, int len) {
	int rv;
	int index = 0;
	while(len > 0) {
		rv = write(fd, buf + index, len);
		if(rv < 0)
			return rv;

		index += rv;
		len -= rv;
	}

	return 0;
}

void remove_newlines(char *str) {
        char *n;
        do {
                n = strchr(str, '\n');
                if(n != NULL)
                        *n = '\0';

        } while(n != NULL);

        do {
                n = strchr(str, '\r');
                if(n != NULL)
                        *n = '\0';

        } while(n != NULL);
}

int strcntchar(char *str, char c) {
	int cnt = 0, i, len = strlen(str);
	for(i=0; i < len; i++) {
		if(str[i] == c)
			cnt++;
	}

	return cnt;
}

char *linebuf;

void linebuf_write(char* buf) {
        char* tmp = concat(linebuf, buf, NULL);
        free(linebuf);
        linebuf = tmp;
}

char *linebuf_read(void) {
        char *term = strchr(linebuf, '\n');
        if(term) {
                char *nlinebuf = concat(term + 1, NULL);
                *term = '\0';
                char *line = concat(linebuf, NULL);

                free(linebuf);
                linebuf = nlinebuf;

                return line;
        }

        return NULL;
}


void fprint_line(FILE* f, char *msg, char *line) {
        char *tmp = concat(line, NULL);
        remove_newlines(tmp);
        fprintf(f, "%s: %s\n", msg, tmp);
        free(tmp);
}

void fprint_bytes(FILE *f, char *msg, char *bytes) {
        fprintf(f, "%s BYTES: ", msg);

        char *ptr = bytes;
        while(*ptr) {
                fprintf(f, "%x ", *ptr);
                ptr++;
        }

        fprintf(f, "\n");
}

void connect(int fd) {
        char buf[dma_chunk_size];
        int rv;

        rv = term_set_buffer(fd, dma_chunk_size, 10);
        if(rv < 0)
                terror(rv);

        rv = term_apply(fd);
        if(rv < 0)
                terror(rv);

        int w = full_write(fd, &stopword, 1);
        if(w < 0)
                exit(1);

        sleep(1);
        tcflush(fd, TCIOFLUSH);

        w = full_write(fd, &syncword, 1);
        if(w < 0)
                exit(1);

        size_t atchars = 0;
        do {
                int r = read(fd, buf, sizeof(buf));
                fprintf(stderr, "read(): %d\n", r);
                if(r < 0) {
                        perror("read(): ");
                        exit(1);
                } else if(r == 0) {
                        break;
                }

                for(size_t i=0; i < r; i++) {
                        if(buf[i] == '@')
                                atchars++;
                        else
                                atchars = 0;
                }

                if(atchars >= 512)
                        break;

        } while(1);

        rv = term_set_buffer(fd, dma_chunk_size + 16, 0);
        if(rv < 0)
                terror(rv);

        rv = term_apply(fd);
        if(rv < 0)
                terror(rv);

}

int read_loop(int fd) {
        int w = full_write(fd, &contword, 1);
        if(w < 0)
                exit(1);

        char buf[dma_chunk_size + 16];


        do {
                int r = read(fd, buf, sizeof(buf));
                /* fprintf(stderr, "read(): %d\n", r); */
                if(r < 0) {
                        perror("read(): ");
                        exit(1);
                } else if(r == 0) {
                        fprintf(stderr, "empty read\n");
                        exit(1);
                }

                struct dma_packet *pkt = (struct dma_packet *)&buf[0];

                printf("%04x:%04x\n", pkt->header.channel, pkt->header.type);

                struct dma_entry *data = &pkt->data[0];

                for(size_t i=0; i < asizeof(pkt->data); i++) {
                        struct dma_entry ent = data[i];

                        ent.duty = ent.duty + 1;
                        ent.per = ent.per + 1;

                        double mag = (double)ent.duty / (double)ent.per;

                        /* assert(mag <= 1.0); */

                        if(pkt->header.channel == 3)
                                printf("%04x-%04x=%0.2f ", ent.per, ent.duty, mag);
                        /* printf("%04x ", mag); */
                }
                printf("\n");

                /* assert(r == dma_chunk_size); */
                /* if(r != dma_chunk_size) { */
                /*         return -1; */
                /* } */

                /* full_write(STDOUT_FILENO, buf, r); */
        } while(1);

}

int main(int argc, char **argv) {
        enum term_errno_e rv;

        rv = term_lib_init();
        if(rv < 0)
                terror(rv);

        int fd = open(argv[1], O_RDWR | O_NOCTTY);
        if(fd < 0)
                perror("open");

        rv = term_set (fd, 0, 1000000, P_NONE, 8, FC_NONE, 1, 1);
        if(rv < 0)
                terror(rv);

        rv = term_set_buffer(fd, 128 + 16, 10);
        if(rv < 0)
                terror(rv);

        rv = term_apply(fd);
        if(rv < 0)
                terror(rv);

        fprintf(stderr, "ready\n");

        do {
                connect(fd);
                read_loop(fd);
        } while(1);

        fprintf(stderr, "connected\n");

        term_remove(fd);

        return 0;
}
