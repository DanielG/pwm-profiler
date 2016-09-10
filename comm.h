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

#ifndef __COMM_H__
#define __COMM_H__

#define dma_chunk_size 128
enum dma_type {
        DMA_TYPE_LOWER = 0,
        DMA_TYPE_UPPER = 1,
};

/* we use a self synchronizing code, each codeword is a dma_header concatinated
 * with a dma_buffer_t, when the host sends a sync command a dma_escape is
 * inserted into the stream consisting of (sizeof(dma_header) +
 * sizeof(dma_buffer_t)) bytes of 0x40 ('@'). The first 4 byte field of the
 * dma_header is required to always be (0x00 0x00 0x0D 0x0A) therefore we can
 * sychronize the stream by looking for the (0xff 0xff... 0x00 0x00 0x0D 0x0A)
 * sequence which cannot appear in the stream itself */

struct dma_header {
        uint32_t preamble;

        uint8_t type;
        uint8_t channel;
        uint8_t __reserved1;
        uint8_t __reserved2;

        uint32_t systick;
        uint32_t __reserved3;
};

typedef uint16_t dma_chunk_t[dma_chunk_size];

#endif
