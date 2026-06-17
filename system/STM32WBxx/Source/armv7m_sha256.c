/*
 * Copyright (c) 2022-2024 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#include "armv7m.h"

#define SWAP(_x) (uint32_t)((((_x) >> 24) & 0x000000ff) | (((_x) >> 8) & 0x0000ff00) | (((_x) << 8) & 0x00ff0000) |  (((_x) << 24) & 0xff000000))

static const uint32_t __attribute__((used)) armv7m_sha256_const_K[64] =
{
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

static const uint32_t __attribute__((used)) armv7m_sha256_const_H[8] =
{
    0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19, 
};

static void __attribute__((naked, noinline)) armv7m_sha256_compress(uint32_t *state, const uint32_t *data, const uint32_t *data_e)
{
    register uint32_t _state  __asm__("r0") = (uint32_t)state;
    register uint32_t _data   __asm__("r1") = (uint32_t)data;
    register uint32_t _data_e __asm__("r2") = (uint32_t)data_e;

    __asm__(
        "    push    { r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, r11, lr }    \n"
        "    mov     lr, sp                                                  \n"
        "    mov     r8, r1                                                  \n"
        "                                                                    \n"
        "1:  sub     sp, #256                                                \n"
        "    mov     r9, sp                                                  \n"
        "                                                                    \n"
        "    ldmia   r8!, { r0, r1, r2, r3, r4, r5, r6, r7 }                 \n"
        "    rev     r0, r0                                                  \n"
        "    rev     r1, r1                                                  \n"
        "    rev     r2, r2                                                  \n"
        "    rev     r3, r3                                                  \n"
        "    rev     r4, r4                                                  \n"
        "    rev     r5, r5                                                  \n"
        "    rev     r6, r6                                                  \n"
        "    rev     r7, r7                                                  \n"
        "    stmia   r9!, { r0, r1, r2, r3, r4, r5, r6, r7 }                 \n"
        "    ldmia   r8!, { r0, r1, r2, r3, r4, r5, r6, r7 }                 \n"
        "    rev     r0, r0                                                  \n"
        "    rev     r1, r1                                                  \n"
        "    rev     r2, r2                                                  \n"
        "    rev     r3, r3                                                  \n"
        "    rev     r4, r4                                                  \n"
        "    rev     r5, r5                                                  \n"
        "    rev     r6, r6                                                  \n"
        "    rev     r7, r7                                                  \n"
        "    stmia   r9!, { r0, r1, r2, r3, r4, r5, r6, r7 }                 \n"
        "    str     r8, [sp, #260]                                          \n"
        "                                                                    \n"
        "    mov     r0, sp                                                  \n"
        "    add     r1, r0, #192                                            \n"
        "    ldrd    r4, r5, [r0, #56]                                       \n"
        "                                                                    \n"
        "2:  ror     r2, r4, #17                                             \n"
        "    eor     r2, r2, r4, ror #19                                     \n"
        "    eor     r2, r2, r4, lsr #10                                     \n"
        "    ror     r3, r5, #17                                             \n"
        "    eor     r3, r3, r5, ror #19                                     \n"
        "    eor     r3, r3, r5, lsr #10                                     \n"
        "    ldrd    r4, r5, [r0, #36]                                       \n"
        "    adds    r2, r4                                                  \n"
        "    adds    r3, r5                                                  \n"
        "    ldmia   r0, { r6, r7, r8 }                                      \n"
        "    ror     r4, r7, #7                                              \n"
        "    eor     r4, r4, r7, ror #18                                     \n"
        "    eor     r4, r4, r7, lsr #3                                      \n"
        "    adds    r4, r2                                                  \n"
        "    ror     r5, r8, #7                                              \n"
        "    eor     r5, r5, r8, ror #18                                     \n"
        "    eor     r5, r5, r8, lsr #3                                      \n"
        "    adds    r5, r3                                                  \n"
        "    adds    r4, r6                                                  \n"
        "    adds    r5, r7                                                  \n"
        "    strd    r4, r5, [r0, #64]                                       \n"
        "    adds    r0, #8                                                  \n"
        "    cmp     r0, r1                                                  \n"
        "    bne     2b                                                      \n"
        "                                                                    \n"
        "    ldr     r12, [sp, #256]                                         \n"
        "    ldmia   r12, { r2, r3, r4, r5, r6, r7, r8, r9 }                 \n"
        "    movw    r12, #:lower16:armv7m_sha256_const_K                    \n"
        "    movt    r12, #:upper16:armv7m_sha256_const_K                    \n"
        "                                                                    \n"
        "3:  ror     r0, r6, #6                                              \n"
        "    eor     r0, r0, r6, ror #11                                     \n"
        "    eor     r0, r0, r6, ror #25                                     \n"
        "    add     r9, r0                                                  \n"
        "    eor     r0, r7, r8                                              \n"
        "    ands    r0, r6                                                  \n"
        "    eor     r0, r8                                                  \n"
        "    add     r9, r0                                                  \n"
        "    ldrd    r0, r10, [r12], #8                                      \n"
        "    add     r9, r0                                                  \n"
        "    ldrd    r1, r11, [sp], #8                                       \n"
        "    add     r9, r1                                                  \n"
        "    add     r5, r9                                                  \n"
        "    ror     r0, r2, #2                                              \n"
        "    eor     r0, r0, r2, ror #13                                     \n"
        "    eor     r0, r0, r2, ror #22                                     \n"
        "    add     r9, r0                                                  \n"
        "    orr     r0, r3, r4                                              \n"
        "    and     r1, r3, r4                                              \n"
        "    ands    r0, r2                                                  \n"
        "    orrs    r0, r1                                                  \n"
        "    add     r9, r0                                                  \n"
        "    ror     r0, r5, #6                                              \n"
        "    eor     r0, r0, r5, ror #11                                     \n"
        "    eor     r0, r0, r5, ror #25                                     \n"
        "    add     r8, r0                                                  \n"
        "    eor     r0, r6, r7                                              \n"
        "    ands    r0, r5                                                  \n"
        "    eors    r0, r7                                                  \n"
        "    add     r8, r0                                                  \n"
        "    add     r8, r10                                                 \n"
        "    add     r8, r11                                                 \n"
        "    add     r4, r8                                                  \n"
        "    ror     r0, r9, #2                                              \n"
        "    eor     r0, r0, r9, ror #13                                     \n"
        "    eor     r0, r0, r9, ror #22                                     \n"
        "    add     r8, r0                                                  \n"
        "    orr     r0, r2, r3                                              \n"
        "    and     r1, r2, r3                                              \n"
        "    and     r0, r9                                                  \n"
        "    orrs    r0, r1                                                  \n"
        "    add     r8, r0                                                  \n"
        "    ror     r0, r4, #6                                              \n"
        "    eor     r0, r0, r4, ror #11                                     \n"
        "    eor     r0, r0, r4, ror #25                                     \n"
        "    adds    r7, r0                                                  \n"
        "    eor     r0, r5, r6                                              \n"
        "    ands    r0, r4                                                  \n"
        "    eors    r0, r6                                                  \n"
        "    adds    r7, r0                                                  \n"
        "    ldrd    r0, r10, [r12], #8                                      \n"
        "    adds    r7, r0                                                  \n"
        "    ldrd    r1, r11, [sp], #8                                       \n"
        "    adds    r7, r1                                                  \n"
        "    adds    r3, r7                                                  \n"
        "    ror     r0, r8, #2                                              \n"
        "    eor     r0, r0, r8, ror #13                                     \n"
        "    eor     r0, r0, r8, ror #22                                     \n"
        "    adds    r7, r0                                                  \n"
        "    orr     r0, r9, r2                                              \n"
        "    and     r1, r9, r2                                              \n"
        "    and     r0, r8                                                  \n"
        "    orrs    r0, r1                                                  \n"
        "    adds    r7, r0                                                  \n"
        "    ror     r0, r3, #6                                              \n"
        "    eor     r0, r0, r3, ror #11                                     \n"
        "    eor     r0, r0, r3, ror #25                                     \n"
        "    adds    r6, r0                                                  \n"
        "    eor     r0, r4, r5                                              \n"
        "    ands    r0, r3                                                  \n"
        "    eors    r0, r5                                                  \n"
        "    adds    r6, r0                                                  \n"
        "    add     r6, r10                                                 \n"
        "    add     r6, r11                                                 \n"
        "    adds    r2, r6                                                  \n"
        "    ror     r0, r7, #2                                              \n"
        "    eor     r0, r0, r7, ror #13                                     \n"
        "    eor     r0, r0, r7, ror #22                                     \n"
        "    adds    r6, r0                                                  \n"
        "    orr     r0, r8, r9                                              \n"
        "    and     r1, r8, r9                                              \n"
        "    ands    r0, r7                                                  \n"
        "    orrs    r0, r1                                                  \n"
        "    adds    r6, r0                                                  \n"
        "    ror     r0, r2, #6                                              \n"
        "    eor     r0, r0, r2, ror #11                                     \n"
        "    eor     r0, r0, r2, ror #25                                     \n"
        "    adds    r5, r0                                                  \n"
        "    eor     r0, r3, r4                                              \n"
        "    ands    r0, r2                                                  \n"
        "    eors    r0, r4                                                  \n"
        "    adds    r5, r0                                                  \n"
        "    ldrd    r0, r10, [r12], #8                                      \n"
        "    adds    r5, r0                                                  \n"
        "    ldrd    r1, r11, [sp], #8                                       \n"
        "    adds    r5, r1                                                  \n"
        "    add     r9, r5                                                  \n"
        "    ror     r0, r6, #2                                              \n"
        "    eor     r0, r0, r6, ror #13                                     \n"
        "    eor     r0, r0, r6, ror #22                                     \n"
        "    adds    r5, r0                                                  \n"
        "    orr     r0, r7, r8                                              \n"
        "    and     r1, r7, r8                                              \n"
        "    ands    r0, r6                                                  \n"
        "    orrs    r0, r1                                                  \n"
        "    adds    r5, r0                                                  \n"
        "    ror     r0, r9, #6                                              \n"
        "    eor     r0, r0, r9, ror #11                                     \n"
        "    eor     r0, r0, r9, ror #25                                     \n"
        "    adds    r4, r0                                                  \n"
        "    eor     r0, r2, r3                                              \n"
        "    and     r0, r9                                                  \n"
        "    eors    r0, r3                                                  \n"
        "    adds    r4, r0                                                  \n"
        "    add     r4, r10                                                 \n"
        "    add     r4, r11                                                 \n"
        "    add     r8, r4                                                  \n"
        "    ror     r0, r5, #2                                              \n"
        "    eor     r0, r0, r5, ror #13                                     \n"
        "    eor     r0, r0, r5, ror #22                                     \n"
        "    adds    r4, r0                                                  \n"
        "    orr     r0, r6, r7                                              \n"
        "    and     r1, r6, r7                                              \n"
        "    ands    r0, r5                                                  \n"
        "    orrs    r0, r1                                                  \n"
        "    adds    r4, r0                                                  \n"
        "    ror     r0, r8, #6                                              \n"
        "    eor     r0, r0, r8, ror #11                                     \n"
        "    eor     r0, r0, r8, ror #25                                     \n"
        "    adds    r3, r0                                                  \n"
        "    eor     r0, r9, r2                                              \n"
        "    and     r0, r8                                                  \n"
        "    eors    r0, r2                                                  \n"
        "    adds    r3, r0                                                  \n"
        "    ldrd    r0, r10, [r12], #8                                      \n"
        "    adds    r3, r0                                                  \n"
        "    ldrd    r1, r11, [sp], #8                                       \n"
        "    adds    r3, r1                                                  \n"
        "    adds    r7, r3                                                  \n"
        "    ror     r0, r4, #2                                              \n"
        "    eor     r0, r0, r4, ror #13                                     \n"
        "    eor     r0, r0, r4, ror #22                                     \n"
        "    adds    r3, r0                                                  \n"
        "    orr     r0, r5, r6                                              \n"
        "    and     r1, r5, r6                                              \n"
        "    ands    r0, r4                                                  \n"
        "    orrs    r0, r1                                                  \n"
        "    adds    r3, r0                                                  \n"
        "    ror     r0, r7, #6                                              \n"
        "    eor     r0, r0, r7, ror #11                                     \n"
        "    eor     r0, r0, r7, ror #25                                     \n"
        "    adds    r2, r0                                                  \n"
        "    eor     r0, r8, r9                                              \n"
        "    ands    r0, r7                                                  \n"
        "    eor     r0, r9                                                  \n"
        "    adds    r2, r0                                                  \n"
        "    add     r2, r10                                                 \n"
        "    add     r2, r11                                                 \n"
        "    adds    r6, r2                                                  \n"
        "    ror     r0, r3, #2                                              \n"
        "    eor     r0, r0, r3, ror #13                                     \n"
        "    eor     r0, r0, r3, ror #22                                     \n"
        "    adds    r2, r0                                                  \n"
        "    orr     r0, r4, r5                                              \n"
        "    and     r1, r4, r5                                              \n"
        "    ands    r0, r3                                                  \n"
        "    orrs    r0, r1                                                  \n"
        "    adds    r2, r0                                                  \n"
        "    cmp     sp, lr                                                  \n"
        "    bne     3b                                                      \n"
        "                                                                    \n"
        "    ldr     r12, [sp, #0]                                           \n"
        "    ldmia   r12, { r0, r1, r10, r11 }                               \n"
        "    adds    r2, r0                                                  \n"
        "    adds    r3, r1                                                  \n"
        "    add     r4, r10                                                 \n"
        "    add     r5, r11                                                 \n"
        "    stmia   r12!, { r2, r3, r4, r5 }                                \n"
        "    ldmia   r12, { r2, r3, r4, r5 }                                 \n"
        "    adds    r2, r6                                                  \n"
        "    adds    r3, r7                                                  \n"
        "    add     r4, r8                                                  \n"
        "    add     r5, r9                                                  \n"
        "    stmia   r12!, { r2, r3, r4, r5 }                                \n"
        "                                                                    \n"
        "    ldrd    r8, r9, [sp, #4]                                        \n"
        "    cmp     r8, r9                                                  \n"
        "    bne     1b                                                      \n"
        "                                                                    \n"
        "    pop     { r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, r11, pc }    \n"
        :
        : "r" (_state), "r" (_data), "r" (_data_e)
        );
}

void armv7m_sha256_init(armv7m_sha256_context_t *sha256_ctx)
{
    sha256_ctx->length = 0;
    sha256_ctx->index = 0;

    memcpy(&sha256_ctx->hash[0], &armv7m_sha256_const_H[0], 32);
}

void armv7m_sha256_update(armv7m_sha256_context_t *sha256_ctx, const uint8_t *data, size_t size)
{
    uint32_t index, count;
  
    sha256_ctx->length += size;

    index = sha256_ctx->index;
    
    while (size)
    {
        if ((index == 0) && !((uint32_t)data & 3) && (size & ~63))
        {
            count = size & ~63;

            armv7m_sha256_compress(&sha256_ctx->hash[0], (const uint32_t*)data, (const uint32_t*)(data + count));

            data += count;
            size -= count;
        }
        else
        {
            sha256_ctx->data[index++] = *data++;
            
            size--;
            
            if (index == 64)
            {
                armv7m_sha256_compress(&sha256_ctx->hash[0], (const uint32_t*)&sha256_ctx->data[0], (const uint32_t*)&sha256_ctx->data[64]);
                
                index = 0;
            }
        }
    }
    
    sha256_ctx->index = index;
}

void armv7m_sha256_final(armv7m_sha256_context_t *sha256_ctx, uint32_t *hash)
{
    sha256_ctx->data[sha256_ctx->index++] = 0x80;

    if (sha256_ctx->index > (64-8))
    {
        memset(&sha256_ctx->data[sha256_ctx->index], 0x00, 64 - sha256_ctx->index);
        
        armv7m_sha256_compress(&sha256_ctx->hash[0], (const uint32_t*)&sha256_ctx->data[0], (const uint32_t*)&sha256_ctx->data[64]);

        sha256_ctx->index = 0;
    }

    if (sha256_ctx->index != (64-8+3))
    {
        memset(&sha256_ctx->data[sha256_ctx->index], 0x00, (64-8+3) - sha256_ctx->index);
    }

    sha256_ctx->data[59] = sha256_ctx->length >> (32-3);
    sha256_ctx->data[60] = sha256_ctx->length >> (24-3);
    sha256_ctx->data[61] = sha256_ctx->length >> (16-3);
    sha256_ctx->data[62] = sha256_ctx->length >>  (8-3);
    sha256_ctx->data[63] = sha256_ctx->length <<    (3);

    armv7m_sha256_compress(&sha256_ctx->hash[0], (const uint32_t*)&sha256_ctx->data[0], (const uint32_t*)&sha256_ctx->data[64]);

    hash[0] = SWAP(sha256_ctx->hash[0]);
    hash[1] = SWAP(sha256_ctx->hash[1]);
    hash[2] = SWAP(sha256_ctx->hash[2]);
    hash[3] = SWAP(sha256_ctx->hash[3]);
    hash[4] = SWAP(sha256_ctx->hash[4]);
    hash[5] = SWAP(sha256_ctx->hash[5]);
    hash[6] = SWAP(sha256_ctx->hash[6]);
    hash[7] = SWAP(sha256_ctx->hash[7]);    
}

