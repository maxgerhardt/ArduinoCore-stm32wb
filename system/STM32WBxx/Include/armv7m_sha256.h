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


#if !defined(_ARMV7M_SHA256_H)
#define _ARMV7M_SHA256_H

#ifdef __cplusplus
extern "C" {
#endif

#define ARMV7M_SHA256_BLOCK_SIZE  64
#define ARMV7M_SHA256_HASH_SIZE   32    

#define ARMV7M_SHA256_NUM_BITS    256
#define ARMV7M_SHA256_NUM_BYTES   32
#define ARMV7M_SHA256_NUM_WORDS   8

typedef struct _armv7m_sha256_context_t {
    uint32_t                   hash[ARMV7M_SHA256_HASH_SIZE / 4];
    uint32_t                   length;
    uint32_t                   index;
    uint8_t                    data[ARMV7M_SHA256_BLOCK_SIZE];
} armv7m_sha256_context_t;

extern void armv7m_sha256_init(armv7m_sha256_context_t *sha256_ctx);
extern void armv7m_sha256_update(armv7m_sha256_context_t *sha256_ctx, const uint8_t *data, size_t size);
extern void armv7m_sha256_final(armv7m_sha256_context_t *sha256_ctx, uint32_t *hash);

#ifdef __cplusplus
}
#endif

#endif /* _ARMV7M_SHA256_H */
