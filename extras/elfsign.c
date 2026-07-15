/*
 * ./Configure --cross-compile-prefix=x86_64-w64-mingw32- mingw64 no-threads no-shared no-dso
 * x86_64-w64-mingw32-gcc elfsign.c -m64 -Iopenssl-OpenSSL_1_1_1s/include -Lopenssl-OpenSSL_1_1_1s/ -lssl -lcrypto -lws2_32 -lcrypt32 -o elfsign.exe
 *
 * ./Configure linux-x86_64 no-threads no-shared no-dso
 * gcc elfsign.c -Iopenssl-OpenSSL_1_1_1s/include -Lopenssl-OpenSSL_1_1_1s/ -lssl -lcrypto -o elfsign
 *
 * export PATH=/opt/osxcross/bin:$PATH
 * export LD_LIBRARY_PATH=/opt/osxcross/lib:$LD_LIBRARY_PATH
 * ./Configure --cross-compile-prefix=x86_64-apple-darwin13- darwin64-x86_64-cc  no-threads no-shared no-dso
 * x86_64-apple-darwin18-cc elfsign.c -Iopenssl-OpenSSL_1_1_1s/include -Lopenssl-OpenSSL_1_1_1s/ -lssl -lcrypto -o elfsign
 */
/*
 * elfsign --as128_key <keyfile> --rsa2048_key <keyfile> <in-file> [<out-file>]
 * 
 * - add option to generate aes128 key and rsa2048 key.
 *
 *  image authentication type as "None", "Signed", "Signed+Encrypted"
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <getopt.h>
#include <sys/time.h>

#ifdef WIN32
#include <fcntl.h>
#include <io.h>
#endif

#include "elfsign.h"

#include "openssl/bn.h"
#include "openssl/pem.h"
#include "openssl/rand.h"
#include "openssl/rsa.h"
#include "openssl/sha.h"
#include "openssl/aes.h"

#define SWAP(_x) (uint32_t)((((_x) >> 24) & 0x000000ff) | (((_x) >> 8) & 0x0000ff00) | (((_x) << 8) & 0x00ff0000) |  (((_x) << 24) & 0xff000000))


typedef struct _stm32wb_boot_vectors_t {
    uint32_t                      reserved_0[8];
    uint32_t                      magic;             // TAG, OPTION, IDCODE
    uint32_t                      base;              // 0x08000000
    uint32_t                      size;              // 16384
    uint32_t                      reserved_1[2];
    uint32_t                      offset;            // offset to boot_info
    uint32_t                      reserved_2[2];
} stm32wb_boot_vectors_t;

typedef struct _stm32wb_boot_ecc256_key_t {
    uint32_t                      x[8];
    uint32_t                      y[8];
} stm32wb_boot_ecc256_key_t;
  
typedef struct _stm32wb_boot_rsa2048_key_t {
    uint32_t                      n[64];
    uint32_t                      r2[64];   /* R^2 as little endian array */
    uint32_t                      u;        /* -1 / n mod 2^32 */
} stm32wb_boot_rsa2048_key_t;

#define STM32WB_BOOT_TAG_MASK                      0xffff0000
#define STM32WB_BOOT_TAG_SHIFT                     16
#define STM32WB_BOOT_TAG_BOOT                      0x54560000 // 'V', 'T'
#define STM32WB_BOOT_TYPE_MASK                     0x00008000
#define STM32WB_BOOT_TYPE_SHIFT                    15
#define STM32WB_BOOT_TYPE_BOOT                     0x00000000
#define STM32WB_BOOT_OPTION_MASK                   0x00007000
#define STM32WB_BOOT_OPTION_SHIFT                  12
#define STM32WB_BOOT_OPTION_PROTECTED              0x00004000
#define STM32WB_BOOT_OPTION_SIGNATURE_MASK         0x00003000
#define STM32WB_BOOT_OPTION_SIGNATURE_NONE         0x00000000
#define STM32WB_BOOT_OPTION_SIGNATURE_ECC256       0x00001000
#define STM32WB_BOOT_OPTION_SIGNATURE_RSA2048      0x00002000
#define STM32WB_BOOT_IDCODE_MASK                   0x00000fff
#define STM32WB_BOOT_IDCODE_SHIFT                  0
#define STM32WB_BOOT_IDCODE_STM32WB55              0x00000495

#define STM32WB_BOOT_BASE                          0x08000000
#define STM32WB_BOOT_SIZE                          0x00004000

#define STM32WB_BOOT_AES128_KEY                    0x08003fb0
#define STM32WB_BOOT_HASH                          0x08003fe0

typedef struct _stm32wb_boot_version_t {
    uint8_t                       major;
    uint8_t                       minor;
    uint16_t                      revision;
} stm32wb_boot_version_t;

typedef struct _stm32wb_boot_info_t {
    uint8_t                       uuid[16];
    stm32wb_boot_version_t        version;
    uint32_t                      application_base;
    uint32_t                      application_limit;
    uint32_t                      fwu_base;
    uint32_t                      fwu_limit;
    uint32_t                      fwu_status;
    uint32_t                      wireless_base;
    union {
        uint32_t                      ecc256_key;
        uint32_t                      rsa2048_key;
    };
    uint32_t                      options;
    uint32_t                      hseclk;
    uint16_t                      lseclk;
    uint8_t                       pin_led;
    uint8_t                       pin_boot;
    uint16_t                      pin_cs;
    uint16_t                      pin_clk;
    uint16_t                      pin_mosi; /* io0 */
    uint16_t                      pin_miso; /* io1 */
    uint16_t                      pin_wp;   /* io2 */
    uint16_t                      pin_hold; /* io3 */
    uint16_t                      pin_vbus;
    uint16_t                      pin_rfu;
} stm32wb_boot_info_t;

typedef struct _stm32wb_system_version_t {
    uint8_t                       major;
    uint8_t                       minor;
    uint16_t                      revision;
} stm32wb_system_version_t;

typedef struct _stm32wb_system_info_t {
    stm32wb_system_version_t      version;
    uint32_t                      options;
    uint32_t                      hseclk;
    uint16_t                      lseclk;
    uint8_t                       pin_led;
    uint8_t                       pin_boot;
    uint16_t                      pin_cs;
    uint16_t                      pin_clk;
    uint16_t                      pin_mosi; /* io0 */
    uint16_t                      pin_miso; /* io1 */
    uint16_t                      pin_wp;   /* io2 */
    uint16_t                      pin_hold; /* io3 */
    uint16_t                      pin_vbus;
    uint16_t                      pin_rfu;
} stm32wb_system_info_t;

#define STM32WB_SYSTEM_OPTION_LSE_BYPASS             0x00000001
#define STM32WB_SYSTEM_OPTION_LSE_MODE_MASK          0x00000006
#define STM32WB_SYSTEM_OPTION_LSE_MODE_SHIFT         1
#define STM32WB_SYSTEM_OPTION_LSE_MODE_0             0x00000000
#define STM32WB_SYSTEM_OPTION_LSE_MODE_1             0x00000002
#define STM32WB_SYSTEM_OPTION_LSE_MODE_2             0x00000004
#define STM32WB_SYSTEM_OPTION_LSE_MODE_3             0x00000006
#define STM32WB_SYSTEM_OPTION_HSE_BYPASS             0x00000008
#define STM32WB_SYSTEM_OPTION_VBAT_CHARGING          0x00000010
#define STM32WB_SYSTEM_OPTION_USART1_SYSCLK          0x00000020
#define STM32WB_SYSTEM_OPTION_SMPS_INDUCTOR_MASK     0x000000c0
#define STM32WB_SYSTEM_OPTION_SMPS_INDUCTOR_SHIFT    6
#define STM32WB_SYSTEM_OPTION_SMPS_INDUCTOR_10uH     0x00000040
#define STM32WB_SYSTEM_OPTION_SMPS_INDUCTOR_2_2uH    0x00000080
#define STM32WB_SYSTEM_OPTION_SMPS_CURRENT_MASK      0x00000f00
#define STM32WB_SYSTEM_OPTION_SMPS_CURRENT_SHIFT     8
#define STM32WB_SYSTEM_OPTION_SMPS_CURRENT_80mA      0x00000000
#define STM32WB_SYSTEM_OPTION_SMPS_CURRENT_100mA     0x00000100
#define STM32WB_SYSTEM_OPTION_SMPS_CURRENT_120mA     0x00000200
#define STM32WB_SYSTEM_OPTION_SMPS_CURRENT_140mA     0x00000300
#define STM32WB_SYSTEM_OPTION_SMPS_CURRENT_160mA     0x00000400
#define STM32WB_SYSTEM_OPTION_SMPS_CURRENT_180mA     0x00000500
#define STM32WB_SYSTEM_OPTION_SMPS_CURRENT_200mA     0x00000600
#define STM32WB_SYSTEM_OPTION_SMPS_CURRENT_220mA     0x00000700
#define STM32WB_SYSTEM_OPTION_USB                    0x00010000
#define STM32WB_SYSTEM_OPTION_SFLASH                 0x00020000

typedef struct _stm32wb_application_vectors_t {
    uint32_t                      reserved_0[8];
    uint32_t                      magic;             // TAG, OPTION, IDCODE
    uint32_t                      base;              // 0x08004000
    uint32_t                      size;              // application size (signature not included)
    uint32_t                      reserved_1[2];
    uint32_t                      offset;            // offset to system_info
    uint32_t                      reserved_2[2];
} stm32wb_application_vectors_t;

#define STM32WB_APPLICATION_TAG_MASK                 0xffff0000
#define STM32WB_APPLICATION_TAG_SHIFT                16
#define STM32WB_APPLICATION_TAG_APPLICATION          0x54560000 // 'V', 'T'
#define STM32WB_APPLICATION_TYPE_MASK                0x00008000
#define STM32WB_APPLICATION_TYPE_SHIFT               15
#define STM32WB_APPLICATION_TYPE_APPLICATION         0x00008000
#define STM32WB_APPLICATION_OPTION_MASK              0x00007000
#define STM32WB_APPLICATION_OPTION_SHIFT             12
#define STM32WB_APPLICATION_OPTION_PROTECTED         0x00004000
#define STM32WB_APPLICATION_OPTION_SIGNATURE_MASK    0x00003000
#define STM32WB_APPLICATION_OPTION_SIGNATURE_NONE    0x00000000
#define STM32WB_APPLICATION_OPTION_SIGNATURE_ECC256  0x00001000
#define STM32WB_APPLICATION_OPTION_SIGNATURE_RSA2048 0x00002000
#define STM32WB_APPLICATION_IDCODE_MASK              0x00000fff
#define STM32WB_APPLICATION_IDCODE_SHIFT             0
#define STM32WB_APPLICATION_IDCODE_STM32WB55         0x00000495

#define STM32WB_APPLICATION_BASE                     0x08004000

#define STM32WB_APPLICATION_SIGNATURE_SIZE_NONE      (0)
#define STM32WB_APPLICATION_SIGNATURE_SIZE_ECC256    (64)
#define STM32WB_APPLICATION_SIGNATURE_SIZE_RSA2048   (256)

typedef struct _stm32wb_application_version_t {
    uint8_t                       major;
    uint8_t                       minor;
    uint16_t                      revision;
} stm32wb_application_version_t;
  
typedef struct _stm32wb_application_info_t {
    uint8_t                       uuid[16];
    stm32wb_application_version_t version;
    uint32_t                      sequence;
    uint32_t                      epoch;
    uint32_t                      crc32;
} stm32wb_application_info_t;

typedef struct _stm32wb_fwu_prefix_t {
    uint32_t                      magic;             // TAG, OPTION, IDCODE
    uint32_t                      base;              // 0x08004000
    uint32_t                      size;              // application size (minus signature size)
    uint32_t                      length;            // image length
    uint32_t                      nonce[3];          // image nonce
    uint32_t                      crc32;             // image crc32
} stm32wb_fwu_prefix_t;

#define STM32WB_FWU_TAG_MASK                         0xffff0000
#define STM32WB_FWU_TAG_SHIFT                        16
#define STM32WB_FWU_TAG_FWU                          0x55530000 // 'S', 'U'
#define STM32WB_FWU_TYPE_MASK                        0x00008000
#define STM32WB_FWU_TYPE_SHIFT                       15
#define STM32WB_FWU_TYPE_UNCOMPRESSED                0x00000000
#define STM32WB_FWU_TYPE_COMPRESSED                  0x00008000
#define STM32WB_FWU_OPTION_MASK                      0x00007000
#define STM32WB_FWU_OPTION_SHIFT                     12
#define STM32WB_FWU_OPTION_PROTECTED                 0x00004000
#define STM32WB_FWU_OPTION_SIGNATURE_MASK            0x00003000
#define STM32WB_FWU_OPTION_SIGNATURE_NONE            0x00000000
#define STM32WB_FWU_OPTION_SIGNATURE_ECC256          0x00001000
#define STM32WB_FWU_OPTION_SIGNATURE_RSA2048         0x00002000
#define STM32WB_FWU_IDCODE_MASK                      0x00000fff
#define STM32WB_FWU_IDCODE_SHIFT                     0
#define STM32WB_FWU_IDCODE_STM32WB55                 0x00000495

#define STM32WB_FWU_PREFIX_SIZE                      32

static struct option opts[] = {
    { "protected",      0, 0, 'l' },
    { "uuid",           1, 0, 'u' },
    { "vid",            1, 0, 'v' },
    { "pid",            1, 0, 'p' },
    { "reference",      1, 0, 'r' },
    { "encryption_key", 1, 0, 'e' },
    { "signature_key",  1, 0, 's' },
    { "bin",            1, 0, 'b' },
    { "dfu",            1, 0, 'd' },
    { "ota",            1, 0, 'o' },
    { "hex",            1, 0, 'x' },
    { NULL,             0, 0, 0   }
};

/**********************************************************************************************************************************/

static const unsigned long crc32_table[] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
    0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
    0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
    0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
    0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
    0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
    0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
    0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
    0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
    0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
    0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
    0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
    0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
    0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
    0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
    0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
    0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
    0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
    0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
    0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
    0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
    0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
    0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
    0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
    0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
    0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
    0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
    0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
    0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
    0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
    0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d};

static uint32_t crc32_byte(uint32_t accum, uint8_t delta)
{
    return crc32_table[(accum ^ delta) & 0xff] ^ (accum >> 8);
}

/**********************************************************************************************************************************/

static uint32_t __attribute__((noinline)) stm32wb_boot_crc32(const uint8_t *data, uint32_t size, uint32_t crc32)
{
    const uint8_t *in, *in_e;
    uint8_t c;

    static const uint32_t stm32wb_boot_crc32_lut[16] = {
        0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
        0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
        0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
        0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c,
    };

    if (size)
    {
        in = (const uint8_t*)data;
        in_e = (const uint8_t*)((const uint8_t*)data + size);

        do
        {
            c = *in++;

            crc32 = (crc32 >> 4) ^ stm32wb_boot_crc32_lut[(crc32 ^ c       ) & 15];
            crc32 = (crc32 >> 4) ^ stm32wb_boot_crc32_lut[(crc32 ^ (c >> 4)) & 15];
        }
        while (in != in_e);
    }

    return crc32;
}

/**********************************************************************************************************************************/

#define STM32WB_BOOT_AES128_BLOCK_SIZE 16

typedef struct _stm32wb_boot_aes128_context_t {
    AES_KEY                    K;
} stm32wb_boot_aes128_context_t;

static void stm32wb_boot_aes128_set_key(stm32wb_boot_aes128_context_t *aes128_ctx, const uint32_t *key)
{
    AES_set_encrypt_key((const uint8_t*)key, 128, &aes128_ctx->K);
}

static void stm32wb_boot_aes128_ecb_encrypt(stm32wb_boot_aes128_context_t *aes128_ctx, const uint32_t *in, uint32_t *out, uint32_t count)
{
    for (; count; count -= 16, in += 4, out += 4)
    {
        AES_encrypt((const uint8_t*)in, (uint8_t*)out, &aes128_ctx->K);
    }
}

static void stm32wb_boot_aes128_ctr_encrypt(stm32wb_boot_aes128_context_t *aes128_ctx, const uint32_t *iv, const uint32_t *in, uint32_t *out, uint32_t count)
{
    uint32_t aes128_iv[4], aes128_xor[4], block;

    aes128_iv[0] = iv[0];
    aes128_iv[1] = iv[1];
    aes128_iv[2] = iv[2];
    
    for (block = SWAP(iv[3]); count; count -= 16, in += 4, out += 4, block += 1)
    {
        aes128_iv[3] = SWAP(block);

        AES_encrypt((const uint8_t*)&aes128_iv[0], (uint8_t*)&aes128_xor[0], &aes128_ctx->K);

        out[0] = in[0] ^ aes128_xor[0];
        out[1] = in[1] ^ aes128_xor[1];
        out[2] = in[2] ^ aes128_xor[2];
        out[3] = in[3] ^ aes128_xor[3];
    }
}

/**********************************************************************************************************************************/

typedef struct _stm32wb_boot_nvm_control_t {
    stm32wb_boot_aes128_context_t  aes128_ctx;
} stm32wb_boot_nvm_control_t;

static stm32wb_boot_nvm_control_t stm32wb_boot_nvm_control;

static const uint8_t *stm32wb_boot_nvm_base = NULL;
static uint8_t __attribute__((aligned(16))) stm32wb_boot_nvm_data[4 * 1024 * 1024];

static void stm32wb_boot_nvm_set_key(const uint32_t *key, uint8_t *nvm_base, uint32_t nvm_size)
{
    memset(stm32wb_boot_nvm_data, 0xff, sizeof(stm32wb_boot_nvm_data));
    memcpy(stm32wb_boot_nvm_data, nvm_base, nvm_size);

    stm32wb_boot_nvm_base = nvm_base;
    
    stm32wb_boot_aes128_set_key(&stm32wb_boot_nvm_control.aes128_ctx, key);
}

static bool stm32wb_boot_nvm_erase(uint8_t *address, uint32_t size)
{
    memset(address, 0xff, size);

    return true;
}

static bool stm32wb_boot_nvm_program(uint8_t *address, const uint8_t *data, uint32_t count, bool encrypted)
{
    uint32_t aes128_iv[4], program_data[4096 / 4];

    memcpy(&stm32wb_boot_nvm_data[address - stm32wb_boot_nvm_base], data, count);
    
    if (encrypted)
    {
        aes128_iv[0] = 0;
        aes128_iv[1] = 0;
        aes128_iv[2] = 0;
        aes128_iv[3] = SWAP((address - stm32wb_boot_nvm_base) / 16);
        
        stm32wb_boot_aes128_ctr_encrypt(&stm32wb_boot_nvm_control.aes128_ctx, &aes128_iv[0], (const uint32_t*)data, (uint32_t*)&program_data[0], count);

        data = (const uint8_t*)&program_data[0];
    }
    
    memcpy(address, data, count);

    return true;
}

static void stm32wb_boot_nvm_read(const uint8_t *address, uint8_t *data, uint32_t count, bool encrypted)
{
    uint32_t aes128_iv[4];

    memcpy(data, address, count);

    if (encrypted)
    {
        aes128_iv[0] = 0;
        aes128_iv[1] = 0;
        aes128_iv[2] = 0;
        aes128_iv[3] = SWAP((address - stm32wb_boot_nvm_base) / 16);
        
        stm32wb_boot_aes128_ctr_encrypt(&stm32wb_boot_nvm_control.aes128_ctx, &aes128_iv[0], (const uint32_t*)data, (uint32_t*)data, count);

        if (memcmp(&stm32wb_boot_nvm_data[address - stm32wb_boot_nvm_base], data, count))
        {
            printf("DECRYPT MISMATCH\n");
        }
    }
}

/**********************************************************************************************************************************/

#define STM32WB_BOOT_SHA256_BLOCK_SIZE  64
#define STM32WB_BOOT_SHA256_HASH_SIZE   32    
  
typedef struct _stm32wb_boot_sha256_context_t {
    uint32_t                   hash[STM32WB_BOOT_SHA256_HASH_SIZE / 4];
    uint32_t                   length;
    uint32_t                   index;
    uint8_t                    data[STM32WB_BOOT_SHA256_BLOCK_SIZE];
} stm32wb_boot_sha256_context_t;

#define SHR(_x,_n)    (uint32_t)(((_x) >> (_n)))
#define ROTR(_x,_n)   (uint32_t)(((_x) >> (_n)) | ((_x) << (32 - (_n))))
#define ROTL(_x,_n)   (uint32_t)(((_x) << (_n)) | ((_x) >> (32 - (_n))))
#define CH(_x,_y,_z)  (uint32_t)(((_x) & (_y)) ^ ( (~(_x)) & (_z)))
#define MAJ(_x,_y,_z) (uint32_t)(((_x) & (_y)) ^ ((_x) & (_z)) ^ ((_y) & (_z)))

#define BSIG0(_x)     (uint32_t)(ROTR((_x), 2) ^ ROTR((_x),13) ^ ROTR((_x),22))
#define BSIG1(_x)     (uint32_t)(ROTR((_x), 6) ^ ROTR((_x),11) ^ ROTR((_x),25))
#define SSIG0(_x)     (uint32_t)(ROTR((_x), 7) ^ ROTR((_x),18) ^  SHR((_x), 3))
#define SSIG1(_x)     (uint32_t)(ROTR((_x),17) ^ ROTR((_x),19) ^  SHR((_x),10))

static uint32_t stm32wb_boot_sha256_const_K[64] =
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

static const uint32_t stm32wb_boot_sha256_const_H[8] =
{
    0x5be0cd19, 0x1f83d9ab, 0x9b05688c, 0x510e527f, 0xa54ff53a, 0x3c6ef372, 0xbb67ae85, 0x6a09e667,
};

static void __attribute__((noinline, optimize("O3"))) stm32wb_boot_sha256_process(uint32_t *H, const uint32_t *Mp)
{
    uint32_t T1, T2, W[72];
    uint32_t *Wp, *Wp_e;
    volatile uint32_t *Hp;
    const uint32_t *Kp;

    for (Wp = &W[0], Wp_e = Wp + 8, Hp = &H[0]; Wp < Wp_e; Wp++, Hp++)
    {
        Wp[0] = Hp[0];
    }

    for (Wp_e = Wp + 16; Wp < Wp_e; Wp++, Mp++)
    {
        Wp[0] = SWAP(Mp[0]);
    }
    
    for (Wp = &W[8], Wp_e = Wp + 48; Wp < Wp_e; Wp++)
    {
        Wp[16] = SSIG1(Wp[14]) + Wp[9] + SSIG0(Wp[1]) + Wp[0];
    }
    
    for (Wp = &W[0], Wp_e = Wp + 64, Kp = &stm32wb_boot_sha256_const_K[0]; Wp < Wp_e; Wp++, Kp++)
    {
        T1 = Wp[0] + BSIG1(Wp[3]) + CH(Wp[3],Wp[2],Wp[1]) + Kp[0] + Wp[8];
        T2 = BSIG0(Wp[7]) + MAJ(Wp[7],Wp[6],Wp[5]);
        
        Wp[4] = Wp[4] + T1;
        Wp[8] = T1 + T2;
    }

    for (Wp_e = Wp + 8, Hp = &H[0]; Wp < Wp_e; Wp++, Hp++)
    {
        Hp[0] += Wp[0];
    }
}

static void __attribute__((noinline)) stm32wb_boot_sha256_init(stm32wb_boot_sha256_context_t *sha256_ctx)
{
    sha256_ctx->length = 0;
    sha256_ctx->index = 0;

    memcpy(&sha256_ctx->hash[0], &stm32wb_boot_sha256_const_H[0], 32);
}

static void __attribute__((noinline)) stm32wb_boot_sha256_update(stm32wb_boot_sha256_context_t *sha256_ctx, const uint8_t *data, size_t size)
{
    uint32_t index;
  
    sha256_ctx->length += size;

    index = sha256_ctx->index;
    
    while (size)
    {
        if ((size >= 64) && (index == 0) && !((uint64_t)data & 3))
        {
            do
            {
                stm32wb_boot_sha256_process(&sha256_ctx->hash[0], (const uint32_t*)data);
                
                data += 64;
                size -= 64;
            }
            while (size >= 64);
        }
        else
        {
            sha256_ctx->data[index++] = *data++;
            
            size--;
            
            if (index == 64)
            {
                stm32wb_boot_sha256_process(&sha256_ctx->hash[0], (const uint32_t*)&sha256_ctx->data[0]);
                
                index = 0;
            }
        }
    }
    
    sha256_ctx->index = index;
}

static void __attribute__((noinline)) stm32wb_boot_sha256_final(stm32wb_boot_sha256_context_t *sha256_ctx, uint32_t *hash)
{
    uint32_t *Hp, *Hp_e;
    
    sha256_ctx->data[sha256_ctx->index++] = 0x80;

    if (sha256_ctx->index > (64-8))
    {
        memset(&sha256_ctx->data[sha256_ctx->index], 0x00, 64 - sha256_ctx->index);
        
        stm32wb_boot_sha256_process(&sha256_ctx->hash[0], (const uint32_t*)&sha256_ctx->data[0]);

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

    stm32wb_boot_sha256_process(&sha256_ctx->hash[0], (const uint32_t*)&sha256_ctx->data[0]);

    for (Hp = &sha256_ctx->hash[0], Hp_e = Hp + 8, hash = hash + 7; Hp < Hp_e; Hp++, hash--)
    {
        hash[0] = SWAP(Hp[0]);
    }
}

/**********************************************************************************************************************************/

static void stm32wb_boot_rsa2048_convert_key(RSA* key, stm32wb_boot_rsa2048_key_t *out)
{
    int i, k, w, e;
    const BIGNUM *key_n, *key_e;
    BIGNUM *N, *B, *R, *Z, *R2, *U, *MSW, *T;
    BN_CTX *bn_ctx = BN_CTX_new();
      
    if (RSA_size(key) != 256)
    {
        fprintf(stderr, "RSA2048 KEY SIZE 256 required\n");

        exit(-1);
    }

    /* Initialize BIGNUMs */
    RSA_get0_key(key, &key_n, &key_e, NULL);

    e = BN_get_word(key_e);

    if (e != 65537)
    {
        fprintf(stderr, "RSA2048 KEY EXPONENT 65537 required\n");

        exit(-1);
    }

    N = BN_dup(key_n);

    B = BN_new();
    R = BN_new();
    Z = BN_new();
    R2 = BN_new();
    U = BN_new();

    MSW = BN_new();
    T = BN_new();

    BN_set_word(B, 1L);
    BN_lshift(B, B, 32);

    k = BN_num_bits(N);
    w = (k + 31) / 32;

    /* Calculate Z for STM32 PKA */
    BN_set_word(R, 1);
    BN_lshift(R, R, k);

    BN_sub(Z, R, N);

    for (i = 0; i < (w + 2); i++)
    {
        BN_lshift(Z, Z, 32);
        BN_rshift(T, Z, k);
        BN_mod(MSW, T, B, bn_ctx);

        while (!BN_is_zero(MSW))
        {
            BN_mul(T, MSW, N, bn_ctx);
            BN_sub(Z, Z, T);

            BN_rshift(T, Z, k);
            BN_mod(MSW, T, B, bn_ctx);
        }
    }
    
    /* Calculate R^2 mod N */
    BN_set_word(R2, 1);
    BN_lshift(R2, R2, (2 * k));
    BN_mod(R2, R2, N, bn_ctx);

    /* Calculate -1 / N mod 2^32 */
    BN_mod_inverse(U, N, B, bn_ctx);
    BN_sub(U, B, U);
    
    for (i = 0; i < w; i++)
    {
        BN_mod(T, N, B, bn_ctx);
        out->n[i] = BN_get_word(T);
        BN_rshift(N, N, 32);
    }

    for (i = 0; i < w; i++)
    {
        BN_mod(T, R2, B, bn_ctx);
        out->r2[i] = BN_get_word(T);
        BN_rshift(R2, R2, 32);
    }

    out->u = BN_get_word(U);
    
    BN_free(N);
    BN_free(B);
    BN_free(R);
    BN_free(Z);
    BN_free(R2);
    BN_free(U);
    BN_free(MSW);
    BN_free(T);

    BN_CTX_free(bn_ctx);
}

#define STM32WB_BOOT_RSA2048_NUM_BYTES  256
#define STM32WB_BOOT_RSA2048_NUM_WORDS  64

static inline uint64_t stm32wb_boot_rsa2048_mula32(uint32_t a, uint32_t b, uint32_t c)
{
    uint64_t ret = a;
    ret *= b;
    ret += c;
    return ret;
}

static inline uint64_t stm32wb_boot_rsa2048_mulaa32(uint32_t a, uint32_t b, uint32_t c, uint32_t d)
{
    uint64_t ret = a;
    ret *= b;
    ret += c;
    ret += d;
    return ret;
}

/**
 * a[] -= mod
 */
static void stm32wb_boot_rsa2048_mod_sub(const stm32wb_boot_rsa2048_key_t *key, uint32_t *a)
{
    const uint32_t *nn, *nn_e;
    uint32_t *aa;
    int64_t A;

    aa = a;
    nn = key->n;
    nn_e = key->n + STM32WB_BOOT_RSA2048_NUM_WORDS;

    A = 0;
    
    do
    {
        A += (uint64_t)aa[0] - (uint64_t)nn[0];
        aa[0] = (uint32_t)A;
        A >>= 32;

        nn++;
        aa++;
    }
    while (nn != nn_e);
}

/**
 * Return sign of a[] - mod
 */
static int stm32wb_boot_rsa2048_mod_cmp(const stm32wb_boot_rsa2048_key_t *key, const uint32_t *a)
{
    const uint32_t *aa, *nn, *nn_e;

    aa = a + STM32WB_BOOT_RSA2048_NUM_WORDS - 1;
    nn = key->n + STM32WB_BOOT_RSA2048_NUM_WORDS - 1;
    nn_e = key->n -1;
    
    do
    {
        if (aa[0] < nn[0])
        {
            return -1;
        }

        if (aa[0] > nn[0])
        {
            return 1;
        }

        aa--;
        nn--;
    }
    while (nn != nn_e);

    return 0;
}

/**
 * Montgomery c[] = a[] * b[] / R % mod
 */
static void stm32wb_boot_rsa2048_mod_mul(const stm32wb_boot_rsa2048_key_t *key, uint32_t *c, const uint32_t *a, const uint32_t *b)
{
    const uint32_t *aa, *aa_e, *bb, *nn, *nn_e;
    uint32_t *cc;
    uint64_t A, B;
    uint32_t d0;
    
    memset((uint8_t*)c, 0, STM32WB_BOOT_RSA2048_NUM_WORDS * 4);

    cc = c;
    
    aa = a;
    aa_e = aa + STM32WB_BOOT_RSA2048_NUM_WORDS;
    nn_e = key->n + STM32WB_BOOT_RSA2048_NUM_WORDS;
    
    do
    {
        nn = key->n;
        bb = b;
        cc = c;
        
        A = stm32wb_boot_rsa2048_mula32(aa[0], bb[0], cc[0]);
        d0 = (uint32_t)A * key->u;
        B = stm32wb_boot_rsa2048_mula32(d0, nn[0], A);
        
        nn++;
        bb++;
        
        do
        {
            A = stm32wb_boot_rsa2048_mulaa32(aa[0], bb[0], cc[1], A >> 32);
            B = stm32wb_boot_rsa2048_mulaa32(d0, nn[0], A, B >> 32);
            cc[0] = (uint32_t)B;
            
            nn++;
            bb++;
            cc++;
        }
        while (nn != nn_e);
        
        A = (A >> 32) + (B >> 32);
        
        cc[0] = (uint32_t)A;
        
        if (A >> 32)
        {
            stm32wb_boot_rsa2048_mod_sub(key, c);
        }

        aa++;
    }
    while (aa != aa_e);
}

static void stm32wb_boot_rsa2048_mod_exp(const stm32wb_boot_rsa2048_key_t *key, uint32_t *b, const uint32_t *in)
{
    const uint32_t *in_e;
    uint32_t *a, *aa, *aaa, *b_e;
    uint32_t *a_r = b + STM32WB_BOOT_RSA2048_NUM_WORDS;
    uint32_t *aa_r = a_r + STM32WB_BOOT_RSA2048_NUM_WORDS;
    int i;

    a = b;
    aa = a + STM32WB_BOOT_RSA2048_NUM_WORDS -1;
    aaa = aa_r;
        
    in_e = in + STM32WB_BOOT_RSA2048_NUM_WORDS;

    do
    {
        aa[0] = SWAP(in[0]);

        aa--;
        in++;
    }
    while (in != in_e);
        
    /* Exponent 65537 */

    stm32wb_boot_rsa2048_mod_mul(key, a_r, a, key->r2); /* a_r = a * R2 / R mod M */

    for (i = 0; i < 16; i += 2)
    {
        stm32wb_boot_rsa2048_mod_mul(key, aa_r, a_r, a_r); /* aa_r = a_r * a_r / R mod M */
        stm32wb_boot_rsa2048_mod_mul(key, a_r, aa_r, aa_r); /* a_r = aa_r * aa_r / R mod M */
    }

    stm32wb_boot_rsa2048_mod_mul(key, aaa, a_r, a); /* aaa = a_r * a / R mod M */

    if (stm32wb_boot_rsa2048_mod_cmp(key, aaa) >= 0)
    {
        stm32wb_boot_rsa2048_mod_sub(key, aaa);
    }

    aa = aaa + STM32WB_BOOT_RSA2048_NUM_WORDS -1;
    b_e = b + STM32WB_BOOT_RSA2048_NUM_WORDS;

    do
    {
        b[0] = SWAP(aa[0]);

        aa--;
        b++;
    }
    while (b != b_e);
}

/*
 * PKCS#1 padding (from the RSA PKCS#1 v2.1 standard)
 *
 * The DER-encoded padding is defined as follows :
 * 0x00 || 0x01 || PS || 0x00 || T
 *
 * T: DER Encoded DigestInfo value which depends on the hash function used,
 * for SHA-256:
 * (0x)30 31 30 0d 06 09 60 86 48 01 65 03 04 02 01 05 00 04 20 || H.
 *
 * Length(T) = 51 octets for SHA-256
 *
 * PS: octet string consisting of {Length(RSA Key) - Length(T) - 3} 0xFF
 */

#define STM32WB_BOOT_RSA2048_SIGNATURE_SIZE 256
#define STM32WB_BOOT_RSA2048_SHA256_DIGEST_INFO_SIZE 51

static const uint8_t stm32wb_boot_rsa2048_digest_info[STM32WB_BOOT_RSA2048_SHA256_DIGEST_INFO_SIZE - STM32WB_BOOT_SHA256_HASH_SIZE + 1] = {
    0x00, 0x30, 0x31, 0x30, 0x0d, 0x06, 0x09, 0x60, 0x86, 0x48, 0x01, 0x65, 0x03, 0x04, 0x02, 0x01, 0x05, 0x00, 0x04, 0x20
};

static bool stm32wb_boot_rsa2048_verify(const stm32wb_boot_rsa2048_key_t *key, const uint32_t *signature, const uint32_t *digest)
{
    const uint8_t *out, *out_e;
    uint32_t temp[1024 / 4];

    stm32wb_boot_rsa2048_mod_exp(key, (uint32_t*)&temp[0], signature);

    out = (const uint8_t*)&temp[0];
    
    if ((out[0] != 0x00) || (out[1] != 0x01))
    {
        return false;
    }

    out += 2;
    out_e = out + (STM32WB_BOOT_RSA2048_SIGNATURE_SIZE - STM32WB_BOOT_SHA256_HASH_SIZE - sizeof(stm32wb_boot_rsa2048_digest_info) - 2);

    do
    {
        if (out[0] != 0xff)
        {
            return false;
        }

        out++;
    }
    while (out != out_e);

    if (memcmp(out, stm32wb_boot_rsa2048_digest_info, sizeof(stm32wb_boot_rsa2048_digest_info)))
    {
        return false;
    }

    out += sizeof(stm32wb_boot_rsa2048_digest_info);

    if (memcmp(out, (const uint8_t*)digest, STM32WB_BOOT_SHA256_HASH_SIZE))
    {
        return false;
    }

    return true;
}

/**********************************************************************************************************************************/

typedef struct _lzfwu_context_t {
  uint32_t *out_data;
  uint32_t out_bits;
  uint32_t out_count;
} lzfwu_context_t;

void lzfwu_compress_setup(lzfwu_context_t *lzfwu_context, uint8_t *out_data)
{
    lzfwu_context->out_data = (uint32_t*)out_data;
    lzfwu_context->out_bits = 0;
    lzfwu_context->out_count = 32;
}

void lzfwu_compress_write(lzfwu_context_t *lzfwu_context, uint32_t bits, uint32_t count)
{
    uint32_t size;
    
    size = count;

    if (size > lzfwu_context->out_count)
    {
        size = lzfwu_context->out_count;
    }

    lzfwu_context->out_bits = (lzfwu_context->out_bits << size) | (bits >> (count - size));
    lzfwu_context->out_count -= size;

    if (lzfwu_context->out_count == 0)
    {
        *lzfwu_context->out_data++ = lzfwu_context->out_bits;

        lzfwu_context->out_bits = 0;
        lzfwu_context->out_count = 32;
    }

    if (size != count)
    {
        lzfwu_context->out_bits = bits & ((1 << (count - size)) -1);
        lzfwu_context->out_count -= (count - size);
    }
}

void lzfwu_compress_finish(lzfwu_context_t *lzfwu_context, uint8_t *out_data)
{
    if (lzfwu_context->out_count != 32)
    {
        *lzfwu_context->out_data++ = (lzfwu_context->out_bits << lzfwu_context->out_count) | (0xffffffff >> (32 - lzfwu_context->out_count));
    }

    while (((const uint8_t*)lzfwu_context->out_data - (const uint8_t*)out_data) & 15)
    {
        *lzfwu_context->out_data++ = 0xffffffff;
    }
}

uint32_t lzfwu_compress_cost(uint32_t length, uint32_t distance)
{
    uint32_t cost;

    if      (length <= 4)       { cost = 3;                 }
    else if (length <= 6)       { cost = (4 + 1);           }
    else if (length <= 14)      { cost = (5 + 3);           }
    else if (length <= 126)     { cost = (5 + 7);           }
    else if (length <= 2048)    { cost = (8 + 11);          }
    else                        { cost = 0x00ffffff;        }

    if (distance)
    {
        if (length == 2)
        {
            if      (distance <= 256)     { cost += (1 + 8);    }
            else if (distance <= 2304)    { cost += (1 + 11);   }
            else                          { cost += 0x00ffffff; }
        }
        else
        {
            if      (distance <= 256)     { cost += (1 + 8);    }
            else if (distance <= 2304)    { cost += (2 + 11);   }
            else if (distance <= 8192)    { cost += (2 + 13);   }
            else                          { cost += 0x00ffffff; }
        }
    }
    else
    {
        cost += (4 + 22);
    }
    
    return cost;
}


#define LZFWU_COPY_LENGTH_MIN    2
#define LZFWU_COPY_LENGTH_MAX    2048
#define LZFWU_COPY_DISTANCE_MIN  1
#define LZFWU_COPY_DISTANCE_MAX  8192

#define LZFWU_HASH_TABLE_ENTRIES 524287

typedef struct _lzfwu_node_t {
    uint32_t prefix;
    uint32_t index;
    struct _lzfwu_node_t *next;
} lzfwu_node_t;

#define LZFWU_LINK_SENTINEL ~0u

typedef struct _lzfwu_entry_t {
    uint32_t cost;
    uint32_t length;
    uint32_t distance;
    uint32_t offset;
    uint32_t link;      /* linked list for match finding */
} lzfwu_entry_t;


uint32_t lzfwu_compress(const uint8_t *source_data, uint32_t source_size, const uint8_t *target_data, uint32_t target_size, uint32_t target_offset, uint8_t *out_data)
{
    lzfwu_context_t lzfwu_context;
    lzfwu_entry_t *table = NULL;
    lzfwu_node_t **hash = NULL;
    lzfwu_node_t *nodes = NULL;
    lzfwu_node_t *node;
    uint32_t map[65536];
    uint32_t length, distance, offset, offset_e, limit, size, slot, link, cost, prefix;
    int32_t index;
    
    if (!table)
    {
        table = (lzfwu_entry_t*)malloc((target_size + 1) * sizeof(lzfwu_entry_t));
    }
    
    for (slot = 0; slot < 65536; slot++)
    {
        map[slot] = LZFWU_LINK_SENTINEL;
    }
    
    for (index = 0; index < (target_size + 1); index++)
    {
        table[index].cost = 0xffffffff;
        table[index].length = 0;
        table[index].distance = 0;
        table[index].offset = 0;
        table[index].link = LZFWU_LINK_SENTINEL;
    }

    if (source_data)
    {
        if (!hash)
        {
            hash = (lzfwu_node_t**)malloc(LZFWU_HASH_TABLE_ENTRIES * sizeof(lzfwu_node_t*));
        }
        
        if (!nodes)
        {
            nodes = (lzfwu_node_t*)malloc(source_size * sizeof(lzfwu_node_t));
        }

        memset(hash, 0, (LZFWU_HASH_TABLE_ENTRIES * sizeof(lzfwu_node_t*)));
        memset(nodes, 0, (source_size * sizeof(lzfwu_node_t)));

        for (index = (source_size - 4); index >= 0; index--)
        {
            prefix = *((const uint32_t*)&source_data[index]);
            slot = prefix % LZFWU_HASH_TABLE_ENTRIES;
            
            nodes[index].prefix = prefix;
            nodes[index].index = index;
            nodes[index].next = hash[slot];
            
            hash[slot] = &nodes[index];
        }
    }
    
    for (index = target_offset; index <= target_size; index++)
    {
        if (index <= target_offset)
        {
            table[target_offset].cost = 0;

            if (index < target_offset)
            {
                continue;
            }
        }
        else
        {
            cost = table[index -1].cost + 9;

            if (table[index].cost >= cost)
            {
                table[index].cost = cost;
                table[index].length = 1;
                table[index].distance = 0;
                table[index].offset = 0;
            }
        }
        
        if (index <= (target_size - 2))
        {
            slot = target_data[index +0] | (target_data[index +1] << 8);

            limit = 0;
            
            for (link = map[slot]; link != LZFWU_LINK_SENTINEL; link = table[link].link)
            {
                distance = index - link;

                if (distance > LZFWU_COPY_DISTANCE_MAX)
                {
                    break;
                }
                
                offset_e = LZFWU_COPY_LENGTH_MAX;

                if (offset_e > (((index & ~2047) + 2048) - index))
                {
                    offset_e = ((index & ~2047) + 2048) - index;
                }
                
                if (offset_e > (target_size - index))
                {
                    offset_e = (target_size - index);
                }
                
                if (offset_e >= 2)
                {
                    if (limit < offset_e)
                    {
                        for (offset = 2; offset < offset_e; offset++)
                        {
                            if (target_data[index - distance + offset] != target_data[index + offset])
                            {
                                break;
                            }
                        }
                        
                        if (limit < offset)
                        {
                            limit = offset;
                            
                            for (length = 2; length <= offset; length++)
                            {
                                cost = table[index].cost + lzfwu_compress_cost(length, distance);
                                
                                if (table[index + length].cost > cost)
                                {
                                    table[index + length].cost = cost;
                                    table[index + length].length = length;
                                    table[index + length].distance = distance;
                                    table[index + length].offset = 0;
                                }
                            }
                        }
                    }
                }
            }
            
            table[index].link = map[slot];

            map[slot] = index;
        }

        if (source_data)
        {
            if (index <= (target_size - 4))
            {
                limit = 0;

                prefix = *((const uint32_t*)&target_data[index]);
                slot = prefix % LZFWU_HASH_TABLE_ENTRIES;

                for (node = hash[slot]; node; node = node->next)
                {
                    if (node->prefix == prefix)
                    {
                        offset_e = LZFWU_COPY_LENGTH_MAX;

                        if (offset_e > (((index & ~2047) + 2048) - index))
                        {
                            offset_e = ((index & ~2047) + 2048) - index;
                        }
                
                        if (offset_e > (target_size - index))
                        {
                            offset_e = (target_size - index);
                        }

                        if (offset_e > (((node->index & ~2047) + 2048) - node->index))
                        {
                            offset_e = ((node->index & ~2047) + 2048) - node->index;
                        }

                        if (offset_e > (source_size - node->index))
                        {
                            offset_e = (source_size - node->index);
                        }

                        if (offset_e >= 4)
                        {
                            if (limit < offset_e)
                            {
                                for (offset = 4; offset < offset_e; offset++)
                                {
                                    if (source_data[node->index + offset] != target_data[index + offset])
                                    {
                                        break;
                                    }
                                }
                                
                                if (limit < offset)
                                {
                                    limit = offset;
                                    
                                    for (length = 4; length <= offset; length++)
                                    {
                                        cost = table[index].cost + lzfwu_compress_cost(length, 0);
                                        
                                        if (table[index + length].cost > cost)
                                        {
                                            table[index + length].cost = cost;
                                            table[index + length].length = length;
                                            table[index + length].distance = 0;
                                            table[index + length].offset = node->index;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /* Walk the way backwards from the end to the start, while leaving a trace in link.
     */
    
    for (index = target_size; index != target_offset; index -= table[index].length)
    {
        table[index - table[index].length].link = index;
    }

    /* Emit the codes while walking forward along the path.
     */

    lzfwu_compress_setup(&lzfwu_context, out_data);

    for (index = target_offset; index != target_size; index = table[index].link)
    {
        length = table[table[index].link].length;
        distance = table[table[index].link].distance;
        offset = table[table[index].link].offset;
        
        // fprintf(stderr, "%08x: ", index);
        
        if (length == 1)
        {

            /*
             * 0                                   [1]      1 byte literal
             */
            lzfwu_compress_write(&lzfwu_context, (0 << 8) + target_data[index], 9);
            
            // fprintf(stderr, "INSERT(%02x)\n", target_data[index]);
        }
        else
        {
            /*
             * 100                                 [3]      2 byte copy
             * 101                                 [3]      3 byte copy
             * 110                                 [3]      4 byte copy
             * 1110x                               [5]      5-6 byte copy (bias 5)
             * 11110xxx                            [8]      7-14 byte copy (bias 7)
             * 11111yyyxxxx (yyy != 111)           [12]     15-126 byte copy (bias 15)
             * 11111111xxxxxxxxxxx                 [19]     127-2048 byte copy (bias 127)
             */
            
            if      (length <= 4)   { lzfwu_compress_write(&lzfwu_context, (  4 << 0)  + (length - 2),   3);  }
            else if (length <= 6)   { lzfwu_compress_write(&lzfwu_context, ( 14 << 1)  + (length - 5),   5);  }
            else if (length <= 14)  { lzfwu_compress_write(&lzfwu_context, ( 30 << 3)  + (length - 7),   8);  }
            else if (length <= 126) { lzfwu_compress_write(&lzfwu_context, ( 31 << 7)  + (length - 15),  12); }
            else                    { lzfwu_compress_write(&lzfwu_context, (255 << 11) + (length - 127), 19); }

            if (distance)
            {
                if (length == 2)
                {
                    /* 
                     * 0xxxxxxxx                           [9]    1 - 256 byte distance (bias 1)
                     * 1xxxxxxxxxxx                        [12]   257 - 2304 byte distance (bias 257)
                     */
                    
                    if (distance <= 256) { lzfwu_compress_write(&lzfwu_context, (0 << 8)  + (distance - 1),   9);  }
                    else                 { lzfwu_compress_write(&lzfwu_context, (1 << 11) + (distance - 257), 12); }
                }
                else
                {
                    /* 
                     * 0xxxxxxxx                            [9]   1 - 256 byte distance (bias 1)
                     * 10xxxxxxxxxxx                       [13]   257 - 2304 byte distance (bias 257)
                     * 11yyxxxxxxxxxxx (yy != 11)          [15]   2305 - 8192 byte distance (bias 2305)
                     */
                    
                    if      (distance <= 256)  { lzfwu_compress_write(&lzfwu_context, ( 0 << 8)  + (distance - 1),    9);  }
                    else if (distance <= 2304) { lzfwu_compress_write(&lzfwu_context, ( 2 << 11) + (distance - 257),  13); }
                    else                       { lzfwu_compress_write(&lzfwu_context, ( 3 << 13) + (distance - 2305), 15); }
                }
                
                // fprintf(stderr, "COPY-TARGET(%d @%d)\n", length, distance);
            }
            else
            {
                /* 
                 * 1111xxxxxxxxxxxxxxxxxxxxxx          [26]   0 - 4194303 byte offset
                 */

                lzfwu_compress_write(&lzfwu_context, (15 << 22) + offset, 26);

                // fprintf(stderr, "COPY-SOURCE(%d @%d)\n", length, offset);
            }
        }
    }

    lzfwu_compress_finish(&lzfwu_context, out_data);

    // fprintf(stderr, "**************************************************************************\n");
    
    size = (uint32_t)((table[target_size].cost + 127) & ~127) / 8;

    if (((uint8_t*)lzfwu_context.out_data - (uint8_t*)out_data) != size)
    {
        fprintf(stderr, "### MISMATCH %ld %d ###\n\n", ((uint8_t*)lzfwu_context.out_data - (uint8_t*)out_data), size);
    }

    if (nodes) free(nodes);
    if (hash) free(hash);
    if (table) free(table);

    return size;
}

/**********************************************************************************************************************************/

static uint8_t __attribute__((aligned(16))) stm32wb_boot_copy_data[2048];
static uint8_t __attribute__((aligned(16))) stm32wb_boot_dict_data[8192];

typedef struct _stm32wb_boot_fwu_context_t {
    bool                          encrypted;
    const uint8_t                 *in_current;
    const uint8_t                 *in_finish;
    const uint8_t                 *in_base;
    const uint8_t                 *in_limit;
    uint8_t                       in_size;
    uint8_t                       in_index;
    uint8_t                       in_shift;
    uint32_t                      in_bits;
    uint32_t                      in_data[64];
    const uint8_t                 *out_base;
    const uint8_t                 *out_limit;
    const uint8_t                 *swap_start;
    const uint8_t                 *swap_base;
    const uint8_t                 *swap_limit;
    const uint8_t                 *copy_current;
    const uint8_t                 *copy_finish;
    uint8_t                       *copy_base;
    uint8_t                       *dict_current;
    uint8_t                       *dict_base;
    uint8_t                       *dict_limit;
    stm32wb_boot_aes128_context_t aes128_ctx;
    uint32_t                      aes128_iv[4];
} stm32wb_boot_fwu_context_t;

static void stm32wb_boot_fwu_data(stm32wb_boot_fwu_context_t *fwu_ctx, uint8_t *data, uint32_t count)
{
    const uint8_t *in_current = fwu_ctx->in_current;
    
    // address multiple of 16, count multiple of 16

    if (count > (fwu_ctx->in_limit - in_current))
    {
        count = fwu_ctx->in_limit - in_current;
    }

    if (count)
    {
        fwu_ctx->in_current += count;

        stm32wb_boot_nvm_read(in_current, data, count, false);
    }
}

static void stm32wb_boot_fwu_decrypt(stm32wb_boot_fwu_context_t *fwu_ctx, uint8_t *data, uint32_t count)
{
    const uint8_t *in_current = fwu_ctx->in_current;
    uint32_t block;

    // address multiple of 16, count multiple of 16

    if (count > (fwu_ctx->in_limit - in_current))
    {
        count = fwu_ctx->in_limit - in_current;
    }
    
    if (count)
    {
        fwu_ctx->in_current += count;

        stm32wb_boot_nvm_read(in_current, data, count, false);
    
        if (fwu_ctx->encrypted)
        {
            block = (in_current - fwu_ctx->in_base) / 16;
        
            fwu_ctx->aes128_iv[3] = SWAP(block);
        
            stm32wb_boot_aes128_ctr_encrypt(&fwu_ctx->aes128_ctx, &fwu_ctx->aes128_iv[0], (const uint32_t*)data, (uint32_t*)data, count);
        }
    }
}

static void stm32wb_boot_fwu_init(stm32wb_boot_fwu_context_t *fwu_ctx, const uint8_t *in_base, const uint8_t *in_finish, const uint8_t *in_limit, const uint8_t *out_base, const uint8_t *out_limit, const uint8_t *swap_start, const uint8_t *swap_base, const uint8_t *swap_limit)
{
    fwu_ctx->in_current = in_base;
    fwu_ctx->in_finish = in_finish;
    fwu_ctx->in_base = in_base;
    fwu_ctx->in_limit = in_limit;

    fwu_ctx->out_base = out_base;
    fwu_ctx->out_limit = out_limit;
    
    fwu_ctx->swap_start = swap_start;
    fwu_ctx->swap_base  = swap_base;
    fwu_ctx->swap_limit = swap_limit;

    fwu_ctx->copy_base = &stm32wb_boot_copy_data[0];

    fwu_ctx->dict_base = &stm32wb_boot_dict_data[0];
    fwu_ctx->dict_limit = &stm32wb_boot_dict_data[8192];
}

static void stm32wb_boot_fwu_save(stm32wb_boot_fwu_context_t *fwu_ctx, const uint8_t ** p_in_current_return, uint8_t * p_in_size_return, uint8_t * p_in_index_return, uint8_t * p_in_shift_return)
{
    *p_in_current_return = fwu_ctx->in_current;
    *p_in_size_return = fwu_ctx->in_size;
    *p_in_index_return = fwu_ctx->in_index;
    *p_in_shift_return = fwu_ctx->in_shift;
}

static void stm32wb_boot_fwu_restore(stm32wb_boot_fwu_context_t *fwu_ctx, const uint8_t *in_current, uint8_t in_size, uint8_t in_index, uint8_t in_shift, const uint8_t *out_current)
{
    uint32_t count;

    fwu_ctx->in_current = in_current;
    fwu_ctx->in_size = in_size;
    fwu_ctx->in_index = in_index;
    fwu_ctx->in_shift = in_shift;

    // starting condition is in_index == 3 and in_shift == 32.
    if (!((fwu_ctx->in_shift == 32) && (fwu_ctx->in_index == (fwu_ctx->in_size -1))))
    {
        fwu_ctx->in_current -= (fwu_ctx->in_size * 4);

        stm32wb_boot_fwu_decrypt(fwu_ctx, (uint8_t*)&fwu_ctx->in_data[0], (fwu_ctx->in_size * 4));

        if (fwu_ctx->in_shift != 32)
        {
            fwu_ctx->in_bits = fwu_ctx->in_data[fwu_ctx->in_index] << fwu_ctx->in_shift;
        }
    }

    fwu_ctx->copy_current = NULL;
    fwu_ctx->copy_finish = NULL;
    
    count = 8192;

    if (count > (out_current - fwu_ctx->out_base))
    {
        count = (out_current - fwu_ctx->out_base);
    }

    memcpy(fwu_ctx->dict_base, (const uint8_t*)(out_current - count), count);

    fwu_ctx->dict_current = fwu_ctx->dict_base + count;

    if (fwu_ctx->dict_current == fwu_ctx->dict_limit)
    {
        fwu_ctx->dict_current = fwu_ctx->dict_base;
    }
}

static uint32_t stm32wb_boot_fwu_bits(stm32wb_boot_fwu_context_t *fwu_ctx, uint32_t count)
{
    uint32_t bits, size;

    if (fwu_ctx->in_shift == 32)
    {
        fwu_ctx->in_shift = 0;
        fwu_ctx->in_index++;

        if (fwu_ctx->in_index == fwu_ctx->in_size)
        {
            fwu_ctx->in_size = 64;
            fwu_ctx->in_index = 0;
            
            if (fwu_ctx->in_size > ((fwu_ctx->in_finish - fwu_ctx->in_current) / 4))
            {
                fwu_ctx->in_size = ((fwu_ctx->in_finish - fwu_ctx->in_current) / 4);
            }

            if (fwu_ctx->in_size)
            {
                stm32wb_boot_fwu_decrypt(fwu_ctx, (uint8_t*)&fwu_ctx->in_data[0], (fwu_ctx->in_size * 4));
            }
        }
        
        fwu_ctx->in_bits = fwu_ctx->in_data[fwu_ctx->in_index];

        bits = fwu_ctx->in_bits >> (32 - count);

        fwu_ctx->in_bits <<= count;
        fwu_ctx->in_shift += count;
    }
    else
    {
        size = 32 - fwu_ctx->in_shift;

        if (size > count)
        {
            size = count;
        }
        
        bits = fwu_ctx->in_bits >> (32 - size);

        fwu_ctx->in_bits <<= size;
        fwu_ctx->in_shift += size;

        if (size != count)
        {
            fwu_ctx->in_shift = 0;
            fwu_ctx->in_index++;

            if (fwu_ctx->in_index == fwu_ctx->in_size)
            {
                fwu_ctx->in_size = 64;
                fwu_ctx->in_index = 0;
                
                if (fwu_ctx->in_size > ((fwu_ctx->in_finish - fwu_ctx->in_current) / 4))
                {
                    fwu_ctx->in_size = ((fwu_ctx->in_finish - fwu_ctx->in_current) / 4);
                }

                if (fwu_ctx->in_size)
                {
                    stm32wb_boot_fwu_decrypt(fwu_ctx, (uint8_t*)&fwu_ctx->in_data[0], (fwu_ctx->in_size * 4));
                }
            }

            fwu_ctx->in_bits = fwu_ctx->in_data[fwu_ctx->in_index];
            
            bits = (bits << (count - size)) | (fwu_ctx->in_bits >> (32 - (count - size)));
            
            fwu_ctx->in_bits <<= (count - size);
            fwu_ctx->in_shift += (count - size);
        }
    }
    
    return bits;
}

static void stm32wb_boot_fwu_uncompress(stm32wb_boot_fwu_context_t *fwu_ctx, uint8_t *data, uint32_t count, uint32_t swap_size)
{
    uint32_t code, literal, length, distance, offset, align;
    const uint8_t *copy_current;
    uint8_t *dict_current;
    uint8_t *data_e;

    data_e = data + count;

    copy_current = fwu_ctx->copy_current;
    dict_current = fwu_ctx->dict_current;

    while (data < data_e)
    {
        if (copy_current != fwu_ctx->copy_finish)
        {
            literal = *copy_current++;

            if (copy_current == fwu_ctx->dict_limit)
            {
                copy_current = fwu_ctx->dict_base;
            }

            *data++ = literal;

            *dict_current++ = literal;

            if (dict_current == fwu_ctx->dict_limit)
            {
                dict_current = fwu_ctx->dict_base;
            }
            
            continue;
        }

        // fprintf(stderr, "%08x: ", data - fwu_ctx->out_base);
        
        code = stm32wb_boot_fwu_bits(fwu_ctx, 3);

        if (code < 4)
        {
            literal = (code << 6) | stm32wb_boot_fwu_bits(fwu_ctx, 6);

            // fprintf(stderr, "INSERT(%02x)\n", literal);
            
            *data++ = literal;

            *dict_current++ = literal;

            if (dict_current == fwu_ctx->dict_limit)
            {
                dict_current = fwu_ctx->dict_base;
            }
        }
        else
        {
            if (code < 7)
            {
                length = (code - 4) + 2;
            }
            else
            {
                code = stm32wb_boot_fwu_bits(fwu_ctx, 2);

                if (code < 2)
                {
                    length = (code + 5);
                }
                else
                {
                    if (code == 2)
                    {
                        length = stm32wb_boot_fwu_bits(fwu_ctx, 3) + 7;
                    }
                    else
                    {
                        code = stm32wb_boot_fwu_bits(fwu_ctx, 7);

                        if (code < 112)
                        {
                            length = code + 15;
                        }
                        else
                        {
                            length = (((code - 112) << 7) + stm32wb_boot_fwu_bits(fwu_ctx, 7)) + 127;
                        }
                    }
                }
            }

            code = stm32wb_boot_fwu_bits(fwu_ctx, 9);
                
            if (code < 256)
            {
                distance = code + 1;
            }
            else
            {
                if (length == 2)
                {
                    distance = (((code - 256) << 3) + stm32wb_boot_fwu_bits(fwu_ctx, 3)) + 257;
                }
                else
                {
                    if (code < 384)
                    {
                        distance = (((code - 256) << 4) + stm32wb_boot_fwu_bits(fwu_ctx, 4)) + 257;
                    }
                    else
                    {
                        code = ((code - 384) << 6) + stm32wb_boot_fwu_bits(fwu_ctx, 6);
                        
                        if (code < 6144)
                        {
                            distance = code + 2305;
                        }
                        else
                        {
                            offset = (((code - 6144) << 11) + stm32wb_boot_fwu_bits(fwu_ctx, 11));
                            
                            if ((offset + length) > (fwu_ctx->out_limit - fwu_ctx->out_base))
                            {
                                // Avoid overfetching, but also avoid in/out size mismatches ... So fetch random stuff
                                copy_current = (const uint8_t*)fwu_ctx->out_base;
                            }
                            else
                            {
                                if (offset >= swap_size)
                                {
                                    copy_current = (const uint8_t*)(fwu_ctx->out_base + offset);
                                }
                                else
                                {
                                    if (offset < (fwu_ctx->swap_limit - fwu_ctx->swap_start))
                                    {
                                        copy_current = (const uint8_t*)(fwu_ctx->swap_start + offset);
                                    }
                                    else
                                    {
                                        copy_current = (const uint8_t*)(fwu_ctx->swap_base + (offset - (fwu_ctx->swap_limit - fwu_ctx->swap_start)));
                                    }
                                }

                                align = ((uint64_t)copy_current & 15);

                                stm32wb_boot_nvm_read((copy_current - align), fwu_ctx->copy_base, ((align + length + 15) & ~15), true);

                                copy_current = fwu_ctx->copy_base + align;
                            }

                            fwu_ctx->copy_finish = copy_current + length;

                            // fprintf(stderr, "COPY-SOURCE(%d @%d)\n", length, offset);
                            
                            continue;
                        }
                    }
                }
            }

            copy_current = dict_current - distance;

            if (copy_current < fwu_ctx->dict_base)
            {
                copy_current += 8192;
            }
            
            fwu_ctx->copy_finish = copy_current + length;

            if (fwu_ctx->copy_finish >= fwu_ctx->dict_limit)
            {
                fwu_ctx->copy_finish -= 8192;
            }

            // fprintf(stderr, "COPY-TARGET(%d @%d)\n", length, distance);
        }
    }

    fwu_ctx->copy_current = copy_current;
    fwu_ctx->dict_current = dict_current;
}

/**********************************************************************************************************************************/

#define ELF_CHUNK_SIZE (1024 * 1024)

static void elf_file_read(const char *elf_file_name, uint8_t **p_elf_data, uint32_t *p_elf_size, uint8_t **p_image_data, uint32_t *p_image_size)
{
    FILE *fp;
    uint8_t *elf_data, *image_data;
    uint32_t elf_size, image_size;
    Elf32_Ehdr *elf_ehdr;
    Elf32_Phdr *elf_phdr, *elf_phdr_e;
    Elf32_Shdr *elf_shdr, *elf_shdr_e;
    Elf32_Sym *elf_sym, *elf_sym_e;
    const char *shstrtab, *strtab;
    uint32_t index;
    size_t n;

    stm32wb_boot_info_t *boot_info;
    
    uint32_t image_base  = 0xffffffff;
    uint32_t image_limit = 0x00000000;
    
    uint32_t __application_base__ = 0xffffffff;
    uint32_t __application_limit__ = 0xffffffff;
    uint32_t __fwu_base__ = 0xffffffff;
    uint32_t __fwu_limit__ = 0xffffffff;
    uint32_t __fwu_status__ = 0xffffffff;
    uint32_t __wireless_base__ = 0xffffffff;

    fp = fopen(elf_file_name, "r");

    if (!fp)
    {
        fprintf(stderr, "Cannot open \"%s\"\n", elf_file_name);

        exit(-1);
    }
    
#ifdef WIN32
    _setmode(_fileno(fp), _O_BINARY);
#endif

    elf_data = NULL;
    elf_size = 0;
    
    do
    {
        elf_data = realloc(elf_data, elf_size + ELF_CHUNK_SIZE);

        n = fread((elf_data + elf_size), 1, ELF_CHUNK_SIZE, fp);

        elf_size += n;
    }
    while (n == ELF_CHUNK_SIZE);

    fclose(fp);
    
    elf_ehdr = (Elf32_Ehdr*)elf_data;

#if 0    
    fprintf(stderr, "e_type      = %04x\n", elf_ehdr->e_type);
    fprintf(stderr, "e_machine   = %04x\n", elf_ehdr->e_machine);
    fprintf(stderr, "e_version   = %08x\n", elf_ehdr->e_version);
    fprintf(stderr, "e_entry     = %08x\n", elf_ehdr->e_entry);
    fprintf(stderr, "e_phoff     = %08x\n", elf_ehdr->e_phoff);
    fprintf(stderr, "e_shoff     = %08x\n", elf_ehdr->e_shoff);
    fprintf(stderr, "e_flags     = %08x\n", elf_ehdr->e_flags);
    fprintf(stderr, "e_ehsize    = %d\n",   elf_ehdr->e_ehsize);
    fprintf(stderr, "e_phentsize = %d\n",   elf_ehdr->e_phentsize);
    fprintf(stderr, "e_phnum     = %d\n",   elf_ehdr->e_phnum);
    fprintf(stderr, "e_shentsize = %d\n",   elf_ehdr->e_shentsize);
    fprintf(stderr, "e_shnum     = %d\n",   elf_ehdr->e_shnum);
    fprintf(stderr, "e_shstrndx  = %d\n",   elf_ehdr->e_shstrndx);
    fprintf(stderr, "\n\n");
#endif
    
    if (elf_ehdr->e_type != ET_EXEC)
    {
        fprintf(stderr, "Bad ELF file (not executable)\n");

        exit(-1);
    }

    if (elf_ehdr->e_machine != EM_ARM)
    {
        fprintf(stderr, "Bad ELF file (not ARM architecture)\n");

        exit(-1);
    }

    elf_shdr = (Elf32_Shdr*)(elf_data + elf_ehdr->e_shoff + elf_ehdr->e_shstrndx * elf_ehdr->e_shentsize);

    shstrtab = (const char*)(elf_data + elf_shdr->sh_offset);

    elf_phdr = (Elf32_Phdr*)(elf_data + elf_ehdr->e_phoff);
    elf_phdr_e = (Elf32_Phdr*)(elf_data + elf_ehdr->e_phoff + (elf_ehdr->e_phentsize * elf_ehdr->e_phnum));
    
    index = 0;
    
    while (elf_phdr < elf_phdr_e)
    {
#if 0        
        fprintf(stderr, "%d:\n", index);
        fprintf(stderr, "p_type      = %08x\n", elf_phdr->p_type);
        fprintf(stderr, "p_offset    = %08x\n", elf_phdr->p_offset);
        fprintf(stderr, "p_vaddr     = %08x\n", elf_phdr->p_vaddr);
        fprintf(stderr, "p_paddr     = %08x\n", elf_phdr->p_paddr);
        fprintf(stderr, "p_filesz    = %08x\n", elf_phdr->p_filesz);
        fprintf(stderr, "p_memsz     = %08x\n", elf_phdr->p_memsz);
        fprintf(stderr, "p_flags     = %08x\n", elf_phdr->p_flags);
        fprintf(stderr, "p_aignment  = %08x\n", elf_phdr->p_align);
        fprintf(stderr, "\n\n");
#endif
        
        index++;
        
        if ((elf_phdr->p_type == PT_LOAD) && elf_phdr->p_memsz)
        {
            if (elf_phdr->p_memsz != elf_phdr->p_filesz)
            {
                fprintf(stderr, "Bad ELF file (memsz != filesz)\n");

                exit(-1);
            }
            
            if (image_base > elf_phdr->p_paddr)
            {
                image_base = elf_phdr->p_paddr;
            }

            if (image_limit < (elf_phdr->p_paddr + elf_phdr->p_memsz))
            {
                image_limit = (elf_phdr->p_paddr + elf_phdr->p_memsz);
            }
        }
        
        elf_phdr = (Elf32_Phdr*)((uint8_t*)elf_phdr + elf_ehdr->e_phentsize);
    }

    elf_shdr = (Elf32_Shdr*)(elf_data + elf_ehdr->e_shoff);
    elf_shdr_e = (Elf32_Shdr*)(elf_data + elf_ehdr->e_shoff + (elf_ehdr->e_shentsize * elf_ehdr->e_shnum));

    strtab = NULL;
    elf_sym = NULL;
    elf_sym_e = NULL;
    
    index = 0;
    
    while (elf_shdr < elf_shdr_e)
    {
#if 0        
        fprintf(stderr, "%d:\n", index);
        fprintf(stderr, "sh_name      = %s\n", &shstrtab[elf_shdr->sh_name]);
        fprintf(stderr, "sh_type      = %08x\n", elf_shdr->sh_type);
        fprintf(stderr, "sh_flags     = %08x\n", elf_shdr->sh_flags);
        fprintf(stderr, "sh_addr      = %08x\n", elf_shdr->sh_addr);
        fprintf(stderr, "sh_offset    = %08x\n", elf_shdr->sh_offset);
        fprintf(stderr, "sh_size      = %08x\n", elf_shdr->sh_size);
        fprintf(stderr, "sh_link      = %08x\n", elf_shdr->sh_link);
        fprintf(stderr, "sh_info      = %08x\n", elf_shdr->sh_info);
        fprintf(stderr, "sh_addralign = %08x\n", elf_shdr->sh_addralign);
        fprintf(stderr, "sh_entsize   = %08x\n", elf_shdr->sh_entsize);
        fprintf(stderr, "\n\n");
#endif
        
        index++;

        if (!strcmp(&shstrtab[elf_shdr->sh_name], ".symtab"))
        {
            elf_sym = (Elf32_Sym*)(elf_data + elf_shdr->sh_offset);
            elf_sym_e = (Elf32_Sym*)(elf_data + elf_shdr->sh_offset + elf_shdr->sh_size);
        }

        if (!strcmp(&shstrtab[elf_shdr->sh_name], ".strtab"))
        {
            strtab = (const char*)(elf_data + elf_shdr->sh_offset);
        }
        
        elf_shdr = (Elf32_Shdr*)((uint8_t*)elf_shdr + elf_ehdr->e_shentsize);
    }

    index = 0;
    
    while (elf_sym < elf_sym_e)
    {
#if 0        
        fprintf(stderr, "%d:\n", index);
        fprintf(stderr, "st_name      = %s\n", &strtab[elf_sym->st_name]);
        fprintf(stderr, "st_value     = %08x\n", elf_sym->st_value);
        fprintf(stderr, "st_size      = %08x\n", elf_sym->st_size);
        fprintf(stderr, "sh_info      = %02x\n", elf_sym->st_info);
        fprintf(stderr, "sh_other     = %02x\n", elf_sym->st_other);
        fprintf(stderr, "sh_shndx     = %04x\n", elf_sym->st_shndx);
        fprintf(stderr, "\n\n");
#endif
        index++;

        if (!strcmp(&strtab[elf_sym->st_name], "__application_base__"))
        {
            __application_base__ = elf_sym->st_value;
        }

        if (!strcmp(&strtab[elf_sym->st_name], "__application_limit__"))
        {
            __application_limit__ = elf_sym->st_value;
        }

        if (!strcmp(&strtab[elf_sym->st_name], "__fwu_base__"))
        {
            __fwu_base__ = elf_sym->st_value;
        }

        if (!strcmp(&strtab[elf_sym->st_name], "__fwu_limit__"))
        {
            __fwu_limit__ = elf_sym->st_value;
        }

        if (!strcmp(&strtab[elf_sym->st_name], "__fwu_status__"))
        {
            __fwu_status__ = elf_sym->st_value;
        }
        
        if (!strcmp(&strtab[elf_sym->st_name], "__wireless_base__"))
        {
            __wireless_base__ = elf_sym->st_value;
        }
        
        elf_sym = (Elf32_Sym*)((uint8_t*)elf_sym + sizeof(Elf32_Sym));
    }
    
    if (__application_base__ == 0xffffffff)
    {
        fprintf(stderr, "Bad ELF file (missing symbol \"__application_base__\")\n");

        exit(-1);
    }

    if (__application_limit__ == 0xffffffff)
    {
        fprintf(stderr, "Bad ELF file (missing symbol \"__application_limit__\")\n");

        exit(-1);
    }

    if (__fwu_base__ == 0xffffffff)
    {
        fprintf(stderr, "Bad ELF file (missing symbol \"__fwu_base__\")\n");

        exit(-1);
    }

    if (__fwu_limit__ == 0xffffffff)
    {
        fprintf(stderr, "Bad ELF file (missing symbol \"__fwu_limit__\")\n");

        exit(-1);
    }

    if (__fwu_status__ == 0xffffffff)
    {
        fprintf(stderr, "Bad ELF file (missing symbol \"__fwu_status__\")\n");

        exit(-1);
    }

    if (__wireless_base__ == 0xffffffff)
    {
        fprintf(stderr, "Bad ELF file (missing symbol \"__wireless_base__\")\n");

        exit(-1);
    }
    
    if (image_base != STM32WB_BOOT_BASE)
    {
        fprintf(stderr, "Bad ELF file (invalid base address)\n");

        exit(-1);
    }

    image_size = image_limit - image_base;
    image_data = (uint8_t*)malloc(image_size);

    /* STM32WB erases FLASH to 0xff. ELF keeps gaps empty. So the image_data
     * needs to emulate that properly.
     */
    memset(image_data, 0xff, image_size);
    
    elf_phdr = (Elf32_Phdr*)(elf_data + elf_ehdr->e_phoff);
    elf_phdr_e = (Elf32_Phdr*)(elf_data + elf_ehdr->e_phoff + (elf_ehdr->e_phentsize * elf_ehdr->e_phnum));

    while (elf_phdr < elf_phdr_e)
    {
        if ((elf_phdr->p_type == PT_LOAD) && elf_phdr->p_filesz)
        {
            memcpy(image_data + (elf_phdr->p_paddr - image_base), elf_data + elf_phdr->p_offset, elf_phdr->p_filesz);
        }
        
        elf_phdr = (Elf32_Phdr*)((uint8_t*)elf_phdr + elf_ehdr->e_phentsize);
    }

    boot_info = (stm32wb_boot_info_t*)(image_data + ((stm32wb_boot_vectors_t*)image_data)->offset);
    
    boot_info->application_base = __application_base__;
    boot_info->application_limit = __application_limit__;
    boot_info->fwu_base = __fwu_base__;
    boot_info->fwu_limit = __fwu_limit__;
    boot_info->fwu_status = __fwu_status__;
    boot_info->wireless_base = __wireless_base__;
    
    *p_elf_data = elf_data;
    *p_elf_size = elf_size;
    *p_image_data = image_data;
    *p_image_size = image_size;
}

/**********************************************************************************************************************************/

static void elf_file_write(const char *elf_file_name, uint8_t *elf_data, uint32_t elf_size, const uint8_t *image_data, uint32_t image_size)
{
    FILE *fp;
    Elf32_Ehdr *elf_ehdr;
    Elf32_Phdr *elf_phdr, *elf_phdr_e;

    elf_ehdr = (Elf32_Ehdr*)elf_data;
    
    elf_phdr = (Elf32_Phdr*)(elf_data + elf_ehdr->e_phoff);
    elf_phdr_e = (Elf32_Phdr*)(elf_data + elf_ehdr->e_phoff + (elf_ehdr->e_phentsize * elf_ehdr->e_phnum));

    while (elf_phdr < elf_phdr_e)
    {
        if ((elf_phdr->p_type == PT_LOAD) && elf_phdr->p_filesz)
        {
            memcpy(elf_data + elf_phdr->p_offset, image_data + (elf_phdr->p_paddr - STM32WB_BOOT_BASE), elf_phdr->p_filesz);
        }
        
        elf_phdr = (Elf32_Phdr*)((uint8_t*)elf_phdr + elf_ehdr->e_phentsize);
    }

    fp = fopen(elf_file_name, "w");

    if (!fp)
    {
        fprintf(stderr, "Cannot open \"%s\"\n", elf_file_name);

        exit(-1);
    }

#ifdef WIN32
    _setmode(_fileno(fp), _O_BINARY);
#endif

    if (fwrite(elf_data, 1, elf_size, fp) != elf_size)
    {
        fprintf(stderr, "Cannot write \"%s\"\n", elf_file_name);

        exit(-1);
    }
        
    fclose(fp);
}

/**********************************************************************************************************************************/

static void bin_file_write(const char *bin_file_name, const uint8_t *image_data, uint32_t image_size)
{
    FILE *fp;

    fp = fopen(bin_file_name, "w");

    if (!fp)
    {
        fprintf(stderr, "Cannot open \"%s\"\n", bin_file_name);

        exit(-1);
    }

#ifdef WIN32
    _setmode(_fileno(fp), _O_BINARY);
#endif

    if (fwrite(image_data, 1, image_size, fp) != image_size)
    {
        fprintf(stderr, "Cannot write \"%s\"\n", bin_file_name);

        exit(-1);
    }
        
    fclose(fp);
}

/**********************************************************************************************************************************/

static void hex_file_write(const char *hex_file_name, const uint8_t *image_data, uint32_t image_size)
{
    FILE *fp;
    uint32_t index, offset, count;
    uint32_t extended_address, image_address;
    uint8_t address[2];
    int cksum;
    
    fp = fopen(hex_file_name, "w");
    
    if (!fp)
    {
        fprintf(stderr, "Cannot open \"%s\"\n", hex_file_name);
        
        exit(-1);
    }

#ifdef WIN32
    _setmode(_fileno(fp), _O_BINARY);
#endif

    extended_address = 0xffffffff;
        
    for (offset = 0; offset < image_size; offset += 16)
    {
        image_address = STM32WB_BOOT_BASE + offset;

        if (extended_address != (image_address & 0xffff0000))
        {
            extended_address = (image_address & 0xffff0000);

            address[0] = ((image_address >> 24) & 0xff);
            address[1] = ((image_address >> 16) & 0xff);

            cksum = 2 + 4 + address[0] + address[1];
                
            fprintf(fp, ":02000004%02X%02X%02X\r\n", address[0], address[1], ((0x100 - (cksum & 0xff)) & 0xff));
        }

        count = 16;

        if (count > (image_size - offset))
        {
            count = (image_size - offset);
        }

        address[0] = ((image_address >> 8) & 0xff);
        address[1] = ((image_address >> 0) & 0xff);

        cksum = count + address[0] + address[1] + 0x00;

        fprintf(fp, ":%02X%02X%02X00", count, address[0], address[1]);
            
        for (index = 0; index < count; index++)
        {
            cksum += image_data[offset + index];

            fprintf(fp, "%02X", image_data[offset + index]);
        }
            
        fprintf(fp, "%02X\r\n", ((0x100 - (cksum & 0xff)) & 0xff));
    }
    
    fprintf(fp, ":00000001FF\r\n");
        
    fclose(fp);
}

/**********************************************************************************************************************************/

static void dfu_file_write(const char *dfu_file_name, const uint8_t *image_data, uint32_t image_size, uint32_t signature_size, const uint32_t *aes128_data, const uint32_t *aes128_nonce, uint16_t vid, uint16_t pid)
{
    FILE *fp;
    const uint8_t *application_data;
    uint8_t *dfu_image;
    uint32_t application_size, dfu_size;
    uint32_t block, offset, size, count;
    uint32_t crc32;
    AES_KEY aes128_key;
    uint32_t aes128_iv[4], aes128_xor[4];
    stm32wb_fwu_prefix_t fwu_prefix;
    const stm32wb_application_info_t *application_info;

    memset(&fwu_prefix, 0, sizeof(stm32wb_fwu_prefix_t));

    application_data = image_data + ((stm32wb_boot_vectors_t*)image_data)->size;
    application_size = image_size - ((stm32wb_boot_vectors_t*)image_data)->size - signature_size;

    application_info = (const stm32wb_application_info_t*)(application_data + ((stm32wb_application_vectors_t*)application_data)->size - sizeof(stm32wb_application_info_t));
    
    if (signature_size)
    {
        dfu_size = application_size + signature_size;
        dfu_image = (uint8_t*)malloc(dfu_size);

        memcpy(dfu_image, application_data, dfu_size);
        
        fwu_prefix.magic = STM32WB_FWU_TAG_FWU | STM32WB_FWU_TYPE_UNCOMPRESSED | (((const stm32wb_application_vectors_t*)application_data)->magic & (STM32WB_APPLICATION_OPTION_MASK | STM32WB_APPLICATION_IDCODE_MASK));
        fwu_prefix.base = ((stm32wb_application_vectors_t*)application_data)->base;
        fwu_prefix.size = application_size;
        fwu_prefix.length = dfu_size;

        AES_set_encrypt_key((const uint8_t*)&aes128_data[0], 128, &aes128_key);
        
        aes128_iv[0] = 0;
        aes128_iv[1] = 0;
        aes128_iv[2] = 0;
        
        for (block = 0, offset = 0, size = dfu_size - sizeof(stm32wb_application_info_t) - signature_size; offset < size; offset += count, block += 1)
        {
            aes128_iv[3] = SWAP(block);

            count = 16;

            if (count > (size - offset))
            {
                count = size - offset;
            }
            
            AES_encrypt((const uint8_t*)&aes128_iv[0], (uint8_t*)&aes128_xor[0], &aes128_key);

            ((uint32_t*)&dfu_image[offset])[0] ^= aes128_xor[0];
            ((uint32_t*)&dfu_image[offset])[1] ^= aes128_xor[1];
            ((uint32_t*)&dfu_image[offset])[2] ^= aes128_xor[2];
            ((uint32_t*)&dfu_image[offset])[3] ^= aes128_xor[3];
        }

        fwu_prefix.nonce[0] = aes128_nonce[0];
        fwu_prefix.nonce[1] = aes128_nonce[1];
        fwu_prefix.nonce[2] = aes128_nonce[2];

        crc32 = 0xffffffff;

        for (offset = 0; offset < offsetof(stm32wb_fwu_prefix_t, crc32); offset++)
        {
            crc32 = crc32_byte(crc32, ((const uint8_t*)&fwu_prefix)[offset]);
        }
    
        for (offset = 0; offset < dfu_size; offset++)
        {
            crc32 = crc32_byte(crc32, dfu_image[offset]);
        }
    
        fwu_prefix.crc32 = crc32;
    }
    else
    {
        dfu_size = image_size;
        dfu_image = (uint8_t*)malloc(dfu_size);

        memcpy(dfu_image, image_data, dfu_size);
    }
    
    const uint8_t dfu_int_to_bcd[] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 
        0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 
        0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 
        0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 
        0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 
        0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 
        0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 
        0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 
    };
          
    unsigned char dfu_suffix[] = {
        0x00, /* bcdDevice lo */
        0x00, /* bcdDevice hi */
        0x00, /* idProduct lo */
        0x00, /* idProduct hi */
        0x00, /* idVendor lo */
        0x00, /* idVendor hi */
        0x00, /* bcdDFU lo */
        0x01, /* bcdDFU hi */
        'U',  /* ucDfuSignature lsb */
        'F',  /* ucDfuSignature --- */
        'D',  /* ucDfuSignature msb */
        16,   /* bLength for this version */
        0x00, /* dwCRC lsb */
        0x00, /* dwCRC --- */
        0x00, /* dwCRC --- */
        0x00  /* dwCRC msb */
    };
    
    dfu_suffix[0] = dfu_int_to_bcd[application_info->version.minor >> 0];
    dfu_suffix[1] = dfu_int_to_bcd[application_info->version.major >> 0];
    dfu_suffix[2] = pid >> 0;
    dfu_suffix[3] = pid >> 8;
    dfu_suffix[4] = vid >> 0;
    dfu_suffix[5] = vid >> 8;
    
    crc32 = 0xffffffff;

    if (signature_size)
    {
        for (offset = 0; offset < sizeof(stm32wb_fwu_prefix_t); offset++)
        {
            crc32 = crc32_byte(crc32, ((const uint8_t*)&fwu_prefix)[offset]);
        }
    }
    
    for (offset = 0; offset < dfu_size; offset++)
    {
        crc32 = crc32_byte(crc32, dfu_image[offset]);
    }
    
    for (offset = 0; offset < 12; offset++)
    {
        crc32 = crc32_byte(crc32, dfu_suffix[offset]);
    }
    
    dfu_suffix[12] = crc32 >> 0;
    dfu_suffix[13] = crc32 >> 8;
    dfu_suffix[14] = crc32 >> 16;
    dfu_suffix[15] = crc32 >> 24;
    
    fp = fopen(dfu_file_name, "w");
    
    if (!fp)
    {
        fprintf(stderr, "Cannot open \"%s\"\n", dfu_file_name);
        
        exit(-1);
    }
        
#ifdef WIN32
    _setmode(_fileno(fp), _O_BINARY);
#endif

    if (signature_size)
    {
        if (fwrite(&fwu_prefix, 1, sizeof(stm32wb_fwu_prefix_t), fp) != sizeof(stm32wb_fwu_prefix_t))
        {
            fprintf(stderr, "Cannot write \"%s\"\n", dfu_file_name);
            
            exit(-1);
        }
    }
    
    if (fwrite(dfu_image, 1, dfu_size, fp) != dfu_size)
    {
        fprintf(stderr, "Cannot write \"%s\"\n", dfu_file_name);
        
        exit(-1);
    }
    
    if (fwrite(dfu_suffix, 1, 16, fp) != 16)
    {
        fprintf(stderr, "Cannot write \"%s\"\n", dfu_file_name);
        
        exit(-1);
    }
    
    fclose(fp);

    free(dfu_image);
}

/**********************************************************************************************************************************/

static void ota_file_write(const char *ota_file_name, const uint8_t *image_data, uint32_t image_size, uint32_t signature_size, const uint32_t *aes128_data, const uint32_t *aes128_nonce, const char *reference_file_name)
{
    FILE *fp;
    const uint8_t *application_data;
    uint8_t *ota_image, *app_image, *reference_data, *fwu_image;
    uint32_t ota_size, application_size, reference_size, fwu_size;
    uint32_t block, offset, size, count;
    const uint8_t *in_current, *in_finish, *in_base, *in_limit, *out_base, *out_limit;
    uint8_t in_size, in_index, in_shift;
    uint8_t *swap_start, *swap_base, *swap_limit, *swap_current;
    uint32_t swap_size;
    uint32_t crc32;
    AES_KEY aes128_key;
    uint32_t aes128_random[4], aes128_iv[4], aes128_xor[4];
    stm32wb_boot_fwu_context_t fwu_ctx;
    stm32wb_fwu_prefix_t fwu_prefix;
    const stm32wb_boot_info_t *boot_info;    
        
    memset(&fwu_prefix, 0, sizeof(fwu_prefix));

    boot_info = (stm32wb_boot_info_t*)(image_data + ((stm32wb_boot_vectors_t*)image_data)->offset);

    application_data = image_data + ((stm32wb_boot_vectors_t*)image_data)->size;
    application_size = image_size - ((stm32wb_boot_vectors_t*)image_data)->size - signature_size;
    
    if (reference_file_name)
    {
        fp = fopen(reference_file_name, "r");
            
        if (!fp)
        {
            fprintf(stderr, "Cannot open \"%s\"\n", reference_file_name);
                
            exit(-1);
        }
            
#ifdef WIN32
        _setmode(_fileno(fp), _O_BINARY);
#endif

        if (signature_size)
        {
            if (fread(&fwu_prefix, 1, sizeof(stm32wb_fwu_prefix_t), fp) != sizeof(stm32wb_fwu_prefix_t))
            {
                fprintf(stderr, "Bad Reference image\n");
                
                exit(-1);
            }
            
            if ((fwu_prefix.magic != (STM32WB_FWU_TAG_FWU | STM32WB_FWU_TYPE_UNCOMPRESSED | (((const stm32wb_application_vectors_t*)application_data)->magic & (STM32WB_APPLICATION_OPTION_MASK | STM32WB_APPLICATION_IDCODE_MASK)))) ||
                (fwu_prefix.base != ((const stm32wb_application_vectors_t*)application_data)->base))
            {
                fprintf(stderr, "Bad Reference image\n");
                
                exit(-1);
            }
            
            reference_size = fwu_prefix.size;
            reference_data = (uint8_t*)malloc(reference_size);
            
            if (fread(reference_data, 1, reference_size, fp) != reference_size)
            {
                fprintf(stderr, "Bad Reference image\n");
                
                exit(-1);
            }
            
            AES_set_encrypt_key((const uint8_t*)&aes128_data[0], 128, &aes128_key);

            aes128_iv[0] = 0;
            aes128_iv[1] = 0;
            aes128_iv[2] = 0;
                
            for (block = 0, offset = 0, size = reference_size - sizeof(stm32wb_application_info_t) - signature_size; offset < size; offset += count, block += 1)
            {
                aes128_iv[3] = SWAP(block);

                count = 16;
                    
                if (count > (size - offset))
                {
                    count = size - offset;
                }
                    
                AES_encrypt((const uint8_t*)&aes128_iv[0], (uint8_t*)&aes128_xor[0], &aes128_key);
                    
                ((uint32_t*)&reference_data[offset])[0] ^= aes128_xor[0];
                ((uint32_t*)&reference_data[offset])[1] ^= aes128_xor[1];
                ((uint32_t*)&reference_data[offset])[2] ^= aes128_xor[2];
                ((uint32_t*)&reference_data[offset])[3] ^= aes128_xor[3];
            }
            
            fclose(fp);
        }
        else
        {
            reference_size = sizeof(stm32wb_application_vectors_t);
            reference_data = (uint8_t*)malloc(reference_size);

            if (fread(reference_data, 1, reference_size, fp) != reference_size)
            {
                fprintf(stderr, "Bad Reference image\n");
                
                exit(-1);
            }

            if ((((const stm32wb_application_vectors_t*)reference_data)->magic != ((const stm32wb_application_vectors_t*)application_data)->magic) ||
                (((const stm32wb_application_vectors_t*)reference_data)->base != ((const stm32wb_application_vectors_t*)application_data)->base))
            {
                fprintf(stderr, "Bad Reference image\n");
                
                exit(-1);
            }

            reference_size = ((const stm32wb_application_vectors_t*)reference_data)->size;
            reference_data = (uint8_t*)realloc(reference_data, reference_size);

            if (fread(reference_data + sizeof(stm32wb_application_vectors_t), 1, (reference_size - sizeof(stm32wb_application_vectors_t)), fp) != reference_size)
            {
                fprintf(stderr, "Bad Reference image\n");
                
                exit(-1);
            }
        }
    }
    else
    {
        reference_data = NULL;
        reference_size = 0;
    }
    
    ota_image = (uint8_t*)malloc(4 * 1024 * 1024);
    app_image = (uint8_t*)malloc(4 * 1024 * 1024);
    fwu_image = (uint8_t*)malloc(4 * 1024 * 1024);

    memset(&ota_image[0], 0, 4 * 1024 *1024);
    memset(&app_image[0], 0, 4 * 1024 *1024);
    memset(&fwu_image[0], 0, 4 * 1024 *1024);
        
    memcpy(&ota_image[0], &application_data[0], sizeof(stm32wb_application_vectors_t)); // ???
            
    ota_size = lzfwu_compress(&reference_data[0], (reference_size - sizeof(stm32wb_application_info_t)), &application_data[0], (application_size - sizeof(stm32wb_application_info_t)), 0, &ota_image[0]);

    memcpy(&ota_image[ota_size], &application_data[application_size - sizeof(stm32wb_application_info_t)], (sizeof(stm32wb_application_info_t) + signature_size));

    ota_size += (sizeof(stm32wb_application_info_t) + signature_size);

    fwu_prefix.magic = STM32WB_FWU_TAG_FWU | STM32WB_FWU_TYPE_COMPRESSED | (((const stm32wb_application_vectors_t*)application_data)->magic & (STM32WB_APPLICATION_OPTION_MASK | STM32WB_APPLICATION_IDCODE_MASK));
    fwu_prefix.base = ((stm32wb_application_vectors_t*)application_data)->base;
    fwu_prefix.size = application_size;
    fwu_prefix.length = ota_size;

    if (signature_size)
    {
        AES_set_encrypt_key((const uint8_t*)&aes128_data[0], 128, &aes128_key);
        
        aes128_iv[0] = 0;
        aes128_iv[1] = 0;
        aes128_iv[2] = 0;

        for (block = 0, offset = 0, size = ota_size - (sizeof(stm32wb_application_info_t) + signature_size); offset < size; offset += count, block += 1)
        {
            aes128_iv[3] = SWAP(block);

            count = 16;

            if (count > (size - offset))
            {
                count = size - offset;
            }
            
            AES_encrypt((const uint8_t*)&aes128_iv[0], (uint8_t*)&aes128_xor[0], &aes128_key);

            ((uint32_t*)&ota_image[offset])[0] ^= aes128_xor[0];
            ((uint32_t*)&ota_image[offset])[1] ^= aes128_xor[1];
            ((uint32_t*)&ota_image[offset])[2] ^= aes128_xor[2];
            ((uint32_t*)&ota_image[offset])[3] ^= aes128_xor[3];
        }

        fwu_prefix.nonce[0] = aes128_nonce[0];
        fwu_prefix.nonce[1] = aes128_nonce[1];
        fwu_prefix.nonce[2] = aes128_nonce[2];
    }
    else
    {
        fwu_prefix.nonce[0] = 0;
        fwu_prefix.nonce[1] = 0;
        fwu_prefix.nonce[2] = 0;
    }
    
    crc32 = 0xffffffff;

    for (offset = 0; offset < offsetof(stm32wb_fwu_prefix_t, crc32); offset++)
    {
        crc32 = crc32_byte(crc32, ((const uint8_t*)&fwu_prefix)[offset]);
    }
            
    for (offset = 0; offset < ota_size; offset++)
    {
        crc32 = crc32_byte(crc32, ota_image[offset]);
    }

    fwu_prefix.crc32 = crc32;


    // PASS 1: Uncompress without save/restore, with CRC32 check.
    
    RAND_bytes((uint8_t*)&aes128_random[0], 16);

    stm32wb_boot_nvm_set_key((uint32_t*)&aes128_random[0], &ota_image[0], ota_size);
    
    memset(&app_image[0], 0, 4 * 1024 * 1024);

    if (reference_data)
    {
        memcpy(&app_image[0], &reference_data[0], reference_size);
    }

    fwu_size = ota_size;
    memcpy(&fwu_image[0], &ota_image[0], fwu_size);
      
    if (signature_size)
    {
        fwu_ctx.encrypted = true;

        stm32wb_boot_aes128_set_key(&fwu_ctx.aes128_ctx, &aes128_data[0]);

        fwu_ctx.aes128_iv[0] = 0;
        fwu_ctx.aes128_iv[1] = 0;
        fwu_ctx.aes128_iv[2] = 0;
        fwu_ctx.aes128_iv[3] = 0;
    }
    else
    {
        fwu_ctx.encrypted = false;
    }
    
    in_base = &fwu_image[0];
    in_finish = &fwu_image[fwu_size - (sizeof(stm32wb_application_info_t) + signature_size)];
    in_limit = &fwu_image[fwu_size];
            
    out_base = &app_image[0];
    out_limit = &app_image[boot_info->application_limit - boot_info->application_base];

    swap_start = &fwu_image[(fwu_size + 4095) & ~4095];
    swap_base = &fwu_image[0];
    swap_limit = &fwu_image[boot_info->fwu_limit - boot_info->fwu_base];
    swap_current = swap_start;
    swap_size = 0;
            
    stm32wb_boot_fwu_init(&fwu_ctx, in_base, in_finish, in_limit, out_base, out_limit, swap_start, swap_base, swap_limit);

    in_current = &fwu_image[0];
    in_size = 64;
    in_index = 63;
    in_shift = 32;

    stm32wb_boot_fwu_restore(&fwu_ctx, in_current, in_size, in_index, in_shift, &app_image[offset]);

    for (offset = 0; offset < (application_size - sizeof(stm32wb_application_info_t)); offset += size)
    {
        stm32wb_boot_nvm_program(swap_current, &app_image[offset], 4096, true);

        swap_current += 4096;
        swap_size += 4096;

        if (swap_current == swap_limit)
        {
            swap_current = swap_base;
        }

        size = 4096;

        if (size > ((application_size - sizeof(stm32wb_application_info_t)) - offset))
        {
            size = (application_size - sizeof(stm32wb_application_info_t)) - offset;
        }

        stm32wb_boot_fwu_uncompress(&fwu_ctx, &app_image[offset], size, swap_size);
    }

    stm32wb_boot_fwu_data(&fwu_ctx, &app_image[offset], (sizeof(stm32wb_application_info_t) + signature_size));
    
    if (fwu_ctx.in_current != fwu_ctx.in_limit)
    {
        fprintf(stderr, "Uncompress Size Mismatch\n");

        exit(-1);
    }
    
    if (memcmp(&app_image[0], &application_data[0], (application_size + signature_size)))
    {
        fprintf(stderr, "Uncompress Data Mismatch\n");

        exit(-1);
    }
    

    // PASS 2: Uncompress with save/restore, without CRC32 check.
    
    RAND_bytes((uint8_t*)&aes128_random[0], 16);

    stm32wb_boot_nvm_set_key((uint32_t*)&aes128_random[0], &ota_image[0], ota_size);
    
    memset(&app_image[0], 0, 4 * 1024 * 1024);

    if (reference_data)
    {
        memcpy(&app_image[0], &reference_data[0], reference_size);
    }

    fwu_size = ota_size;
    memcpy(&fwu_image[0], &ota_image[0], fwu_size);

    if (signature_size)
    {
        fwu_ctx.encrypted = true;

        stm32wb_boot_aes128_set_key(&fwu_ctx.aes128_ctx, &aes128_data[0]);

        fwu_ctx.aes128_iv[0] = 0;
        fwu_ctx.aes128_iv[1] = 0;
        fwu_ctx.aes128_iv[2] = 0;
        fwu_ctx.aes128_iv[3] = 0;
    }
    else
    {
        fwu_ctx.encrypted = false;
    }
    
    in_base = &fwu_image[0];
    in_finish = &fwu_image[fwu_size - (sizeof(stm32wb_application_info_t) + signature_size)];
    in_limit = &fwu_image[fwu_size];
            
    out_base = &app_image[0];
    out_limit = &app_image[boot_info->application_limit - boot_info->application_base];

    swap_start = &fwu_image[(fwu_size + 4095) & ~4095];
    swap_base = &fwu_image[0];
    swap_limit = &fwu_image[boot_info->fwu_limit - boot_info->fwu_base];
    swap_current = swap_start;
    swap_size = 0;
            
    stm32wb_boot_fwu_init(&fwu_ctx, in_base, in_finish, in_limit, out_base, out_limit, swap_start, swap_base, swap_limit);

    in_current = &fwu_image[0];
    in_size = 64;
    in_index = 63;
    in_shift = 32;

    for (offset = 0; offset < (application_size - sizeof(stm32wb_application_info_t)); offset += size)
    {
        stm32wb_boot_nvm_program(swap_current, &app_image[offset], 4096, true);

        swap_current += 4096;
        swap_size += 4096;

        if (swap_current == swap_limit)
        {
            swap_current = swap_base;
        }

        size = 4096;

        if (size > ((application_size - sizeof(stm32wb_application_info_t)) - offset))
        {
            size = (application_size - sizeof(stm32wb_application_info_t)) - offset;
        }

        stm32wb_boot_fwu_restore(&fwu_ctx, in_current, in_size, in_index, in_shift, &app_image[offset]);

        stm32wb_boot_fwu_uncompress(&fwu_ctx, &app_image[offset], size, swap_size);

        stm32wb_boot_fwu_save(&fwu_ctx, &in_current, &in_size, &in_index, &in_shift);
    }

    stm32wb_boot_fwu_data(&fwu_ctx, &app_image[offset], (sizeof(stm32wb_application_info_t) + signature_size));

    if (fwu_ctx.in_current != fwu_ctx.in_limit)
    {
        fprintf(stderr, "Uncompress Size Mismatch\n");

        exit(-1);
    }
    
    if (memcmp(&app_image[0], &application_data[0], (application_size + signature_size)))
    {
        fprintf(stderr, "Uncompress Data Mismatch\n");

        exit(-1);
    }
    
    fp = fopen(ota_file_name, "w");
        
    if (!fp)
    {
        fprintf(stderr, "Cannot open \"%s\"\n", ota_file_name);
            
        exit(-1);
    }

#ifdef WIN32
    _setmode(_fileno(fp), _O_BINARY);
#endif

    if (fwrite(&fwu_prefix, 1, sizeof(fwu_prefix), fp) != sizeof(fwu_prefix))
    {
        fprintf(stderr, "Cannot write \"%s\"\n", ota_file_name);
            
        exit(-1);
    }
        
    if (fwrite(ota_image, 1, ota_size, fp) != ota_size)
    {
        fprintf(stderr, "Cannot write \"%s\"\n", ota_file_name);
                
        exit(-1);
    }
        
    fclose(fp);

    free(app_image);
    free(ota_image);

    if (reference_data)
    {
        free(reference_data);
    }
}

/**********************************************************************************************************************************/

static int ascii2hex(char c)
{
    if ((c >= '0') && (c <= '9')) { return ((c - '0') + 0x00); }
    if ((c >= 'a') && (c <= 'f')) { return ((c - 'a') + 0x0a); }
    if ((c >= 'A') && (c <= 'F')) { return ((c - 'A') + 0x0a); }

    return -1;
}

/**********************************************************************************************************************************/

int main(int argc, char **argv)
{
    FILE *fp;
    struct timeval tv;
    int c, option_index;
    stm32wb_boot_sha256_context_t sha256_ctx;
    uint8_t sha256_data[256], sha256_hash[64];
    uint32_t signature_size;
    int index, offset, data_l, data_h;
    unsigned int length;
    uint32_t aes128_data[4], aes128_nonce[3], *aes128_code;
    AES_KEY aes128_key;
    uint64_t timestamp;
    EC_KEY *ecc256_key;
    BN_CTX *bn_ctx;
    uint8_t *ecc256_key_data;
    size_t ecc256_key_size;
    uint32_t *key_x, *key_y;
    uint8_t signature[256], *signature_r, *signature_s;
    RSA *rsa2048_key;
    uint8_t *elf_data, *image_data, *boot_data, *application_data;
    uint32_t elf_size, image_size, boot_size, application_size;
    stm32wb_boot_info_t *boot_info;
    stm32wb_system_info_t *system_info;
    stm32wb_application_info_t *application_info;

    char *uuid_string = NULL;
    char *reference_file_name = NULL;
    char *encryption_key_file_name = NULL;
    char *signature_key_file_name = NULL;
    char *bin_file_name = NULL;
    char *dfu_file_name = NULL;
    char *ota_file_name = NULL;
    char *hex_file_name = NULL;
    char *in_file_name = NULL;
    char *out_file_name = NULL;

    uint8_t uuid[16];

    uint16_t vid = 0xffff;
    uint16_t pid = 0xffff;

    bool protected = false;
    
    while (1)
    {
        c = getopt_long(argc, argv, "lu:v:p:r:e:s:b:d:o:x:", opts, &option_index);

        if (c == -1)
        {
            break;
        }
        
        switch (c) {
        case 'l':
            protected = true;
            break;

        case 'u':
            uuid_string = optarg;
            break;
                
        case 'v':
            vid = strtol(optarg, NULL, 16);
            break;

        case 'p':
            pid = strtol(optarg, NULL, 16);
            break;
           
        case 'r':
            reference_file_name = optarg;
            break;

        case 'e':
            encryption_key_file_name = optarg;
            break;

        case 's':
            signature_key_file_name = optarg;
            break;

        case 'b':
            bin_file_name = optarg;
            break;

        case 'd':
            dfu_file_name = optarg;
            break;

        case 'o':
            ota_file_name = optarg;
            break;
            
        case 'x':
            hex_file_name = optarg;
            break;
            
        default:
            break;
        }
    }

    if (optind < argc)
    {
        in_file_name = out_file_name = argv[optind++];
    }

    if (optind < argc)
    {
        out_file_name = argv[optind++];
    }

    if (optind < argc)
    {
        fprintf(stderr, "Too many argments\n");

        return -1;
    }

    if (!in_file_name || !out_file_name)
    {
        fprintf(stderr, "No in-file-name");

        return -1;
    }
    
    if (!uuid_string)
    {
        fprintf(stderr, "Missing UUID");

        return -1;
    }
    
    length = strlen(uuid_string);

    if ((length != 32) && (length != 36))
    {
        fprintf(stderr, "Bad UUID\n");
        
        return -1;
    }
    
    for (index = 15; index >= 0; index--)
    {
        data_h = ascii2hex(*uuid_string++);
        data_l = ascii2hex(*uuid_string++);
        
        if ((data_h < 0) || (data_l < 0))
        {
            fprintf(stderr, "Bad UUID\n");
            
            return -1;
        }
        
        uuid[index] = (data_h << 4) | (data_l << 0);
        
        if ((length == 36) && ((1 << index) & ((1 << 12) | (1 << 10) | (1 << 8) | (1 << 6))))
        {
            if (*uuid_string++ != '-')
            {
                fprintf(stderr, "Bad UUID\n");
                
                return -1;
            }
        }
    }

    gettimeofday(&tv, NULL);
    
    elf_file_read(in_file_name, &elf_data, &elf_size, &image_data, &image_size);

    boot_data = image_data;
    boot_size = ((stm32wb_boot_vectors_t*)boot_data)->size;
    
    application_data = image_data + boot_size;
    application_size = ((stm32wb_application_vectors_t*)application_data)->size;

    boot_info = (stm32wb_boot_info_t*)(boot_data + ((stm32wb_boot_vectors_t*)boot_data)->offset);
    system_info = (stm32wb_system_info_t*)(application_data + ((stm32wb_application_vectors_t*)application_data)->offset);
    application_info = (stm32wb_application_info_t*)(application_data + ((stm32wb_application_vectors_t*)application_data)->size - sizeof(stm32wb_application_info_t));

    memcpy(&boot_info->options, &system_info->options, (sizeof(stm32wb_system_info_t) - offsetof(stm32wb_system_info_t, options)));

    if (((uint32_t*)&application_info->uuid[0])[0] | ((uint32_t*)&application_info->uuid[0])[1] | ((uint32_t*)&application_info->uuid[0])[2] | ((uint32_t*)&application_info->uuid[0])[3])
    {
        memcpy(&boot_info->uuid[0], &application_info->uuid[0], 16);
    }
    else
    {
        memcpy(&boot_info->uuid[0], &uuid[0], 16);
        memcpy(&application_info->uuid[0], &uuid[0], 16);
    }
    
    application_info->epoch = tv.tv_sec;
    application_info->crc32 = stm32wb_boot_crc32(application_data, (application_size - sizeof(uint32_t)), 0xffffffff);
    
    if (signature_key_file_name)
    { 
        signature_size = STM32WB_APPLICATION_SIGNATURE_SIZE_NONE;

        if (protected)
        {
            ((stm32wb_boot_vectors_t*)boot_data)->magic |= STM32WB_BOOT_OPTION_PROTECTED;
        }
        else
        {
            ((stm32wb_boot_vectors_t*)boot_data)->magic &= ~STM32WB_BOOT_OPTION_PROTECTED;
        }

        ((stm32wb_application_vectors_t*)application_data)->magic = ((((stm32wb_boot_vectors_t*)boot_data)->magic & ~STM32WB_BOOT_TYPE_MASK) | STM32WB_APPLICATION_TYPE_APPLICATION);

        if ((((stm32wb_boot_vectors_t*)boot_data)->magic & STM32WB_BOOT_OPTION_SIGNATURE_MASK) == STM32WB_BOOT_OPTION_SIGNATURE_ECC256)
        {
            signature_size = STM32WB_APPLICATION_SIGNATURE_SIZE_ECC256;

            fp = fopen(signature_key_file_name, "r");
        
            if (!fp)
            {
                fprintf(stderr, "Cannot open \"%s\"\n", signature_key_file_name);
                
                return -1;
            }
            
            ecc256_key = PEM_read_ECPrivateKey(fp, NULL, NULL, NULL);

            fclose(fp);
            
            if (!ecc256_key)
            {
                fprintf(stderr, "Cannot read \"%s\"\n", signature_key_file_name);
                
                return -1;
            }
            
            bn_ctx = BN_CTX_new();
        
            ecc256_key_size = EC_KEY_key2buf(ecc256_key, POINT_CONVERSION_UNCOMPRESSED, &ecc256_key_data, bn_ctx);
            
            if (ecc256_key_size != 65)
            {
                fprintf(stderr, "Wrong ECC key size\n");
                
                return -1;
            }
            
            key_x = &((stm32wb_boot_ecc256_key_t*)(boot_data + (boot_info->ecc256_key - STM32WB_BOOT_BASE)))->x[0];
            key_y = &((stm32wb_boot_ecc256_key_t*)(boot_data + (boot_info->ecc256_key - STM32WB_BOOT_BASE)))->y[0];
            
            for (index = 0; index < 8; index++)
            {
                key_x[index] = SWAP(((const uint32_t*)&ecc256_key_data[1])[7 - index]);
                key_y[index] = SWAP(((const uint32_t*)&ecc256_key_data[33])[7 - index]);
            }
            
            SHA256((uint8_t*)application_data, application_size, &sha256_hash[0]);
            
            if (!ECDSA_sign(0, (const uint8_t*)&sha256_hash[0], 32, &signature[0], &length, ecc256_key))
            {
                fprintf(stderr, "Internal ECC mismatch error\n");
                
                return -1;
            }
            
            signature_r = (signature[3] == 32) ? &signature[4] : &signature[5];
            signature_s = (signature[((signature[3] == 32) ? 4 : 5) + 32 +1] == 32) ? &signature[((signature[3] == 32) ? 4 : 5) + 32 +2] : &signature[((signature[3] == 32) ? 4 : 5) + 32 +3];
            
            for (index = 0; index < 8; index++)
            {
                ((uint32_t*)((uint8_t*)application_info + sizeof(stm32wb_application_info_t) +  0))[index] = SWAP(((const uint32_t*)signature_r)[7 - index]);
                ((uint32_t*)((uint8_t*)application_info + sizeof(stm32wb_application_info_t) + 32))[index] = SWAP(((const uint32_t*)signature_s)[7 - index]);
            }
        }

        if ((((stm32wb_boot_vectors_t*)boot_data)->magic & STM32WB_BOOT_OPTION_SIGNATURE_MASK) == STM32WB_BOOT_OPTION_SIGNATURE_RSA2048)
        {
            signature_size = STM32WB_APPLICATION_SIGNATURE_SIZE_RSA2048;

            fp = fopen(signature_key_file_name, "r");
        
            if (!fp)
            {
                fprintf(stderr, "Cannot open \"%s\"\n", signature_key_file_name);
                
                return -1;
            }
            
            rsa2048_key = PEM_read_RSAPrivateKey(fp, NULL, NULL, NULL);
            
            fclose(fp);

            if (!rsa2048_key)
            {
                fprintf(stderr, "Cannot read \"%s\"\n", signature_key_file_name);
                
                return -1;
            }
            
            SHA256((uint8_t*)application_data, application_size, &sha256_hash[0]);
            
            RSA_sign(NID_sha256, (const uint8_t*)&sha256_hash[0], 32, ((uint8_t*)application_data + application_size), &length, rsa2048_key);
            
            stm32wb_boot_rsa2048_convert_key(rsa2048_key, (stm32wb_boot_rsa2048_key_t*)(boot_data + (boot_info->rsa2048_key - STM32WB_BOOT_BASE)));
            
            stm32wb_boot_sha256_init(&sha256_ctx);
            stm32wb_boot_sha256_update(&sha256_ctx, (uint8_t*)application_data, application_size);
            stm32wb_boot_sha256_final(&sha256_ctx, (uint32_t*)&sha256_hash[0]);
            
            if (!stm32wb_boot_rsa2048_verify((stm32wb_boot_rsa2048_key_t*)(boot_data + (boot_info->rsa2048_key - STM32WB_BOOT_BASE)), (uint32_t*)((uint8_t*)application_info + application_size), (uint32_t*)&sha256_hash[0]))
            {
                fprintf(stderr, "Internal RSA mismatch error\n");
                
                return -1;
            }
        }
        
        if (signature_size == STM32WB_APPLICATION_SIGNATURE_SIZE_NONE)
        {
            fprintf(stderr, "Invalid signature key\n");
            
            return -1;
        }
        
        if (!encryption_key_file_name)
        {
            fprintf(stderr, "Missing encryption key\n");
                
            return -1;
        }

        fp = fopen(encryption_key_file_name, "r");

        if (!fp)
        {
            fprintf(stderr, "Cannot open \"%s\"\n", encryption_key_file_name);
                
            return -1;
        }
    
#ifdef WIN32
        _setmode(_fileno(fp), _O_BINARY);
#endif
        if (fread((uint8_t*)&aes128_data[0], 1, 16, fp) != 16)
        {
            fprintf(stderr, "Cannot read \"%s\"\n", encryption_key_file_name);
                
            return -1;
        }

        fclose(fp);
        
        aes128_code = (uint32_t*)(boot_data + STM32WB_BOOT_AES128_KEY - STM32WB_BOOT_BASE);

        for (offset = 0; offset < 4; offset++)
        {
            aes128_code[0 + 3 * offset] = (0x0100f240 |
                                           (((aes128_data[offset] & 0x000000ff) >>  0) << 16) |
                                           (((aes128_data[offset] & 0x00000700) >>  8) << 28) |
                                           (((aes128_data[offset] & 0x00000800) >> 11) << 10) |
                                           (((aes128_data[offset] & 0x0000f000) >> 12) <<  0));

            aes128_code[1 + 3 * offset] = (0x0100f2c0 |
                                           (((aes128_data[offset] & 0x00ff0000) >> 16) << 16) |
                                           (((aes128_data[offset] & 0x07000000) >> 24) << 28) |
                                           (((aes128_data[offset] & 0x08000000) >> 27) << 10) |
                                           (((aes128_data[offset] & 0xf0000000) >> 28) <<  0));

            aes128_code[2 + 3 * offset] = ((offset != 3) ? 0x1b04f840 : 0x47706001);
        }

        /* 42 bit timestamp in units of 500us since 1/1/2024 midnight.
         */
        timestamp = ((uint64_t)tv.tv_sec * 2000) + ((uint64_t)tv.tv_usec / 500) - ((uint64_t)1704067200 * 2000);

        RAND_bytes((uint8_t*)&aes128_nonce[0], 12);

        aes128_nonce[0] = (aes128_nonce[0] & ~0xffffffff) | ((uint32_t)(timestamp >>  0) & 0xffffffff);
        aes128_nonce[1] = (aes128_nonce[1] & ~0x000003ff) | ((uint32_t)(timestamp >> 32) & 0x000003ff);
        
        memcpy(&sha256_data[ 0], &boot_info->uuid[0], 16);
        sha256_data[16] = (uint8_t)(STM32WB_BOOT_IDCODE_STM32WB55 >> 0);
        sha256_data[17] = (uint8_t)(STM32WB_BOOT_IDCODE_STM32WB55 >> 8);
        sha256_data[18] = 0;
        sha256_data[19] = 0;
        memcpy(&sha256_data[20], &aes128_nonce[0], 12);
        
        SHA256(&sha256_data[0], 32, &sha256_hash[0]);
        
        AES_set_encrypt_key((const uint8_t*)&aes128_data[0], 128, &aes128_key);
        
        AES_encrypt((const uint8_t*)&sha256_hash[0], (uint8_t*)&aes128_data[0], &aes128_key);
    }
    else
    {
        signature_size = STM32WB_APPLICATION_SIGNATURE_SIZE_NONE;

        ((stm32wb_boot_vectors_t*)boot_data)->magic = ((((stm32wb_boot_vectors_t*)boot_data)->magic & ~(STM32WB_BOOT_OPTION_PROTECTED | STM32WB_BOOT_OPTION_SIGNATURE_MASK)) | STM32WB_BOOT_OPTION_SIGNATURE_NONE);

        ((stm32wb_application_vectors_t*)application_data)->magic = ((((stm32wb_boot_vectors_t*)boot_data)->magic & ~STM32WB_BOOT_TYPE_MASK) | STM32WB_APPLICATION_TYPE_APPLICATION);

        aes128_code = (uint32_t*)(boot_data + STM32WB_BOOT_AES128_KEY - STM32WB_BOOT_BASE);

        for (offset = 0; offset < 4; offset++)
        {
            aes128_code[0 + 3 * offset] = 0xbe00be00;
            aes128_code[1 + 3 * offset] = 0xbe00be00;
            aes128_code[2 + 3 * offset] = 0xbe00be00;
        }

        memset((uint8_t*)&aes128_nonce[0], 0, 12);
    }
    
    SHA256((uint8_t*)boot_data, (STM32WB_BOOT_HASH - STM32WB_BOOT_BASE), (boot_data + STM32WB_BOOT_HASH - STM32WB_BOOT_BASE));
    
    if (out_file_name)
    {
        elf_file_write(out_file_name, elf_data, elf_size, image_data, image_size);
    }
    
    if (bin_file_name)
    {
        bin_file_write(bin_file_name, image_data, (boot_size + application_size + signature_size));
    }

    if (hex_file_name)
    {
        hex_file_write(hex_file_name, image_data, (boot_size + application_size + signature_size));
    }
    
    if (dfu_file_name)
    {
        dfu_file_write(dfu_file_name, image_data, (boot_size + application_size + signature_size), signature_size, aes128_data, aes128_nonce, vid, pid);
    }

    if (ota_file_name)
    {
        ota_file_write(ota_file_name, image_data, (boot_size + application_size + signature_size), signature_size, aes128_data, aes128_nonce, reference_file_name);
    }

    free(elf_data);
    free(image_data);
    
    return 0;
}

