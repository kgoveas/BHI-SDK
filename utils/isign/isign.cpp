////////////////////////////////////////////////////////////////////////////////
///
/// @file       utils/isign/isign.cpp
///
/// @project    EM7189
///
/// @brief
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2018 EM Microelectronic
/// @cond
///
/// All rights reserved.
///
/// Disclosure to third parties or reproduction in any form what-
/// soever, without prior written consent, is strictly forbidden
///
////////////////////////////////////////////////////////////////////////////////
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
/// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
/// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
/// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
/// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
/// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
/// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
/// @endcond
////////////////////////////////////////////////////////////////////////////////
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <io.h>
#define unlink(__x__) _unlink(__x__)
#else
#include <unistd.h>
#endif
#include "Typedef.h"
#include "Keys.h"
#include "SHA256.h"
#include "ECCStatus.h"
#include "ECCkeyManagement.h"
#include "ECDSA_Sign.h"
#include "ECDSA_Verify.h"
#include "fwHeader.h"

#define KB                   (1024)
#define CRC_SEED             (0xFFFFFFFF)
#define USER_HDROFFSET_LOC   0
#define KERNEL_HDROFFSET_LOC (0x0E * 4)
#define DEFAULT_KEY_LOCATION KEY_LOCATION_OTP
#define DEFAULT_KEY_TYPE     KEY_TYPE_ECDSA
#define KEY_LOCATION_KERNEL  0x4 // Pick RAM or Flash according to ImgTypeFlash fw header image flag

#define DMA_PADDING_NEEDED   128u
#define DMA_TRANSACTION_SIZE 1024u

#define FIXED_RAM_BANK_SIZE    0x4000
#define OPTIONAL_RAM_BANK_SIZE 0x8000
#define CODE_RAM_START         0x124000

#define KERNEL_IMAGE_IDX (0u)
#define USER_IMAGE_IDX   (1u)
#define ALL_IMAGES       (0xff)

typedef enum
{
    DEFAULT_TASK,
    USAGE,
    SIGN_FW,
    INFO,
    SIGN_DIGEST,
    VERIFY_FW,
    TEST_IMAGES,
} task_t;


typedef struct
{
    uint32_t   zeroPad;
    FWHeader_t hdr;
} FWHeaderBuffer_t;

typedef enum
{
    SUCCESS,                // 0
    ERR_NO_FW_FILE,         // 1
    ERR_FILE_READ,          // 2
    ERR_FILE_WRITE,         // 3
    ERR_NO_FW_HEADER,       // 4
    ERR_IMAGE_SIZE,         // 5
    ERR_SSBL_MISMATCH,      // 6
    ERR_INVALID_ARGUMENT,   // 7
    ERR_IMAGE_CRC,          // 8
    ERR_HDR_CRC,            // 9
    ERR_IMAGE_VERIFICATION, // 10
    ERR_NO_PUBKEY_IN_FW, // 11
    ERR_NO_PUBKEY_FILE,  // 12
    ERR_SHA,             // 13
    ERR_EXT_SIGNING,     // 14
    ERR_KEY_CRC,         // 15
    ERR_NO_PRIVATE_KEY_FILE, // 16
    ERR_DIGEST_FORMAT,       // 17
    ERR_ECDSA,               // 18
    MAX_ERR,
} error_t;

const char *errors[] = {
    "Success",                  // 0
    "No FW file",               // 1
    "Error reading file",       // 2
    "Error writing file",       // 3
    "No FW header",             // 4
    "Image size error",         // 5
    "Mismatched PairWithSSBL",  // 6
    "Invalid argument",         // 7
    "Invalid image CRC",        // 8
    "Invalid header CRC",       // 9
    "Error verifying firmware", // 10
    "No public key in firmware", // 11
    "No public key file",        // 12
    "Invalid SHA",               // 13
    "External signing error",    // 14
    "Invalid key CRC",           // 15
    "No private key file or external sign tool specified", // 16
    "Invalid digest format",                               // 17
    "Invalid signature",                                   // 18
};

static inline const char *error_to_string(error_t error)
{
    if (error >= MAX_ERR)
    {
        return "unknown";
    }
    else
    {
        return errors[error];
    }
}

uint32_t update_crc_32(uint32_t crc_accum, uint32_t *data_blk_ptr, uint32_t data_blk_size);
void     print_usage(const char *execName);
error_t print_header(const char *FwFilename);
error_t get_fw_info(const char *inFwFilename);
error_t sign_fw(const char *inFwFilename, const char *outFwFilename,
                bool secure, bool extSign, const char *KeyPath,
                bool addKey, const char *pubKeyFilename[], uint8_t imgIdx);
error_t sign_digest(const char *digest, const char *KeyFilename);
error_t verify_fw(const char *FwFilename, const char *KeyFilename, uint8_t imgIdx);
error_t generate_test_firmware(const char *fwFileName, const char *KeyFilename, const char *pubKeyFilename[]);

// command line options
uint8_t verbose = 0;

// Default header values, can be overwritten on the command line
uint8_t noexec    = 0;
uint8_t lastImage = 1;
uint8_t nextImgUnsigned = 0;
uint8_t keyLocation     = DEFAULT_KEY_LOCATION;
uint8_t keyType         = DEFAULT_KEY_TYPE;
uint8_t keyIndex        = 0;
bool gInjectCRCError = 0;
bool gInjectSHAError = 0;
bool gDisableChecks  = 0;

// Help string
static const char USAGE_STR[] =
    "EM Microelectronic Firmware Signing Utility\n"
    "Usage:\n\t%s [<options>]\n\n"
    "Options:\n"
    "  -h             = Print this usage message\n"
    "  -V             = Turn on verbose output\n"
    "  -c             = Finalize a firmware image with CRC (unsigned)\n"
    "  -s             = Sign a firmware image\n"
    "  -f <fwFile>    = Specify the firmware file name to sign/verify\n"
    "  -o <outFile>   = Specify the output firmware file\n"
    "  -a <digest>    = Sign a SHA digest of a firmware image\n"
    "                   <digest> as 256bit hex string\n"
    "  -v             = Verify a firmware image\n"
    "  -k <keyfile>   = Specify the key file name to use to sign/verify firmware\n"
    "  -i             = Get information about the firmware file\n"
    "  -t             = Generate test images from the specified firmware file\n"
    "Signing Options:\n"
    "  --chain        = Contents are in a chain\n"
    "  --keyloc <x>   = Set key location\n"
    "                   0 - OTP\n"
    "                   1 - ROM\n"
    "                   2 - RAM\n"
    "                   3 - FLASH\n"
    "                   4 - KERNEL (Pick RAM or FLASH according to FW Header ImgTypeFlash Image Flag value\n"
    "  --keyid <x>    = Set key index\n"
    "                   0-14 - key index\n"
    "                   15 - No authentication\n"
    "  --pubkey <idx> <pubKeyFile> = Add a public key to firmware image at index idx\n"
    "  --extsign <path> = Use external program to retrieve signature from digest\n"
    "  --niu          = Allow images following this one to be unsigned\n"
    "  --imgIdx <x>   = Sign image x in concatenated image\n"
    "                   Index starts at 0, default is 0\n"
    "  --noexec       = For Flash images, do not execute the image immediately after a hard reset\n"
    "  --exec         = For Flash images, attempt to execute the image immediately after a hard reset \n"
    "  --keytype <x>  = Authentication type\n"
    "                   0 - ECDSA\n"
    ;

int main(int argc, char *argv[])
{
    const char *fwFileName  = NULL;
    const char *outFileName = NULL;
    uint8_t     imageIndex  = ALL_IMAGES;
    task_t      task        = DEFAULT_TASK;
    error_t     ret         = SUCCESS;
    int         i;
    const char *pubKeyFilename[MAX_PUB_KEYS] = {0};
    const char *keyPath                      = NULL;
    uint8_t     pubKeyIndex;
    bool        addKey  = false;
    bool        extSign = false;
    bool        secure  = true;
    const char *digestHex = NULL;

    // Process command line arguments
    for (i = 1; i < argc; i++)
    {
        if (argv[i][0] == '-')
        {
            switch (argv[i][1])
            {
                case 'h':
                    task = USAGE;
                    break;
                case 'V':
                    verbose = 1;
                    break;
                case 'f':
                    if (argc <= (i + 1))
                    {
                        fprintf(stderr, "Not enough arguments to -f\n");
                        return ERR_INVALID_ARGUMENT;
                    }
                    fwFileName = argv[++i];
                    break;
                case 'o':
                    if (argc <= (i + 1))
                    {
                        fprintf(stderr, "Not enough arguments to -o\n");
                        return ERR_INVALID_ARGUMENT;
                    }
                    outFileName = argv[++i];
                    break;
                case 's':
                    task   = SIGN_FW;
                    secure = true;
                    break;
                case 'i':
                    task = INFO;
                    break;
                case 'c':
                    task   = SIGN_FW;
                    secure = false;
                    break;
                case 'a':
                    task = SIGN_DIGEST;
                    if (argc <= (i + 1))
                    {
                        fprintf(stderr, "Not enough arguments to -a\n");
                        return ERR_INVALID_ARGUMENT;
                    }
                    digestHex = argv[++i];
                    break;
                case 'v':
                    task = VERIFY_FW;
                    break;
                case 'k':
                    if (argc <= (i + 1))
                    {
                        fprintf(stderr, "Not enough arguments to -k\n");
                        return ERR_INVALID_ARGUMENT;
                    }
                    keyPath = argv[++i];
                    break;
                case 't':
                    task = TEST_IMAGES;
                    break;
                case '-':
                {
                    // extended args
                    char *cp = argv[i];
                    cp += 2; // skip over the --
                    if (strncmp(cp, "chain", strlen("chain")) == 0)
                    {
                        lastImage = 0;
                    }
                    else if (strncmp(cp, "imgIdx", strlen("imgIdx")) == 0)
                    {
                        imageIndex = atoi(argv[++i]);
                    }
                    else if (strncmp(cp, "noexec", strlen("noexec")) == 0)
                    {
                        noexec = 1;
                    }
                    else if (strncmp(cp, "exec", strlen("exec")) == 0)
                    {
                        noexec = 0;
                    }
                    else if (strncmp(cp, "keyloc", strlen("keyloc")) == 0)
                    {
                        keyLocation = atoi(argv[++i]);
                    }
                    else if (strncmp(cp, "keyid", strlen("keyid")) == 0)
                    {
                        keyIndex = atoi(argv[++i]);
                    }
                    else if (strncmp(cp, "pubkey", strlen("pubkey")) == 0)
                    {
                        if (argc <= (i + 2))
                        {
                            fprintf(stderr, "Not enough arguments to --pubkey\n");
                            return ERR_INVALID_ARGUMENT;
                        }
                        pubKeyIndex = atoi(argv[++i]);
                        if (pubKeyIndex >= MAX_PUB_KEYS)
                        {
                            fprintf(stderr, "Maximum public key index = %d\n", MAX_PUB_KEYS - 1);
                            return ERR_INVALID_ARGUMENT;
                        }
                        pubKeyFilename[pubKeyIndex] = argv[++i];
                        addKey                      = true;
                    }
                    else if (strncmp(cp, "extsign", strlen("extsign")) == 0)
                    {
                        if (argc <= (i + 1))
                        {
                            fprintf(stderr, "Not enough arguments to --extsign\n");
                            return ERR_INVALID_ARGUMENT;
                        }
                        extSign = true;
                        keyPath = argv[++i];
                    }
                    else if (strncmp(cp, "niu", strlen("niu")) == 0)
                    {
                        nextImgUnsigned = 1;
                    }
                    else if (strncmp(cp, "keytype", strlen("keytype")) == 0)
                    {
                        keyType = atoi(argv[++i]);
                    }
                    else
                    {
                        fprintf(stderr, "Unknown option: --%s\n", cp);
                        return ERR_INVALID_ARGUMENT;
                    }
                    break;
                }
                default:
                    fprintf(stderr, "Unknown option: -%c\n", argv[i][1]);
                    return ERR_INVALID_ARGUMENT;

                    break;
            }
        }
    }

    if (verbose)
    {
        for (int i = 0; i < argc; i++)
        {
            printf("%s ", argv[i]);
        }
        printf("\n");
    }

    // Do the task
    switch (task)
    {
        case SIGN_FW:
            ret = sign_fw(fwFileName, outFileName, secure, extSign, keyPath, addKey, pubKeyFilename, imageIndex);
            if (ret)
            {
                fprintf(stderr, "Error signing firmware %s: error=%d: %s\n", fwFileName, ret, error_to_string(ret));
            }
            break;
        case INFO:
            ret = get_fw_info(fwFileName);
            if (ret)
            {
                fprintf(stderr, "Error retrieving firmware info %s: error=%d: %s\n", fwFileName, ret, error_to_string(ret));
            }
            break;
        case SIGN_DIGEST:
            ret = sign_digest(digestHex, keyPath);
            if (ret)
            {
                fprintf(stderr, "Error signing firmware digest: error=%d: %s\n", ret, error_to_string(ret));
            }
            break;

        case VERIFY_FW:
            ret = verify_fw(fwFileName, keyPath, imageIndex);
            if (ret)
            {
                fprintf(stderr, "Error verifying firmware %s: error=%d: %s\n", fwFileName, ret, error_to_string(ret));
            }
            break;
        case TEST_IMAGES:
            ret = generate_test_firmware(fwFileName, keyPath, pubKeyFilename);
            if (ret)
            {
                fprintf(stderr, "Error generate test firmware from %s: error=%d: %s\n", fwFileName, ret, error_to_string(ret));
            }
            break;
        case DEFAULT_TASK:
            if (fwFileName)
            {
                ret = print_header(fwFileName);
            }
            else
            {
                print_usage(argv[0]);
            }
            break;

        case USAGE:
            print_usage(argv[0]);
            break;

        default:
            break;
    }

    return ret;
}

void print_usage(const char *ExecName)
{
    printf(USAGE_STR, ExecName);
}

/* Returns pointer to buffer where file has been read.  fileLen will
   be updated to reflect the length of the file read. */
uint8_t *read_file(const char *filename, int &fileLen, bool padForDMA = false)
{
    int      bytesRead;
    FILE    *file;
    uint8_t *fileBuffer = NULL;
    int      padLen     = 0;

    fileLen = 0;

    if (filename == NULL)
    {
        return NULL;
    }

    file = fopen(filename, "rb");
    if (file == NULL)
    {
        return NULL;
    }

    // Find the file size and allocate a buffer
    fseek(file, 0, SEEK_END);
    fileLen = ftell(file);

    // Allocate enough buffer to hold max DMA padding
    if (padForDMA)
    {
        padLen += DMA_PADDING_NEEDED;
    }

    fileBuffer = (uint8_t *)malloc(fileLen + padLen);
    if (!fileBuffer)
    {
        fileLen = 0;
        return NULL;
    }

    // Read in the firmware file
    fseek(file, 0, SEEK_SET);
    bytesRead = fread(fileBuffer, 1, fileLen, file);
    fclose(file);
    if (bytesRead != fileLen)
    {
        fileLen = 0;
        free(fileBuffer);
        return NULL;
    }

    if (padLen > 0)
    {
        // Clear the padding
        memset(fileBuffer + fileLen, 0, padLen);
    }

    return fileBuffer;
}

// Calculate CRC over len bytes in buffer
uint32_t fileCrc(uint8_t *buffer, uint32_t len)
{
    uint8_t *fileBuf   = buffer;
    uint32_t crc       = CRC_SEED;
    uint32_t bytesLeft = len;
    uint32_t crcLen;

    while (bytesLeft > 0)
    {
        crcLen = (bytesLeft < SHA256_BLOCK_SIZE) ? bytesLeft : SHA256_BLOCK_SIZE;
        crc    = update_crc_32(crc, (uint32_t *)fileBuf, crcLen / 4);
        bytesLeft -= crcLen;
        fileBuf += crcLen;
    }

    return crc;
}

FWHeader_t *seekHdrCopy(FWHeader_t *fwHdr)
{
    FWHeader_t *fwHdrCopy = NULL;
    uint8_t    *fwBuffer  = (uint8_t *)(fwHdr) + sizeof(FWHeader_t);
    uint32_t    hdrCopyOffset;
    uint32_t    hdrOffsetVals[2] = {KERNEL_HDROFFSET_LOC, USER_HDROFFSET_LOC};

    // There is no knowledge whether this is a kernel or user image, so check
    // both header copy offsets
    for (int i = 0; i < 2; i++)
    {
        // Make sure the offset we are going to read is within the image
        if ((hdrOffsetVals[i] + sizeof(uint32_t)) > fwHdr->ImageLen)
        {
            continue;
        }

        hdrCopyOffset = *(uint32_t *)&fwBuffer[hdrOffsetVals[i]];

        // Make sure the offset plus size of header is within the image
        if ((hdrCopyOffset + sizeof(FWHeader_t)) > fwHdr->ImageLen)
        {
            continue;
        }

        fwHdrCopy = (FWHeader_t *)&fwBuffer[hdrCopyOffset];
        // Check if the header copy has been found
        if ((fwHdrCopy->Magic == FW_MAGIC_2B) &&
            (fwHdrCopy->Version == fwHdr->Version) &&
            (fwHdrCopy->ExpVer == fwHdr->ExpVer))
        {
            // Found the header copy
            break;
        }
        fwHdrCopy = NULL;
    }

    if (fwHdrCopy == NULL)
    {
        fprintf(stderr, "Firmware header copy not found\n");
    }

    return fwHdrCopy;
}

FWHeader_t *seekFwHdr(uint8_t *buffer, uint8_t imgIndex, int bufferLen, bool verboseError = true)
{
    int         index = 0;
    FWHeader_t *fwHdr;
    int         hdrPos;

    hdrPos = 0;
    fwHdr  = (FWHeader_t *)&buffer[hdrPos];

    if ((fwHdr->Magic != FW_MAGIC_2B) && (fwHdr->Magic != FW_MAGIC_2C))
    {
        // No fw header
        if (verboseError)
        {
            fprintf(stderr, "No FW header found\n");
        }
        return NULL;
    }

    if ((fwHdr->ImageLen == 0) && (imgIndex == 0))
    {
        // Set the length
        fwHdr->ImageLen = bufferLen - sizeof(FWHeader_t);
    }

    // Find the image at index imgIndex
    while (index != imgIndex)
    {
        if (fwHdr->ImageLen == 0)
        {
            // Attempting to sign a concatenated image,
            // but the fw length in the header is 0.
            if (verboseError)
            {
                fprintf(stderr, "Unsupported option combination\n");
            }
            return NULL;
        }
        hdrPos += sizeof(FWHeader_t) + fwHdr->ImageLen;
        if ((int)(hdrPos + sizeof(FWHeader_t)) > bufferLen)
        {
            // No fw header
            if (verboseError)
            {
                fprintf(stderr, "No FW header found - header position beyond end of image for %s image\n",
                        (imgIndex == KERNEL_IMAGE_IDX) ? "kernel" : "user");
            }
            return NULL;
        }
        fwHdr = (FWHeader_t *)&buffer[hdrPos];
        if ((fwHdr->Magic != FW_MAGIC_2B) && (fwHdr->Magic != FW_MAGIC_2C))
        {
            // No fw header
            if (verboseError)
            {
                fprintf(stderr, "No FW header found - magic invalid for %s image\n",
                        (imgIndex == KERNEL_IMAGE_IDX) ? "kernel" : "user");
            }
            return NULL;
        }

        if (fwHdr->ImageLen == 0)
        {
            // Set the length for this image
            fwHdr->ImageLen = (bufferLen - hdrPos) - sizeof(FWHeader_t);
        }
        index++;
    }

    return fwHdr;
}

void initFwHdr(FWHeader_t *fwHdr, bool secure)
{
    fwHdr->ImageCrc = 0;
    fwHdr->HdrCrc   = 0;
    memset(fwHdr->Sha, 0, SHA256_DIGEST_SIZE);
    memset(&fwHdr->Certificate, 0, sizeof(Cert_t));

    // Set default values (or values from command line)
    fwHdr->ImageFlags.bits.NoExec        = noexec;
    fwHdr->ImageFlags.bits.EndOfImgChain = lastImage;
    fwHdr->ImageFlags.bits.NextImgUnsigned = nextImgUnsigned;
    if (nextImgUnsigned && (fwHdr->Magic == FW_MAGIC_2B))
    {
        fwHdr->ImageFlags.bits.EndOfImgChain = 1;
    }
    fwHdr->ImageFlags.bits.Reserved = 0;

    if (secure)
    {
        if (keyLocation == KEY_LOCATION_KERNEL)
        {
            if (fwHdr->ImageFlags.bits.ImgTypeFlash == 1)
            {
                keyLocation = KEY_LOCATION_FLASH;
            }
            else
            {
                keyLocation = KEY_LOCATION_RAM;
            }
        }
    }
    else
    {
        keyLocation = KEY_NO_AUTHENTICATION;
    }

    fwHdr->KeyFlags.bits.Location = keyLocation;
    fwHdr->KeyFlags.bits.Index    = keyIndex;
    fwHdr->KeyFlags.bits.Type     = keyType;
    fwHdr->KeyFlags.bits.Reserved = 0;
}

void initFwHdrCopy(FWHeader_t *fwHdr, FWHeader_t *fwHdrCopy)
{
    if ((fwHdr->Magic == FW_MAGIC_2B) && (fwHdrCopy != NULL))
    {
        // Copy the header
        memcpy(fwHdrCopy, fwHdr, sizeof(FWHeader_t));
    }
}

void dump_header(FWHeader_t *hdr)
{
    printf("\n");
    printf("FW Header:\n");
    printf("Magic:      0x%04x\n", hdr->Magic);
    printf("ImageFlags: 0x%04x\n", hdr->ImageFlags.value);
    printf("  NoExec: %d\n", hdr->ImageFlags.bits.NoExec);
    printf("  EndOfImgChain: %d\n", hdr->ImageFlags.bits.EndOfImgChain);
    printf("  NextImgUnsigned: %d\n", hdr->ImageFlags.bits.NextImgUnsigned);
    printf("  ImgTypeFlash: %d\n", hdr->ImageFlags.bits.ImgTypeFlash);
    printf("  SSBL: %d\n", hdr->ImageFlags.bits.SSBL);
    printf("  PairWithSSBL: %d\n", hdr->ImageFlags.bits.PairWithSSBL);
    printf("  EncryptionSupported: %d\n", hdr->ImageFlags.bits.EncryptionSupported);
    printf("  Encrypted: %d\n", hdr->ImageFlags.bits.Encrypted);
    printf("KeyFlags:   0x%04x\n", hdr->KeyFlags.value);
    printf("  Location: %d\n", hdr->KeyFlags.bits.Location);
    printf("  Index: %d\n", hdr->KeyFlags.bits.Index);
    printf("  Type: %d\n", hdr->KeyFlags.bits.Type);
    printf("Version:    0x%04x\n", hdr->Version);
    printf("Sha:");
    for (int i = 0; i < SHA256_DIGEST_SIZE; i++)
    {
        if ((i % 16) == 0)
        {
            printf("\n");
        }
        printf("0x%02x ", hdr->Sha[i]);
    }
    printf("\n");
    printf("ECDSA Certificate:");
    for (int i = 0; i < CERT_LEN; i++)
    {
        if ((i % 16) == 0)
        {
            printf("\n");
        }
        printf("0x%02x ", hdr->Certificate.value[i]);
    }
    printf("\n");
    printf("ImageLen:   0x%08x\n", hdr->ImageLen);
    printf("ImageCrc:   0x%08x\n", hdr->ImageCrc);
    printf("KeyOffset:  0x%08x\n", hdr->KeyOffset);
    printf("ExpVer:     0x%08x\n", hdr->ExpVer);
    printf("HdrCrc:     0x%08x\n", hdr->HdrCrc);
    printf("\n");
}

int size_t2u8(size_t val)
{
    return (val <= UINT8_MAX) ? (uint8_t)((size_t)val) : 1;
}

void mySHA256(uint8_t *buffer, uint8_t *shaResult, FWHeader_t *fwHdr)
{
    uint32_t bytesLeft = fwHdr->ImageLen;
    uint8_t *crcBuffer = buffer;

    memset(shaResult, 0, SHA256_DIGEST_SIZE);
    SHA256_Init();

    if (fwHdr->Magic != FW_MAGIC_2B)
    {
        // Post DI02, include the header plus a prepended 4 byte
        // zero pad in the SHA calculation
        FWHeaderBuffer_t fwHdrBuffer;
        uint8_t         *hdrBuf = (uint8_t *)&fwHdrBuffer;

        fwHdrBuffer.zeroPad = 0;
        memcpy(&fwHdrBuffer.hdr, fwHdr, sizeof(FWHeader_t));
        memset(fwHdrBuffer.hdr.Sha, 0, SHA256_DIGEST_SIZE);
        memset(&fwHdrBuffer.hdr.Certificate, 0, sizeof(Cert_t));
        SHA256_Update(hdrBuf);
        SHA256_Update(hdrBuf + SHA256_BLOCK_SIZE);
    }

    while (bytesLeft >= SHA256_BLOCK_SIZE)
    {
        SHA256_Update(crcBuffer);
        bytesLeft -= SHA256_BLOCK_SIZE;
        crcBuffer += SHA256_BLOCK_SIZE;
    }

    // Finish the SHA calculation
    // NOTE: SHA256_Final should be called with max size of
    // SHA256_BLOCK_SIZE-1 bytes
    SHA256_Final(crcBuffer, size_t2u8(bytesLeft), shaResult);
}

error_t fwAddPubKey(KeyList_t *keyList, const char *pubKeyFilename[])
{
    FILE   *pubKeyFile;
    Key_t  *pubKeys;
    uint8_t numPubKeys = 0;
    int     i;
    uint32_t key[sizeof(POINT) / 4] = {0};

    if (keyList->hdr.keyMagic != KEY_MAGIC)
    {
        fprintf(stderr, "Key magic incorrect: keyMagic=0x%x\n", keyList->hdr.keyMagic);
        return ERR_NO_PUBKEY_IN_FW;
    }

    numPubKeys = keyList->hdr.numKeys;
    pubKeys    = keyList->keys;

    // Read in the public key files
    for (i = 0; i < numPubKeys; i++)
    {
        if (pubKeyFilename[i] != NULL)
        {
            pubKeyFile = fopen(pubKeyFilename[i], "rb");
            if (pubKeyFile == NULL)
            {
                fprintf(stderr, "Unable to read key file %s\n", pubKeyFilename[i]);
                return ERR_NO_PUBKEY_FILE;
            }
            fread(&pubKeys[i].key, 1, 40, pubKeyFile);
            pubKeys[i].crc = 0xffffffff;
            memcpy(key, &pubKeys[i].key, sizeof(POINT));
            pubKeys[i].crc = update_crc_32(pubKeys[i].crc, key, sizeof(POINT) / 4);
            fclose(pubKeyFile);
        }
    }

    return SUCCESS;
}

error_t fwSignExternal(uint8_t *shaDigest, const char *extSignPath, uint8_t *signature_char)
{
    size_t i, num_chars;
    char   shaDigestHex[SHA256_DIGEST_SIZE * 2 + 1];
    char   signature_string[2 * sizeof(PointCertificate) + 2]; // allocate space for new-line and null-termination after the certificate string
    char   extSignCommand[1024];
    char  *endPtr = NULL;

    // Assemble hex string of SHA digest
    for (i = 0; i < SHA256_DIGEST_SIZE; i++)
    {
        // every sprintf call overwrites 3 bytes.
        // the \0 is overwritten with each loop except for the last
        sprintf(&shaDigestHex[2 * i], "%02X", shaDigest[i]);
    }

    // Assemble command line

    // check extSignPath length for maximum here!
    if (strlen(extSignPath) > (sizeof(extSignCommand) - sizeof(shaDigestHex) - 1))
    {
        fprintf(stderr, "Command %s exceeds length limit of %lu chars\n", extSignCommand, sizeof(extSignCommand));
        return ERR_EXT_SIGNING;
    }
    strcpy(extSignCommand, extSignPath);
    strcat(extSignCommand, " ");
    strcat(extSignCommand, shaDigestHex);

    // call the external sign program and parse the return value
    FILE *handle = popen(extSignCommand, "r");

    if (handle == NULL)
    {
        fprintf(stderr, "Cannot execute command %s\n", extSignCommand);
        return ERR_EXT_SIGNING;
    }

    if (fgets(signature_string, sizeof(signature_string), handle))
    {
        num_chars = strlen(signature_string);
        if ((num_chars - 1) != 2 * sizeof(PointCertificate)) // reduce num_chars by 1 to exclude new-line char, (2*sizeof) as it is 2 chars per byte
        {
            fprintf(stderr, "Length of received certificate too short: %zu chars, certificate: %s\n", (num_chars - 1), signature_string);
            return ERR_EXT_SIGNING;
        }
        else if (signature_string[(num_chars - 1)] != '\n') // reduce num_chars by 1 to retrieve last char of the string (as array starts with 0)
        {
            fprintf(stderr, "Length of received certificate too long: >=%zu chars, certificate: %s\n", num_chars, signature_string);
            return ERR_EXT_SIGNING;
        }
        // convert the output from command into hex
        for (i = 0; i < sizeof(PointCertificate); i++)
        {
            char buf[3]       = {signature_string[2 * i], signature_string[(2 * i) + 1], 0};
            signature_char[i] = (uint8_t)strtoul(buf, &endPtr, 16);
            if (*endPtr != '\0') // endPtr is only '\0' if all chars have been successfully parsed
            {
                fprintf(stderr, "Error reading certificate, non hex char found in byte %zu ( %c ), certificate: %s\n", i + 1, *endPtr, signature_string);
                return ERR_EXT_SIGNING;
            }
        }
    }
    else
    {
        fprintf(stderr, "Error reading certificate\n");
        return ERR_EXT_SIGNING;
    }

    pclose(handle);

    if (verbose)
    {
        printf("Read signature: %s", signature_char);
    }
    return SUCCESS;
}

const char unsigned_all_str[]         = "UNSIGNED_ALL";
const char signed_all_str[]           = "SIGNED_ALL";
const char signed_unsigned_str[]      = "SIGNED_UNSIGNED";
const char ram_str[]                  = "RAM";
const char flash_str[]                = "FLASH";
const char plaintext_str[]            = "PLAINTEXT";
const char encrypted_str[]            = "ENCRYPTED";
const char encryption_supported_str[] = "ENCRYPTION_SUPPORTED";
const char encryption_invlaid_str[]   = "ENCRYPTION_INVALID";

error_t get_fw_info(const char *inFwFilename)
{
    int      fileSize;
    uint8_t *fileBuffer;
    error_t  ret = SUCCESS;

    FWHeader_t *fwHdrKernel;
    FWHeader_t *fwHdrUser = NULL;

    int         numImages = 0;
    bool        chained   = false;
    const char *securityType;
    const char *imageType;
    const char *encryption_type;

    // Read the firmware file into a buffer
    fileBuffer = read_file(inFwFilename, fileSize);
    if (fileBuffer == NULL)
    {
        return ERR_FILE_READ;
    }

    // Find the kernel firmware header
    if ((fwHdrKernel = seekFwHdr(fileBuffer, KERNEL_IMAGE_IDX, fileSize)) == NULL)
    {
        // Can't find the first header, give up
        free(fileBuffer);
        return ERR_NO_FW_HEADER;
    }

    numImages++;

    /* Set the Image type  */
    if (fwHdrKernel->ImageFlags.bits.ImgTypeFlash == 1)
    {
        imageType = flash_str;
    }
    else
    {
        imageType = ram_str;
    }

    /* Determine the security type and chaining */
    if ((fwHdrKernel->Magic == FW_MAGIC_2B) &&
        (!fwHdrKernel->ImageFlags.bits.EndOfImgChain || fwHdrKernel->ImageFlags.bits.NextImgUnsigned))
    {
        // This kernel is meant to be chained.
        if (fwHdrKernel->KeyFlags.bits.Location == KEY_NO_AUTHENTICATION)
        {
            securityType = unsigned_all_str;
            if (fwHdrKernel->ImageFlags.bits.EndOfImgChain)
            {
                chained = false;
            }
            else
            {
                chained = true;
            }
        }
        else if (fwHdrKernel->ImageFlags.bits.EndOfImgChain && fwHdrKernel->ImageFlags.bits.NextImgUnsigned)
        {
            if (fwHdrKernel->ImageFlags.bits.NextImgUnsigned)
            {
                securityType = signed_unsigned_str;
                chained      = true;
            }
            else
            {
                securityType = signed_all_str;
                chained      = false;
            }
        }
        else if ((!fwHdrKernel->ImageFlags.bits.EndOfImgChain) && (fwHdrKernel->KeyFlags.bits.Location != KEY_NO_AUTHENTICATION))
        {
            chained      = true;
            securityType = signed_all_str;
        }
    }
    else if ((fwHdrKernel->Magic != FW_MAGIC_2B) && !fwHdrKernel->ImageFlags.bits.EndOfImgChain)
    {
        // Chained image for DI04 or later
        chained = true;
        if (fwHdrKernel->KeyFlags.bits.Location == KEY_NO_AUTHENTICATION)
        {
            securityType = unsigned_all_str;
        }
        else if (fwHdrKernel->ImageFlags.bits.NextImgUnsigned)
        {
            securityType = signed_unsigned_str;
        }
        else
        {
            securityType = signed_all_str;
        }
    }
    else
    {
        chained = false;
        // This a stand alone kernel application
        if (fwHdrKernel->KeyFlags.bits.Location == KEY_NO_AUTHENTICATION)
        {
            securityType = unsigned_all_str;
        }
        else
        {
            securityType = signed_all_str;
        }
    }

    /* Determine if additional images exist, when chained. */
    if (chained)
    {
        // Find the user firmware header
        if ((fwHdrUser = seekFwHdr(fileBuffer, USER_IMAGE_IDX, fileSize, false)) != NULL)
        {
            numImages++;
        }
    }

    /* Determine the encryption type */
    if (fwHdrUser)
    {
        // User and kernel payload found.
        if (fwHdrUser->ImageFlags.bits.EncryptionSupported)
        {
            // Bit should never be set.
            encryption_type = encryption_invlaid_str;
        }
        else if ((fwHdrKernel->ImageFlags.bits.EncryptionSupported) &&
                 (fwHdrUser->ImageFlags.bits.Encrypted))
        {
            // Encryption supported and image encrypted.
            encryption_type = encrypted_str;
        }
        else if ((fwHdrKernel->ImageFlags.bits.EncryptionSupported) &&
                 (!fwHdrUser->ImageFlags.bits.Encrypted))
        {
            // Encryption supported but image not encrypted.
            encryption_type = encryption_supported_str;
        }
        else if ((!fwHdrKernel->ImageFlags.bits.EncryptionSupported) &&
                 (!fwHdrUser->ImageFlags.bits.Encrypted))
        {
            // Encryption not supported and image not encrypted.
            encryption_type = plaintext_str;
        }
        else if ((!fwHdrKernel->ImageFlags.bits.EncryptionSupported) &&
                 (fwHdrUser->ImageFlags.bits.Encrypted))
        {
            // Encryption not supported but image encrypted.
            encryption_type = encryption_invlaid_str;
        }
    }
    else if (fwHdrKernel->ImageFlags.bits.EncryptionSupported)
    {
        // Only kernel payload found and it supports encryption
        encryption_type = encryption_supported_str;
    }
    else
    {
        // Only kernel payload found and it does not support encryption.
        encryption_type = plaintext_str;
    }

    printf("%d,%s,%s,%s,%s\n", numImages, chained ? "CHAINED" : "SINGLE", securityType, imageType, encryption_type);

    free(fileBuffer);
    return ret;
}


error_t sign_fw(const char *inFwFilename, const char *outFwFilename,
                bool secure, bool extSign, const char *KeyPath,
                bool addKey, const char *pubKeyFilename[], uint8_t imgIdx)
{
    FILE    *fwOutFile;
    uint8_t *fileBuffer;
    uint8_t *fwBuffer;
    int      fileSize;
    int      bytes;
    uint8_t  imgToSign = imgIdx;
    uint8_t shaDigest[SHA256_DIGEST_SIZE];
    Cert_t  signature;
    error_t ret = SUCCESS;
    FILE    *keyFile;
    uint32_t privateKey[SCALAR_SIZE];

    FWHeader_t *fwHdr;
    FWHeader_t *fwHdrCopy   = NULL;
    FWHeader_t *kernelFwHdr = NULL;

    if (ALL_IMAGES == imgToSign)
    {
        fprintf(stderr, "Specify image to sign using the --imgIdx option\n");
        return ERR_INVALID_ARGUMENT;
    }

    // Read the firmware file into a buffer
    fileBuffer = read_file(inFwFilename, fileSize, true);
    if (fileBuffer == NULL)
    {
        return ERR_FILE_READ;
    }

    // Find the image/header to sign
    if ((fwHdr = seekFwHdr(fileBuffer, imgToSign, fileSize)) == NULL)
    {
        free(fileBuffer);
        return ERR_NO_FW_HEADER;
    }

    // Set up the firmware headers
    initFwHdr(fwHdr, secure);

    if (fwHdr->Magic == FW_MAGIC_2B)
    {
        if ((fwHdrCopy = seekHdrCopy(fwHdr)) == NULL)
        {
            free(fileBuffer);
            return ERR_NO_FW_HEADER;
        }
    }

    // Signing the user image, do a few sanity checks
    if (USER_IMAGE_IDX == imgToSign)
    {
        uint32_t kernelLen;
        uint32_t totalLen;

        // Find the kernel header
        if ((kernelFwHdr = seekFwHdr(fileBuffer, KERNEL_IMAGE_IDX, fileSize)) == NULL)
        {
            free(fileBuffer);
            return ERR_NO_FW_HEADER;
        }

        kernelLen = kernelFwHdr->ImageLen;
        totalLen  = kernelLen + sizeof(FWHeader_t) + fwHdr->ImageLen;

        // Check for maximum firmware size
        if (totalLen > (kernelFwHdr->maxFwSizeKB * KB))
        {
            printf("ERROR: Size of firmware %s exceeds maximum firmware size\n", inFwFilename);
            printf("       Max firmware size: 0x%x\n", kernelFwHdr->maxFwSizeKB * KB);
            printf("       Firmware size:     0x%x\n", totalLen);
            free(fileBuffer);
            return ERR_IMAGE_SIZE;
        }

        // For RAM SIGNED_UNSIGNED RAM images, check for possible firmware upload error
        if (!fwHdr->ImageFlags.bits.ImgTypeFlash && !secure &&
            kernelFwHdr->ImageFlags.bits.EndOfImgChain && kernelFwHdr->ImageFlags.bits.NextImgUnsigned)
        {
            // When a SIGNED_UNSIGNED image has a user image that spans more than 2 RAM banks
            // there is a possibility that the last needed RAM bank may not be turned on during
            // firmware upload, causing a firmware verification error.
            uint32_t kernelRamBank = ((kernelLen - FIXED_RAM_BANK_SIZE) + (OPTIONAL_RAM_BANK_SIZE - 1)) / OPTIONAL_RAM_BANK_SIZE;
            uint32_t userRamBank   = ((totalLen - FIXED_RAM_BANK_SIZE) + (OPTIONAL_RAM_BANK_SIZE - 1)) / OPTIONAL_RAM_BANK_SIZE;
            if ((userRamBank - kernelRamBank) > 1)
            {
                printf("WARNING: The user image for %s exceeds the limit of this SDK.\n", inFwFilename);
                printf("         Please request an extended SDK from Bosch, allowing a larger user image.\n");
                printf("         Last kernel RAM bank=%d, last user RAM bank=%d (kernel length=0x%x, total length=0x%x)\n", kernelRamBank, userRamBank, kernelLen, totalLen);
            }
        }

        if (fwHdr->ImageFlags.bits.PairWithSSBL != kernelFwHdr->ImageFlags.bits.PairWithSSBL)
        {
            fprintf(stderr, "ERROR: All payloads must have the same PairWithSSBL flag\n");
            free(fileBuffer);
            return ERR_SSBL_MISMATCH;
        }
    }

    if (fwHdr->Magic == FW_MAGIC_2B)
    {
        fwHdrCopy = seekHdrCopy(fwHdr);
        initFwHdrCopy(fwHdr, fwHdrCopy);

        // DMA workaround for DI02/DI03
        if (fwHdr->ImageFlags.bits.EndOfImgChain && !fwHdr->ImageFlags.bits.NextImgUnsigned) // last image
        {
            uint32_t totalLen;
            uint32_t dmaPadding;

            totalLen = fileSize - sizeof(FWHeader_t);

            // Total image length must meet DMA workaround requirements (at least 128 bytes over a DMA boundary)
            dmaPadding = totalLen % DMA_TRANSACTION_SIZE; // Current overflow past DMA boundary
            if (dmaPadding && dmaPadding < DMA_PADDING_NEEDED)
            {
                dmaPadding = DMA_PADDING_NEEDED - dmaPadding;
                fileSize += dmaPadding;
                fwHdr->ImageLen += dmaPadding;
                fwHdrCopy->ImageLen = fwHdr->ImageLen;
            }
        }
    }

    if (verbose)
    {
        printf("Signing image at index %d\n", imgToSign);
    }

    fwBuffer = (uint8_t *)(fwHdr) + sizeof(FWHeader_t);

    if (addKey)
    {
        KeyList_t *pubKeys = (KeyList_t *)(fwBuffer + fwHdr->KeyOffset);
        if ((ret = fwAddPubKey(pubKeys, pubKeyFilename)) != SUCCESS)
        {
            free(fileBuffer);
            return ret;
        }
    }



    // Calculate the crc of the fw image
    fwHdr->ImageCrc = fileCrc(fwBuffer, fwHdr->ImageLen);
    if (gInjectCRCError)
    {
        // Simulate a full image CRC error by inverting the expected crc.
        fwHdr->ImageCrc ^= 0xFFFFFFFF;
    }

    if (fwHdr->Magic != FW_MAGIC_2B)
    {
        // For DI03 and later, calculate the fw header CRC now (with SHA and ECDSA 0'd)
        fwHdr->HdrCrc = update_crc_32(CRC_SEED, (uint32_t *)fwHdr, offsetof(FWHeader_t, HdrCrc) / 4);
    }

    if (secure)
    {
        // Calculate the SHA of the fw image
        mySHA256(fwBuffer, shaDigest, fwHdr);

        if (!extSign)
        {
            // Read in the private key file
            if (KeyPath == NULL)
            {
                free(fileBuffer);
                return ERR_NO_PRIVATE_KEY_FILE;
            }

            if (!extSign)
            {
                keyFile = fopen(KeyPath, "rb");
                if (keyFile == NULL)
                {
                    free(fileBuffer);
                    return ERR_NO_PRIVATE_KEY_FILE;
                }
                fread(privateKey, sizeof(privateKey), 1, keyFile);
                fclose(keyFile);
            }
            // Calculate the ECDSA signature
            if (ECDSA_Sign(privateKey, shaDigest, &signature.ecdsaCert) != SW_OK)
            {
                free(fileBuffer);
                return ERR_ECDSA;
            }
        }
        else
        {
            // retrieve signature via call of external program
            if ((ret = fwSignExternal(shaDigest, KeyPath, signature.value)) != SUCCESS)
            {
                free(fileBuffer);
                return ret;
            }
        }

        memcpy(fwHdr->Sha, shaDigest, SHA256_DIGEST_SIZE);
        if (gInjectSHAError)
        {
            // Simulate a SHA error by providing garbage data.
            memset(fwHdr->Sha, 0xAABBCCDD, SHA256_DIGEST_SIZE);
        }
        memcpy(&fwHdr->Certificate, &signature.ecdsaCert, sizeof(fwHdr->Certificate));

        if (fwHdr->Magic == FW_MAGIC_2B)
        {
            // For DI02, calculate the fw header CRC now (with SHA and ECDSA filled in)
            fwHdr->HdrCrc = update_crc_32(CRC_SEED, (uint32_t *)fwHdr, offsetof(FWHeader_t, HdrCrc) / 4);
        }
    }
    else
    {
        fwHdr->HdrCrc = update_crc_32(CRC_SEED, (uint32_t *)fwHdr, offsetof(FWHeader_t, HdrCrc) / 4);
    }

    if (verbose)
    {
        printf("Main header:");
        dump_header(fwHdr);

        if ((fwHdr->Magic == FW_MAGIC_2B) && (fwHdrCopy != NULL))
        {
            printf("Copy header:");
            dump_header(fwHdrCopy);
        }
    }

    // Finally write the output file
    fwOutFile = fopen(outFwFilename, "w+b");
    if (fwOutFile == NULL)
    {
        free(fileBuffer);
        return ERR_NO_FW_FILE;
    }

    if ((bytes = fwrite(fileBuffer, 1, fileSize, fwOutFile)) != fileSize)
    {
        fprintf(stderr, "Failed to write file %s\n", outFwFilename);
        free(fileBuffer);
        return ERR_FILE_WRITE;
    }

    free(fileBuffer);
    fclose(fwOutFile);
    return SUCCESS;
}

error_t sign_digest(const char *digestHex, const char *KeyFilename)
{
    FILE            *keyFile;
    uint8_t          shaDigest[SHA256_DIGEST_SIZE];
    char             signatureString[2 * sizeof(PointCertificate) + 1];
    uint32_t         privateKey[SCALAR_SIZE];
    PointCertificate signature;
    size_t           i;
    char            *endPtr = NULL;

    // Convert digest input string to binary format
    if (strlen(digestHex) != sizeof(shaDigest) * 2) // hex string has 2 characters per byte
    {
        fprintf(stderr, "Invalid format of digest string (should be 64 hex characters)\n");
        return ERR_DIGEST_FORMAT;
    }

    for (i = 0; i < sizeof(shaDigest); i++)
    {
        char buf[3]  = {digestHex[2 * i], digestHex[(2 * i) + 1], 0}; // temporary string for 1 hex byte including terminating null
        shaDigest[i] = (uint8_t)strtoul(buf, &endPtr, 16);
        if (*endPtr != '\0') // endPtr is only '\0' if all chars have been successfully parsed
        {
            fprintf(stderr, "Error reading in digest, non hex char found in byte %zu ( %c ), digest: %s\n", i + 1, *endPtr, digestHex);
            return ERR_DIGEST_FORMAT;
        }
    }

    if (verbose)
    {
        printf("Read in digest: ");
        for (i = 0; i < sizeof(shaDigest); i++)
        {
            printf("%02X,", shaDigest[i]);
        }
        printf("\n");
    }

    // Read in the private key file
    if (KeyFilename == NULL)
    {
        return ERR_NO_PRIVATE_KEY_FILE;
    }

    keyFile = fopen(KeyFilename, "rb");
    if (keyFile == NULL)
    {
        return ERR_NO_PRIVATE_KEY_FILE;
    }
    fread(privateKey, sizeof(privateKey), 1, keyFile);
    fclose(keyFile);

    // Calculate the ECDSA signature
    if (ECDSA_Sign(privateKey, shaDigest, (PointCertificate *)&signature) != SW_OK)
    {
        return ERR_ECDSA;
    }

    unsigned char *signature_char = (unsigned char *)&signature; // dirty object type casting...

    // Print point certificate (should be the signature)
    for (i = 0; i < sizeof(PointCertificate); i++)
    {
        sprintf(&signatureString[2 * i], "%02X", signature_char[i]);
    }
    printf("%s\n", signatureString);

    return SUCCESS;
}

// Check for valid firmware image
// - Image CRC, SHA, ECDSA signature, public key CRCs, Header CRC
error_t check_image(FWHeader_t *fwHdr, uint8_t *fwBuffer, POINT *publicKey)
{
    uint32_t key[sizeof(POINT) / 4] = {0};
    if (verbose)
    {
        printf("Firmware header\n");
        dump_header(fwHdr);
    }

    // Check the crc of the fw image
    uint32_t crc = CRC_SEED;
    crc          = fileCrc(fwBuffer, fwHdr->ImageLen);
    if (crc != fwHdr->ImageCrc)
    {
        // Image CRC mismatch
        fprintf(stderr, "Image verification error in image CRC:\n");
        fprintf(stderr, "Reported:   0x%08x\n", fwHdr->ImageCrc);
        fprintf(stderr, "Calculated: 0x%08x\n", crc);
        fprintf(stderr, "\n");
        return ERR_IMAGE_CRC;
    }

    if (fwHdr->KeyFlags.bits.Location != KEY_NO_AUTHENTICATION)
    {
        // Calculate the SHA
        uint8_t shaDigest[SHA256_DIGEST_SIZE];

        mySHA256(fwBuffer, shaDigest, fwHdr);
        if (memcmp(shaDigest, fwHdr->Sha, sizeof(shaDigest)) != 0)
        {
            fprintf(stderr, "Image verification error in image SHA:\n");
            fprintf(stderr, "Reported:");
            for (size_t i = 0; i < SHA256_DIGEST_SIZE; i++)
            {
                if (0 == (i % 16))
                {
                    fprintf(stderr, "\n");
                }
                fprintf(stderr, "0x%02x ", fwHdr->Sha[i]);
            }
            fprintf(stderr, "\n");
            fprintf(stderr, "Calculated:");
            for (size_t i = 0; i < SHA256_DIGEST_SIZE; i++)
            {
                if (0 == (i % 16))
                {
                    fprintf(stderr, "\n");
                }
                fprintf(stderr, "0x%02x ", shaDigest[i]);
            }
            fprintf(stderr, "\n\n");
            return ERR_SHA;
        }

        if (publicKey == NULL)
        {
            fprintf(stderr, "Image verification error - no public key specified\n\n");
            return ERR_NO_PUBKEY_FILE;
        }

        // Validate the signature
        if (ECDSA_Verify(&fwHdr->Certificate.ecdsaCert, publicKey, shaDigest) != SW_OK)
        {
            fprintf(stderr, "Image verification error in image ECDSA\n\n");
            return ERR_ECDSA;
        }
    }
    else
    {
        if (publicKey != NULL)
        {
            fprintf(stderr, "Image verification error - public key specified but payload is unsigned\n\n");
            return ERR_IMAGE_VERIFICATION;
        }

        fprintf(stderr, "Payload is unsigned and no public key specified\n\n");
    }

    // Check the CRC for any included public keys
    if (fwHdr->KeyOffset != 0)
    {
        KeyList_t *keyList    = (KeyList_t *)(fwBuffer + fwHdr->KeyOffset);
        uint8_t    numPubKeys = keyList->hdr.numKeys;
        Key_t     *pubKeys    = keyList->keys;
        for (int i = 0; i < numPubKeys; i++)
        {
            if (pubKeys[i].crc != 0)
            {
                crc = CRC_SEED;
                memcpy(key, &pubKeys[i].key, sizeof(POINT));
                crc = update_crc_32(crc, key, sizeof(POINT) / 4);
                if (crc != pubKeys[i].crc)
                {
                    fprintf(stderr, "Image verification error in embedded public key CRC %d\n", i);
                    fprintf(stderr, "Reported   0x%08x\n", pubKeys[i].crc);
                    fprintf(stderr, "Calculated 0x%08x\n", crc);
                    fprintf(stderr, "\n");
                    return ERR_KEY_CRC;
                }
            }
        }
    }

    // Check the header CRC
    if (fwHdr->Magic != FW_MAGIC_2B)
    {
        memset(&fwHdr->Certificate, 0, sizeof(Cert_t));
        memset(fwHdr->Sha, 0, SHA256_DIGEST_SIZE);
    }

    crc = CRC_SEED;
    crc = update_crc_32(crc, (uint32_t *)fwHdr, offsetof(FWHeader_t, HdrCrc) / 4);
    if (crc != fwHdr->HdrCrc)
    {
        // Header CRC mismatch
        fprintf(stderr, "Image verification error in firmware header CRC\n");
        fprintf(stderr, "Reported   0x%08x\n", fwHdr->HdrCrc);
        fprintf(stderr, "Calculated 0x%08x\n", crc);
        fprintf(stderr, "\n");
        return ERR_HDR_CRC;
    }

    fprintf(stderr, "Image verification successful\n\n");
    return SUCCESS;
}

bool isChained(FWHeader_t *fwHdr)
{
    if (FW_MAGIC_2B == fwHdr->Magic)
    {
        return (!fwHdr->ImageFlags.bits.EndOfImgChain ||
                fwHdr->ImageFlags.bits.NextImgUnsigned);
    }
    else
    {
        return (!fwHdr->ImageFlags.bits.EndOfImgChain);
    }
}

error_t verify_fw(const char *FwFilename, const char *KeyFilename, uint8_t imgIdx)
{
    uint8_t    *fileBuffer;
    int         fileSize;
    error_t     ret         = SUCCESS;
    bool        halt        = false;
    uint8_t     imgToVerify = imgIdx;
    FWHeader_t *fwHdr;
    uint8_t    *fwBuffer;
    uint8_t     prevRamBank = 0;
    int         totalLen    = 0;

    // Read the firmware file into a buffer
    fileBuffer = read_file(FwFilename, fileSize);
    if (NULL == fileBuffer)
    {
        fprintf(stderr, "Verification error: Unable to read input file %s\n", FwFilename);
        return ERR_FILE_READ;
    }

    // Read the key file into a buffer
    int        keySize;
    KeyList_t *keyList         = NULL;
    uint8_t   *keyBuffer       = NULL;
    POINT     *verificationKey = NULL;
    POINT      keyTemp;

    if (NULL != KeyFilename)
    {
        keyBuffer       = read_file(KeyFilename, keySize);
        verificationKey = (POINT *)keyBuffer;
    }

    if (ALL_IMAGES == imgToVerify)
    {
        // Start at the beginning and check all images
        imgToVerify = 0;
    }
    else
    {
        // Check a single image and then stop
        halt = true;
    }

    do
    {
        fprintf(stderr, "Verifying image at index %d:\n", imgToVerify);

        // Find the header
        if (NULL != (fwHdr = seekFwHdr(fileBuffer, imgToVerify, fileSize)))
        {
            fwBuffer = (uint8_t *)(fwHdr) + sizeof(FWHeader_t);

            // Keep a tally of the total length
            totalLen += sizeof(FWHeader_t) + fwHdr->ImageLen;

            // For RAM SIGNED_UNSIGNED RAM images, check for possible firmware upload error
            if (!fwHdr->ImageFlags.bits.ImgTypeFlash)
            {
                // When a SIGNED_UNSIGNED image has a user image that spans more than 2 RAM banks
                // there is a possibility that the last needed RAM bank may not be turned on during
                // firmware upload, causing a firmware verification error.
                if (0 != prevRamBank)
                {
                    uint8_t currRamBank;
                    currRamBank = (((totalLen - sizeof(FWHeader_t)) - FIXED_RAM_BANK_SIZE) + (OPTIONAL_RAM_BANK_SIZE - 1)) / OPTIONAL_RAM_BANK_SIZE;
                    if ((currRamBank - prevRamBank) > 1)
                    {
                        uint32_t endAddr   = CODE_RAM_START - sizeof(FWHeader_t) + totalLen;
                        uint32_t startAddr = (endAddr - fwHdr->ImageLen) - sizeof(FWHeader_t);
                        fprintf(stderr, "\tChained image verification error: Image spans too many RAM banks\n");
                        fprintf(stderr, "\tFirst RAM bank=%d, Last RAM bank=%d\n", prevRamBank, currRamBank);
                        fprintf(stderr, "\tImage start address=0x%x, Image end address=0x%x\n", startAddr, endAddr);
                        ret = ERR_IMAGE_SIZE;
                    }
                }
                else if (fwHdr->ImageFlags.bits.EndOfImgChain && fwHdr->ImageFlags.bits.NextImgUnsigned)
                {
                    prevRamBank = (((totalLen - sizeof(FWHeader_t)) - FIXED_RAM_BANK_SIZE) + (OPTIONAL_RAM_BANK_SIZE - 1)) / OPTIONAL_RAM_BANK_SIZE;
                }
            }

            // If the key location points to the RAM/FLASH kernel image, get it from the current fw image.
            // Otherwise try the passed in key.
            if (KEY_LOCATION_OTP == fwHdr->KeyFlags.bits.Location)
            {
                // ROM key must be passed in on the command line
                verificationKey = (POINT *)keyBuffer;
            }
            else if ((keyList != NULL) &&
                     ((!fwHdr->ImageFlags.bits.ImgTypeFlash &&
                       (KEY_LOCATION_RAM == fwHdr->KeyFlags.bits.Location)) ||
                      (fwHdr->ImageFlags.bits.ImgTypeFlash &&
                       (KEY_LOCATION_FLASH == fwHdr->KeyFlags.bits.Location))))
            {
                memcpy(&keyTemp, &keyList->keys[fwHdr->KeyFlags.bits.Index].key, sizeof(POINT));
                verificationKey = &keyTemp;

            }

            if (SUCCESS != check_image(fwHdr, fwBuffer, verificationKey))
            {
                ret = ERR_IMAGE_VERIFICATION;
            }
            if (!halt && isChained(fwHdr))
            {
                // Go to the next image
                imgToVerify++;
                // Check if there are keys in this image
                keyList = (KeyList_t *)(fwBuffer + fwHdr->KeyOffset);
            }
            else
            {
                // All done
                halt = true;
            }
        }
        else
        {
            fprintf(stderr, "Chained image verification error: Unable to find header for image %d\n", imgToVerify);
            halt = true;
            ret  = ERR_NO_FW_HEADER;
        }
    } while (!halt);

    // Extra checks when verifying the entire image
    if ((ALL_IMAGES == imgIdx) && (NULL != fwHdr))
    {
        // Total length according to headers vs. file size
        if (totalLen != fileSize)
        {
            fprintf(stderr, "Chained image verification error: File size does not match headers\n");
            fprintf(stderr, "Header image length: 0x%x\n", totalLen);
            fprintf(stderr, "File size:           0x%x\n", fileSize);
            ret = ERR_IMAGE_SIZE;
        }

        // Remaining test are on size minus the first header
        totalLen -= sizeof(FWHeader_t);

        // Check for maximum firmware size
        if (totalLen > (int)(fwHdr->maxFwSizeKB * KB))
        {
            fprintf(stderr, "Chained image verification error: Size of firmware exceeds maximum firmware size\n");
            fprintf(stderr, "Note: Excludes size of first firmware header\n");
            fprintf(stderr, "Image length:   0x%x\n", totalLen);
            fprintf(stderr, "Maximum length: 0x%x\n", fwHdr->maxFwSizeKB * KB);
            ret = ERR_IMAGE_SIZE;
        }

        // DMA workaround
        if (fwHdr->Magic == FW_MAGIC_2B)
        {
            uint32_t dmaPadding = 0;
            // Total image length must meet DMA workaround requirements (at least 128 bytes over a DMA boundary)
            dmaPadding = totalLen % DMA_TRANSACTION_SIZE; // Find current overflow past DMA boundary
            if (dmaPadding && dmaPadding < DMA_PADDING_NEEDED)
            {
                fprintf(stderr, "Chained image verification error: Insufficient DMA padding\n");
                fprintf(stderr, "Actual DMA padding:   0x%x\n", dmaPadding);
                fprintf(stderr, "Required DMA padding: 0x%x\n", DMA_PADDING_NEEDED);
                ret = ERR_IMAGE_SIZE;
            }
        }
    }

    free(fileBuffer);
    free(keyBuffer);

    return ret;
}

error_t print_header(const char *FwFilename)
{
    verbose = true;
    error_t ret = verify_fw(FwFilename, NULL, ALL_IMAGES);
    if (ret)
    {
        fprintf(stderr, "Error verifying firmware %s: error=%d: %s\n", FwFilename, ret, error_to_string(ret));
    }
    return ret;
}

typedef enum
{
    UNSIGNED_VALID,     /* The image is unsigned and has a valid crc */
    UNSIGNED_CRC_ERROR, /* The image is unsigned with an invalid crc */
    SIGNED_VALID,       /* The image is signed with a valid sha/ecdsa */
    SIGNED_SHA_ERROR,   /* the image is signed with an invalid sha/ecdsa */
} signature_status_t;

/* Firmware Image Types */
signature_status_t gKernelSignatureOptions[] =
{
    UNSIGNED_VALID,
    UNSIGNED_CRC_ERROR,
    SIGNED_VALID,
    SIGNED_SHA_ERROR,
};

signature_status_t gUserSignatureOptions[] =
{
    UNSIGNED_VALID,
    UNSIGNED_CRC_ERROR,
    SIGNED_VALID,
    SIGNED_SHA_ERROR,
};

static const char *getSignatureString(signature_status_t sig)
{
    switch (sig)
    {
        case UNSIGNED_VALID:
            return "Uv";
        case UNSIGNED_CRC_ERROR:
            return "Uc";
        case SIGNED_VALID:
            return "Sv";
        case SIGNED_SHA_ERROR:
            return "Ss";
    }

    return "unknown";
}

#define ARRAY_ELEMENTS(__x__) (sizeof(__x__) / sizeof(__x__[0]))
error_t generate_test_firmware(const char *fwFileName, const char *KeyFilename, const char *pubKeyFilename[])
{
    FWHeader_t *fwHdr;
    FWHeader_t *fwHdrUser;
    int         fileSize;
    uint8_t    *fileBuffer;

    // Allow illegal firmware combinations.
    gDisableChecks = true;

    // Read the firmware file into a buffer
    fileBuffer = read_file(fwFileName, fileSize);
    if (fileBuffer == NULL)
    {
        return ERR_FILE_READ;
    }

    // Find the kernel header
    if ((fwHdr = seekFwHdr(fileBuffer, KERNEL_IMAGE_IDX, fileSize)) == NULL)
    {
        free(fileBuffer);
        return ERR_NO_FW_HEADER;
    }

    // Find the user header
    if ((fwHdrUser = seekFwHdr(fileBuffer, USER_IMAGE_IDX, fileSize)) == NULL)
    {
        free(fileBuffer);
        return ERR_NO_FW_HEADER;
    }

    error_t status = SUCCESS;
    for (unsigned int kernel_sig_idx = 0; kernel_sig_idx < ARRAY_ELEMENTS(gKernelSignatureOptions); kernel_sig_idx++)
    {
        signature_status_t kernel_sig     = gKernelSignatureOptions[kernel_sig_idx];
        const char        *kernel_sig_str = getSignatureString(kernel_sig);

        for (unsigned int user_sig_idx = 0; user_sig_idx < ARRAY_ELEMENTS(gUserSignatureOptions); user_sig_idx++)
        {
            signature_status_t user_sig     = gUserSignatureOptions[user_sig_idx];
            const char        *user_sig_str = getSignatureString(user_sig);

            const char *file_string = "EmBootTest-Kernel_%s-User_%s.fw";
            char       *outFileName = (char *)malloc(strlen(file_string) + strlen(kernel_sig_str) + strlen(user_sig_str));
            sprintf(outFileName, file_string, kernel_sig_str, user_sig_str);

            printf("Generating test firmware image: %s\n", outFileName);

            bool secure;

            // Sign the kernel payload
            secure          = (SIGNED_VALID == kernel_sig) || (SIGNED_SHA_ERROR == kernel_sig);
            nextImgUnsigned = (UNSIGNED_VALID == user_sig) || (UNSIGNED_CRC_ERROR == user_sig);
            gInjectCRCError = (UNSIGNED_CRC_ERROR == kernel_sig);
            gInjectSHAError = (SIGNED_SHA_ERROR == kernel_sig);
            keyLocation     = KEY_LOCATION_OTP;
            keyIndex        = 0;
            lastImage       = 0;
            error_t ret     = sign_fw(fwFileName, outFileName, secure, false, KeyFilename, !nextImgUnsigned, pubKeyFilename, KERNEL_IMAGE_IDX);
            if (SUCCESS != ret)
            {
                fprintf(stderr, "Error: Unable to prepare kernel payload %s from %s: error=%d: %s\n", outFileName, fwFileName, ret, error_to_string(ret));
                status = ret;
            }

            // Sign the user payload
            secure            = (SIGNED_VALID == user_sig) || (SIGNED_SHA_ERROR == user_sig);
            nextImgUnsigned   = 0;
            lastImage         = 1;
            gInjectCRCError   = (UNSIGNED_CRC_ERROR == user_sig);
            gInjectSHAError   = (SIGNED_SHA_ERROR == user_sig);
            keyLocation       = secure ? KEY_LOCATION_KERNEL : KEY_NO_AUTHENTICATION;
            keyIndex          = 0;
            bool need_signing = true;

            if (SUCCESS == status && need_signing)
            {
                // The user firmware needs to be encrypted or errors need to be injected.
                ret = sign_fw(outFileName, outFileName, secure, false, KeyFilename, false, pubKeyFilename, USER_IMAGE_IDX);
                if (SUCCESS != ret)
                {
                    fprintf(stderr, "Error: Unable to prepare user payload %s: error=%d: %s\n", outFileName, ret, error_to_string(ret));
                    unlink(outFileName);
                    status = ret;
                }
            }

            free(outFileName);
        }
    }

    free(fileBuffer);

    // Restore strict checks.
    gDisableChecks = false;

    return status;
}
