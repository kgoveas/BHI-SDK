////////////////////////////////////////////////////////////////////////////////
///
/// @file       utils/isign/icrc.cpp
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
#include "fwHeader.h"

#define KB                   (1024)
#define CRC_SEED             (0xFFFFFFFF)
#define USER_HDROFFSET_LOC   0
#define KERNEL_HDROFFSET_LOC (0x0E * 4)

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
} task_t;


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
error_t crc_fw(const char *inFwFilename, const char *outFwFilename, uint8_t imgIdx);

// command line options
uint8_t verbose = 0;

// Default header values, can be overwritten on the command line
uint8_t noexec    = 0;
uint8_t lastImage = 1;

// Help string
static const char USAGE_STR[] =
    "EM Microelectronic Firmware Finishing Utility\n"
    "Usage:\n\t%s [<options>]\n\n"
    "Options:\n"
    "  -h             = Print this usage message\n"
    "  -V             = Turn on verbose output\n"
    "  -f <fwFile>    = Specify the firmware file name to finish\n"
    "  -o <outFile>   = Specify the output firmware file\n"
    "  --chain        = Contents are in a chain\n"
    "  --imgIdx <x>   = Finish image x in concatenated image\n"
    "                   Index starts at 0, default is 0\n"
    "  --noexec       = For Flash images, do not execute the image immediately after a hard reset\n"
    "  --exec         = For Flash images, attempt to execute the image immediately after a hard reset \n"
    ;

int main(int argc, char *argv[])
{
    const char *fwFileName  = NULL;
    const char *outFileName = NULL;
    uint8_t     imageIndex  = ALL_IMAGES;
    task_t      task        = DEFAULT_TASK;
    error_t     ret         = SUCCESS;
    int         i;

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
        case DEFAULT_TASK:
            if (fwFileName)
            {
                ret = crc_fw(fwFileName, outFileName, imageIndex);
                if (ret)
                {
                    fprintf(stderr, "Error finalizing firmware %s: error=%d: %s\n", fwFileName, ret, error_to_string(ret));
                }
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
    fwHdr->ImageFlags.bits.NextImgUnsigned = 0;
    fwHdr->ImageFlags.bits.Reserved        = 0;

    fwHdr->KeyFlags.bits.Location = KEY_NO_AUTHENTICATION;
    fwHdr->KeyFlags.bits.Index    = 0;
    fwHdr->KeyFlags.bits.Type     = 0;
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
    printf("ImageLen:   0x%08x\n", hdr->ImageLen);
    printf("ImageCrc:   0x%08x\n", hdr->ImageCrc);
    printf("KeyOffset:  0x%08x\n", hdr->KeyOffset);
    printf("ExpVer:     0x%08x\n", hdr->ExpVer);
    printf("HdrCrc:     0x%08x\n", hdr->HdrCrc);
    printf("\n");
}


error_t crc_fw(const char *inFwFilename, const char *outFwFilename, uint8_t imgIdx)
{
    FILE    *fwOutFile;
    uint8_t *fileBuffer;
    uint8_t *fwBuffer;
    int      fileSize;
    int      bytes;
    uint8_t  imgToSign = imgIdx;
    bool secure = false;

    FWHeader_t *fwHdr;
    FWHeader_t *fwHdrCopy   = NULL;
    FWHeader_t *kernelFwHdr = NULL;

    if (ALL_IMAGES == imgToSign)
    {
        imgToSign = 0;
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



    // Calculate the crc of the fw image
    fwHdr->ImageCrc = fileCrc(fwBuffer, fwHdr->ImageLen);

    fwHdr->HdrCrc = update_crc_32(CRC_SEED, (uint32_t *)fwHdr, offsetof(FWHeader_t, HdrCrc) / 4);

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

// Check for valid firmware image
// - Image CRC, Header CRC
error_t check_image(FWHeader_t *fwHdr, uint8_t *fwBuffer)
{
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

error_t print_fw(const char *FwFilename, uint8_t imgIdx)
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

            if (SUCCESS != check_image(fwHdr, fwBuffer))
            {
                ret = ERR_IMAGE_VERIFICATION;
            }
            if (!halt && isChained(fwHdr))
            {
                // Go to the next image
                imgToVerify++;
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

    return ret;
}

error_t print_header(const char *FwFilename)
{
    verbose = true;
    error_t ret = print_fw(FwFilename, ALL_IMAGES);
    if (ret)
    {
        fprintf(stderr, "Error printing firmware %s: error=%d: %s\n", FwFilename, ret, error_to_string(ret));
    }
    return ret;
}

