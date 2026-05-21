#define _CRT_SECURE_NO_WARNINGS

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * CRC32 查表法验包示例
 *
 * 这个示例程序用于演示：
 * 1. 如何使用“查表法”计算 CRC32
 * 2. 如何校验本项目打包后的 BIN 文件
 * 3. 如何读取打包尾部中的版本、时间、Git 哈希和 CRC 信息
 *
 * 本示例使用的 CRC32 参数与 bin_packer.c 完全一致：
 * - 多项式  : 0xEDB88320
 * - 初值    : 0xFFFFFFFF
 * - 结束异或: 0xFFFFFFFF
 *
 * 说明：
 * - 这个程序只用于“校验已经打包后的 BIN”
 * - 它会先检查尾部签名 BPKR128
 * - 如果没有这个签名，就说明该文件不是本项目生成的有效打包文件
 */

#define PACK_TIME_LEN 20
#define GIT_HASH_LEN 64
#define BIN_TAIL_RESERVED_LEN 36
#define BIN_TAIL_TOTAL_LEN 128
#define BIN_TAIL_MAGIC "BPKR128"

#pragma pack(push, 1)
typedef struct BinTailTag {
    /* 软件版本号，例如 0x0003 */
    uint16_t software_version;

    /* 硬件版本号，例如 0x0000 */
    uint16_t hardware_version;

    /* 打包时间字符串，当前格式：YYYY-MM-DD HH:MM:SS */
    char pack_time[PACK_TIME_LEN];

    /* Git 提交哈希，长度预留 64 字节 */
    char git_hash[GIT_HASH_LEN];

    /*
     * 预留区：
     * - 前几个字节目前写入固定签名 BPKR128
     * - 剩余空间保留给后续扩展使用
     */
    char reserved[BIN_TAIL_RESERVED_LEN];

    /*
     * 整包 CRC32：
     * 对整个打包文件除最后 4 字节之外的所有内容计算 CRC32，
     * 然后把结果写到这里
     */
    uint32_t package_crc32;
} BinTail;
#pragma pack(pop)

/* 编译期检查：如果结构体总长度不是 128 字节，直接编译报错 */
typedef char BinTailSizeCheck[(sizeof(BinTail) == BIN_TAIL_TOTAL_LEN) ? 1 : -1];

/*
 * CRC32 查找表
 *
 * 这张表对应的就是当前 CRC32 算法使用的反射多项式 0xEDB88320。
 * 使用查表法时，不再对每个字节逐 bit 处理 8 次，而是直接通过表项
 * 快速更新 CRC 值，因此速度通常比逐位算法更快。
 */
static const uint32_t g_crc32_table[256] = {
    0x00000000U, 0x77073096U, 0xEE0E612CU, 0x990951BAU, 0x076DC419U, 0x706AF48FU, 0xE963A535U, 0x9E6495A3U,
    0x0EDB8832U, 0x79DCB8A4U, 0xE0D5E91EU, 0x97D2D988U, 0x09B64C2BU, 0x7EB17CBDU, 0xE7B82D07U, 0x90BF1D91U,
    0x1DB71064U, 0x6AB020F2U, 0xF3B97148U, 0x84BE41DEU, 0x1ADAD47DU, 0x6DDDE4EBU, 0xF4D4B551U, 0x83D385C7U,
    0x136C9856U, 0x646BA8C0U, 0xFD62F97AU, 0x8A65C9ECU, 0x14015C4FU, 0x63066CD9U, 0xFA0F3D63U, 0x8D080DF5U,
    0x3B6E20C8U, 0x4C69105EU, 0xD56041E4U, 0xA2677172U, 0x3C03E4D1U, 0x4B04D447U, 0xD20D85FDU, 0xA50AB56BU,
    0x35B5A8FAU, 0x42B2986CU, 0xDBBBC9D6U, 0xACBCF940U, 0x32D86CE3U, 0x45DF5C75U, 0xDCD60DCFU, 0xABD13D59U,
    0x26D930ACU, 0x51DE003AU, 0xC8D75180U, 0xBFD06116U, 0x21B4F4B5U, 0x56B3C423U, 0xCFBA9599U, 0xB8BDA50FU,
    0x2802B89EU, 0x5F058808U, 0xC60CD9B2U, 0xB10BE924U, 0x2F6F7C87U, 0x58684C11U, 0xC1611DABU, 0xB6662D3DU,
    0x76DC4190U, 0x01DB7106U, 0x98D220BCU, 0xEFD5102AU, 0x71B18589U, 0x06B6B51FU, 0x9FBFE4A5U, 0xE8B8D433U,
    0x7807C9A2U, 0x0F00F934U, 0x9609A88EU, 0xE10E9818U, 0x7F6A0DBBU, 0x086D3D2DU, 0x91646C97U, 0xE6635C01U,
    0x6B6B51F4U, 0x1C6C6162U, 0x856530D8U, 0xF262004EU, 0x6C0695EDU, 0x1B01A57BU, 0x8208F4C1U, 0xF50FC457U,
    0x65B0D9C6U, 0x12B7E950U, 0x8BBEB8EAU, 0xFCB9887CU, 0x62DD1DDFU, 0x15DA2D49U, 0x8CD37CF3U, 0xFBD44C65U,
    0x4DB26158U, 0x3AB551CEU, 0xA3BC0074U, 0xD4BB30E2U, 0x4ADFA541U, 0x3DD895D7U, 0xA4D1C46DU, 0xD3D6F4FBU,
    0x4369E96AU, 0x346ED9FCU, 0xAD678846U, 0xDA60B8D0U, 0x44042D73U, 0x33031DE5U, 0xAA0A4C5FU, 0xDD0D7CC9U,
    0x5005713CU, 0x270241AAU, 0xBE0B1010U, 0xC90C2086U, 0x5768B525U, 0x206F85B3U, 0xB966D409U, 0xCE61E49FU,
    0x5EDEF90EU, 0x29D9C998U, 0xB0D09822U, 0xC7D7A8B4U, 0x59B33D17U, 0x2EB40D81U, 0xB7BD5C3BU, 0xC0BA6CADU,
    0xEDB88320U, 0x9ABFB3B6U, 0x03B6E20CU, 0x74B1D29AU, 0xEAD54739U, 0x9DD277AFU, 0x04DB2615U, 0x73DC1683U,
    0xE3630B12U, 0x94643B84U, 0x0D6D6A3EU, 0x7A6A5AA8U, 0xE40ECF0BU, 0x9309FF9DU, 0x0A00AE27U, 0x7D079EB1U,
    0xF00F9344U, 0x8708A3D2U, 0x1E01F268U, 0x6906C2FEU, 0xF762575DU, 0x806567CBU, 0x196C3671U, 0x6E6B06E7U,
    0xFED41B76U, 0x89D32BE0U, 0x10DA7A5AU, 0x67DD4ACCU, 0xF9B9DF6FU, 0x8EBEEFF9U, 0x17B7BE43U, 0x60B08ED5U,
    0xD6D6A3E8U, 0xA1D1937EU, 0x38D8C2C4U, 0x4FDFF252U, 0xD1BB67F1U, 0xA6BC5767U, 0x3FB506DDU, 0x48B2364BU,
    0xD80D2BDAU, 0xAF0A1B4CU, 0x36034AF6U, 0x41047A60U, 0xDF60EFC3U, 0xA867DF55U, 0x316E8EEFU, 0x4669BE79U,
    0xCB61B38CU, 0xBC66831AU, 0x256FD2A0U, 0x5268E236U, 0xCC0C7795U, 0xBB0B4703U, 0x220216B9U, 0x5505262FU,
    0xC5BA3BBEU, 0xB2BD0B28U, 0x2BB45A92U, 0x5CB36A04U, 0xC2D7FFA7U, 0xB5D0CF31U, 0x2CD99E8BU, 0x5BDEAE1DU,
    0x9B64C2B0U, 0xEC63F226U, 0x756AA39CU, 0x026D930AU, 0x9C0906A9U, 0xEB0E363FU, 0x72076785U, 0x05005713U,
    0x95BF4A82U, 0xE2B87A14U, 0x7BB12BAEU, 0x0CB61B38U, 0x92D28E9BU, 0xE5D5BE0DU, 0x7CDCEFB7U, 0x0BDBDF21U,
    0x86D3D2D4U, 0xF1D4E242U, 0x68DDB3F8U, 0x1FDA836EU, 0x81BE16CDU, 0xF6B9265BU, 0x6FB077E1U, 0x18B74777U,
    0x88085AE6U, 0xFF0F6A70U, 0x66063BCAU, 0x11010B5CU, 0x8F659EFFU, 0xF862AE69U, 0x616BFFD3U, 0x166CCF45U,
    0xA00AE278U, 0xD70DD2EEU, 0x4E048354U, 0x3903B3C2U, 0xA7672661U, 0xD06016F7U, 0x4969474DU, 0x3E6E77DBU,
    0xAED16A4AU, 0xD9D65ADCU, 0x40DF0B66U, 0x37D83BF0U, 0xA9BCAE53U, 0xDEBB9EC5U, 0x47B2CF7FU, 0x30B5FFE9U,
    0xBDBDF21CU, 0xCABAC28AU, 0x53B39330U, 0x24B4A3A6U, 0xBAD03605U, 0xCDD70693U, 0x54DE5729U, 0x23D967BFU,
    0xB3667A2EU, 0xC4614AB8U, 0x5D681B02U, 0x2A6F2B94U, 0xB40BBE37U, 0xC30C8EA1U, 0x5A05DF1BU, 0x2D02EF8DU
};

static void fixed_field_to_string(const char *field, size_t field_size, char *out, size_t out_size)
{
    size_t copy_len = 0;

    /* 找到字段中第一个 '\0'，避免把尾部填充的 0 一起带出去 */
    while (copy_len < field_size && field[copy_len] != '\0') {
        ++copy_len;
    }

    if (out_size == 0) {
        return;
    }

    /* 保证输出字符串一定以 '\0' 结束 */
    if (copy_len >= out_size) {
        copy_len = out_size - 1;
    }

    memcpy(out, field, copy_len);
    out[copy_len] = '\0';
}

static uint8_t *read_file_all(const char *file_path, size_t *file_size)
{
    FILE *fp;
    long size_long;
    size_t read_size;
    uint8_t *buffer;

    /* 以二进制只读方式打开文件 */
    fp = fopen(file_path, "rb");
    if (fp == NULL) {
        fprintf(stderr, "Error: cannot open file: %s\n", file_path);
        return NULL;
    }

    /* 先移动到文件末尾，获取总长度 */
    if (fseek(fp, 0, SEEK_END) != 0) {
        fclose(fp);
        fprintf(stderr, "Error: cannot seek file: %s\n", file_path);
        return NULL;
    }

    size_long = ftell(fp);
    if (size_long < 0) {
        fclose(fp);
        fprintf(stderr, "Error: cannot get file size: %s\n", file_path);
        return NULL;
    }

    /* 再回到文件开头，准备一次性读取 */
    if (fseek(fp, 0, SEEK_SET) != 0) {
        fclose(fp);
        fprintf(stderr, "Error: cannot rewind file: %s\n", file_path);
        return NULL;
    }

    *file_size = (size_t)size_long;

    /* 为整个文件申请一段连续内存 */
    buffer = (uint8_t *)malloc(*file_size);
    if (buffer == NULL) {
        fclose(fp);
        fprintf(stderr, "Error: not enough memory\n");
        return NULL;
    }

    /* 一次性读入整个文件 */
    read_size = fread(buffer, 1, *file_size, fp);
    fclose(fp);

    if (read_size != *file_size) {
        free(buffer);
        fprintf(stderr, "Error: failed to read file: %s\n", file_path);
        return NULL;
    }

    return buffer;
}

static uint32_t crc32_calculate_table(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFU;
    size_t i;

    /*
     * 查表法计算过程：
     * 1. 初值先置为 0xFFFFFFFF
     * 2. 每处理一个字节：
     *    - 取当前 crc 低 8 位与 data[i] 异或
     *    - 用结果作为查表索引
     *    - crc 右移 8 位后，再与表项异或
     * 3. 全部处理完以后，再做一次结束异或 0xFFFFFFFF
     */
    for (i = 0; i < length; ++i) {
        crc = (crc >> 8) ^ g_crc32_table[(crc ^ data[i]) & 0xFFU];
    }

    return crc ^ 0xFFFFFFFFU;
}

static int verify_packed_bin(const char *file_path)
{
    uint8_t *file_data;
    size_t file_size;
    BinTail tail;
    uint32_t stored_crc32;
    uint32_t calc_crc32;
    char pack_time[PACK_TIME_LEN + 1];
    char git_hash[GIT_HASH_LEN + 1];

    /* 先把整包读入内存，便于后续统一解析 */
    file_data = read_file_all(file_path, &file_size);
    if (file_data == NULL) {
        return 1;
    }

    /* 文件比 128 字节尾部还小，肯定不是有效打包文件 */
    if (file_size < sizeof(BinTail)) {
        fprintf(stderr, "Verify failed: file is smaller than %u-byte tail\n", (unsigned int)sizeof(BinTail));
        free(file_data);
        return 1;
    }

    /*
     * 尾部结构固定放在文件最后 128 字节：
     * file_data + file_size - sizeof(BinTail)
     * 就是尾部结构起始地址
     */
    memcpy(&tail, file_data + file_size - sizeof(BinTail), sizeof(BinTail));

    /* 文件最后 4 字节就是存储的 CRC32 */
    memcpy(&stored_crc32, file_data + file_size - sizeof(uint32_t), sizeof(uint32_t));

    /*
     * 先检查尾部签名：
     * 只有 reserved 前几个字节匹配 BPKR128，才认为是本项目打包出来的文件。
     * 这样可以避免把普通原始 BIN 错误当成打包文件去解析。
     */
    if (memcmp(tail.reserved, BIN_TAIL_MAGIC, sizeof(BIN_TAIL_MAGIC)) != 0) {
        fprintf(stderr, "Verify failed: tail signature not found, not a valid packed bin\n");
        free(file_data);
        return 1;
    }

    /*
     * 重新计算整包 CRC32。
     * 注意：最后 4 字节本身不参与 CRC 计算，
     * 因为它只是“保存 CRC 结果”的字段。
     */
    calc_crc32 = crc32_calculate_table(file_data, file_size - sizeof(uint32_t));

    /* 把固定长度字段转换为正常 C 字符串，便于打印 */
    fixed_field_to_string(tail.pack_time, sizeof(tail.pack_time), pack_time, sizeof(pack_time));
    fixed_field_to_string(tail.git_hash, sizeof(tail.git_hash), git_hash, sizeof(git_hash));

    printf("Table CRC32 verify report\n");
    printf("Packed file      : %s\n", file_path);
    printf("Software version : 0x%04X\n", tail.software_version);
    printf("Hardware version : 0x%04X\n", tail.hardware_version);
    printf("Pack time        : %s\n", pack_time[0] != '\0' ? pack_time : "UNKNOWN");
    printf("Git hash         : %s\n", git_hash[0] != '\0' ? git_hash : "UNKNOWN");
    printf("Stored CRC32     : 0x%08X\n", stored_crc32);
    printf("Actual CRC32     : 0x%08X\n", calc_crc32);
    printf("File size        : %u bytes\n", (unsigned int)file_size);

    free(file_data);

    /* 比较“尾部记录值”和“重新计算值”是否一致 */
    if (stored_crc32 != calc_crc32) {
        fprintf(stderr, "Verify result    : FAILED\n");
        return 1;
    }

    printf("Verify result    : OK\n");
    return 0;
}

int main(int argc, char *argv[])
{
    /*
     * 用法：
     *   crc32_table_verify_example.exe <packed.bin>
     *
     * 示例：
     *   crc32_table_verify_example.exe ..\\examples\\sample_pack.bin
     */
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <packed.bin>\n", argv[0]);
        fprintf(stderr, "Example: %s ..\\examples\\sample_pack.bin\n", argv[0]);
        return 1;
    }

    return verify_packed_bin(argv[1]);
}
