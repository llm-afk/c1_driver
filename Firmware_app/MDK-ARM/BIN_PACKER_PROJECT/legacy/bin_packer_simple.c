#define _CRT_SECURE_NO_WARNINGS

#include <ctype.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define PACK_DATE_LEN 12
#define PATH_BUFFER_LEN 1024

typedef enum ModeTag {
    MODE_PACK = 0,
    MODE_VERIFY = 1
} Mode;

#pragma pack(push, 1)
typedef struct SimpleHeaderTag {
    uint16_t software_version;
    char pack_date[PACK_DATE_LEN];
    uint32_t payload_crc32;
    uint16_t hardware_version;
} SimpleHeader;
#pragma pack(pop)

typedef struct OptionsTag {
    Mode mode;
    const char *input_path;
    const char *output_path;
    const char *raw_path;
    const char *software_version_text;
    const char *hardware_version_text;
    size_t target_size;
    char default_output_path[PATH_BUFFER_LEN];
} Options;

static void print_usage(const char *program_name)
{
    printf("Usage:\n");
    printf("  Pack  : %s -i <input.bin> [-o <output.bin>] -s <target_size> -sv <0x0001> [-hv <0x0000>]\n", program_name);
    printf("  Verify: %s --verify -i <packed.bin> -r <original.bin>\n", program_name);
    printf("\n");
    printf("Options:\n");
    printf("  -i         Input file path\n");
    printf("  -o         Output file path, default is app_pack.bin in input directory\n");
    printf("  -r         Original raw bin path, required by verify mode\n");
    printf("  -s         Target size, supports 81920 / 80KB / 80KiB / 1MB\n");
    printf("  -sv        Software version, for example 0x0001\n");
    printf("  -hv        Hardware version, for example 0x0002, default is 0x0000\n");
    printf("  --verify   Verify packed bin with original raw bin\n");
    printf("  -h         Show help\n");
    printf("\n");
    printf("Header order:\n");
    printf("  software_version -> pack_date -> payload_crc32 -> hardware_version\n");
}

static int equals_ignore_case(const char *a, const char *b)
{
    while (*a != '\0' && *b != '\0') {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) {
            return 0;
        }
        ++a;
        ++b;
    }
    return (*a == '\0' && *b == '\0');
}

static int parse_size_string(const char *text, size_t *size_out)
{
    char *end_ptr;
    unsigned long long value;
    char suffix[16];
    size_t i;
    unsigned long long multiplier = 1ULL;

    if (text == NULL || size_out == NULL) {
        return 0;
    }

    while (*text != '\0' && isspace((unsigned char)*text)) {
        ++text;
    }

    if (*text == '\0') {
        return 0;
    }

    errno = 0;
    value = strtoull(text, &end_ptr, 10);
    if (errno != 0 || end_ptr == text) {
        return 0;
    }

    while (*end_ptr != '\0' && isspace((unsigned char)*end_ptr)) {
        ++end_ptr;
    }

    for (i = 0; *end_ptr != '\0' && i < sizeof(suffix) - 1; ++i, ++end_ptr) {
        suffix[i] = (char)tolower((unsigned char)*end_ptr);
    }
    suffix[i] = '\0';

    if (*end_ptr != '\0') {
        return 0;
    }

    if (suffix[0] == '\0' || equals_ignore_case(suffix, "b")) {
        multiplier = 1ULL;
    } else if (equals_ignore_case(suffix, "k") || equals_ignore_case(suffix, "kb") || equals_ignore_case(suffix, "kib")) {
        multiplier = 1024ULL;
    } else if (equals_ignore_case(suffix, "m") || equals_ignore_case(suffix, "mb") || equals_ignore_case(suffix, "mib")) {
        multiplier = 1024ULL * 1024ULL;
    } else {
        return 0;
    }

    if (value > ((unsigned long long)-1) / multiplier) {
        return 0;
    }

    value *= multiplier;

    if (value > (unsigned long long)((size_t)-1)) {
        return 0;
    }

    *size_out = (size_t)value;
    return 1;
}

static int parse_u16_value(const char *text, uint16_t *value_out)
{
    char *end_ptr;
    unsigned long value;

    if (text == NULL || value_out == NULL) {
        return 0;
    }

    errno = 0;
    value = strtoul(text, &end_ptr, 0);
    if (errno != 0 || end_ptr == text || *end_ptr != '\0' || value > 0xFFFFUL) {
        return 0;
    }

    *value_out = (uint16_t)value;
    return 1;
}

static int build_default_output_path(const char *input_path, char *output_path, size_t output_size)
{
    const char *last_slash;
    const char *last_backslash;
    const char *separator;
    size_t prefix_len;

    if (input_path == NULL || output_path == NULL || output_size == 0) {
        return 0;
    }

    last_slash = strrchr(input_path, '/');
    last_backslash = strrchr(input_path, '\\');
    separator = last_slash;
    if (last_backslash != NULL && (separator == NULL || last_backslash > separator)) {
        separator = last_backslash;
    }

    if (separator == NULL) {
        strncpy(output_path, "app_pack.bin", output_size - 1);
        output_path[output_size - 1] = '\0';
        return 1;
    }

    prefix_len = (size_t)(separator - input_path + 1);
    if (prefix_len + strlen("app_pack.bin") + 1 > output_size) {
        return 0;
    }

    memcpy(output_path, input_path, prefix_len);
    output_path[prefix_len] = '\0';
    strcat(output_path, "app_pack.bin");
    return 1;
}

static int parse_arguments(int argc, char *argv[], Options *options)
{
    int i;

    memset(options, 0, sizeof(*options));
    options->mode = MODE_PACK;

    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Error: missing input file path after -i\n");
                return 0;
            }
            options->input_path = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Error: missing output file path after -o\n");
                return 0;
            }
            options->output_path = argv[++i];
        } else if (strcmp(argv[i], "-r") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Error: missing raw file path after -r\n");
                return 0;
            }
            options->raw_path = argv[++i];
        } else if (strcmp(argv[i], "-s") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Error: missing target size after -s\n");
                return 0;
            }
            if (!parse_size_string(argv[++i], &options->target_size)) {
                fprintf(stderr, "Error: invalid target size %s\n", argv[i]);
                return 0;
            }
        } else if (strcmp(argv[i], "-sv") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Error: missing software version after -sv\n");
                return 0;
            }
            options->software_version_text = argv[++i];
        } else if (strcmp(argv[i], "-hv") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Error: missing hardware version after -hv\n");
                return 0;
            }
            options->hardware_version_text = argv[++i];
        } else if (strcmp(argv[i], "--verify") == 0) {
            options->mode = MODE_VERIFY;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return -1;
        } else {
            fprintf(stderr, "Error: unknown option %s\n", argv[i]);
            return 0;
        }
    }

    if (options->input_path == NULL) {
        fprintf(stderr, "Error: input file must be specified with -i\n");
        return 0;
    }

    if (options->mode == MODE_PACK) {
        if (options->software_version_text == NULL) {
            fprintf(stderr, "Error: software version must be specified with -sv\n");
            return 0;
        }
        if (options->target_size == 0) {
            fprintf(stderr, "Error: a valid target size must be specified with -s\n");
            return 0;
        }
        if (options->output_path == NULL) {
            if (!build_default_output_path(options->input_path, options->default_output_path, sizeof(options->default_output_path))) {
                fprintf(stderr, "Error: failed to build default output path\n");
                return 0;
            }
            options->output_path = options->default_output_path;
        }
    } else {
        if (options->raw_path == NULL) {
            fprintf(stderr, "Error: verify mode requires original raw bin path by -r\n");
            return 0;
        }
    }

    return 1;
}

static uint8_t *read_binary_file(const char *file_path, size_t *file_size)
{
    FILE *fp;
    long size_long;
    uint8_t *buffer;
    size_t read_size;

    if (file_size != NULL) {
        *file_size = 0;
    }

    fp = fopen(file_path, "rb");
    if (fp == NULL) {
        fprintf(stderr, "Error: failed to open file %s\n", file_path);
        return NULL;
    }

    if (fseek(fp, 0, SEEK_END) != 0) {
        fclose(fp);
        fprintf(stderr, "Error: failed to seek to end of file %s\n", file_path);
        return NULL;
    }

    size_long = ftell(fp);
    if (size_long < 0) {
        fclose(fp);
        fprintf(stderr, "Error: failed to get file size %s\n", file_path);
        return NULL;
    }

    if (fseek(fp, 0, SEEK_SET) != 0) {
        fclose(fp);
        fprintf(stderr, "Error: failed to seek to start of file %s\n", file_path);
        return NULL;
    }

    *file_size = (size_t)size_long;

    if (*file_size == 0) {
        fclose(fp);
        return NULL;
    }

    buffer = (uint8_t *)malloc(*file_size);
    if (buffer == NULL) {
        fclose(fp);
        fprintf(stderr, "Error: memory allocation failed for file %s\n", file_path);
        return NULL;
    }

    read_size = fread(buffer, 1, *file_size, fp);
    fclose(fp);

    if (read_size != *file_size) {
        free(buffer);
        fprintf(stderr, "Error: failed to read file %s\n", file_path);
        return NULL;
    }

    return buffer;
}

static uint32_t crc32_calculate(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFU;
    size_t i;
    int bit;

    for (i = 0; i < length; ++i) {
        crc ^= (uint32_t)data[i];
        for (bit = 0; bit < 8; ++bit) {
            if (crc & 1U) {
                crc = (crc >> 1) ^ 0xEDB88320U;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc ^ 0xFFFFFFFFU;
}

static int get_pack_date_string(char *date_out, size_t date_out_size)
{
    time_t now;
    struct tm *local_time;
    int year;
    int written;

    now = time(NULL);
    if (now == (time_t)-1) {
        return 0;
    }

    local_time = localtime(&now);
    if (local_time == NULL) {
        return 0;
    }

    year = (local_time->tm_year + 1900) % 100;
    written = snprintf(date_out, date_out_size, "%02d.%d.%d", year, local_time->tm_mon + 1, local_time->tm_mday);
    if (written <= 0 || (size_t)written >= date_out_size) {
        return 0;
    }

    return 1;
}

static void fill_header(SimpleHeader *header, uint16_t software_version, const char *pack_date, uint32_t crc32, uint16_t hardware_version)
{
    memset(header, 0, sizeof(*header));
    header->software_version = software_version;
    strncpy(header->pack_date, pack_date, sizeof(header->pack_date) - 1);
    header->pack_date[sizeof(header->pack_date) - 1] = '\0';
    header->payload_crc32 = crc32;
    header->hardware_version = hardware_version;
}

static int write_output_file(const char *file_path, const SimpleHeader *header, const uint8_t *payload, size_t payload_size, size_t target_size)
{
    FILE *fp;
    size_t written;
    size_t current_size;
    uint8_t zero_buffer[256];

    memset(zero_buffer, 0, sizeof(zero_buffer));

    fp = fopen(file_path, "wb");
    if (fp == NULL) {
        fprintf(stderr, "Error: failed to create output file %s\n", file_path);
        return 0;
    }

    written = fwrite(header, 1, sizeof(*header), fp);
    if (written != sizeof(*header)) {
        fclose(fp);
        fprintf(stderr, "Error: failed to write header\n");
        return 0;
    }

    written = fwrite(payload, 1, payload_size, fp);
    if (written != payload_size) {
        fclose(fp);
        fprintf(stderr, "Error: failed to write payload data\n");
        return 0;
    }

    current_size = sizeof(*header) + payload_size;
    while (current_size < target_size) {
        size_t chunk = target_size - current_size;
        if (chunk > sizeof(zero_buffer)) {
            chunk = sizeof(zero_buffer);
        }

        written = fwrite(zero_buffer, 1, chunk, fp);
        if (written != chunk) {
            fclose(fp);
            fprintf(stderr, "Error: failed to write padding bytes\n");
            return 0;
        }
        current_size += chunk;
    }

    fclose(fp);
    return 1;
}

static int buffers_equal(const uint8_t *a, const uint8_t *b, size_t length)
{
    size_t i;

    for (i = 0; i < length; ++i) {
        if (a[i] != b[i]) {
            return 0;
        }
    }

    return 1;
}

static int padding_is_all_zero(const uint8_t *data, size_t length)
{
    size_t i;

    for (i = 0; i < length; ++i) {
        if (data[i] != 0U) {
            return 0;
        }
    }

    return 1;
}

static int run_pack_mode(const Options *options)
{
    uint16_t software_version;
    uint16_t hardware_version = 0U;
    char pack_date[PACK_DATE_LEN];
    size_t original_size = 0;
    size_t minimum_size;
    uint8_t *input_data;
    uint32_t payload_crc32;
    SimpleHeader header;

    if (!parse_u16_value(options->software_version_text, &software_version)) {
        fprintf(stderr, "Error: invalid software version %s\n", options->software_version_text);
        return 1;
    }

    if (options->hardware_version_text != NULL) {
        if (!parse_u16_value(options->hardware_version_text, &hardware_version)) {
            fprintf(stderr, "Error: invalid hardware version %s\n", options->hardware_version_text);
            return 1;
        }
    }

    if (!get_pack_date_string(pack_date, sizeof(pack_date))) {
        fprintf(stderr, "Error: failed to generate pack date\n");
        return 1;
    }

    input_data = read_binary_file(options->input_path, &original_size);
    if (input_data == NULL) {
        fprintf(stderr, "Error: input bin must not be empty\n");
        return 1;
    }

    minimum_size = sizeof(SimpleHeader) + original_size;
    if (options->target_size < original_size) {
        fprintf(stderr, "Error: target size %u bytes is smaller than original bin size %u bytes\n",
                (unsigned int)options->target_size,
                (unsigned int)original_size);
        free(input_data);
        return 1;
    }

    if (options->target_size < minimum_size) {
        fprintf(stderr, "Error: target size %u bytes is too small for header(%u) + payload(%u), minimum required is %u bytes\n",
                (unsigned int)options->target_size,
                (unsigned int)sizeof(SimpleHeader),
                (unsigned int)original_size,
                (unsigned int)minimum_size);
        free(input_data);
        return 1;
    }

    payload_crc32 = crc32_calculate(input_data, original_size);
    fill_header(&header, software_version, pack_date, payload_crc32, hardware_version);

    if (!write_output_file(options->output_path, &header, input_data, original_size, options->target_size)) {
        free(input_data);
        return 1;
    }

    printf("Pack success\n");
    printf("Input file       : %s\n", options->input_path);
    printf("Output file      : %s\n", options->output_path);
    printf("Software version : 0x%04X\n", software_version);
    printf("Pack date        : %s\n", pack_date);
    printf("Hardware version : 0x%04X\n", hardware_version);
    printf("Original size    : %u bytes\n", (unsigned int)original_size);
    printf("Header size      : %u bytes\n", (unsigned int)sizeof(SimpleHeader));
    printf("CRC32            : 0x%08X\n", payload_crc32);
    printf("Final size       : %u bytes\n", (unsigned int)options->target_size);

    free(input_data);
    return 0;
}

static int run_verify_mode(const Options *options)
{
    size_t packed_size = 0;
    size_t raw_size = 0;
    uint8_t *packed_data;
    uint8_t *raw_data;
    SimpleHeader header;
    uint32_t actual_crc32;
    size_t payload_offset;
    size_t padding_size;
    int payload_matches;
    int padding_zero;

    packed_data = read_binary_file(options->input_path, &packed_size);
    if (packed_data == NULL) {
        fprintf(stderr, "Error: packed bin must not be empty\n");
        return 1;
    }

    raw_data = read_binary_file(options->raw_path, &raw_size);
    if (raw_data == NULL) {
        free(packed_data);
        fprintf(stderr, "Error: original raw bin must not be empty\n");
        return 1;
    }

    if (packed_size < sizeof(SimpleHeader)) {
        fprintf(stderr, "Verify failed: packed file is smaller than header size\n");
        free(raw_data);
        free(packed_data);
        return 1;
    }

    memcpy(&header, packed_data, sizeof(header));
    payload_offset = sizeof(SimpleHeader);

    if (packed_size < payload_offset + raw_size) {
        fprintf(stderr, "Verify failed: packed file is too small for header + original payload\n");
        free(raw_data);
        free(packed_data);
        return 1;
    }

    actual_crc32 = crc32_calculate(raw_data, raw_size);
    payload_matches = buffers_equal(raw_data, packed_data + payload_offset, raw_size);
    padding_size = packed_size - payload_offset - raw_size;
    padding_zero = padding_is_all_zero(packed_data + payload_offset + raw_size, padding_size);

    printf("Verify report\n");
    printf("Packed file      : %s\n", options->input_path);
    printf("Original file    : %s\n", options->raw_path);
    printf("Software version : 0x%04X\n", header.software_version);
    printf("Pack date        : %s\n", header.pack_date);
    printf("Header CRC32     : 0x%08X\n", header.payload_crc32);
    printf("Actual CRC32     : 0x%08X\n", actual_crc32);
    printf("Hardware version : 0x%04X\n", header.hardware_version);
    printf("Header size      : %u bytes\n", (unsigned int)sizeof(SimpleHeader));
    printf("Packed size      : %u bytes\n", (unsigned int)packed_size);
    printf("Original size    : %u bytes\n", (unsigned int)raw_size);
    printf("Padding bytes    : %u bytes\n", (unsigned int)padding_size);

    if (header.payload_crc32 != actual_crc32) {
        fprintf(stderr, "Verify failed: CRC32 mismatch\n");
        free(raw_data);
        free(packed_data);
        return 1;
    }

    if (!payload_matches) {
        fprintf(stderr, "Verify failed: payload content does not match original bin\n");
        free(raw_data);
        free(packed_data);
        return 1;
    }

    if (!padding_zero) {
        fprintf(stderr, "Verify failed: padding area is not all zero\n");
        free(raw_data);
        free(packed_data);
        return 1;
    }

    printf("Verify result    : OK\n");

    free(raw_data);
    free(packed_data);
    return 0;
}

int main(int argc, char *argv[])
{
    Options options;
    int parse_result;

    parse_result = parse_arguments(argc, argv, &options);
    if (parse_result == -1) {
        return 0;
    }

    if (!parse_result) {
        if (argc == 1) {
            print_usage(argv[0]);
        }
        return 1;
    }

    if (options.mode == MODE_VERIFY) {
        return run_verify_mode(&options);
    }

    return run_pack_mode(&options);
}
