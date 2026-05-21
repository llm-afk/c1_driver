#define _CRT_SECURE_NO_WARNINGS

#include <ctype.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define GIT_HASH_LEN 64
#define PACK_TIME_LEN 20
#define BIN_TAIL_RESERVED_LEN 36
#define BIN_TAIL_TOTAL_LEN 128
#define BIN_TAIL_MAGIC "BPKR128"
#define PATH_BUFFER_LEN 1024

typedef enum ModeTag {
    MODE_PACK = 0,
    MODE_VERIFY = 1
} Mode;

#pragma pack(push, 1)
typedef struct BinTailTag {
    uint16_t software_version;
    uint16_t hardware_version;
    char pack_time[PACK_TIME_LEN];
    char git_hash[GIT_HASH_LEN];
    char reserved[BIN_TAIL_RESERVED_LEN];
    uint32_t package_crc32;
} BinTail;
#pragma pack(pop)

typedef char BinTailSizeCheck[(sizeof(BinTail) == BIN_TAIL_TOTAL_LEN) ? 1 : -1];

typedef struct OptionsTag {
    Mode mode;
    const char *input_path;
    const char *output_path;
    const char *software_version_text;
    const char *software_version_file;
    const char *hardware_version_text;
    size_t target_size;
    char default_output_path[PATH_BUFFER_LEN];
} Options;

static void safe_copy_string(char *dst, size_t dst_size, const char *src);
static void fixed_field_to_string(const char *field, size_t field_size, char *out, size_t out_size);

static void print_usage(const char *program_name)
{
    printf("Usage:\n");
    printf("  Pack  : %s -i <input.bin> [-o <output.bin>] -s <target_size> (-v <software_version> | -vf <version_file>) [-hv <hardware_version>]\n", program_name);
    printf("  Verify: %s --verify -i <packed.bin>\n", program_name);
    printf("\n");
    printf("Options:\n");
    printf("  -i         Input file path\n");
    printf("  -o         Output file path, default is app_pack.bin in input directory\n");
    printf("  -s         Target size, supports 81920 / 80KB / 80KiB / 1MB\n");
    printf("  -v         Software version, for example 0x0001\n");
    printf("  -vf        Read software version from a text file\n");
    printf("  -hv        Hardware version, for example 0x0002, default is 0x0000\n");
    printf("  --verify   Verify packed bin only\n");
    printf("  -h         Show help\n");
    printf("\n");
    printf("Tail order:\n");
    printf("  software_version -> hardware_version -> pack_time -> git_hash -> reserved -> package_crc32\n");
    printf("  Tail size is fixed to %u bytes\n", (unsigned int)sizeof(BinTail));
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

static char *left_trim(char *text)
{
    while (*text != '\0' && isspace((unsigned char)*text)) {
        ++text;
    }
    return text;
}

static void right_trim(char *text)
{
    size_t len;

    if (text == NULL) {
        return;
    }

    len = strlen(text);
    while (len > 0 && isspace((unsigned char)text[len - 1])) {
        text[len - 1] = '\0';
        --len;
    }
}

static int contains_version_keyword(const char *text)
{
    char upper_line[256];
    size_t i;
    size_t len = strlen(text);
    const char *ver_pos;

    if (len >= sizeof(upper_line)) {
        len = sizeof(upper_line) - 1;
    }

    for (i = 0; i < len; ++i) {
        upper_line[i] = (char)toupper((unsigned char)text[i]);
    }
    upper_line[len] = '\0';

    if (strstr(upper_line, "VERSION") != NULL) {
        return 1;
    }

    ver_pos = upper_line;
    while ((ver_pos = strstr(ver_pos, "VER")) != NULL) {
        int left_ok = (ver_pos == upper_line) || !isalnum((unsigned char)ver_pos[-1]);
        int right_ok = (ver_pos[3] == '\0') || !isalnum((unsigned char)ver_pos[3]);
        if (left_ok && right_ok) {
            return 1;
        }
        ++ver_pos;
    }

    return 0;
}

static void strip_inline_comment(char *text)
{
    char *comment_pos = strstr(text, "//");
    if (comment_pos != NULL) {
        *comment_pos = '\0';
    }

    comment_pos = strchr(text, '#');
    if (comment_pos != NULL && comment_pos != text) {
        *comment_pos = '\0';
    }

    comment_pos = strstr(text, "/*");
    if (comment_pos != NULL) {
        *comment_pos = '\0';
    }
}

static int normalize_version_value(char *text)
{
    char *start;
    char *end;

    strip_inline_comment(text);
    right_trim(text);
    start = left_trim(text);

    if (*start == '\0') {
        return 0;
    }

    if ((*start == '"' || *start == '\'') && strlen(start) >= 2) {
        char quote = *start;
        ++start;
        end = strrchr(start, quote);
        if (end != NULL) {
            *end = '\0';
        }
    }

    right_trim(start);
    if (*start == '\0') {
        return 0;
    }

    if (start != text) {
        memmove(text, start, strlen(start) + 1);
    }

    end = text + strlen(text);
    while (end > text && (end[-1] == ';' || end[-1] == ',')) {
        end[-1] = '\0';
        --end;
    }

    right_trim(text);
    return (text[0] != '\0');
}

static int extract_version_from_line(char *line, char *version_out, size_t version_out_size)
{
    char *trimmed;
    char *delimiter;
    uint16_t parsed_value;

    trimmed = left_trim(line);
    right_trim(trimmed);

    if (*trimmed == '\0') {
        return 0;
    }

    if (strncmp(trimmed, "//", 2) == 0 || *trimmed == ';') {
        return 0;
    }

    if (strncmp(trimmed, "#define", 7) == 0 && isspace((unsigned char)trimmed[7])) {
        char *name;
        char *value;

        trimmed += 7;
        trimmed = left_trim(trimmed);
        name = trimmed;

        while (*trimmed != '\0' && !isspace((unsigned char)*trimmed)) {
            ++trimmed;
        }

        if (*trimmed == '\0') {
            return 0;
        }

        *trimmed++ = '\0';
        value = left_trim(trimmed);

        if (contains_version_keyword(name) && normalize_version_value(value) && parse_u16_value(value, &parsed_value)) {
            safe_copy_string(version_out, version_out_size, value);
            return 1;
        }

        return 0;
    }

    delimiter = strchr(trimmed, '=');
    if (delimiter == NULL) {
        delimiter = strchr(trimmed, ':');
    }

    if (delimiter != NULL) {
        *delimiter = '\0';
        ++delimiter;
        right_trim(trimmed);
        delimiter = left_trim(delimiter);

        if (contains_version_keyword(trimmed) && normalize_version_value(delimiter) && parse_u16_value(delimiter, &parsed_value)) {
            safe_copy_string(version_out, version_out_size, delimiter);
            return 1;
        }

        return 0;
    }

    if (*trimmed == '#') {
        return 0;
    }

    if (normalize_version_value(trimmed) && parse_u16_value(trimmed, &parsed_value)) {
        safe_copy_string(version_out, version_out_size, trimmed);
        return 1;
    }

    return 0;
}

static int read_version_from_file(const char *file_path, char *version_out, size_t version_out_size)
{
    FILE *fp;
    char line[512];

    fp = fopen(file_path, "r");
    if (fp == NULL) {
        fprintf(stderr, "Error: failed to open version file %s\n", file_path);
        return 0;
    }

    while (fgets(line, sizeof(line), fp) != NULL) {
        if (extract_version_from_line(line, version_out, version_out_size)) {
            fclose(fp);
            return 1;
        }
    }

    fclose(fp);
    fprintf(stderr, "Error: failed to parse software version from file %s\n", file_path);
    return 0;
}

static void safe_copy_string(char *dst, size_t dst_size, const char *src)
{
    if (dst_size == 0) {
        return;
    }

    if (src == NULL) {
        dst[0] = '\0';
        return;
    }

    strncpy(dst, src, dst_size - 1);
    dst[dst_size - 1] = '\0';
}

static void fixed_field_to_string(const char *field, size_t field_size, char *out, size_t out_size)
{
    size_t length = field_size;

    if (out == NULL || out_size == 0) {
        return;
    }

    while (length > 0 && field[length - 1] == '\0') {
        --length;
    }

    if (length >= out_size) {
        length = out_size - 1;
    }

    memcpy(out, field, length);
    out[length] = '\0';
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
        safe_copy_string(output_path, output_size, "app_pack.bin");
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

static void get_directory_from_path(const char *path, char *directory_out, size_t directory_out_size)
{
    const char *last_slash;
    const char *last_backslash;
    const char *separator;
    size_t prefix_len;

    if (directory_out == NULL || directory_out_size == 0) {
        return;
    }

    directory_out[0] = '\0';
    if (path == NULL) {
        safe_copy_string(directory_out, directory_out_size, ".");
        return;
    }

    last_slash = strrchr(path, '/');
    last_backslash = strrchr(path, '\\');
    separator = last_slash;
    if (last_backslash != NULL && (separator == NULL || last_backslash > separator)) {
        separator = last_backslash;
    }

    if (separator == NULL) {
        safe_copy_string(directory_out, directory_out_size, ".");
        return;
    }

    prefix_len = (size_t)(separator - path);
    if (prefix_len == 0) {
        safe_copy_string(directory_out, directory_out_size, ".");
        return;
    }

    if (prefix_len == 2 && path[1] == ':') {
        prefix_len = 3;
    }

    if (prefix_len >= directory_out_size) {
        prefix_len = directory_out_size - 1;
    }

    memcpy(directory_out, path, prefix_len);
    directory_out[prefix_len] = '\0';
}

static int get_git_hash_for_directory(const char *directory_path, char *git_hash_out, size_t git_hash_out_size)
{
    char command[PATH_BUFFER_LEN + 64];
    FILE *pipe_fp;
    char line[160];
    size_t len;

    if (git_hash_out == NULL || git_hash_out_size == 0) {
        return 0;
    }

    git_hash_out[0] = '\0';

    if (directory_path == NULL || directory_path[0] == '\0') {
        return 0;
    }

    if (snprintf(command, sizeof(command), "git -C \"%s\" rev-parse HEAD 2>nul", directory_path) < 0) {
        return 0;
    }

    pipe_fp = _popen(command, "r");
    if (pipe_fp == NULL) {
        return 0;
    }

    if (fgets(line, sizeof(line), pipe_fp) == NULL) {
        _pclose(pipe_fp);
        return 0;
    }

    _pclose(pipe_fp);

    len = strlen(line);
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n' || isspace((unsigned char)line[len - 1]))) {
        line[len - 1] = '\0';
        --len;
    }

    if (len == 0) {
        return 0;
    }

    safe_copy_string(git_hash_out, git_hash_out_size, line);
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
        } else if (strcmp(argv[i], "-s") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Error: missing target size after -s\n");
                return 0;
            }
            if (!parse_size_string(argv[++i], &options->target_size)) {
                fprintf(stderr, "Error: invalid target size %s\n", argv[i]);
                return 0;
            }
        } else if (strcmp(argv[i], "-v") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Error: missing software version after -v\n");
                return 0;
            }
            options->software_version_text = argv[++i];
        } else if (strcmp(argv[i], "-vf") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Error: missing software version file after -vf\n");
                return 0;
            }
            options->software_version_file = argv[++i];
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
        if (options->software_version_text == NULL && options->software_version_file == NULL) {
            fprintf(stderr, "Error: software version must be specified with -v or -vf\n");
            return 0;
        }
        if (options->software_version_text != NULL && options->software_version_file != NULL) {
            fprintf(stderr, "Error: -v and -vf cannot be used together\n");
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

static int get_current_time_string(char *time_out, size_t time_out_size)
{
    time_t now;
    struct tm *local_time;

    now = time(NULL);
    if (now == (time_t)-1) {
        return 0;
    }

    local_time = localtime(&now);
    if (local_time == NULL) {
        return 0;
    }

    if (strftime(time_out, time_out_size, "%Y-%m-%d %H:%M:%S", local_time) == 0) {
        return 0;
    }

    return 1;
}

static void fill_tail(BinTail *tail, uint16_t software_version, uint16_t hardware_version, const char *pack_time, const char *git_hash)
{
    size_t pack_time_len;
    size_t git_hash_len;

    memset(tail, 0, sizeof(*tail));
    tail->software_version = software_version;
    tail->hardware_version = hardware_version;

    if (pack_time != NULL) {
        pack_time_len = strlen(pack_time);
        if (pack_time_len > sizeof(tail->pack_time)) {
            pack_time_len = sizeof(tail->pack_time);
        }
        memcpy(tail->pack_time, pack_time, pack_time_len);
    }

    if (git_hash != NULL) {
        git_hash_len = strlen(git_hash);
        if (git_hash_len > sizeof(tail->git_hash)) {
            git_hash_len = sizeof(tail->git_hash);
        }
        memcpy(tail->git_hash, git_hash, git_hash_len);
    }

    memcpy(tail->reserved, BIN_TAIL_MAGIC, sizeof(BIN_TAIL_MAGIC));
}

static void tail_git_hash_to_string(const BinTail *tail, char *git_hash_out, size_t git_hash_out_size)
{
    fixed_field_to_string(tail->git_hash, sizeof(tail->git_hash), git_hash_out, git_hash_out_size);
}

static int run_pack_mode(const Options *options)
{
    uint16_t software_version;
    uint16_t hardware_version = 0U;
    char version_text[64];
    char input_directory[PATH_BUFFER_LEN];
    char git_hash[GIT_HASH_LEN + 1];
    char pack_time[PACK_TIME_LEN + 1];
    size_t original_size = 0;
    size_t minimum_size;
    uint8_t *original_data;
    uint8_t *final_data;
    BinTail tail;
    size_t tail_offset;
    uint32_t package_crc32;
    FILE *fp;

    memset(version_text, 0, sizeof(version_text));
    if (options->software_version_text != NULL) {
        safe_copy_string(version_text, sizeof(version_text), options->software_version_text);
    } else {
        if (!read_version_from_file(options->software_version_file, version_text, sizeof(version_text))) {
            return 1;
        }
    }

    if (!parse_u16_value(version_text, &software_version)) {
        fprintf(stderr, "Error: invalid software version %s\n", version_text);
        return 1;
    }

    if (options->hardware_version_text != NULL) {
        if (!parse_u16_value(options->hardware_version_text, &hardware_version)) {
            fprintf(stderr, "Error: invalid hardware version %s\n", options->hardware_version_text);
            return 1;
        }
    }

    get_directory_from_path(options->input_path, input_directory, sizeof(input_directory));
    if (!get_git_hash_for_directory(input_directory, git_hash, sizeof(git_hash))) {
        safe_copy_string(git_hash, sizeof(git_hash), "UNKNOWN");
    }

    if (!get_current_time_string(pack_time, sizeof(pack_time))) {
        fprintf(stderr, "Error: failed to get pack time\n");
        return 1;
    }

    original_data = read_binary_file(options->input_path, &original_size);
    if (original_data == NULL) {
        fprintf(stderr, "Error: input bin must not be empty\n");
        return 1;
    }

    minimum_size = original_size + sizeof(BinTail);
    if (options->target_size < original_size) {
        fprintf(stderr, "Error: target size %u bytes is smaller than original bin size %u bytes\n",
                (unsigned int)options->target_size,
                (unsigned int)original_size);
        free(original_data);
        return 1;
    }

    if (options->target_size < minimum_size) {
        fprintf(stderr, "Error: target size %u bytes is too small for payload(%u) + tail(%u), minimum required is %u bytes\n",
                (unsigned int)options->target_size,
                (unsigned int)original_size,
                (unsigned int)sizeof(BinTail),
                (unsigned int)minimum_size);
        free(original_data);
        return 1;
    }

    final_data = (uint8_t *)calloc(options->target_size, 1);
    if (final_data == NULL) {
        fprintf(stderr, "Error: memory allocation failed for output buffer\n");
        free(original_data);
        return 1;
    }

    memcpy(final_data, original_data, original_size);

    tail_offset = options->target_size - sizeof(BinTail);
    fill_tail(&tail, software_version, hardware_version, pack_time, git_hash);
    memcpy(final_data + tail_offset, &tail, sizeof(tail));

    package_crc32 = crc32_calculate(final_data, options->target_size - sizeof(uint32_t));
    memcpy(final_data + options->target_size - sizeof(uint32_t), &package_crc32, sizeof(package_crc32));

    fp = fopen(options->output_path, "wb");
    if (fp == NULL) {
        fprintf(stderr, "Error: failed to create output file %s\n", options->output_path);
        free(final_data);
        free(original_data);
        return 1;
    }

    if (fwrite(final_data, 1, options->target_size, fp) != options->target_size) {
        fprintf(stderr, "Error: failed to write output file %s\n", options->output_path);
        fclose(fp);
        free(final_data);
        free(original_data);
        return 1;
    }

    fclose(fp);

    printf("Pack success\n");
    printf("Input file       : %s\n", options->input_path);
    printf("Output file      : %s\n", options->output_path);
    printf("Software version : 0x%04X\n", software_version);
    printf("Hardware version : 0x%04X\n", hardware_version);
    printf("Pack time        : %s\n", pack_time);
    printf("Git hash         : %s\n", git_hash);
    printf("Original size    : %u bytes\n", (unsigned int)original_size);
    printf("Tail size        : %u bytes\n", (unsigned int)sizeof(BinTail));
    printf("Reserved size    : %u bytes\n", (unsigned int)sizeof(tail.reserved));
    printf("Package CRC32    : 0x%08X\n", package_crc32);
    printf("Final size       : %u bytes\n", (unsigned int)options->target_size);

    if (strcmp(git_hash, "UNKNOWN") == 0) {
        printf("Git status       : git hash not found, stored as UNKNOWN\n");
    }

    free(final_data);
    free(original_data);
    return 0;
}

static int run_verify_mode(const Options *options)
{
    size_t packed_size = 0;
    uint8_t *packed_data;
    BinTail tail;
    uint32_t stored_crc32;
    uint32_t actual_crc32;
    char git_hash[GIT_HASH_LEN + 1];
    char pack_time[PACK_TIME_LEN + 1];

    packed_data = read_binary_file(options->input_path, &packed_size);
    if (packed_data == NULL) {
        fprintf(stderr, "Error: packed bin must not be empty\n");
        return 1;
    }

    if (packed_size < sizeof(BinTail)) {
        fprintf(stderr, "Verify failed: packed file is smaller than tail size\n");
        free(packed_data);
        return 1;
    }

    memcpy(&tail, packed_data + packed_size - sizeof(BinTail), sizeof(BinTail));
    memcpy(&stored_crc32, packed_data + packed_size - sizeof(uint32_t), sizeof(uint32_t));

    if (memcmp(tail.reserved, BIN_TAIL_MAGIC, sizeof(BIN_TAIL_MAGIC)) != 0) {
        fprintf(stderr, "Verify failed: tail signature not found, this file is not a valid packed bin\n");
        free(packed_data);
        return 1;
    }

    actual_crc32 = crc32_calculate(packed_data, packed_size - sizeof(uint32_t));
    tail_git_hash_to_string(&tail, git_hash, sizeof(git_hash));
    fixed_field_to_string(tail.pack_time, sizeof(tail.pack_time), pack_time, sizeof(pack_time));

    printf("Verify report\n");
    printf("Packed file      : %s\n", options->input_path);
    printf("Software version : 0x%04X\n", tail.software_version);
    printf("Hardware version : 0x%04X\n", tail.hardware_version);
    printf("Pack time        : %s\n", pack_time);
    printf("Git hash         : %s\n", git_hash[0] != '\0' ? git_hash : "UNKNOWN");
    printf("Tail size        : %u bytes\n", (unsigned int)sizeof(BinTail));
    printf("Reserved size    : %u bytes\n", (unsigned int)sizeof(tail.reserved));
    printf("Stored CRC32     : 0x%08X\n", stored_crc32);
    printf("Actual CRC32     : 0x%08X\n", actual_crc32);
    printf("File size        : %u bytes\n", (unsigned int)packed_size);

    if (stored_crc32 != actual_crc32) {
        fprintf(stderr, "Verify failed: CRC32 mismatch\n");
        free(packed_data);
        return 1;
    }

    printf("Verify result    : OK\n");

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
