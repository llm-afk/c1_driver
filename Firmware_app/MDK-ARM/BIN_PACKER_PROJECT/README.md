# BIN_PACKER_PROJECT

`BIN_PACKER_PROJECT` 是一个用于对原始 `bin` 固件做二次封装的打包项目。
它会在不破坏原始固件主体内容的前提下，在文件尾部插入固定 `128` 字节的版本与校验信息区，便于升级、追踪、读取和验包。

## 0. 项目目录

```text
BIN_PACKER_PROJECT/
  README.md
  src/
    bin_packer.c
  include/
    README.md
  bin/
    bin_packer.exe
    bin_packer_new.exe
  examples/
    sample.bin
    sample_pack.bin
    sample_version.txt
  legacy/
    bin_packer_simple.c
```

目录说明：

- `README.md`：项目总说明，面向使用者
- `src/`：正式源码目录
- `src/bin_packer.c`：当前主打包程序源码
- `include/`：预留头文件目录
- `bin/`：编译后的可执行文件
- `examples/`：示例输入、版本文件和打包输出
- `legacy/`：旧版或简化版代码

源码主文件：
- [bin_packer.c](/BIN_PACKER_PROJECT/src/bin_packer.c)

## 1. 设计目的

这个打包程序的核心目的，是在不破坏原始 `bin` 主体内容的前提下，在最终固件末尾追加一段固定格式、固定大小的版本信息区，方便后续使用、升级、追踪和校验。

1. 方便后续程序升级  
   升级程序、上位机、产线工具或单片机自身都可以直接从固件中读取版本信息，不需要再单独维护一份版本表。

2. 做软、硬件版本比较  
   单片机启动后可以读取软件版本和硬件版本，并与当前设备型号、板卡版本或升级条件做比对，避免误升级。

3. 可以通过 Git 提交方便追踪定位到哪一版本  
   打包时会尝试写入当前工程对应的 Git 提交哈希，后续定位问题时可以快速追溯到具体源码版本和对应提交记录。

4. 预留的存储空间  
   当前结构中专门预留了固定空间，后续即使要增加字段，也可以优先使用预留区。

5. 追溯到是谁编写的、何时编写、理清版本控制  
   当前尾部直接保存打包时间，并保存 Git 提交哈希；结合代码仓库的提交记录，可以继续追溯到对应提交人、提交时间和变更内容。需要注意，当前程序本身并不直接把“编写者姓名”写入 `bin`，而是通过 Git 提交记录和仓库历史间接追溯。

6. 在严谨控制环境中，单片机内部运行前可以先用 CRC 校验确保程序正确  
   在烧录前、升级前，或者由 BootLoader 在程序正式跳转运行前，都可以先对打包后的固件做 CRC32 校验，确认文件内容正确、未损坏、未被误改。

7. 方便做 CRC 校验和验包  
   打包程序本身提供了验包能力，任何人拿到打包后的 `bin` 文件，都可以通过 `--verify` 重新校验文件完整性，确认当前文件是否与打包时一致。


## 2. 打包后文件结构

打包程序不会改动原始固件正文内容，而是在最终输出文件的末尾插入一段固定 `128` 字节的尾部信息区。

最终文件结构如下：

```text
+------------------------------+
| 原始 bin 内容                 |
+------------------------------+
| 中间补 0x00                   |
+------------------------------+
| 128 字节尾部信息区            |
+------------------------------+
```

其中：

- 原始 `bin` 内容保持不变
- 如果目标大小大于原始 `bin` 大小，空余部分会用 `0x00` 补齐
- 最后 `128` 字节用于存放版本和校验信息等

## 3. 128 字节尾部信息格式

当前尾部结构固定如下：

```c
#pragma pack(push, 1)
typedef struct BinTailTag {
    uint16_t software_version;   /* 2 bytes */
    uint16_t hardware_version;   /* 2 bytes */
    char     pack_time[20];      /* 20 bytes */
    char     git_hash[64];       /* 64 bytes */
    char     reserved[36];       /* 36 bytes */
    uint32_t package_crc32;      /* 4 bytes */
} BinTail;
#pragma pack(pop)
```

字段顺序固定为：

1. `software_version`
2. `hardware_version`
3. `pack_time`
4. `git_hash`
5. `reserved`
6. `package_crc32`

各字段偏移如下：

|        字段        | 偏移 | 长度 |     说明     |
|--------------------|----:|-----:|-------------|
| `software_version` | `0` | `2`  | 软件版本号   |
| `hardware_version` | `2` | `2`  | 硬件版本号   |
| `pack_time`        | `4` | `20` | 打包时间     |
| `git_hash`         | `24` | `64`| Git 提交哈希 |
| `reserved`         | `88` | `36`| 预留区       |
| `package_crc32`    | `124`| `4` | 文件 CRC32   |

说明：

- `pack_time[20]` 当前写入的时间文本格式为 `YYYY-MM-DD HH:MM:SS`
- `git_hash[64]` 可以兼容 `40` 位 SHA-1，保留了更大扩展空间
- `reserved[36]` 当前默认填 `0x00`
- `package_crc32` 位于整个文件最后 `4` 字节

## 4. CRC32 规则

尾部中的 `package_crc32` 用于校验整个打包后文件的完整性。

CRC32 计算规则为：

- 从文件第 `0` 字节开始
- 一直计算到倒数第 `5` 字节为止
- 不包含最后 `4` 字节的 `package_crc32` 字段本身

可以理解为：

```text
CRC32 = 对整个打包后 bin 文件除最后 4 字节外的所有数据做 CRC32
```

当前程序使用的 CRC32 参数为：

- 多项式：`0xEDB88320`
- 初值：`0xFFFFFFFF`
- 结束异或：`0xFFFFFFFF`

## 5. Git 信息写入规则

程序会根据输入 `bin` 所在目录，尝试执行：

```text
git -C "<输入bin所在目录>" rev-parse HEAD
```

如果成功，则把当前提交哈希写入 `git_hash[64]`。  
如果失败，则写入：

```text
UNKNOWN
```

常见失败原因：

- 当前电脑没有安装 `git`
- 输入 `bin` 所在目录不是 Git 仓库
- 当前环境不能正常执行 Git 命令

## 6. 打包程序如何使用

## 6.1 命令格式

打包：

```bash
bin\\bin_packer.exe -i <input.bin> [-o <output.bin>] -s <target_size> (-v <software_version> | -vf <version_file>) [-hv <hardware_version>]
```

校验：

```bash
bin\\bin_packer.exe --verify -i <packed.bin>
```

## 6.2 参数说明

- `-i`：输入原始 `bin` 文件
- `-o`：输出文件路径，不写时默认生成 `app_pack.bin`
- `-s`：目标输出文件总大小
- `-v`：直接指定软件版本号，例如 `0x0001`
- `-vf`：从文本文件中读取软件版本号
- `-hv`：硬件版本号，例如 `0x0002`，不写时默认 `0x0000`
- `--verify`：对已经打包完成的 `bin` 做尾部校验

## 6.3 版本号来源

软件版本支持两种方式：

1. 直接通过 `-v` 指定  
   示例：

```bash
bin\\bin_packer.exe -i app.bin -s 300KB -v 0x0001 -hv 0x0002
```

2. 通过 `-vf` 从文件中提取  
   示例：

```bash
bin\\bin_packer.exe -i app.bin -s 300KB -vf version.h -hv 0x0002
```

版本文件中支持类似以下内容：

```c
#define APP_VERSION 0x0001
```

```c
#define SOFTWARE_VER "0x0001"
```

```text
VERSION=0x0001
```

```text
VER:0x0001
```

```text
0x0001
```

要求最终能解析成 `uint16_t`，也就是范围必须在：

```text
0x0000 ~ 0xFFFF
```

## 6.4 使用示例

示例 1：手动指定软件版本和硬件版本

```bash
bin\\bin_packer.exe -i rtthread.bin -o rtthread_pack.bin -s 300KB -v 0x0102 -hv 0x0003
```

示例 2：从版本文件读取软件版本

```bash
bin\\bin_packer.exe -i rtthread.bin -s 300KB -vf version.h -hv 0x0003
```

示例 3：只验证打包后的文件是否完整

```bash
bin\\bin_packer.exe --verify -i rtthread_pack.bin
```

校验通过时，程序会输出类似：

```text
Verify result    : OK
```

## 7. 使用流程建议

1. 准备原始编译生成的 `bin` 文件
2. 确认本次发布要使用的软件版本号和硬件版本号
3. 明确最终固件总大小，例如 `128KB`、`256KB`、`300KB`
4. 运行打包程序生成新的打包后 `bin`
5. 对打包后的 `bin` 执行一次 `--verify`
6. 将校验通过的打包文件交给烧录、升级或生产环节
7. 单片机程序按约定位置读取尾部信息，用于运行时显示、上报或升级判断

## 8. 单片机读取尾部信息

单片机要读取尾部信息，前提需要知道：

1. 打包后固件总大小
2. 尾部固定大小是 `128` 字节
3. 固件烧录到 Flash 的起始地址

### 8.1 直接用指针读取 Flash

类似在很多 ARM Cortex-M 单片机上可以这样查询：

```c
/* 读取 32 位数据 */
uint32_t *ptr = (uint32_t *)0x08001000;
uint32_t data = *ptr;

/* 按字节读取 */
uint8_t *byte_ptr = (uint8_t *)0x08001000;
uint8_t byte_data = *byte_ptr;
```

### 8.2 读取起始地址的计算方式

假设：

- 固件烧录起始地址为 `APP_BASE_ADDR`
- 打包后固件总大小为 `IMAGE_SIZE`
- 尾部大小固定为 `128`

则尾部起始地址为：

```c
tail_addr = APP_BASE_ADDR + IMAGE_SIZE - 128;
```

例如：如果最终文件固定为 300KB

这时相对于固件起始地址的字段偏移就是：

|        字段        | 文件内偏移 | 十六进制  |
|--------------------|----------:|---------:|
| `software_version` | `307072` | `0x4AF80` |
| `hardware_version` | `307074` | `0x4AF82` |
| `pack_time`        | `307076` | `0x4AF84` |
| `git_hash`         | `307096` | `0x4AF98` |
| `reserved`         | `307160` | `0x4AFD8` |
| `package_crc32`    | `307196` | `0x4AFFC` |

## 9. 如何做 CRC 校验

## 9.1 在 PC 端校验

最简单的方式就是直接执行：

```bash
bin\\bin_packer.exe --verify -i app_pack.bin
```

这个命令会：

1. 读取末尾 `128` 字节尾部信息
2. 先检查尾部签名是否存在，确认该文件确实是本工具打包出来的文件
3. 取出最后 `4` 字节的 `package_crc32`
4. 对整个文件除最后 `4` 字节外的所有内容重新计算 CRC32
5. 比较两个 CRC32 是否一致

如果一致，则说明文件内容完整。
如果是普通原始 `bin`，通常会直接报错提示不是有效的打包文件，不会继续按尾部信息解析。

## 9.2 在单片机端校验

如果升级包先被写入外部 Flash、内部 Flash 或下载到某个缓存区域，也可以在 MCU 端重算 CRC32。

核心步骤如下：

1. 读取尾部最后 `4` 字节中的 `package_crc32`
2. 对整个固件区除最后 `4` 字节外的数据重新做 CRC32
3. 比较计算值和尾部记录值
4. 一致则认为固件完整，不一致则拒绝升级或提示错误

示意代码：

```c
#include <stdint.h>

#define APP_BASE_ADDR   0x08000000UL
#define IMAGE_SIZE      (300U * 1024U)

static uint32_t App_Crc32(const uint8_t *data, uint32_t len);

int App_VerifyPackedBin(void)
{
    const uint8_t *image = (const uint8_t *)APP_BASE_ADDR;
    const uint32_t stored_crc =
        ((uint32_t)image[IMAGE_SIZE - 4]) |
        ((uint32_t)image[IMAGE_SIZE - 3] << 8) |
        ((uint32_t)image[IMAGE_SIZE - 2] << 16) |
        ((uint32_t)image[IMAGE_SIZE - 1] << 24);
    uint32_t calc_crc = App_Crc32(image, IMAGE_SIZE - 4);

    return (stored_crc == calc_crc) ? 1 : 0;
}
```

注意：

- `package_crc32` 在当前文件中按小端方式存放
- 如果你的单片机是大端平台，请按字节重新组包
- MCU 端 CRC 算法参数必须与 PC 端保持一致，否则结果会不同
- 如果要做到“程序正式运行前先校验”，更适合把这段逻辑放在 `BootLoader` 或启动阶段，而不是放在应用主业务已经开始运行之后

## 10. 使用者必须注意的事项


### 10.1 目标总大小必须留出尾部空间

现在尾部固定为 `128` 字节，所以：

```text
目标总大小 >= 原始 bin 大小 + 128
```

如果目标总大小不够，程序会报错，无法生成文件。

### 10.2 单片机 Flash 空间必须足够

如果你把输出总大小固定为 `300KB`，那么 MCU 可用于这个应用镜像的实际存储空间就必须至少能够容纳 `300KB`。

请特别确认：

- 应用分区大小是否足够
- BootLoader 是否允许该大小的应用镜像
- 升级缓存区是否足够
- 双备份升级时两份镜像空间是否都足够

不要只看“原始程序大小”，要看“打包后的最终大小”。

### 10.3 尽量固定输出大小

如果单片机代码里采用固定地址读取尾部信息，那么输出大小一旦变化，尾部地址也会变化。

因此建议：

- 同一产品固定使用同一个 `-s`
- 单片机程序中也固定使用同一个 `IMAGE_SIZE`

这样读取偏移不会漂移。

## 11. 推荐的实际落地流程

一个比较实用的落地流程如下：

1. 开发编译生成原始 `bin`
2. 使用打包程序生成固定大小的发布 `bin`
3. 运行 `--verify` 确认 CRC 正确
4. 将发布 `bin` 交给测试、产线或升级系统
5. MCU 启动后读取尾部信息并打印或上报
6. 升级程序在升级前检查硬件版本、软件版本和 CRC
7. 出问题时通过 `software_version`、`pack_time`、`git_hash` 追溯版本来源

## 12. 编译说明

如果需要重新编译该工具，直接使用现有源码 [bin_packer.c](/BIN_PACKER_PROJECT/src/bin_packer.c) 即可，例如在项目根目录使用 GCC：

```bash
gcc -O2 -Wall -Wextra -o bin\\bin_packer.exe src\\bin_packer.c
```
