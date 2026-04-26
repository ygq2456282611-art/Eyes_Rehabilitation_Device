# VS Code STLink 调试 - 快速配置指南

## 📋 已为你创建的文件

1. **launch.json** - 调试配置文件（3个调试配置）
2. **stm32h7.cfg** - OpenOCD配置文件  
3. **tasks.json** - 编译任务配置
4. **README_DEBUG.md** - 详细说明文档

---

## ⚙️ 必要的修改

### 修改 1: 更新ELF文件路径
在 **launch.json** 中，修改 `executable` 指向你的编译输出：

```json
"executable": "${workspaceFolder}/CtrBoard-H7_IMU/CtrBoard-H7_IMU.elf"
```

如果Keil编译输出在其他路径，更新为正确路径。

### 修改 2: 设置要监视的变量
在 **launch.json** 中，修改 `watch` 数组添加要监视的变量名：

```json
"watch": [
    "imu_data.accel_x",
    "imu_data.accel_y", 
    "imu_data.accel_z",
    "temperature",
    "sensor_status"
]
```

### 修改 3: 配置GDB工具链路径
在 **launch.json** 中，设置ARM工具链路径（如需要）：

```json
"armToolchainPath": "C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin"
```

### 修改 4: 更新构建命令（可选）
在 **tasks.json** 中，确保Keil路径正确：

```json
"command": "C:\\Keil_v5\\UV4.exe"
```

---

## 🚀 快速开始

### 选择调试方式

**方式1: 使用 OpenOCD（推荐）**
```bash
# 1. 安装 OpenOCD
#    Windows: 从 https://gnutoolchain.com/arm-eabi/windows/ 下载
#    或: vcpkg install openocd:x64-windows

# 2. 在 launch.json 中选择配置 "STM32H723 - ST-Link (OpenOCD)"

# 3. 按 F5 启动调试
```

**方式2: 使用 ST-Link Utility**
```bash
# 1. 从 ST官网下载: https://www.st.com/en/development-tools/stsw-link004.html

# 2. 在 launch.json 中选择配置 "STM32H723 - ST-Link (ST-Util)"

# 3. 按 F5 启动调试
```

---

## 📝 launch.json 关键配置项说明

| 配置项 | 说明 | 示例 |
|--------|------|------|
| `executable` | 编译输出的ELF文件路径 | `${workspaceFolder}/build/app.elf` |
| `servertype` | 调试服务器类型 | `openocd`, `st-util`, `j-link` |
| `device` | 目标芯片型号 | `STM32H723VG` |
| `interface` | 调试接口 | `swd` (推荐), `jtag` |
| `watch` | 监视的变量数组 | `["var1", "var2"]` |
| `runToEntryPoint` | 运行到函数 | `main` |
| `postInitCommands` | 初始化后执行的命令 | `["load", "continue"]` |

---

## 🐛 调试时的常用操作

| 操作 | 快捷键/方式 |
|------|-----------|
| 启动调试 | F5 |
| 继续执行 | F5 或 Ctrl+F5 |
| 暂停 | F6 |
| 单步执行 | F10 |
| 进入函数 | F11 |
| 跳出函数 | Shift+F11 |
| 设置断点 | 点击行号左侧 |
| 添加监视变量 | 在"监视"面板+按钮 |

---

## 📊 监视变量示例

在代码中定义变量后，在launch.json的watch中添加：

```json
"watch": [
    "counter",
    "temperature",
    "sensor_data.x",
    "sensor_data.y",
    "sensor_data.z",
    "system_state"
]
```

调试时这些变量会显示在左侧"调试"面板的"监视"部分。

---

## ❌ 常见问题排查

### 问题1: "找不到可执行文件"
✓ 检查ELF文件是否存在  
✓ 路径是否正确（用 `/` 而不是 `\`）  
✓ 在Keil中编译生成ELF文件

### 问题2: "无法连接到ST-Link"  
✓ 检查硬件连接  
✓ 在Keil中测试调试确认硬件正常  
✓ 检查ST-Link驱动是否安装  

### 问题3: "程序不停在断点"
✓ 检查编译选项中是否启用调试符号 (-g)  
✓ 确保加载了正确的ELF文件  
✓ 尝试重启调试会话

### 问题4: "找不到源文件"
✓ 检查includePath配置是否正确  
✓ 验证源文件路径

---

## 💡 高级配置

### 添加 SVD 文件查看寄存器
SVD（System View Description）文件可以让你查看芯片的寄存器和外设。

```json
"svdFile": "${workspaceFolder}/.vscode/STM32H723.svd"
```

从这里下载：https://github.com/posborne/cmsis-svd/tree/master/data/STMicroelectronics

### 自定义GDB命令
```json
"postInitCommands": [
    "set print pretty on",
    "set print array on",
    "define refresh",
    "  info locals",
    "end"
]
```

---

## 📚 参考资源

- Cortex-Debug 扩展: https://github.com/Marus25/cortex-debug
- OpenOCD: https://openocd.org/
- STM32H7 参考: https://www.st.com/en/microcontrollers-microprocessors/stm32h7-series.html

