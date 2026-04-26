# VS Code STLink 调试配置指南

## 前置要求

### 1. 安装必要的VS Code扩展
- **Cortex-Debug** (marus25.cortex-debug) - 核心调试扩展
- **C/C++** (ms-vscode.cpptools) - C/C++支持
- **Embedded Tools** (ms-vscode.vscode-embedded) - 嵌入式开发工具

### 2. 安装系统工具

#### Windows 安装步骤：
```bash
# 使用 vcpkg 或直接下载
# 需要安装以下工具之一：
1. ST-Link Utility (STM官方工具)
   https://www.st.com/en/development-tools/stsw-link004.html

2. OpenOCD (开源替代品，推荐)
   - 下载: https://gnutoolchain.com/arm-eabi/windows/
   - 或使用 vcpkg: vcpkg install openocd:x64-windows

3. arm-none-eabi-gdb (GDB工具链)
   - 下载: https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm
```

### 3. 确保编译输出
- 项目需要编译生成 `.elf` 或 `.axf` 文件
- 在Keil中编译时勾选生成调试信息

## 配置说明

**launch.json** 文件已为你创建，主要配置项：
- `executable`: 指向编译生成的ELF文件
- `servertype`: 使用的GDB服务器（st-link 或 openocd）
- `device`: STM32H723VG 目标设备
- `cwd`: 工作目录
- `watch`: 要监视的变量列表

## 使用方式

1. **启动调试**：
   - 按 `F5` 或通过菜单 Run → Start Debugging
   - 或点击左侧"运行和调试"面板中的启动按钮

2. **监视变量**：
   - 在 launch.json 的 `watch` 数组中添加变量名
   - 或在调试期间在"监视"面板手动添加

3. **设置断点**：
   - 在代码行号处左键单击设置断点（红点）
   - 程序会在断点处暂停

## 常见问题

### 找不到GDB服务器
确保 openocd.exe 或 st-util.exe 在系统PATH中，或修改launch.json中的完整路径

### 连接失败
1. 检查ST-Link硬件连接
2. 检查Keil中的调试配置（Settings → Device）
3. 尝试先在Keil中运行调试确认硬件工作正常

### 程序不停在断点
确保编译时启用了调试符号（-g 选项）
