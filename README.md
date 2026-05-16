# 河南科技大学本科毕业设计（论文）LaTeX 模板

> **仅供毕业论文写作使用，请勿用于其他用途。**

基于 ctexbook 定制，适配河南科技大学 2025 版毕业设计（论文）撰写规范。

## 项目说明

本模板基于个人毕业设计项目——**基于 STM32H7 的脑卒中眼部康复头戴式装置设计与实现**——定制而成，实现了河南科技大学规定的全部格式要求。

## 格式特性

| 项目 | 说明 |
|------|------|
 | 页面 | A4，上/下 2.54cm，左 2.5cm + 1cm 装订线，右 2.5cm |
 | 页眉 | "河南科技大学毕业设计(论文)"，宋体小五居中，双横线 |
 | 章标题 | 三号黑体居中，另起一页，下空两行 |
 | 节标题 | §1.1 四号黑体左起，上下各空一行 |
 | 小节标题 | §1.1.1 小四黑体左起 |
 | 正文 | 小四宋体，1.5 倍行距 |
 | 图题/表题 | 五号楷体 |
 | 参考文献 | GB/T 7714-2015 格式 |

## 使用方法

### 环境要求

- TeX Live 2026+（推荐）或 MacTeX
- 编译引擎：XeLaTeX（**不支持 pdfLaTeX**）
- Windows 系统字体（宋体 / 黑体 / 楷体 / Times New Roman）

### 编译

**命令行（Windows）：**
```bat
cd 论文目录
make.bat
```

**VS Code + LaTeX Workshop：**
1. 打开项目根目录
2. 打开 `main.tex`
3. Ctrl+Shift+P → `LaTeX Workshop: Build with recipe` → `latexmk (xelatex)`

### Overleaf

如需在线使用，将项目打包为 ZIP 上传至 Overleaf，设置：
- 编译器：XeLaTeX
- 主入口：`main.tex`

## 文件结构

```
├── hustthesis.cls          % 格式类文件
├── main.tex                % 主入口
├── make.bat                % Windows 编译脚本
├── latexmkrc               % latexmk 配置文件
├── chapters/               % 章节文件
│   ├── 00_cover.tex        % 封面
│   ├── 00_declaration.tex  % 声明
│   ├── 00_abstract.tex     % 中英文摘要
│   ├── 01_intro.tex        % 第1章
│   ├── ...
│   └── 99_ack.tex          % 致谢
├── figures/                % 图片目录
└── bib/                    % 参考文献
    └── ref.bib
```

## 许可

仅供学习与毕业论文写作使用。
