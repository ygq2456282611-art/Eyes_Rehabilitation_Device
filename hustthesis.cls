% ============================================================
%  hustthesis.cls  —  河南科技大学本科毕业设计(论文) LaTeX 模板
%  基于 ctexbook，适配 2025 版撰写规范
%  版本: v1.0  日期: 2026/05/16
% ============================================================
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{hustthesis}[2026/05/16 v1.0 HUST Thesis Template]

% -------- 基础类 --------
\LoadClass[zihao=-4, openany, UTF8, oneside]{ctexbook}

% -------- 必需宏包 --------
\RequirePackage{geometry}
\RequirePackage{fancyhdr}
\RequirePackage{setspace}
\RequirePackage{caption}
\RequirePackage{subcaption}
\RequirePackage{amsmath,amssymb}
\RequirePackage{graphicx}
\RequirePackage{booktabs}
\RequirePackage{longtable}
\RequirePackage{array}
\RequirePackage{multirow}
\RequirePackage{float}
\RequirePackage[colorlinks=true,linkcolor=black,citecolor=black,urlcolor=black]{hyperref}
\RequirePackage{cleveref}
\RequirePackage{enumitem}
\RequirePackage{xcolor}
\RequirePackage{titlesec}

% ============================================================
%  页面设置
%  上/下 2.54cm, 左 2.5cm + 装订线 1cm, 右 2.5cm
% ============================================================
\geometry{
  a4paper,
  top    = 2.54cm,
  bottom = 2.54cm,
  left   = 2.5cm,
  right  = 2.5cm,
  bindingoffset = 1cm,
  headheight = 15pt,
  headsep    = 5mm,
  footskip   = 10mm,
}

% ============================================================
%  字体设置 —— 使用 Windows 系统字体
% ============================================================
\setCJKmainfont{宋体}[BoldFont=黑体, ItalicFont=楷体]
\setCJKsansfont{黑体}
\setCJKmonofont{楷体}
\setmainfont{Times New Roman}
\setsansfont{Arial}
\setmonofont{Courier New}

% ============================================================
%  行距：正文 1.5 倍
% ============================================================
\onehalfspacing

% ============================================================
%  页眉页脚
%  页眉: "河南科技大学毕业设计(论文)" 小五宋体居中，下两横线
%  页脚: 页码居中
% ============================================================
\pagestyle{fancy}
\fancyhf{}
\fancyhead[C]{\zihao{-5}\songti 河南科技大学毕业设计(论文)}
\fancyfoot[C]{\thepage}
\renewcommand{\headrulewidth}{0pt}

% 双横线页眉装饰
\makeatletter
\def\headrule{%
  \if@fancyplain\let\headrulewidth\plainheadrulewidth\fi
  \hrule\@height 0.4pt \@width\headwidth
  \vskip 1.2pt
  \hrule\@height 0.4pt \@width\headwidth
  \vskip -1.2pt}
\makeatother

% plain 风格(章首页): 无页眉线, 页码居中
\fancypagestyle{plain}{
  \fancyhf{}
  \fancyhead[C]{\zihao{-5}\songti 河南科技大学毕业设计(论文)}
  \fancyfoot[C]{\thepage}
  \renewcommand{\headrule}{%
    \hrule\@height 0.4pt \@width\headwidth
    \vskip 1.2pt
    \hrule\@height 0.4pt \@width\headwidth
    \vskip -1.2pt}
}

% ============================================================
%  章/节/小节 标题格式
%  章: 三号黑体居中, "第1章  标题", 章下空两行
%  节: 四号黑体左起, "§1.1  标题", 上下各空一行
%  小节: 小四黑体左起, "§1.1.1  标题", 不空行
% ============================================================
\ctexset{
  chapter = {
    format      = \centering\zihao{3}\heiti,
    name        = {第,章},
    number      = \arabic{chapter},
    aftername   = \quad,
    beforeskip  = 0pt,
    afterskip   = 36pt,
    pagestyle   = fancy,
    fixskip     = true,
  },
  section = {
    format      = \zihao{4}\heiti\raggedright,
    name        = {§,},
    number      = \arabic{chapter}.\arabic{section},
    aftername   = \quad,
    beforeskip  = 18pt,
    afterskip   = 18pt,
    fixskip     = true,
  },
  subsection = {
    format      = \zihao{-4}\heiti\raggedright,
    name        = {§,},
    number      = \arabic{chapter}.\arabic{section}.\arabic{subsection},
    aftername   = \quad,
    beforeskip  = 0pt,
    afterskip   = 0pt,
    fixskip     = true,
  },
  subsubsection = {
    format      = \zihao{-4}\heiti\raggedright,
    aftername   = \quad,
    beforeskip  = 0pt,
    afterskip   = 0pt,
    fixskip     = true,
  },
}

% ============================================================
%  目录标题
% ============================================================
\renewcommand{\contentsname}{目\ \ 录}

% ============================================================
%  图表标题: 五号楷体
%  图: 编号居中在下方, 表: 左对齐在上方
%  编号: 图3-2 / 表5-4 (按章编号)
% ============================================================
\DeclareCaptionFont{kaifive}{\zihao{5}\kaishu}
\captionsetup{
  font        = kaifive,
  labelsep    = quad,
  justification = centering,
  format      = hang,
}
\captionsetup[figure]{ position = bottom }
\captionsetup[table]{
  position = top,
  justification = raggedright,
  singlelinecheck = false,
}

\renewcommand{\figurename}{图}
\renewcommand{\tablename}{表}
\renewcommand{\thefigure}{\thechapter-\arabic{figure}}
\renewcommand{\thetable}{\thechapter-\arabic{table}}

% 表格内: 五号宋体, 单倍行距
\AtBeginEnvironment{table}{\zihao{5}\songti\singlespacing}

% ============================================================
%  公式编号: (章-序号), 如 (3-2)
% ============================================================
\renewcommand{\theequation}{\thechapter-\arabic{equation}}

% ============================================================
%  参考文献标题格式
% ============================================================
\AtBeginDocument{%
  \defbibheading{bibliography}[\bibname]{%
    \chapter*{#1}%
  }%
}

% ============================================================
%  附录标题格式
% ============================================================
\ctexset{
  appendix = {
    name    = {附录,},
    number  = \Alph{chapter},
  }
}

% ============================================================
%  封面命令
% ============================================================
\newcommand{\husttitle}{}
\newcommand{\hustauthor}{}
\newcommand{\hustinstitute}{}
\newcommand{\hustmajor}{}
\newcommand{\hustadvisor}{}
\newcommand{\hustdate}{}

\newcommand{\thesistitle}[1]{\renewcommand{\husttitle}{#1}}
\newcommand{\thesisauthor}[1]{\renewcommand{\hustauthor}{#1}}
\newcommand{\thesisinstitute}[1]{\renewcommand{\hustinstitute}{#1}}
\newcommand{\thesismajor}[1]{\renewcommand{\hustmajor}{#1}}
\newcommand{\thesisadvisor}[1]{\renewcommand{\hustadvisor}{#1}}
\newcommand{\thesisdate}[1]{\renewcommand{\hustdate}{#1}}

\endinput
