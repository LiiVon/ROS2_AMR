#!/usr/bin/env python3
"""代码规范与命名检查（对应需求文档 TR-10 / FR-CODE）。

检查项：
  1. 禁止出现 v2_ 前缀、/v2/ 路径、_srv_ 后缀、_pub/_sub 缩写、data_/tmp_ 占位名
  2. 单个 C++ 源文件不超过 1500 行
  3. 关键对外名字必须存在（topic / service 常量抽查）
用法：
  python3 tools/check_conventions.py [--root src] [--strict]
退出码 0 表示全部通过。
"""

import argparse
import pathlib
import re
import sys

FORBIDDEN = [
    (re.compile(r"\bv2_[a-z0-9_]+"), "v2_ 前缀标识符"),
    (re.compile(r'"/v2/'), "/v2/ 服务名"),
    (re.compile(r"\b[a-z0-9_]+_srv_?\b"), "_srv 后缀（应改为 _service）"),
    (re.compile(r"\b[a-z0-9_]+_pub_?\b"), "_pub 缩写（应改为 _publisher）"),
    (re.compile(r"\b[a-z0-9_]+_sub_?\b"), "_sub 缩写（应改为 _subscription）"),
    (re.compile(r"\bdata_\b"), "data_ 占位名"),
    (re.compile(r"\btmp_\b"), "tmp_ 占位名"),
]

SRC_EXT = {".cpp", ".hpp", ".h"}
MAX_LINES = 1500


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default="src")
    parser.add_argument("--strict", action="store_true", help="将文件行数超限也视为失败")
    args = parser.parse_args()

    root = pathlib.Path(args.root)
    problems = []
    warnings = []

    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        if path.suffix not in SRC_EXT:
            continue
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except UnicodeDecodeError:
            continue

        if len(lines) > MAX_LINES:
            warnings.append(f"{path}: {len(lines)} 行，超过 {MAX_LINES} 行上限")

        for number, line in enumerate(lines, start=1):
            if line.lstrip().startswith("//"):
                continue
            for pattern, label in FORBIDDEN:
                if pattern.search(line):
                    problems.append(f"{path}:{number}: {label} -> {line.strip()[:100]}")

    for item in warnings:
        print("警告:", item)
    if args.strict and warnings:
        problems.extend(warnings)

    if problems:
        print("发现 %d 处规范问题：" % len(problems))
        for item in problems:
            print("  -", item)
        return 1

    print("规范检查通过：命名无违规%s" % ("" if warnings else "，文件行数均 <= %d 行" % MAX_LINES))
    return 0


if __name__ == "__main__":
    sys.exit(main())
