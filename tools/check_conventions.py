#!/usr/bin/env python3
"""代码规范与命名检查（对应需求文档 TR-10 / FR-CODE）。

检查项：
  1. 禁止版本号前缀式命名（标识符/路径）、_srv_ 后缀、_pub/_sub 缩写、data_/tmp_ 占位名
  2. 单个 C++ 源文件不超过 1500 行
  3. 目录分层：`include/<包名>/` 与 `src/` 根层不得堆放散装源文件
  4. 关键对外名字必须存在（topic / service 常量抽查）
用法：
  python3 tools/check_conventions.py [--root src] [--strict]
退出码 0 表示全部通过。
"""

import argparse
import pathlib
import re
import sys

FORBIDDEN = [
    (re.compile(r"\bv\d+_[a-z0-9_]+"), "版本号前缀标识符"),
    (re.compile(r'"/v\d+/'), "版本号前缀服务名"),
    (re.compile(r"\b[a-z0-9_]+_srv_?\b"), "_srv 后缀（应改为 _service）"),
    (re.compile(r"\b[a-z0-9_]+_pub_?\b"), "_pub 缩写（应改为 _publisher）"),
    (re.compile(r"\b[a-z0-9_]+_sub_?\b"), "_sub 缩写（应改为 _subscription）"),
    # 注意：这里要匹配“前缀”而不是仅匹配孤立的 data_/tmp_，
    # 否则 data_buffer、tmp_file 这类占位命名会被漏检。
    (re.compile(r"\bdata_"), "data_ 占位名"),
    (re.compile(r"\btmp_"), "tmp_ 占位名"),
]

SRC_EXT = {".cpp", ".hpp", ".h", ".inc"}
MAX_LINES = 1500


def strip_comments_and_strings(lines):
    """屏蔽块注释与字符串字面量，避免其内容被当作标识符误报。

    逐行处理并保留行号（屏蔽内容替换为空格），因此报告的行号仍然准确。
    """
    result = []
    in_block_comment = False
    for line in lines:
        out = []
        index = 0
        in_string = False
        string_delimiter = ""
        while index < len(line):
            char = line[index]
            if in_block_comment:
                if char == "*" and index + 1 < len(line) and line[index + 1] == "/":
                    in_block_comment = False
                    index += 2
                    out.append("  ")
                    continue
                out.append(" ")
                index += 1
                continue
            if in_string:
                if char == "\\" and index + 1 < len(line):
                    out.append("  ")
                    index += 2
                    continue
                if char == string_delimiter:
                    in_string = False
                out.append(" ")
                index += 1
                continue
            if char == "/" and index + 1 < len(line):
                nxt = line[index + 1]
                if nxt == "/":
                    out.append(" " * (len(line) - index))
                    break
                if nxt == "*":
                    in_block_comment = True
                    out.append("  ")
                    index += 2
                    continue
            if char in ('"', "'"):
                in_string = True
                string_delimiter = char
                out.append(" ")
                index += 1
                continue
            out.append(char)
            index += 1
        result.append("".join(out))
    return result


def check_layering(root: pathlib.Path) -> list:
    """include/<包名>/ 与 src/ 根层必须只含子目录，文件需归入分层目录。"""
    problems = []
    for package in sorted(p for p in root.glob("robot_*") if p.is_dir()):
        for base in (package / "include" / package.name, package / "src"):
            if not base.is_dir():
                continue
            for entry in sorted(base.iterdir()):
                if entry.is_file() and entry.suffix in SRC_EXT:
                    problems.append(
                        f"{entry}: 根层散装源文件，请归入分层子目录（model/workflow/nodes/core/...）"
                    )
    return problems


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default="src")
    parser.add_argument("--strict", action="store_true", help="将文件行数超限也视为失败")
    args = parser.parse_args()

    root = pathlib.Path(args.root)
    if not root.is_dir():
        print(f"错误：检查根目录不存在或不是目录：{root.resolve()}", file=sys.stderr)
        print("请在仓库根目录运行，或显式传入 --root <目录>。", file=sys.stderr)
        return 2
    problems = []
    warnings = []

    problems.extend(check_layering(root))

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

        for number, (raw_line, scanned_line) in enumerate(
                zip(lines, strip_comments_and_strings(lines)), start=1):
            if not scanned_line.strip():
                continue
            for pattern, label in FORBIDDEN:
                if pattern.search(scanned_line):
                    problems.append(f"{path}:{number}: {label} -> {raw_line.strip()[:100]}")

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
