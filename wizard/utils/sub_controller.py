import ast


def extract_function_blocks(source_code):
    """
    source_code: 템플릿 원본 문자열
    return: { func_name: full_text_block_of_that_function }
    """
    tree = ast.parse(source_code)
    lines = source_code.split("\n")

    blocks = {}

    for node in tree.body:
        if isinstance(node, ast.FunctionDef):
            start = node.lineno - 1                        # 0-based index
            end = node.end_lineno                          # end_lineno is inclusive, so slicing is fine
            block = "\n".join(lines[start:end])
            blocks[node.name] = block

    return blocks


def extract_non_function_blocks(source_code):
    tree = ast.parse(source_code)
    lines = source_code.split("\n")

    function_ranges = []
    
    for node in tree.body:
        if isinstance(node, ast.FunctionDef):
            function_ranges.append((node.lineno-1, node.end_lineno))

    # mark function ranges
    mask = [False] * len(lines)
    for s, e in function_ranges:
        for i in range(s, e):
            mask[i] = True

    # collect all non-function lines
    output_lines = []
    for i, flag in enumerate(mask):
        if not flag:
            output_lines.append(lines[i])

    return "\n".join(output_lines)


def merge_templates(*template_sources):
    merged_non_funcs = []
    merged_funcs = {}
    
    for src in template_sources:
        # 1) 함수 전체 블록
        func_blocks = extract_function_blocks(src)

        # 2) 함수 아닌 부분
        non_func = extract_non_function_blocks(src)
        merged_non_funcs.append(non_func)

        # 3) 함수 추가 (중복은 제거)
        for fname, block in func_blocks.items():
            if fname not in merged_funcs:
                merged_funcs[fname] = block

    # join everything
    final = "\n\n".join(merged_non_funcs) + "\n\n"
    final += "\n\n".join(merged_funcs.values()) + "\n"

    return final

FINAL_CODE = merge_templates(
    TEMPLATE_BASE,
    TEMPLATE_MOBILE,
    TEMPLATE_MANIPULATOR
)

with open("SubController.py", "w", encoding="utf-8") as f:
    f.write(FINAL_CODE)
