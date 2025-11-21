"""
지정된 파일의 위치를 기준으로 프로젝트 루트를 계산하고 sys.path에 추가합니다.

Args:
    module_file (str): 현재 모듈의 __file__ 값.
    depth (int): 루트까지 몇 단계 올라갈지. 예: avatar/robot/xxx.py → depth=2
"""

import os, sys
def bootstrap(module_file: str, depth: int = 2):
    root = os.path.abspath(os.path.join(os.path.abspath(module_file), *[".."] * depth))
    if root not in sys.path:
        sys.path.insert(0, root)
