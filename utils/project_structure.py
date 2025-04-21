import os
import argparse

def print_directory_structure(root_dir, indent='', exclude=None, show_contents=False):
    if exclude is None:
        exclude = []

    for item in os.listdir(root_dir):
        if item in exclude:
            continue
        item_path = os.path.join(root_dir, item)
        if os.path.isdir(item_path):
            print(f"{indent}├── {item}/")
            print_directory_structure(item_path, indent + '    ', exclude, show_contents)
        else:
            print(f"{indent}├── {item}")
            if show_contents:
                print_file_contents(item_path, indent + '    ')

def print_file_contents(file_path, indent=''):
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            print(f"{indent}--- {file_path} ---")
            for line in file:
                print(f"{indent}{line.rstrip()}")
    except Exception as e:
        print(f"{indent}Could not read {file_path}: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Print project directory structure and optionally file contents.")
    parser.add_argument('--show-contents', action='store_true', help="Show contents of the files.")
    args = parser.parse_args()

    # 현재 스크립트의 디렉토리를 프로젝트 루트 디렉토리로 설정
    project_root = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(project_root, ".."))
    exclude_dirs = ['venv', '__pycache__', '.git', '.idea', '.gitignore']

    # 프로젝트 디렉토리 구조 및 파일 내용 출력
    print(f"{os.path.basename(project_root)}/")
    print_directory_structure(project_root, exclude=exclude_dirs, show_contents=False)
    # print_directory_structure(project_root, exclude=exclude_dirs, show_contents=True)
    # print_directory_structure(project_root, exclude=exclude_dirs, show_contents=args.show_contents)
