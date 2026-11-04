import sys
from pathlib import Path

# Đảm bảo import được từ thư mục gốc
root_dir = Path(__file__).resolve().parent.parent.parent
if str(root_dir) not in sys.path:
    sys.path.insert(0, str(root_dir))


def run_this_test(file_path):
    pass
