import os
import shutil
import subprocess


def get_mounted_usb_drives():
    """
    ✅ /media 또는 /mnt 아래에 마운트된 USB 디스크 검색
    """
    media_dirs = ["/media", "/mnt"]
    usb_mounts = []

    for base_dir in media_dirs:
        if os.path.exists(base_dir):
            for root, dirs, files in os.walk(base_dir):
                for dirname in dirs:
                    mount_path = os.path.join(root, dirname)
                    if os.path.ismount(mount_path):
                        usb_mounts.append(mount_path)
    return usb_mounts


def get_disk_usage(path):
    """💾 디스크 용량 정보 확인"""
    try:
        total, used, free = shutil.disk_usage(path)
        return {
            'total_gb': round(total / (1024 ** 3), 2),
            'used_gb': round(used / (1024 ** 3), 2),
            'free_gb': round(free / (1024 ** 3), 2),
        }
    except FileNotFoundError:
        return None


# ✅ 실행 예시
if __name__ == "__main__":
    drives = get_mounted_usb_drives()

    if not drives:
        print("⚠️ 외장하드(USB 디스크)가 연결되어 있지 않거나 마운트되지 않았습니다.")
    else:
        for path in drives:
            print(f"🔌 외장하드 마운트됨: {path}")
            usage = get_disk_usage(path)
            if usage:
                print(f"   - 전체 용량: {usage['total_gb']} GB")
                print(f"   - 사용 중: {usage['used_gb']} GB")
                print(f"   - 남은 용량: {usage['free_gb']} GB\n")
            else:
                print(f"   🚨 디스크 사용 정보를 불러올 수 없습니다: {path}")
