import subprocess
import re

def find_camera_indices_by_name(camera_name="ZED-M"):
    """
    'v4l2-ctl --list-devices'를 실행하여
    특정 camera_name을 가진 장치의 모든 /dev/video 인덱스를 찾아 리스트로 반환합니다.
    
    :param camera_name: 찾고자 하는 카메라 이름 (예: "ZED-M", "RealSense")
    :return: 찾은 비디오 인덱스 번호(int)의 리스트 (예: [6, 7])
    """
    video_indices = []
    
    try:
        # 1. 'v4l2-ctl --list-devices' 명령어 실행
        # 'check=True'는 오류 발생 시 예외를 발생시킵니다.
        # 'text=True'는 stdout/stderr를 자동으로 디코딩합니다.
        result = subprocess.run(
            ['v4l2-ctl', '--list-devices'],
            capture_output=True,
            text=True,
            check=True
        )
        
        output = result.stdout
        
    except FileNotFoundError:
        print("❌ Error: 'v4l2-ctl' command not found.")
        print("Please install it using: sudo apt-get install v4l-utils")
        return []
    except subprocess.CalledProcessError as e:
        # v4l2-ctl이 0이 아닌 코드를 반환할 때 (예: 권한 문제)
        print(f"❌ Error running command: {e.stderr}")
        return []
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return []

    # 2. 명령어 출력 결과(stdout)를 파싱
    lines = output.splitlines()
    found_camera_section = False
    
    for line in lines:
        # 라인이 탭으로 시작하지 않으면, 새 장치 헤더임
        if not line.startswith('\t'):
            # 현재 라인이 우리가 찾는 카메라 이름인지 확인
            if camera_name in line:
                found_camera_section = True
            else:
                # 다른 장치이므로 플래그 리셋
                found_camera_section = False
        
        # 3. ZED-M 섹션을 찾았고, 탭으로 시작하는 라인일 경우
        elif found_camera_section:
            stripped_line = line.strip()
            
            # 이 라인이 /dev/video 경로인지 확인
            if stripped_line.startswith('/dev/video'):
                # 정규 표현식으로 숫자 부분만 추출
                match = re.search(r'/dev/video(\d+)', stripped_line)
                if match:
                    # 숫자를 정수(int)로 변환하여 리스트에 추가
                    video_indices.append(int(match.group(1)))

    # 중복을 제거하고 정렬하여 반환
    return sorted(list(set(video_indices)))[0]

# --- 스크립트 실행 ---
if __name__ == "__main__":
    
    zed_indices = find_camera_indices_by_name("ZED-M")
    
    if zed_indices:
        print(f"✅ ZED-M 카메라 인덱스: {zed_indices}")
        # 예: OpenCV에서 첫 번째 인덱스 사용
        # first_index = zed_indices[0]
        # cap = cv2.VideoCapture(first_index)
    else:
        print("❌ ZED-M 카메라를 찾지 못했습니다.")

    # (참고) RealSense 카메라도 동일하게 찾을 수 있습니다.
    # rs_indices = find_camera_indices_by_name("RealSense")
    # if rs_indices:
    #     print(f"✅ RealSense 카메라 인덱스: {rs_indices}")