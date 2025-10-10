
import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.rtc2kng.robot_client import RobotClient
from _and_.rtc2kng.camera_manager import CameraManager
from _and_.rtc2kng.audio_track import AudioSendTrack
from _and_.rtc2kng.pyaudio_manager import AudioRecorder, AudioPlayer

def create_channels(robot_info, server_info, video_info, audio_info, command = None):
    """
    AdaptiveNetworkDaemon의 'rtc2kng' 백엔드를 위한 채널을 생성합니다.

    Args:
        robot_info (dict): 로봇 기본 정보.
            예: {'id': 'test_robot'}
        server_info (any): 서버 관련 정보
            예: {'room_id': 'test_room','server_ip': '125.131.105.165', 'server_port': 25000}
        video_info (dict): 비디오 스트림 정보.
            예: {'front_cam': {'source': 0, 'width': 1920, 'height': 1080, 'debug': False},
                 'rear_cam': {'source': 1, 'width': 1280, 'height': 720, 'debug': False}}
        audio_info (dict): 오디오 스트림 정보.
            예: {'enabled': True, 'player_volume': 0.0, 'silence_threshold': 50,
                 'recorder_params': {'rate': 16000, 'channels': 1}}

    Returns:
        dict: 생성된 채널 객체를 담은 딕셔너리.
              키는 채널의 이름, 값은 connect() 메소드를 가진 채널 객체입니다.
              실패 시 빈 딕셔너리를 반환할 수 있습니다.
    """
    print(f"RTC2KNG 백엔드: create_channels 호출됨")
    print(f"  ROBOT_INFO: {robot_info}")
    print(f"  SERVER_INFO: {server_info}")
    print(f"  VIDEO_INFO: {video_info}")
    print(f"  AUDIO_INFO: {audio_info}")

    # 1. RobotClient 초기화에 필요한 파라미터 추출
    # robot_client.py의 __main__ 및 클래스 기본값 참고
    robot_id = robot_info.get('id', robot_info.get('name', 'test_robot')) # 'id' 또는 'name' 사용
    room_id = robot_info.get('room_id', 'test_room')
    server_ip = server_info.get('server_ip', '125.131.105.165') # robot_client.py의 외부 IP 기본값
    server_port = server_info.get('server_port', 25000) # robot_client.py의 포트 기본값

    # 2. 비디오 트랙 설정 (VIDEO_INFO 기반)
    # robot_client.py의 __main__에서는 video_track={"front_cam": CameraManager(...)} 형태로 전달
    video_tracks_dict = {}
    if video_info and isinstance(video_info, dict):
        for cam_label, cam_config in video_info.items():
            if isinstance(cam_config, dict):
                try:
                    #  1. 'manager' 키를 가져오고, 없으면 기본 CameraManager를 사용
                    ManagerClass = cam_config.pop('manager', CameraManager)
                    
                    #  2. 가져온 클래스(ManagerClass)로 객체를 생성
                    video_tracks_dict[cam_label] = ManagerClass(**cam_config)

                    print(f"  비디오 트랙 '{cam_label}'용 {ManagerClass.__name__} 생성 완료.")
                except Exception as e:
                    print(f"  오류: 비디오 트랙 '{cam_label}'용 매니저 생성 실패: {e}")
            else:
                print(f"  경고: VIDEO_INFO의 '{cam_label}'에 대한 설정이 올바르지 않습니다. 무시합니다.")
    else:
        print("  VIDEO_INFO가 없거나 형식이 올바르지 않아 비디오 트랙을 설정하지 않습니다.")


    # video_tracks_dict = {}
    # if video_info and isinstance(video_info, dict):
    #     for cam_label, cam_config in video_info.items():
    #         if isinstance(cam_config, dict):
    #             try:
    #                 video_tracks_dict[cam_label] = CameraManager(
    #                     index=cam_config.get('source', 0),
    #                     width=cam_config.get('width', 1920), # 기본 해상도 설정
    #                     height=cam_config.get('height', 1080),
    #                     debug=cam_config.get('debug', False)
    #                 )
    #                 print(f"  비디오 트랙 '{cam_label}'용 CameraManager 생성 완료.")
    #             except Exception as e:
    #                 print(f"  오류: 비디오 트랙 '{cam_label}'용 CameraManager 생성 실패: {e}")
    #         elif isinstance(cam_config, CameraManager): # 이미 CameraManager 인스턴스인 경우
    #              video_tracks_dict[cam_label] = cam_config
    #              print(f"  비디오 트랙 '{cam_label}'에 기존 CameraManager 인스턴스 사용.")
    #         else:
    #             print(f"  경고: VIDEO_INFO의 '{cam_label}'에 대한 설정이 올바르지 않습니다 ({type(cam_config)}). 무시합니다.")
    # else:
    #     print("  VIDEO_INFO가 없거나 형식이 올바르지 않아 비디오 트랙을 설정하지 않습니다.")

    # 3. 오디오 트랙 및 플레이어 설정 (AUDIO_INFO 기반)
    audio_track_instance = None
    audio_player_instance = None
    # audio_info 자체가 None이거나 dict가 아닌 경우를 먼저 확인
    if audio_info and isinstance(audio_info, dict):
        audio_settings = audio_info.get('audio') # 'audio' 키 아래 설정을 가져옴
        if audio_settings and isinstance(audio_settings, dict):
            input_device = audio_settings.get('input')
            output_device = audio_settings.get('output')

            # 'input'과 'output' 값이 모두 존재하고, 빈 문자열이 아닐 때 오디오 활성화
            if input_device and output_device: # (input_device is not None and input_device != "") and (output_device is not None and output_device != "")
                print("  오디오 활성화 조건 충족. 오디오 처리 중...")
                try:
                    # recorder_params, player_volume, silence_threshold도 audio_settings에서 가져오도록 수정
                    recorder_params = audio_settings.get('recorder_params', {})
                    recorder = AudioRecorder(**recorder_params)
                    audio_track_instance = AudioSendTrack(recorder)
                    print("  AudioRecorder 및 AudioSendTrack 생성 완료.")
                except Exception as e:
                    print(f"  오류: AudioRecorder 또는 AudioSendTrack 생성 실패: {e}")

                try:
                    audio_player_instance = AudioPlayer(
                        volume=audio_settings.get('player_volume', 100.0),
                        silence_threshold=audio_settings.get('silence_threshold', 50)
                    )
                    print("  AudioPlayer 생성 완료.")
                except Exception as e:
                    print(f"  오류: AudioPlayer 생성 실패: {e}")
            else:
                print("  AUDIO_INFO의 'audio' 설정 내 'input' 또는 'output' 장치 정보가 유효하지 않아 오디오를 비활성화합니다.")
        else:
            print("  AUDIO_INFO에 'audio' 키가 없거나 그 값이 딕셔너리가 아니므로 오디오를 비활성화합니다.")
    else:
        print("  AUDIO_INFO가 제공되지 않았거나 형식이 올바르지 않아 오디오를 비활성화합니다.")


    # 4. RobotClient 인스턴스 생성
    try:
        robot_client_instance = RobotClient(
            room_id=room_id,
            robot_id=robot_id,
            server_ip=server_ip,
            server_port=server_port,
            video_track=video_tracks_dict if video_tracks_dict else None, # 빈 딕셔너리가 아닌 None으로 전달
            audio_track=audio_track_instance,
            audio_player=audio_player_instance
        )
        print(f"  RobotClient 인스턴스 생성 완료 (Robot ID: {robot_id}, Room ID: {room_id}).")
    except Exception as e:
        print(f"  치명적 오류: RobotClient 인스턴스 생성 실패: {e}")
        return {} # 실패 시 빈 채널 딕셔너리 반환

    # 5. 생성된 RobotClient 인스턴스를 채널 딕셔너리에 담아 반환
    # AdaptiveNetworkDaemon은 이 딕셔너리의 값 객체들의 connect() 메소드를 호출합니다.
    channels = {
        f"{robot_id}": robot_client_instance
        # 필요에 따라 다른 채널들 (예: 명령 전용 채널)을 추가할 수 있으나,
        # 현재 RobotClient는 WebRTC 데이터 채널을 통해 명령 수신도 가능할 것으로 보입니다.
    }
    print(f"RTC2KNG 백엔드: 채널 생성 완료. 채널 키: {list(channels.keys())[0]}")
    return channels