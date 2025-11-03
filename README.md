1. 기존 robot_controller 를 robot_interface 로 명칭 변경
- (명령 -> robot_interface -> 실제 로봇 하드웨어)

2. base_controller + sub_controller => krpc_controller로 변경
하나의 krpc_controller에 robot_interface or {id:robot_interface, id2:robot_interface ...}
형태로 전달하면 다중로봇 제어가 가능함을 확인함
- 다중로봇 컨트롤 테스트 코드 -> mulit_robot_case_test

3. first_setup 툴 추가 (UI 기반 초반세팅 툴)
<img width="592" height="811" alt="image" src="https://github.com/user-attachments/assets/8ed2f3d3-1734-4a39-b612-80067ec37edb" />
<img width="878" height="603" alt="image" src="https://github.com/user-attachments/assets/6a6e6f2b-2edd-4ab5-a61f-396c5392a20f" />
- UI크기조정 필요함...
  
- robot/operator_config 코드 생성
- robot/operator_launch 코드 생성 (미구현)
- opertor_commander 코드 생성
- 모든 로봇이 공통적으로 사용할수있는 UI 생성 (동적 명령버튼, 키보드마우스컨트롤러 맵핑UI)
- 결과적으로 필요한 코드가 전부 들어있는 디렉토리 생성
    - 경로 제한 필요함
