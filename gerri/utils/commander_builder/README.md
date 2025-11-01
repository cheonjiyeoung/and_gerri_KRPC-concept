Commander Builder
개요

commander_builder는 로봇 제어 스크립트(robot_controller.py)를 분석해 오퍼레이터용 명령 전송 코드(robor_commander.py)를 자동 생성하는 도구입니다.
이 스크립트는 로봇의 제어 함수와 파라미터 정보를 추출하여, 오퍼레이터가 동일한 명령 구조로 제어 코드를 실행할 수 있도록 도와줍니다.

주요 기능

    - AST 기반 함수 파싱: 파이썬 파일에서 함수명, 인자, 기본값, docstring을 자동 추출

    - 내부 메서드 필터링: _로 시작하는 내부 전용 메서드는 자동 제외

    - 자동 코드 생성: 추출한 정보를 기반으로 오퍼레이터 명령 클래스 생성

    - 백업 처리: 동일한 파일이 이미 존재할 경우, backup/ 폴더에 자동 백업

사용 방법
    1. 기본 실행
    python commander_builder.py <로봇 컨트롤러 스크립트 파일경로> <로봇명>
    ex : python commander_builder.py ./gerri/robot/examples/spot/spot_controller.py spot

    결과물은 gerri/utils/commander_builder/<로봇명>_commander.py 로 생성되며
    이미 commander가 존재할때 다시 실행하게되면 이전의 스크립트는 backup으로 넘어갑니다.

디렉터리 구조
 ├── commander_builder.py
 ├── robot_sub_controller.py
 ├── operator_client.py
 ├── backup/
 │    ├── operator_client_20251102_101522.py
 │    └── operator_client_20251102_121030.py
 └── README.md
