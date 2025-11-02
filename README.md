<img width="921" height="530" alt="image" src="https://github.com/user-attachments/assets/01085041-1f74-498c-8c6a-ca5d0e1534bf" />


"우리가 보장하는 사전 정의된 커맨드 제공" 개념 제거, "로봇의 구현된 함수를 webrtc {topic:<함수명> value:<파라미터>}전송 형태로 실행할수있게 해준다" 컨셉 (KRPC : KETI Remote Procedure Call)

ex) def move(vx,vy,vth) 함수실행시 move(vx=0.4,vy=0.0,vth=0.0) = {"topic":"move", "value":{"vx":0.4,"vy":0.0,"vth":0.0}
  - 사전 정의된 커맨드가 있어도 결국 유저가 파싱,실행 과정을 코딩해야함. 사전정의된 커맨드들을 만들고 보장한다는게 사실 크게 의미가 없는것같고 코드만 복잡해짐. 파싱,실행을 쉽고 직관적으로 만들어주는게 더 의미가 있지않나..

## 로봇쪽 변경점
Robot Side : base_controller 삭제. WebRTC_Bridge에서 메시지 수신, 메시지를 받고 로봇이 이 함수 가지고 있나? 판단, 파라미터 잘 왔나? 판단 후 실행시켜줌.
유저가 할건 로봇 제어 코드, config만 건드리면 끝.
WebRTC_Bridge도 모든로봇이 공통적으로 사용할수 있게되면서 examples/robot/안에는 config, robot_controller만 있으면 사실상 바로 제어 가능

## 오퍼레이터쪽 변경저
Operator Side : base_commander, sub_commander 통합. 사전정의된 커맨드가 사라지면서 base_commander는 의미가 거의 없음.
대신 robot_controller.py 읽어서 작성되있는 함수명, 파라미터를 topic, value 형태의 json 커맨드 생성, 전송까지 해주는 코드 빌더 제공.
결과적으로 오퍼레이터쪽에서는 빌더 실행, config작성, 인터페이스 연결만해주면 끝.

추가적으로 러버넥 조이스틱 명령 통일 시킨 후 우리가 규격화해놓은 함수명으로 코드작성 시 인터페이스 연결과정이 필요없도록 하면 더 좋을것 같음
ex)
axis 0 입력들어올시
if "move" is callable:
  move 실행
else:
  <유저작성>

