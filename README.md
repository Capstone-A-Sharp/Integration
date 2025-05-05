# Integration
All integration

001_code_integration: motor_driver + MPU-9250, RX1616M FSR

002_add calibswitch: calibswitch 데이터 추가(JSON 형식으로)

003_add motordriver: 모터드라이버 제어함수까지 추가(로직은 작년과 동일하게 구현)

004_final: 최종 코드

005_speed_change: 속도 변경 가능 코드(speed 부분만 수정해서 사용해면 됨.)

006_immediate_reaction: 시리얼 입력에서 값을 입력했을 때, 즉각 멈추도록 코드 수정(Timeout 사용)

007_JSON_change: JSON 데이터 포맷 변경
(자이로 모든 값->피치 값만 / 압력센서 모든 값 -> 왼쪽,오른쪽 LOW SUM 값만)
