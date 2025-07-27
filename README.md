# 소개
AMR 에뮬레이터는 자율이동로봇(AMR)의 동작을 가상환경에서 시뮬레이션하고 테스트하기 위해 개발된 도구입니다.
AMR의 실제동작(구동부가감속,센서,배터리소모)을 모방함으로써 보다 현실적이고 정교한 시뮬레이션이 가능합니다.

# 기능
![Diagram](image/amr_emulator_diagram.png)

# Project Structure
'''sh
├──── config
│      └─── amr_params.yaml   # amr 파라메터파일
├──── src      
│      ├─── app
│      ├─── domain      
│      ├─── infrastructure
│      └─── presentation
├──── test
└──── third_party

├── .lintstagedrc
├── LICENSE
├── README.md
├── action.yml
├─> build
├── package-lock.json
├── package.json
├── renovate.json
├─> src
│   └── index.ts
└── tsconfig.json
'''
# 설치
# 사용법
# 테스트
# Todo
 1. 개발환경 및 레포지토리 구축 - done
 2. 기능정의 설계 - done
 3. 프로젝트 폴더 및 파일구조 설계 - done
 4. Motor controller - done 
 5. FMS
 6. Localizer
 7. Navigation
 8. Obstacle detection
 9. Battery management
 10. 통합 테스트
 
# 문의
