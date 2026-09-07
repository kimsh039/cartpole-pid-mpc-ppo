# CartPole: PID · MPC · PPO 비교

비선형 CartPole에 Cascade PID, 선형 MPC, PPO를 적용하고 동일한 초기조건과 제약 아래에서 성능을 비교한 제어 프로젝트다. MATLAB에서 모델링·학습·평가하고, 결과 궤적은 PyBullet로 동시에 재생한다.

![Controller comparison](docs/images/controller_response_comparison.png)

## 핵심 결과

30개 무작위 초기조건, 각 10초 시뮬레이션 결과다. 성공은 카트 위치와 봉 각도가 설정된 한계를 벗어나지 않은 경우로 정의했다.

| 제어기 | 성공률 | 성공 trial 평균 각도 RMSE | 평균 제어 에너지 |
|---|---:|---:|---:|
| Cascade PID | 100% | 0.931° | 2.933 N²·s |
| Linear MPC | 100% | 0.669° | 12.849 N²·s |
| PPO | 96.7% | 1.065° | 0.998 N²·s |

- **MPC**: 각도 추종 오차가 가장 작지만 제어 에너지가 가장 큼
- **PID**: 30/30 성공과 비교적 낮은 연산 복잡도의 균형
- **PPO**: 에너지는 가장 적지만 30회 중 1회 실패

원시 결과는 [Monte Carlo 요약](results/tables/monte_carlo_summary.csv)과 [trial별 결과](results/tables/monte_carlo_trials.csv)에서 확인할 수 있다.

![Monte Carlo result](docs/images/monte_carlo_controller_comparison.png)

## 구현 범위

- 비선형 CartPole 동역학과 RK4 적분
- 평형점 선형화 및 비선형 모델과의 오차 비교
- 위치 외부 루프 + 각도 내부 루프의 Cascade PID
- 이산 선형 모델 기반 예측 제어
- MATLAB Reinforcement Learning Toolbox 기반 PPO 환경·학습·평가
- 동일 초기조건을 사용하는 단일 실험 및 Monte Carlo 비교
- CSV 궤적을 읽는 PyBullet 3D 시각화

## 재현 방법

요구 환경:

- MATLAB R2025b 또는 호환 버전
- Control System Toolbox
- Model Predictive Control Toolbox
- Reinforcement Learning Toolbox
- Python 3.10+

```matlab
cd matlab
compare_controllers
run_monte_carlo_comparison
```

PPO 재학습은 `train_ppo_cartpole`을 실행한다. Python 시각화는 다음과 같다.

```powershell
python -m venv .venv
.venv\Scripts\activate
pip install -r python\requirements.txt
python python\visualize_cartpole_3d.py
```

## 저장소 구조

```text
matlab/                동역학, PID, MPC, PPO 및 평가 코드
models/                선형 모델과 사전 학습 PPO 에이전트
python/                PyBullet 3D 시각화
results/tables/        정량 평가 CSV
results/trajectories/  제어기별 궤적 CSV
docs/images/           결과 그래프
docs/media/            비교 영상
```

## 해석 시 주의점

결과는 이 저장소의 모델 파라미터, 제약, 보상함수와 초기조건 범위에 대한 시뮬레이션이다. 실제 장비의 마찰, 센서 잡음, 지연과 액추에이터 포화는 별도 검증이 필요하다.

## License

[MIT License](LICENSE)
