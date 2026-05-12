# Reusable-Launch-Rocket

파이썬과 Matplotlib을 사용하는 6DOF 재사용 발사체 착륙 시뮬레이션입니다.

## 실행

처음 한 번만 가상환경을 만들고 의존성을 설치합니다.

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

그 다음 시뮬레이션을 실행합니다.

```bash
.venv/bin/python sim.py
```

터미널에서 착륙 과정을 ASCII 애니메이션으로 보여줍니다.

빠른 결과만 보려면:

```bash
.venv/bin/python sim.py --no-render
```

CSV 텔레메트리를 저장하려면:

```bash
.venv/bin/python sim.py --no-render --csv trajectory.csv
```

## Matplotlib 시각화

3D 착륙 궤적, 속도, 스로틀, 자세 그래프를 보려면:

```bash
.venv/bin/python sim.py --no-render --plot
```

그래프를 이미지 파일로 저장하려면:

```bash
.venv/bin/python sim.py --no-render --plot-file landing.png
```

Matplotlib 애니메이션을 보려면:

```bash
.venv/bin/python sim.py --no-render --animate-plot
```

GIF로 저장하려면:

```bash
.venv/bin/python sim.py --no-render --animation-file landing.gif
```

## 주요 옵션

```bash
.venv/bin/python sim.py --x 600 --y -300 --altitude 2200 --vx -55 --vy 30 --vz -90 --fuel 9000
```

- `--x`: 초기 수평 위치 오프셋, 미터
- `--y`: 초기 횡방향 위치 오프셋, 미터
- `--altitude`: 초기 고도, 미터
- `--vx`: 초기 수평 속도, m/s
- `--vy`: 초기 횡방향 속도, m/s
- `--vz`: 초기 수직 속도, m/s
- `--roll`: 초기 롤 각도, 도
- `--pitch`: 초기 피치 각도, 도
- `--yaw`: 초기 요 각도, 도
- `--fuel`: 초기 연료량, kg
- `--dt`: 시뮬레이션 시간 간격, 초
- `--realtime`: 느린 실시간 애니메이션
- `--csv`: CSV 파일로 시간별 상태 저장
- `--plot`: Matplotlib 그래프 창 표시
- `--plot-file`: Matplotlib 그래프 이미지 저장
- `--animate-plot`: Matplotlib 애니메이션 창 표시
- `--animation-file`: Matplotlib 애니메이션 저장

## 모델

시뮬레이션은 `x/y/z` 위치, `vx/vy/vz` 속도, `roll/pitch/yaw` 자세, `p/q/r` 각속도를 적분하는 단순 6DOF 모델입니다. 중력, 대기 밀도에 따른 항력, 연료 소모, 엔진 추력, 롤/피치 짐벌 토크, 요 RCS 토크, 수평/수직 유도 제어, 자세 제어를 포함합니다. 실제 로켓을 정밀하게 재현한 모델은 아니지만, 재사용 발사체 착륙 제어의 핵심 흐름을 실험하기 좋게 단순화했습니다.
