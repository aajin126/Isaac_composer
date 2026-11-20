# NavMesh subgoal grid

월드 바닥을 일정한 칸으로 나누고, 중심점이 유효한 보행자 목적지인 칸만 유지한다.
물체 의미 추출, Cosmos 호출, 시나리오 실행은 이 도구의 범위에 포함하지 않는다.
실제 월드 RGB도 저장하려면 `--scene-image`를 추가한다.
이를 사용하는 후속 계획·실행은 [semantic scenario planner](semantic_scenario.md)를 참고한다.

```bash
/home/ewhaglab/develop/isaacsim/python.sh \
  /home/ewhaglab/isaac_ws/src/isaac_composer/scripts/export_scene_grid.py \
  warehouse_a2_random --cell-size-m 2
```

다른 환경은 `hospital_a2_random` 같은 기존 시나리오 프리셋으로 선택한다.
출력 위치는 기본 `generated/scene_grid/<scenario>/`이며 `--output-dir`로 변경한다.
로봇과 보행자를 생성하지 않고 환경만 로딩한다. 기존 데이터 수집 실행과 별개다.

## 생성 방식

1. 기존 `build_pedestrian_navmesh()`로 같은 바닥·반경·지도 제약의 NavMesh를 bake한다.
2. native NavMesh 삼각형을 큰 격자 사각형과 교차시킨다. 기존 0.12m raster는 사용하지 않는다.
3. 칸의 기하학적 중심만 native `query_closest_point()`로 검사한다.
4. 조회 결과가 중심에서 XY 기준 1mm보다 멀어지면 칸을 제외한다. 같은 칸 안의
   다른 이동 가능 지점으로 보정되는 경우도 제외한다. 1mm는 수치 오차 허용값이다.
5. 바닥·허용 구역 검사를 통과하면 원래 중심의 XY와 조회한 바닥 Z를 목적지로 저장한다.
   칸 안의 다른 후보를 찾는 fallback은 없다.

칸의 범위는 `[x_min, y_min, x_max, y_max]`다. 좌표가 속할 때 오른쪽·위쪽 경계는
제외한다. 전체 격자는 월드 좌표에서 칸 크기의 배수에 정렬한다. 열은 왼쪽부터
A, B, …, Z, AA이고 행은 위쪽(+Y)부터 1, 2, …이다. 제거한 칸의 ID는 당겨 쓰지 않는다.

**유지한 칸은 중심점에서 사람이 설 수 있다는 뜻이다.** `target_world`의 XY는
정확히 칸 중심이고, 칸의 가장자리까지 모두 비어 있다는 의미는 아니다.
칸을 순서대로 방문할 때 연결성은 기존 native `shortest_path()`로 별도 검사해야 한다. 새 길찾기나 topology 그래프를 만들지 않는다.

## 출력

- `grid_cells.json`: 유지한 칸의 ID, 영역, 대표 목적지(world 좌표, m), 조회 반경,
  전체 칸 개수와 제거 개수. 시나리오·환경·USD 출처도 기록한다.
- `grid_navmesh.png`: 파란 영역은 bake된 NavMesh 바닥, 어두운 영역은 장애물·제외 구역,
  빨간 점은 검증한 목적지다. 유효한 칸에만 테두리와 ID를 표시한다.

이미지는 NavMesh 도식이며 실제 월드의 RGB 렌더가 아니다. USD에 격자 geometry를
추가하지 않으므로 collider와 LiDAR에는 영향을 주지 않는다. 바닥 도식은 bake 반경,
대표 목적지는 보행자 query 반경 및 기존 지도 제약으로 검사한다.

## 코드

- `scene_grid.py`: native geometry를 이용한 칸 생성과 JSON·PNG 저장.
- `scene_grid_export.py`: 기존 환경 로딩과 NavMesh 생성 기능을 호출하는 실행 조립.
- `scripts/export_scene_grid.py`: Isaac Python에서 실행하는 얇은 진입점.

검증은 중심이 막힌 부분 칸의 제외, 같은 칸 안으로의 위치 보정 거부, 수치 오차, 다른 칸으로 투영된 점의 거부, 허용 구역 필터,
음수 좌표, Z 이후 열 이름, 제거 후 ID 유지, 파일 출력을 포함한다.

2026-10-15 중심점 기준으로 native Isaac 실행 결과를 갱신했다.
전체 테스트 164개가 통과했다.

| 환경 | 칸 크기 | 전체 | 유지 | 제외 |
|---|---:|---:|---:|---:|
| warehouse | 2m | 150 | 123 | 27 |
| warehouse | 1m | 551 | 481 | 70 |
| hospital | 2m | 836 | 188 | 648 |

모든 저장된 목적지의 XY가 해당 칸의 기하학적 중심과 정확히 일치함을 확인했다.
