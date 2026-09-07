# 어린이 더미 + 구루마 재점검

## 직접 검수 완료 상태

사용자가 `labels_clean` 편집 화면에서 비반전 child 이미지 68장(train 42/val 26)을
직접 검수했다. 최초 보정안 대비 train 원본 10개 파일이 다시 저장되었고, 이 중
8장은 bbox 좌표가 실제로 변경되었으며 2장은 좌표값 변화 없는 재저장이다.

직접 검수 후 train의 원본-좌우반전 42쌍을 모두 다시 계산하여 동기화했다.
실제 좌표 갱신이 필요했던 반전 라벨은 8개였으며, 결과는 `flip_sync.json`, 동기화
직전 전체 복사본은 `labels_clean_before_flip_sync/`에 보관했다.

- 최종 수정본: `dataset_obstacles/labels_clean/` (181개 라벨)
- class 0 `child_dummy`: train 84 boxes, val 26 boxes
- class 1 `vehicle_obstacle`: train 60 boxes, val 2 boxes
- 빈 hard-negative 라벨: 9개
- 범위 오류/잘못된 행 형식/중복 bbox: 0건
- 원본 `labels/`, 이미지, `data.yaml`, weights: 변경 없음(기록 해시 검증 완료)
- `labels_clean`은 아직 학습 설정에 연결하지 않았으며 재학습하지 않음

후속 승인 후 `labels_clean`을 별도 `dataset_obstacles_childclean/` 스냅샷으로
고정하여 `obstacle_childclean_50ep` 50-epoch 시험학습을 완료했다. 원본 라벨이나
기존 weights를 교체한 것은 아니며, 상세 성능은 해당 run의 `REPORT.md`에 기록했다.

여기서 child bbox는 보이는 어린이 더미와 구루마 전체를 하나로 묶는다. 성인,
우산 또는 화면 밖/가림 영역은 추정해서 포함하지 않는다. 이번 검수는 child가
들어 있던 이미지 기준이므로 vehicle 이미지 전체에 대한 역방향 오라벨 검사는
별도 범위로 남는다.

현재 장애물 파생 dataset의 비반전 child 이미지 68장(train42/val26)을 GT 확대 시트로 검사했다.
반전 train42장은 원본에 연결된 파생본으로 집계하며 독립 샘플로 세지 않는다.

- 수정 필요 확인: 원본 이미지 9장(train1/val8), 해당 반전 라벨1개까지 총10파일.
- 추가 정밀 검수 후보: 원본15장. 여백 과다와 손잡이/구루마 끝 누락 의심이며 확정 수정 수에 포함하지 않음.
- 현재 dataset181장 모두 bbox 최대1개여서 한 이미지 내 중복 bbox 0건.
- child/vehicle 오라벨 확정 0건(검수한 child 이미지 범위). 차량-only 전체 오라벨 검수 완료를 의미하지 않음.
- 화면 밖으로 잘린 부분은 bbox를 확장해 추측하지 않는다. 가린 성인은 별도 대상이 아니다.

## 파일 목록

|상태|이미지|검수 이유|
|---|---|---|
|review_candidate|train/20260905_b_photos/P20260905_152743213_19C8D57D-3657-49D2-AC47-329C67067DF6.jpg|Loose background margin.|
|needs_correction|train/20260905_b_video_152846/frame_00000360.jpg|Visible hat top lies above bbox.|
|review_candidate|train/20260905_b_video_152846/frame_00000630.jpg|Large top margin.|
|review_candidate|train/20260905_b_video_152846/frame_00000720.jpg|Check trolley handle left edge at full resolution.|
|review_candidate|train/20260905_b_video_152846/frame_00000750.jpg|Check visible bottom of trolley against bbox.|
|review_candidate|train/20260905_b_video_152846/frame_00000810.jpg|Check visible trolley left edge.|
|review_candidate|train/20260905_b_video_152846/frame_00001080.jpg|Large bottom margin.|
|review_candidate|train/20260905_b_video_152846/frame_00001110.jpg|Large bottom/right margin.|
|review_candidate|val/20260906_d_video_141652/frame_00000870.jpg|Loose margins around body/trolley.|
|review_candidate|val/20260906_d_video_141652/frame_00001080.jpg|Large bottom margin.|
|review_candidate|val/20260906_d_video_141652/frame_00001110.jpg|Large side margin.|
|needs_correction|val/20260906_d_video_141652/frame_00001140.jpg|Visible hat and trolley right edge need review/expansion.|
|needs_correction|val/20260906_d_video_141652/frame_00001230.jpg|Visible hat lies above bbox.|
|review_candidate|val/20260906_d_video_141652/frame_00001320.jpg|Large bottom/right margin.|
|review_candidate|val/20260906_d_video_141652/frame_00001350.jpg|Large bottom/right margin.|
|needs_correction|val/20260906_d_video_141652/frame_00001380.jpg|Visible hat above bbox.|
|needs_correction|val/20260906_d_video_141652/frame_00001500.jpg|Visible hat above bbox.|
|needs_correction|val/20260906_d_video_141652/frame_00001650.jpg|Visible hat above bbox.|
|needs_correction|val/20260906_d_video_141652/frame_00001680.jpg|Visible hat above bbox.|
|needs_correction|val/20260906_d_video_141652/frame_00001710.jpg|Visible hat above bbox.|
|review_candidate|val/20260906_d_video_141652/frame_00001740.jpg|Large right/bottom margin; adult occlusion, do not include adult.|
|review_candidate|val/20260906_d_video_141652/frame_00001800.jpg|Large right/bottom margin.|
|review_candidate|val/20260906_d_video_141652/frame_00001830.jpg|Loose side margin.|
|needs_correction|val/20260906_d_video_141652/frame_00001860.jpg|Visible hat above bbox.|

## 보존 및 후속 기준

원본/현재 labels/반전본/모델을 변경하지 않았다. 좌표 수정본도 아직 만들지 않았고,
findings.json에 현재 좌표와 라벨 revision, 의심 이유를 보관했다. 수정 단계에서는
별도 labels_clean에만 작성하고 해당 원본의 반전 라벨도 동일 기준으로 파생해야 한다.

차량-only 이미지는 child_dummy에 대한 음성으로 쓰되 기존 vehicle_obstacle bbox는
유지한다. 차량까지 있는 이미지를 빈 라벨로 바꾸면 차량 정답이 누락된다.

직전 반전 학습은 이번 요청을 확인했을 때 이미50epoch 종료되어 실행 중 프로세스가
없었다. 이번 점검 중 새 학습/평가/모델 교체는 실행하지 않았다.
