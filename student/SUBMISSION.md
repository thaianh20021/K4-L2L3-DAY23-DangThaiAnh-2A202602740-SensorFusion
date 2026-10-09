# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Đặng Thái Anh
- MSSV: 2A202602740
- Email: bepro13042002@gmail.com
- Link repo (fork): https://github.com/thaianh20021/K4-L2L3-DAY23-DangThaiAnh-2A202602740-SensorFusion
- Commit hash nộp (`git rev-parse HEAD`):

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: `compare`, `[0, 197]`, `training_segment-10072231702153043603_5725_000_5745_000_with_camera_labels.tfrecord`, `0`
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: `0.6455`, `0.7396`, `71 / 39 / 25`
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: `0.2193 m`, `55`, `2.6443 m^2`, `14`, `41`, `0.3485`
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: `0.2105 m`, `55`, `2.4378 m^2`, `14`, `41`, `0.3485`
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss:
  + Về sai số vị trí (RMSE): Chế độ `fused` đạt RMSE = 0.2105 m, tốt hơn chế độ `lidar` (0.2193 m), giảm 0.0088 m sai số và giảm tổng bình phương sai số `sum_sq_err` từ 2.6443 m² xuống 2.4378 m². Cả hai chế độ đều đạt mức RMSE ≤ 0.45 m (mức điểm tối đa của rubric). Điều này chứng tỏ việc tích hợp đo lường pixel 2D từ camera FRONT mang lại thêm ràng buộc phương vị, giúp EKF hiệu chỉnh vị trí tốt hơn so với chỉ dùng LiDAR.
  + Về số lượng cặp ghép và độ ổn định track: Cả hai chế độ đều có cùng số lượng `matches = 55`, `ghost_track_frames = 14`, `missed_gt_frames = 41` và `mean_confirmed_tracks = 0.3485`. Số liệu này minh chứng tính đúng đắn của thiết kế **track-then-fuse**: chỉ có LiDAR quyết định việc khởi tạo, chấm điểm tồn tại và xoá track; Camera chỉ tham gia bước EKF update để tinh chỉnh trạng thái mà không thay đổi score. Nhờ vậy, hệ thống không bị sinh thêm ghost tracks và không làm mất các track thật.
  + Chất lượng tracking tổng thể: Hệ số chất lượng đều vượt ngưỡng chuẩn của rubric với `precision_track = 55 / (55 + 14) = 79.71%` (chuẩn ≥ 75%) và `coverage = 55 / 71 = 77.46%` (chuẩn ≥ 70%).


Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. Khác biệt đo lidar 3D và camera 2D trong EKF (`z`, `R`)?
- Đo LiDAR: Vector đo lường $z \in \mathbb{R}^3$ là toạ độ vị trí không gian 3D $[x, y, z]^T$ trong hệ toạ độ cảm biến (đơn vị: mét). Ma trận hiệp phương sai nhiễu $R$ có kích thước $3 \times 3$ với đường chéo $\text{diag}(\sigma_{lidar\_x}^2, \sigma_{lidar\_y}^2, \sigma_{lidar\_z}^2)$ (đơn vị: $\text{m}^2$). Mô hình đo lường $h(x)$ là phép chiếu tuyến tính từ không gian trạng thái 6D sang 3D vị trí, do đó ma trận Jacobian $H$ là tuyến tính và hằng số ($3 \times 6$).
- Đo Camera: Vector đo lường $z \in \mathbb{R}^2$ là toạ độ điểm ảnh $[u, v]^T$ trên mặt phẳng ảnh của camera FRONT (đơn vị: pixel). Ma trận hiệp phương sai nhiễu $R$ có kích thước $2 \times 2$ với đường chéo $\text{diag}(\sigma_{cam\_i}^2, \sigma_{cam\_j}^2)$ (đơn vị: $\text{pixel}^2$). Mô hình đo lường $h(x)$ là phi tuyến (phép chiếu pinhole kết hợp phép biến đổi toạ độ ngoại vi $R, t$), và $H$ là ma trận Jacobian phi tuyến ($2 \times 6$) được tính bằng quy tắc đạo hàm hàm hợp (chain rule).

2. Vì sao cần gating Mahalanobis trước khi gán?
- Gating bằng khoảng cách Mahalanobis $d^2 = \gamma^T S^{-1} \gamma$ kết hợp phân phối $\chi^2$ nhằm loại bỏ trước những cặp đo lường - track không có khả năng thuộc về nhau (xác suất xuất hiện cực thấp).
- Nếu không gating, các thuật toán ghép cặp (greedy hay Hungarian) sẽ buộc phải gán cả những detection ngoại lai hoặc track ở rất xa, gây hiện tượng vỡ track (divergence) hoặc nhầm lẫn định danh (ID switch).
- Khác với khoảng cách Euclidean bỏ qua độ bất định, khoảng cách Mahalanobis chuẩn hoá khoảng cách theo ma trận hiệp phương sai phần dư $S = H P H^T + R$. Khi một track có độ bất định $P$ lớn (mới tạo hoặc chạy dự báo nhiều frame không có đo), elip gating tự động mở rộng để dễ đón nhận đo lường; khi track đã rất tin cậy ($P$ nhỏ), cổng sẽ tự động siết chặt lại để tránh nhận nhầm nhiễu.

3. Pipeline là track-then-fuse hay fuse-then-track? Chỉ ra trên log `fusion-run-lab`.
- Pipeline của lab là **track-then-fuse**: Hệ thống duy trì duy nhất một danh sách track. Trong mỗi frame, bộ lọc EKF predict một lần, sau đó thực hiện gán và cập nhật LiDAR trước (AssocL), rồi tiếp tục gán và cập nhật thêm Camera (AssocC) trên chính các track đó để tinh chỉnh trạng thái toạ độ.
- Thể hiện trên log `fusion-run-lab`: Trong mỗi frame, pipeline chạy tuần tự: EKF predict cho toàn bộ track -> AssocL (gán LiDAR, EKF update, cập nhật lifecycle manage_tracks) -> AssocC (gán Camera, EKF update để tinh chỉnh x và P, không tạo hay xoá track). Camera không tạo ra các track độc lập rồi mới gộp với track LiDAR như trong kiến trúc fuse-then-track.

4. Nếu camera lệch calibration, triệu chứng gì trên innovation/residual?
- Nếu calibration camera bị lệch (ngoại vi extrinsic bị lệch góc/tịnh tiến hoặc nội vi intrinsic sai tiêu cự/tâm ảnh), hàm dự báo pixel $h(x)$ sẽ tính ra toạ độ pixel bị lệch một lượng có hệ thống (systematic bias).
- Triệu chứng:
  + Innovation $\gamma = z - h(x)$ sẽ không còn dao động quanh 0 mà có kỳ vọng khác 0 (non-zero mean / residual bias).
  + Khoảng cách Mahalanobis $d^2 = \gamma^T S^{-1} \gamma$ tăng vọt, khiến phần lớn các đo lường camera bị cổng $\chi^2$ loại bỏ (out-of-gate rejects), làm giảm tỷ lệ ghép camera thành công.
  + Nếu vẫn lọt qua cổng, bước Kalman update sẽ kéo trạng thái ước lượng của track lệch khỏi vị trí thật, làm tăng RMSE của tracking ở chế độ fused so với lidar-only.

5. Vì sao `associate_and_update(..., sensor)` cần sensor tường minh ở frame rỗng?
   Giải thích vì sao lidar quyết định score/init/delete còn camera chỉ EKF update.
- Cần `sensor` tường minh vì khi `meas_list` rỗng, nếu không truyền sensor thì hàm không thể phân biệt đây là lượt quét LiDAR hay Camera. Tracker bắt buộc phải biết lượt quét LiDAR để đánh dấu các track nằm trong FOV mà không có đo lường là bị "miss", từ đó trừ điểm tồn tại (score $-= 1/window$) và xoá track nếu score $\le 0$.
- LiDAR quyết định score/init/delete vì LiDAR đo trực tiếp toạ độ 3D và có tầm bao quát rộng, đáng tin cậy để xác định sự xuất hiện và biến mất của vật thể. Ngược lại, Camera trong lab bị giới hạn góc nhìn (chỉ có FRONT) và chỉ cung cấp toạ độ góc/pixel 2D (không có chiều sâu trực tiếp độc lập), do đó chỉ được dùng để bổ sung ràng buộc hình học nhằm tinh chỉnh EKF, tránh việc tạo ra ghost track ở vùng camera không bao quát được.

6. Nêu điều kiện xác nhận, giữ confirmed sau miss, và điều kiện xóa track.
- Điều kiện xác nhận (`confirmed`): Khi track nhận được hit LiDAR, score được cộng $1/window$. Khi `score > confirmed_threshold` (0.8), track chuyển sang trạng thái `confirmed`.
- Giữ confirmed sau miss: Khi một track đã ở trạng thái `confirmed`, nếu gặp frame miss trong FOV LiDAR thì score bị trừ $1/window$, nhưng trạng thái vẫn được bảo toàn là `confirmed` (không bị hạ cấp về tentative).
- Điều kiện xoá track (`should_delete_track`):
  + Độ bất định mặt phẳng ngang vượt ngưỡng: $P_{xx} > max\_P$ hoặc $P_{yy} > max\_P$ (với $max\_P = 9.0\text{ m}^2$).
  + Hoặc track đã `confirmed` nhưng bị mất dấu kéo dài khiến $score < delete\_threshold$ (0.6).
  + Hoặc track chưa `confirmed` (`initialized`/`tentative`) có $score \le 0$.

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- Không

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Antigravity (Gemini 2.5 Pro)
- Dùng cho phần nào (hàm, câu hỏi, debug): Phân tích yêu cầu đề bài, gợi ý công thức toán EKF 6D và data association, giải thích chi tiết các câu hỏi lý thuyết
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): Chạy toàn bộ bộ test pytest đạt 128/128 passed (100%), đối chiếu công thức bộ lọc Kalman, pinhole projection và Mahalanobis gating với giáo trình Probabilistic Robotics và hướng dẫn kỹ thuật


## Checklist nộp

- [ ] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [ ] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [ ] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [ ] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [ ] Đã điền đủ file này, gồm khai báo AI
- [ ] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [ ] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [ ] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
