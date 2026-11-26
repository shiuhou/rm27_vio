# RM27 VIO — SPEC

本倉負責連續 Camera＋IMU 狀態估計、時序、標定與定位輸出；現行機載目標是
RK3576。地圖重定位、飛控融合及上層任務在各自元件驗收。

- [Camera＋IMU 資料契約](../../rm27/perception/vision/docs/RM27_VIO_DATA_CONTRACT.md)
  定義來源、時鐘、感測器資料和完整性；保留量測／發布時間、尺度、有效性和定位重置。
- 軟體回放或合成接口不能證明實際 Camera＋IMU 同步、標定、定位精度、
  延遲或穩定板端運行。相關評分及門檻沿用原契約與已確認方案。
- 通用OpenVINS離線入口接受既有sealed sensor recording；保留原始像素、raw SI IMU、
  整數來源時間／時鐘映射、epoch／序號與來源hash。單相機校準須綁定實際解析度，
  IMU噪聲／重力／frame與時偏須有明確來源，缺項不得用公共資料預設值補齊。
  不自動把MP4＋飛控姿態log對齊成可信VIO。無GT允許輸入／初始化／連續性／
  accepted corrections／離線耗時診斷，精度與physical／operational資格保留未驗收。
  Native中止、資料／輸出不一致與逾時須保留失敗與raw，不以exit0視為完整通過。
- M3C 研究保留歷史證據，不作現行硬體驗收或默認回退。
- 使用者2026-11-24明確要求先查上游支援與他人單目使用經驗，**不自行把SchurVINS改成單目**。
  上游協作者明確表示原版只支援雙目；現有RM27單目改動及失敗保留為實驗證據，
  不當作官方單目能力，也不繼續自行適配。來源與第三方實作界線見
  [上游支援核對](SCHURVINS_INTEGRATION.md#upstream-support-scope-2026-11-24)。
- 單目兩後端比較使用同一來源cam0＋imu0、完整序列及相同速率；
  全長覆蓋、初始化／失鎖、固定尺度SE3精度與運算成本分別報告，不能只選短片段低誤差。
  非執行準備只能標 `PREPARED_NOT_RUN`，不升格為runtime／實板／operational資格。
  具體準備與未完成的實拍接入見 [比較協議](../../rm27/perception/vision/docs/MONO_VIO_COMPARISON.md)。
- RK3576 原生診斷候選需鎖完整 ARM64 userspace／source／devel、patches 與無真值輸入；
  準備不得啟動板端操作。原生架構／依賴、資源、程序退出與可評分軌跡分開驗收，
  空／不足 pose 不得由 exit0、初始化事件或 QEMU 結果升格為單目通過。
  [板端診斷入口](../../tools/schurvins/README.md#rk3576-native-diagnostic-candidate)
  不取代原回放評分、75-gate matrix、1cm門檻或實際同步標定要求。
