# RM27 VIO — DESIGN

[localization schema](../../rm27/perception/vision/localization/schema.py)
保留估計器輸出的時間、座標、尺度與不確定性。
沿用 [OpenVINS 工具](../../tools/openvins/README.md) 及
[SchurVINS 研究紀錄](SCHURVINS_INTEGRATION.md) 中各自版本與結果邊界。
[interface_worker.py](../../interface_worker.py) 只驗證本地型別轉換，不執行估計器。

實作在 [rm27/](../../rm27/)、[tools/](../../tools/)，回歸在 [tests/](../../tests/)。
VIO 發布估計，接收端負責准入與融合；缺失資訊不得補成可控制狀態。

[通用錄製適配](../../tools/openvins/recording.py) 重用`experiments.sensors`封存契約，
選定單一camera／IMU及source epoch，導出ASL目錄格式供既有ROS-free headless
runner讀取；ASL只是transport，不改寫真實來源為EuRoC。來源config／events／seal、
原始與mapped時間／ID／序號及校準來源保留，readback重建CSV／YAML／像素核對。
[離線執行入口](../../tools/openvins/recording_replay.py) 綁定build manifest／binary／
immutable image，限時管理所屬程序，對照native frames／poses／summary生成無GT診斷。
不複製估測器算法、Onboard runner、資料契約或姿態轉換，也不新增硬體／PX4出口。
介面、參數與失敗語義見[工具說明](../../tools/openvins/README.md#sealed-camera--imu-recording--offline-diagnostics)。

[ROS-free native適配來源](../../tools/openvins/headless/README.md)由本倉
`tools/openvins/headless/`維護；2026-11-26從實際PC回放所用歷史來源核對hash後匯入。
`upstream.lock.json`鎖上游commit／archive及202份原始／prepared檔案，保留原有
observation counters與單目設定的精確patch，不變更估測器算法。上游source payload、
native binary、已安裝library／image、真實錄製及硬體標定分開取得與驗收；
source-only測試和source preparation核對不視為新native build或RK3576資格。

[單目比較準備工具](../../tools/prepare_mono_comparison.py) 重用既有
`prehardware.validate_matrix/config_locks`，只讀選定的同來源、同速率EuRoC mono配對，
封存來源與未執行的smoke／repeat設定，不提供execute模式、不更改原矩陣或runtime。
準備介面與真實感測器缺項在 [比較協議](../../rm27/perception/vision/docs/MONO_VIO_COMPARISON.md)。

[RK3576候選組裝](../../tools/schurvins/rk3576_prepare.py) 從既有base＋overlay封存
生成獨立native Docker映像context，保留原manifest，完整userspace隔離於映像內。
[板端doctor／raw回放](../../tools/schurvins/rk3576_board.py) 重用既有inner transport，
只管理准入、資源觀察、限時與owned-container清理；不新增估測器或評分器、不接Onboard／PX4。
PRoot/QEMU原入口與OpenVINS基線各自保留；原生候選不冒用emulation helper身份。
