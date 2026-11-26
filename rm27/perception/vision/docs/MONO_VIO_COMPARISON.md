# OpenVINS／SchurVINS 單目 VIO 對比準備

**2026-11-24方向更正：上游原版SchurVINS只支援雙目，使用者要求停止自行單目適配。**
作者明確回覆與他人單目失敗紀錄見[上游支援核對](../../../../.agents/docs/SCHURVINS_INTEGRATION.md#upstream-support-scope-2026-11-24)。
以下準備包與失敗結果保留原研究身份，不能當作兩套官方單目系統的可用比較；
第三方ov_SchurVINS是獨立重作，尚未採用或驗證。

更新：2026-11-24（原始配對回放為11-23）。使用者先要求準備同一組單相機＋IMU比較，隨後明確要求啟動。本文件定義非執行準備包、首次有界回放結果及未完成的接入；不改動原75-gate／67-cohort資格矩陣、1cm repeatability門檻或任何估測器。

## 已準備到哪裡

`tools/prepare_mono_comparison.py` 只讀取既有matrix及兩個同速率mono gate，檢查原始來源、設定與輸入hash，產生一份 `PREPARED_NOT_RUN` 配對記錄及兩套各6份未執行設定（smoke＋5 repeats）。不呼叫估測器或容器。

本輪可用的本地基準是EuRoC V1_01_easy：完整cam0 2912影格、imu0 29120樣本。兩者原始archive SHA256皆為 `a920fe5b5e69a6ad19b32f1cfaf90ac2f59d45dfd1ca18cc9722e07684ba45fa`；輸入與來源校準一致，準備格式分別為Schur sensors.zip及OpenVINS ROS bag，檔案bytes不要求相同。

**這只是來源及設定鎖定。** 沒有重新讀回每張影像／bag、遞迴核對安裝依賴、啟動環境或檢驗native ingestion；solver實際校準參數／線上估計是否等同亦未由來源校準相同而得到證明。結果不能稱為runtime preflight通過、單目演算法合格或RK3576實板合格。

原Schur研究仍依 `plan_schurvins_1.md` §0 的USER_STOPPED狀態保留；本項不恢復bootstrap、geometry、失鎖修正或既有矩陣的其他實驗。

## 產生一組可審閱的比較設定

從VIO元件根目錄使用現有Python環境。本地資料／安裝不隨clone提供；缺檔或hash不符須先核對原始來源，不重建或改寫舊鎖冒充同一版本。

```bash
/home/shiuhou/venvs/mujoco/bin/python -m tools.prepare_mono_comparison \
  --matrix /home/shiuhou/Projects/rm27-schurvins-research/configs/prehardware/matrix.json \
  --schur-gate schur-v101-mono-half-input-boundaries-matrix-v1 \
  --openvins-gate openvins-v101-mono-half-initializer-owned-workers-matrix-v1 \
  --output artifacts/mono-comparison-half

/home/shiuhou/venvs/mujoco/bin/python -m tools.prepare_mono_comparison \
  --matrix /home/shiuhou/Projects/rm27-schurvins-research/configs/prehardware/matrix.json \
  --schur-gate schur-v101-mono-1x-input-boundaries-matrix-v1 \
  --openvins-gate openvins-v101-mono-1x-initializer-owned-workers-matrix-v1 \
  --output artifacts/mono-comparison-1x
```

`--output`必須是新目錄。0.5×與1×分開封存，不能混成同一效能組。工具產生的命令是資料；非執行準備本身不授權執行，也不觸發舊矩陣的75個gate。若先有smoke失敗，該backend的5次repeat不自動繼續；失敗保留，不用新run替換失敗成員。

實際啟動OpenVINS比較前，主機分析亦須載入ROS Jazzy。原生程式在容器中完成不代表主機能解碼CDR；缺少`rclpy`會在回放後的camera consumption分析失敗。使用本機既有環境時，先在**同一個執行shell**確認以下入口，再啟動原生回放；不需要為非執行準備載入ROS。

```bash
source /opt/ros/jazzy/setup.bash
/home/shiuhou/venvs/mujoco/bin/python -c 'from rclpy.serialization import deserialize_message; from sensor_msgs.msg import Image, Imu; import rosbag2_py, cv2'
```

環境修正後的新診斷必須有新run ID／output，保留原失敗；不得覆寫原cohort或以補跑替換失敗成員。

## 首次實際回放（2026-11-23）

使用者明確要求啟動後，已在Intel i5-14400／amd64 PC完成6次headless EuRoC回放，各容器4CPU／12GiB、唯讀來源與sensor mounts、network none；沒有執行RK3576或實拍感測器。

| 版本／批次 | 0.5× | 1× |
|---|---|---|
| 原Schur固定版本smoke | 0 poses，NATIVE_OUTPUT_EMPTY | 1 pose、0 scored epochs，TRAJECTORY_NOT_EVALUABLE |
| 原OpenVINS smoke | native正常退出，但host缺rclpy，分析FAIL | 同左 |
| 補載既有ROS後的獨立OpenVINS確認 | 回放／native input核對PASS，2800／2912 poses | 同左 |

原4smokes全FAIL，20個固定repeats全部NOT_RUN；另兩次環境確認是事前宣告的新ID／output，沒有替換原失敗成員、放行repeats或形成5次重複性成績。
兩次OpenVINS確認各有112幀初始化前綴，沒有內部或尾端pose缺口；固定尺度SE3 ATE皆0.07011501076521645m，1秒RPE平移0.04523709741003572m／旋轉0.478052290355176°。完整2912 camera callbacks／29120 IMU callbacks與native camera source/CDR/frontend consumption檢查PASS；operational health、covariance及live資格未驗。
2800個輸出中實際GT評分2780個，另20個在GT覆蓋外，評分時長138.950883865s；保留全來源2912分母，不能將輸出／有真值可評分的數量混用。continuous_trajectory_qualified仍為false，原orientation reference caveat保留。

Schur兩次都有內部metric初始化，原生accepted visual corrections分別191／2次，但這不能代替可評分的發佈軌跡。另有獨立summary錯誤：用同timestamp將較早native quality=0與初始化後Info quality=2當成同一階段，0.5×發生在camera154、1×在camera151。它不是0／1 pose失敗的原因；原run.json、原生事件與secondary exception均保留，未修改估測器或scorer。

6次輸出的264份封存artifacts、24份原prepared configs及9份受保護舊來源核對PASS，6個owned containers均已確認不存在。原Open smoke的run.json仍有EXECUTING字樣，是host import exception留下的狀態，不能拿它當作目前有程序在跑。最終派生判定為 **配對精度／性能比較BLOCKED**；本輪不對兩算法排名，也不宣稱原1cm repeatability gate通過。

本機證據：[comparison.json](../../../../.agents/runs/mono_comparison_execution/comparison.json)、[原批次](../../../../.agents/runs/mono_comparison_execution/execution.json)、[環境確認](../../../../.agents/runs/mono_comparison_execution/ros_environment_confirmation/confirmation.json)。這些ignored產物不隨clone提供。後續資料流診斷見下節；實拍／RK3576的缺項仍見下文。

## 0／1 pose 的資料流診斷（2026-11-23）

已核對同一凍結RM27 derivative的原始事件、native文字軌跡與ROS記錄，**輸出不足發生於native追蹤品質／stage門檻**。
`monoCallback → processImageBundle → mono processFrameBundle → publishResults` 中，metric初始化不會直接進入Tracking；後續accepted correction還須至少40個當前Schur inliers、滿足品質判定，才會進Tracking。`publishResults`只有Tracking發pose，其他stage仍發Info。

0.5×雖有191次accepted corrections，每次當前有效inliers僅1–25，全部低於40；2912筆Info都停在Initializing，文字軌跡與ROS皆0pose。1×在source camera154達49inliers，進Tracking並發唯一pose；camera155降至0，轉Relocalization，五次重定位失敗後reset為Paused，2752筆Info維持Paused（automatic_reinitialization=false）。`Info.num_matches`是另一個frontend特徵統計，不能用它代替Schur inliers。

原生文字軌跡與ROS的0／1數量及唯一pose內容相符，沒有observer單獨漏掉大量pose的證據。1×的source camera152另未進frontend／Info，其native回調邊界未被記錄，原因仍未知；publisher／observer輸入完整不等於native完整消費。source camera index為0-based，不能與漏幀後的native bundle ID混用。

進一步已看到更新後重投影殘差惡化：0.5× camera156的≤4px內點79→25、mean0.9387→4.5801px；1× camera155為37→0、mean4.1273→11.2542px。這把下一步數值診斷定位到Schur更新／狀態寫回，**尚未證明唯一數學根因**；本輪沒有native重跑、算法／門檻改動或實板操作。

已修正獨立summary的phase配對錯誤，分開保存中間quality判定及稍後Info；同幀初始化後quality變更不再觸發錯誤。兩份實際sealed summaries均可產生、各保留1次phase差異及原FAILED碼／0或1pose；沒有將stage1／quality2當作Good tracking。相關569項測試PASS（2.15s），原raw/manifests及RK3576候選未改。

[完整來源說明](../../../../.agents/docs/SCHURVINS_INTEGRATION.md#monocular-state-to-pose-diagnosis-2026-11-23) ·
[本機資料流重建](../../../../.agents/runs/schurvins_pose_flow/flow.json) ·
[新派生summaries](../../../../.agents/runs/schurvins_pose_flow/summaries.json)。

## 後續有界修復嘗試（2026-11-24）

使用者「請嘗試解決」後，兩個獨立PC候選各完成同來源全長0.5×回放，
仍各僅1pose／TRAJECTORY_NOT_EVALUABLE。更新順序或一致點回代均未恢復持續
追蹤，無效正式patch已撤除。新快照將下一設計邊界收窄到初始化資訊交接、
單目深度與map一致性，尚無唯一根因；見[完整結果與限制](../../../../.agents/docs/SCHURVINS_INTEGRATION.md#bounded-monocular-repair-attempt-2026-11-24)。
這些是含有界診斷的額外負結果，沒有替換上方原始smokes／20repeats，
亦未擴跑1×、跨序列或RK3576；配對精度／性能比較仍BLOCKED。

## 未來比較時固定的條件

- 每個配對都使用同一原始cam0＋imu0、完整序列與來源校準；保留每張影像與IMU原時間、序號、來源缺口。不能一邊降解析度、一邊裁掉失敗前綴或單獨調速後仍稱同輸入。
- 各自固定source／patch／compiler／安裝／參數，記錄線上calibration、初始化、tracking gate及自動重啟策略的差異。若需要調整，作為另一個明示配置，不能覆蓋原結果。OpenVINS保留基線、Schur保留獨立入口。
- 同一RK3576、電源、CPU／thread配置、散熱與背景負載下依序測量；先單獨運行，與Tracking／其他程序並行是另一組。兩者同時搶同一CPU不能拿來評算法效率。執行前固定交錯順序與每組次數，保留啟動／冷暖機條件；本輪未指定或啟動排程。
- 0.5×作為既有公開資料集accuracy protocol；1×另評固定影像週期下的處理超期。演算法duration、queue wait、transport receipt、sensor-to-output各有時鐘和scope，不互相替代。

## 比較哪些結果

| 結果 | 必須保留的內容 |
|---|---|
| 輸入完整性 | 原始影像／IMU全分母、發布／接收／native消費各自計數與缺口；publish成功不能代替native使用 |
| 初始化 | 第一筆pose、原生metric初始化完成、第一個accepted visual correction分開；只有觀測到native事件才填值 |
| 連續追蹤 | 每個epoch的覆蓋率、原始前綴／內部／尾端缺口、失鎖與reset；不能把重啟軌跡拼成單段 |
| 精度 | 使用既有 `tools.schurvins.evaluate`／OpenVINS共同SE3工具，固定scale=1的ATE及1秒RPE；不擬合尺度或時間偏移 |
| 重複性 | 已宣告的smoke＋5 repeats保留全部成員；沿用既有1cm repeatability gate，不調寬或換分母 |
| 性能 | 每幀duration P50／P95／max、deadline misses、CPU／RSS及板溫；完整記錄採樣方法與背景負載 |
| 狀態可信度 | numeric pose、有效tracking、freshness、covariance資格、reset與operational validity分開；未知維持UNKNOWN／NOT_RUN |

主結果先看全長覆蓋與失鎖，再看誤差。短片段低ATE不能勝過完整軌跡。共同區間只能作補充：現有 `compare_rows()` 用epoch間重疊的影格**範圍**，不是精確相同camera-ID集合；若內部有缺口，需另列各自sample IDs／exact intersection後才能說是同一批sample比較。不得裁掉全長失敗來取得較漂亮分數。

Ground truth只在估測器退出後供host評分，不進感測器輸入。若實拍沒有獨立真值，ATE／RPE精度判定為NOT_RUN；兩條估測軌跡互相接近不能充當真值。

## 真實RK3576相機＋IMU對比仍缺什麼

目前判定為 **BLOCKED：實拍雙後端共用入口未完成**，不能因公開資料集預檢通過而改判。

| 項目 | 現況／下一個可驗證交付 |
|---|---|
| 感測器識別 | 實際相機／鏡頭／模式、獨立IMU、安裝與時鐘來源待核對；不從板名或舊M3C配置推定 |
| 共享recording | 已有 `experiments.sensors` 的sealed recording，保留sensor IDs／frame IDs／epoch、raw與mapped timestamps、SI IMU、images及hash；原始sensor檔需保留 |
| 同步與標定 | 真實曝光／時間語義、時鐘映射、Camera–IMU外參／時偏、IMU噪聲／bias未驗收；UNKNOWN不可用EuRoC值填入 |
| Schur native接入 | 已有 `SEALED_SENSOR_RECORDING_REPLAY`／`prepare_recording()`；須提供有來源的標定與IMU參數，未知標定會被拒絕 |
| OpenVINS native接入 | 2026-11-24獨立ROS-free入口已接通sealed recording、明確校準／IMU binding、clock/source IDs及無GT診斷；PC公共完整輸入2912影格／29120 IMU回放通過。此處原ROS比較入口仍只支援EuRoC original／artificial，未完成兩後端共同matrix／實拍對比，亦不恢復已暫停Schur工作；見[錄製入口](../../../../tools/openvins/README.md#sealed-camera--imu-recording--offline-diagnostics) |
| matrix generic接入 | 現有 `config_locks()`假設reference archive為path；Schur generic錄製的null reference尚未接入共同矩陣 |
| 板端資格 | OpenVINS資料集實板回放與Schur ARM64／QEMU各保留原範圍；未建立Schur實板ABI、兩者相同板端條件、真實live與長時性能 |

OpenVINS單後端的通用離線軟體入口已獨立交付；兩後端source/calibration等價檢查與共同matrix仍未實作，Schur維持USER_PAUSED，不由此自動接續。硬體到位後按感測器品質→實拍離線→live順序驗證；未知值不得用EuRoC參數代填。詳細時序／標定要求沿用 [資料契約](RM27_VIO_DATA_CONTRACT.md)，runtime及既有結果見 [OpenVINS工具](../../../../tools/openvins/README.md) 與 [SchurVINS工具](../../../../tools/schurvins/README.md)。

單獨MP4、模擬時間、host receipt或以前M3C校準均不能取代這條實際Camera＋IMU證據鏈。本文件不新增採集、校準、韌體、訓練、部署、PX4融合、飛行或Git發布授權。
