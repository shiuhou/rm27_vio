# M3C laptop 測試與回傳操作手冊

這份文件給「連著 M3C 和飛控的 laptop」及該電腦上的 Codex 使用。
目前這台開發主機負責編譯、程式修正與結果分析；laptop 負責設備辨識、
部署和板端離線回放。測完把原始結果回傳主機，不能只回覆「跑成功了」。
目前驗收結論在 [M3C_DEPLOYMENT.md](M3C_DEPLOYMENT.md)。

## 本輪範圍與已知負結果

- 分支只能用 `research/m3c-vio`，repo 是 `shiuhou/rm27_vio`。
  不修改任何 RK3576 worktree，不 merge、不強制 push、不改其他分支。
- 本輪測試用 EuRoC 錄製的單目影像＋IMU。飛控即使已接線，也不是這輪的資料來源。
  不開 UART、不發 MAVLink 到實體飛控、不啟動相機採集、不執行舊硬體協調腳本。
- **單目已知 FAIL**：PC 和 ARM64 QEMU 完整 V1_01 均只有一個 pose，然後失鎖，
  metric initialization 未驗證。這次板測可以重現失敗和量測資源，不能預設會修好。
  雙目資料僅作軟體控制組，不能算 OS04A10 單目通過。
- ARM64 包是 Bionic 參考 ABI；尚未在實際 M3C 跑過。先辨識設備，再驗證啟動。
  不替換系統 libc／loader、不全域安裝函式庫、不改韌體、EKF、串口或相機設定。
- 新 Camera 採集仍需使用者明確授權。物理 UART 測試還需確認拆槳、Disarmed、
  串口獨占及現有配置，維持 115200／8N1；本文件沒有提供物理 UART 執行步驟。

可直接給 laptop Codex 的任務：

> 閱讀 AGENTS.md 和 .agents/docs/M3C_LAPTOP_HANDOFF.md，依序完成本文件的
> laptop 準備、M3C 唯讀審計、參考包啟動、單目離線及 0.5×／1× 診斷，保留所有
> 失敗及原始輸出。缺依賴或身份不明時記錄 BLOCKED，繼續可獨立完成的檢查。
> 不接實體相機／UART 資料，不改飛控／系統設定，不把過程退出 0 算 VIO PASS。
> 完成後打包本次 session，回傳開發主機，列明每項 PASS／FAIL／BLOCKED／NOT_RUN。

## 1. laptop：拉取分支與建立獨立 session

以下命令使用 Linux Bash。Windows laptop 可用能連到 M3C 的 WSL Bash，
但先確認 WSL 自己能 SSH 到設備；Windows 能連線不等於 WSL 能連線。
所有 `REPLACE_...` 都是要實際填寫的值，不能當成已有的路由或帳號。

新 checkout：

```bash
mkdir -p "$HOME/Projects"
git clone --single-branch --branch research/m3c-vio \
  https://github.com/shiuhou/rm27_vio.git "$HOME/Projects/rm27_vio-m3c"
```

若 laptop 已經有這個 checkout，先檢查，不覆蓋 dirty work：

```bash
export REPO="$HOME/Projects/rm27_vio-m3c"
git -C "$REPO" status --short --branch
test "$(git -C "$REPO" branch --show-current)" = research/m3c-vio
test -z "$(git -C "$REPO" status --porcelain)"
git -C "$REPO" pull --ff-only origin research/m3c-vio
```

後續所有 laptop 命令在同一 Bash session 執行：

```bash
set -euo pipefail
export REPO="$HOME/Projects/rm27_vio-m3c"
export TRANSFER_BASE="$HOME/rm27-m3c-test"
export SESSION_ID="m3c-$(date -u +%Y%m%dT%H%M%SZ)"
export SESSION="$TRANSFER_BASE/sessions/$SESSION_ID"
mkdir -p "$TRANSFER_BASE"
test ! -e "$SESSION"
mkdir -p "$SESSION"
cd "$REPO"
git rev-parse HEAD > "$SESSION/code-commit.txt"
git status --porcelain > "$SESSION/initial-git-status.txt"
uname -a > "$SESSION/laptop-uname.txt"
cat /etc/os-release > "$SESSION/laptop-os-release.txt"
```

不要在未完成的 session 中 pull 更新或改測試參數。若確實需要改程式，保留此輪，
另建 session，記錄新的 commit／diff／binary hash。

## 2. laptop：取得 Git 以外的測試包

**只 pull 不足以跑板測。** Git 沒有 ARM64 binary、依賴和 EuRoC PNG。
開發主機已另備好下列私用離線包，約 2.1 GiB 原始資料加 runtime，實際壓縮大小
為 **1,242,413,504 bytes**。laptop 建議預留至少 6 GiB；M3C 至少 3 GiB 可用空間。
archive SHA256（下載後必須與此值及 checksum 檔都一致）：

```text
787c6d7be7016f84be52f833f668b8902c22a101ab8a70d8c071ff1274ce489b
```

```text
開發主機檔案：
/home/shiuhou/Projects/rm27-m3c-schurvins-research/handoff/rm27-m3c-transfer-v1.tar.gz
/home/shiuhou/Projects/rm27-m3c-schurvins-research/handoff/rm27-m3c-transfer-v1.tar.gz.sha256

解壓後：rm27-m3c-transfer-v1/
  transfer.json            整包檔案 hash、原始 EuRoC ZIP hash、binary identity
  verify.py                Python 標準庫校驗，不需要 NumPy
  runtime/                 checked ARM64 binary、loader、依賴、參數、manifest
  input/mav0/              V1_01 cam0、cam1、imu0；不含 ground truth
  input/preparation.json   原始 sensor-only 準備證據
  provenance/              checked build、完整 stereo camera ledger 等
```

包內 checked binary SHA256 必須是：

```text
697c26e36fe2f0325f315b9c8da6bb1848e8ebcb0883a158cbb8d62a6d1bbe37
```

`transfer.json` 中 code commit 為產生 runtime 的 `8b87181...`；文件後續 commit
不同是正常的，不能因此換成未知 binary。標準 EuRoC ZIP 的 SHA256 為：

```text
a920fe5b5e69a6ad19b32f1cfaf90ac2f59d45dfd1ca18cc9722e07684ba45fa
```

設定 laptop 可到達的開發主機 SSH 帳號／位址。**不是 M3C 的位址**：

```bash
export BUILD_HOST='REPLACE_WITH_USER_AT_DEVELOPMENT_HOST'
test "$BUILD_HOST" != REPLACE_WITH_USER_AT_DEVELOPMENT_HOST
scp "$BUILD_HOST:/home/shiuhou/Projects/rm27-m3c-schurvins-research/handoff/rm27-m3c-transfer-v1.tar.gz" "$TRANSFER_BASE/"
scp "$BUILD_HOST:/home/shiuhou/Projects/rm27-m3c-schurvins-research/handoff/rm27-m3c-transfer-v1.tar.gz.sha256" "$TRANSFER_BASE/"
cd "$TRANSFER_BASE"
sha256sum -c rm27-m3c-transfer-v1.tar.gz.sha256 | tee "$SESSION/transfer-archive-check.txt"
printf '%s  %s\n' \
  787c6d7be7016f84be52f833f668b8902c22a101ab8a70d8c071ff1274ce489b \
  rm27-m3c-transfer-v1.tar.gz | sha256sum -c -
test ! -e "$TRANSFER_BASE/rm27-m3c-transfer-v1"
tar -xzf rm27-m3c-transfer-v1.tar.gz -C "$TRANSFER_BASE"
export KIT="$TRANSFER_BASE/rm27-m3c-transfer-v1"
python3 "$KIT/verify.py" --kit "$KIT" > "$SESSION/transfer-files-check.json"
cp "$KIT/transfer.json" "$SESSION/transfer.json"
```

上面若已解壓，重新測試時只執行 `verify.py`，不要覆蓋既有 kit。亦可透過 USB
把同兩個 archive／checksum 檔案搬過去；不能自創 SSH 位址、假設主機已開 SSH
或把密碼／私鑰寫進 repo。無法取得包就記為 **BLOCKED: TRANSFER_ARTIFACT_UNAVAILABLE**，
仍可做下一節設備審計。不要在 laptop 安裝整套 ROS 來繞過這個缺件。

## 3. laptop：最小 Python 工具檢查

laptop 的分析工具需要 Python 3.10+。可用既有合格環境；以下只建立個人 venv：

```bash
python3 -c 'import sys; assert sys.version_info >= (3, 10), sys.version'
python3 -m venv "$TRANSFER_BASE/venv"
. "$TRANSFER_BASE/venv/bin/activate"
python -m pip install numpy opencv-python-headless pytest
python -m pip freeze > "$SESSION/laptop-python-packages.txt"
cd "$REPO"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q \
  tests/test_schurvins_m3c.py \
  tests/test_schurvins_headless_contract.py \
  tests/test_schurvins_headless_evaluate.py \
  tests/test_schurvins_benchmark.py \
  --junitxml="$SESSION/laptop-software-junit.xml" \
  > "$SESSION/laptop-software.log" 2>&1
```

這是 laptop 工具測試，不是 ARM64 執行、完整 583 項環境回歸或 VIO 驗收。
安裝／測試失敗要保留 log；不為通過而刪除測試。

## 4. laptop → M3C：先做唯讀設備審計

歷史管理地址是 `root@10.18.198.1`，必須在接著設備的 laptop 重新確認。
第一次 SSH 應核對實際設備的 host key；不要關閉 host-key 檢查或刪除原 known_hosts。

```bash
export M3C_SSH='root@10.18.198.1'
ssh "$M3C_SSH" 'uname -a; cat /etc/os-release'
cd "$REPO"
set +e
python -m tools.schurvins.m3c --host "$M3C_SSH" \
  --output "$SESSION/board-audit" > "$SESSION/board-audit-console.log" 2>&1
audit_rc=$?
set -e
printf '%s\n' "$audit_rc" > "$SESSION/board-audit-exit.txt"
cat "$SESSION/board-audit-console.log"
```

檢查 `board-audit/audit.json` 中的 DT model／compatible、SoC、uname、RAM、
Python ELF machine 183／AArch64、libc／loader、SDK paths、process／device owner。
審計 `PASS` 只表示有拿到 inventory，**仍需人工確認這台確實是 M3C**。
`10.18.198.1` 不可達、誤連其他設備、不是 ARM64、Python collector 無法執行，
皆先記錄 BLOCKED，不繼續部署；把審計回傳主機處理。

飛控接線不妨礙這個唯讀盤點，但 device owner 不代表允許你接管該程序。
不要殺掉既有相機／UART 程序。若資源競爭，記錄 owner、時間及影響。

## 5. laptop → M3C：獨立目錄部署與 loader 測試

確認 root SSH 及設備身份後，使用 `/root/rm27-vio-test/<SESSION_ID>/`。
若 SSH 使用者不是 root，改成該使用者可寫的絕對路徑，所有後續路徑同步修改。
不把測試包放進 `/usr/lib` 或既有 camera SDK 目錄。

```bash
export BOARD_SESSION="/root/rm27-vio-test/$SESSION_ID"
ssh "$M3C_SSH" "df -h /root; test ! -e '$BOARD_SESSION' && mkdir -p '$BOARD_SESSION'"
scp -r "$KIT" "$M3C_SSH:$BOARD_SESSION/kit"
ssh "$M3C_SSH" "python3 '$BOARD_SESSION/kit/verify.py' --kit '$BOARD_SESSION/kit'" \
  > "$SESSION/board-kit-check.json"
set +e
ssh "$M3C_SSH" "'$BOARD_SESSION/kit/runtime/run' --help" \
  > "$SESSION/board-loader-help.log" 2>&1
loader_rc=$?
set -e
printf '%s\n' "$loader_rc" > "$SESSION/board-loader-exit.txt"
cat "$SESSION/board-loader-help.log"
test "$loader_rc" -eq 0
```

只在校驗成功且 help 正常退出 0 後進入回放。`Exec format error`、loader／symbol
缺失、`Illegal instruction`、Segfault 均是 **FAIL/BLOCKED**，保留完整 stderr，
不要用系統升級／換 libc／拷入 x86 庫處理。`--help` 成功只是啟動 PASS，不是 estimator PASS。
如果 M3C 沒有 Python3，包內標準庫校驗也無法跑，先交回主機，不默默跳過校驗。

## 6. M3C：單目 smoke、完整離線、0.5×／1× 診斷

這段由 laptop 發送，但所有 estimator／GNU time／timeout 都在 **M3C 本機**執行。
需要板上 Bash、支援 `-k` 的 `timeout` 與 GNU `/usr/bin/time -v`。
先把版本／help 留存；若缺工具，本階段記 BLOCKED，無須全域安裝套件或改系統。

```bash
ssh -T "$M3C_SSH" bash -s -- "$SESSION_ID" <<'BOARD'
set -eu
case "$1" in ''|*[!A-Za-z0-9_-]*) exit 2;; esac
S="/root/rm27-vio-test/$1"
K="$S/kit"
mkdir "$S/runs"
uname -a > "$S/board-uname.txt"
cat /etc/os-release > "$S/board-os-release.txt"
cat /proc/meminfo > "$S/meminfo-before.txt"
df -h "$S" > "$S/disk-before.txt"
command -v timeout > "$S/timeout-path.txt"
timeout --help > "$S/timeout-help.txt" 2>&1
/usr/bin/time --version > "$S/time-version.txt" 2>&1

snapshot() {
  date -u +%Y-%m-%dT%H:%M:%SZ
  for p in /sys/class/thermal/thermal_zone*/temp /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq; do
    if test -r "$p"; then printf '%s ' "$p"; cat "$p"; fi
  done
}

run_case() {
  name=$1; mode=$2; rate=$3; limit=$4; seconds=$5
  D="$S/runs/$name"
  mkdir "$D"
  snapshot > "$D/system-before.txt"
  set -- "$K/runtime/run" --dataset "$K/input" --mode "$mode" \
    --calib "$K/runtime/params/calib/euroc_${mode}.yaml" \
    --params "$K/runtime/params/vio_${mode}.yaml" \
    --output "$D/native" --rate "$rate"
  if test "$limit" -gt 0; then set -- "$@" --max-frames "$limit"; fi
  printf '%s\n' "$@" > "$D/argv.txt"
  set +e
  LC_ALL=C timeout -k 10s "${seconds}s" /usr/bin/time -v \
    -o "$D/native-resource.txt" "$@" > "$D/process.log" 2>&1
  rc=$?
  set -e
  printf '%s\n' "$rc" > "$D/exit-code.txt"
  snapshot > "$D/system-after.txt"
  printf '%s exit=%s\n' "$name" "$rc"
  if test "$rc" -ne 0; then return 1; fi
}

# 200 幀只用於啟動／失效診斷，不可當完整序列驗收。
run_case mono-smoke mono 0 200 180
# 完整序列，不加 --max-frames、不跳過前綴、不自動重啟。
run_case mono-offline-full mono 0 0 900
run_case mono-half-full mono 0.5 0 900
run_case mono-one-full mono 1 0 900
cat /proc/meminfo > "$S/meminfo-after.txt"
BOARD
```

若 remote 腳本非零退出，停止依賴它的後續回放，保留已產生的目錄和輸出，繼續第 8 節回傳。
timeout exit 124／137、SIGILL／crash、輸入／校驗失敗都不能當 PASS。
SSH 中斷後，先重跑唯讀 inventory 確認本次程序是否仍在，不盲目重試或 `pkill`。
必要終止只針對本次 argv／PID 已辨識的程序，不停止其他任務的程序。

native process 退出 0 而單目只一個 pose，仍是已知 **estimator FAIL**；
可以完成以上 0.5×／1× 診斷以收集板端效能，但不得宣稱有有效 VIO 輸出。
預期 V1_01 完整輸入為 2912 個 cam0／29120 個 IMU，第一張無嚴格更早 IMU
會被明確拒絕；不補假樣本。輸入數量不等於內部積分已合格。

可選的 stereo 30-frame 控制組必須用新目錄，保持 `mode=stereo`、stereo calibration
及 stereo params 一致；只證明公開資料雙目路徑，不能替代單目與真實相機驗收。
單目完整軌跡／metric initialization 未過前，不做「五次 PASS」或 60 分鐘合格穩定性宣稱。

## 7. 選做：板上 native benchmark sampler

第 6 節的 GNU time 是板端 CPU／Peak RSS 原始證據，frontend／forward／backward
timing 在 `native-events.jsonl`，schedule backlog 在 `frames.csv`。
thermal／frequency 只有前後快照，**不是全程熱穩定性測量**。

若板上已有 Python 3.10+、NumPy 和 GNU time，可另跑現有 sampler 取得 process-tree
採樣。不要為這步改系統 Python。laptop 把**當次程式**送到同一 session：

```bash
cd "$REPO"
git archive --format=tar HEAD > "$SESSION/repo-source.tar"
scp "$SESSION/repo-source.tar" "$M3C_SSH:$BOARD_SESSION/repo-source.tar"
ssh -T "$M3C_SSH" bash -s -- "$SESSION_ID" <<'BOARD'
set -eu
case "$1" in ''|*[!A-Za-z0-9_-]*) exit 2;; esac
S="/root/rm27-vio-test/$1"
K="$S/kit"
python3 -c 'import sys; assert sys.version_info >= (3,10); import numpy'
mkdir "$S/repo"
tar -xf "$S/repo-source.tar" -C "$S/repo"
cd "$S/repo"
python3 -m tools.schurvins.benchmark run \
  --target m3c-native --rate 0.5 --timeout-s 900 \
  --output "$S/runs/mono-half-sampled" \
  --artifact "$K/runtime" --artifact "$K/input" \
  -- "$K/runtime/run" --dataset "$K/input" --mode mono \
  --calib "$K/runtime/params/calib/euroc_mono.yaml" \
  --params "$K/runtime/params/vio_mono.yaml" \
  --output '{output}/native' --rate 0.5
BOARD
```

這個 sampler 要求實際 AArch64 與可辨識的 M3C/MaixCAM2 DT identity。
若拒絕身份、缺 Python／NumPy，記 sampler BLOCKED，回傳第 6 節的證據。
不能把 target 改成 `pc-native` 假冒身份，也不能在 laptop 量 SSH client 當板端 CPU。
sampler `PASS` 只代表測量完成，estimator 判定仍要看原始單目證據。

## 8. laptop：取回原始結果、做初檢

成功、失敗、卡在審計／ABI 的 session 都要回傳。若第 5 節已建立板端目錄，
用以下方式只取結果；不把 2 GiB input 和函式庫重複搬回：

```bash
mkdir "$SESSION/board-results"
ssh -T "$M3C_SSH" "cd '$BOARD_SESSION' && tar --exclude=./kit --exclude=./repo --exclude=./repo-source.tar -cf - ." \
  > "$SESSION/board-results.tar"
tar -tf "$SESSION/board-results.tar" > "$SESSION/board-results-members.txt"
tar -xf "$SESSION/board-results.tar" -C "$SESSION/board-results"
```

先確認 SSH 命令退出成功再解壓；不要把斷線留下的半個 tar 當完整結果。
如果板端尚未部署，就跳過這段，回傳 laptop 審計與失敗 log 即可。
保留板上原始目錄，等主機確認收件／雜湊／解析後再另行清理。

laptop 初檢會保留每個 raw summary、exit code、recording 一致性和原始資源；
不在 laptop 擬合尺度、不提供 ground truth 給 M3C。

```bash
cd "$REPO"
python - "$SESSION" <<'PY'
import json, sys
from pathlib import Path
from rm27.perception.vision.localization.experiments.schurvins_headless import load_run
from tools.schurvins.benchmark import timeline_summary
from tools.schurvins.summarize import native_resources
s = Path(sys.argv[1])
results = []
for run in sorted((s/'board-results/runs').glob('*')):
    item = {'run': run.name, 'scope': 'BOARD_DIAGNOSTIC_REVIEW',
            'estimator_acceptance': 'NOT_RUN', 'operational_validity': 'NOT_RUN'}
    for name in ('exit-code.txt', 'benchmark.json'):
        path = run/name
        if path.exists(): item[name] = path.read_text()
    try:
        summary, frames, poses = load_run(run/'native')
        item.update(recording_integrity='PASS', native_summary=summary)
        item['timeline'] = timeline_summary(run/'native/frames.csv')
    except (ValueError, OSError) as exc:
        item.update(recording_integrity='FAIL', error=str(exc))
    if (run/'native-resource.txt').exists():
        item['resources'] = native_resources((run/'native-resource.txt').read_text())
    results.append(item)
(s/'laptop-review.json').write_text(json.dumps(
    {'scope': 'PRELIMINARY_ONLY_NO_ATE_OR_FLIGHT_QUALIFICATION', 'runs': results}, indent=2)+'\n')
print('Collected runs:', len(results))
PY
```

`recording_integrity=PASS` 只代表檔案內部一致，不代表單目初始化／尺度／完整軌跡通過。
初檢無 run（例如 audit BLOCKED）也要回傳，不能憑空建立成功 run。

## 9. laptop → 開發主機：回傳 session 與交接說明

在 `$SESSION/RETURN.md` 填寫以下資料，沒有做的明確寫 NOT_RUN：

```text
Session ID / UTC 起迄：
repo branch / commit / 是否有本地修改：
M3C SSH route（不含密碼）/ 實際 DT identity / SoC / OS / libc / RAM：
設備身份由誰、以何種現場證據確認：
飛控是否接線；本輪是否開過 UART / Camera（預期均否）：
板端部署路徑 / binary SHA256 / input SHA256：
每輪 argv / exit / frame、IMU、pose 數 / tracking losses：
PC build：本機未重建則 NOT_RUN，引用主機既有 PASS
ARM64 build：本機未重建則 NOT_RUN，引用 transfer manifest
M3C ABI/help：PASS / FAIL / BLOCKED / NOT_RUN + 證據路徑
M3C mono offline / half / one：各自列 process 與 estimator 判定
M3C sampler：PASS / FAIL / BLOCKED / NOT_RUN
真實 Camera+IMU VIO / physical UART：NOT_RUN，除非另有明確授權及證據
已觀察失敗 / 原始 log 位置 / unresolved blocker：
CPU / GNU Peak RSS / native timing / backlog；未知項不可填零：
thermal 是否只有快照、系統其他程序／負載、是否曾中斷：
實際修改過什麼設定或程式；若沒有請寫沒有：
需要開發主機接手的問題：
```

保留 raw `summary.json`、`frames.csv`、`poses.csv`、`native-events.jsonl`、
`effective-params.yaml`、trace、`process.log`、GNU time、stderr、exit、audit、
manifest、argv、測試 JUnit；失敗輪次也全部保留。不要只帶截圖或 ATE 數字。

若 laptop 修改了程式，額外保存差異，不擅自 merge／覆蓋主機或 RK3576：

```bash
git -C "$REPO" status --porcelain > "$SESSION/final-git-status.txt"
git -C "$REPO" diff HEAD --binary > "$SESSION/laptop-changes.patch"
git -C "$REPO" ls-files --others --exclude-standard > "$SESSION/untracked-source-files.txt"
```

patch 不含 untracked 檔案本體。若有新 source／測試檔，逐項檢查清單後，把必要檔案
複製至 `$SESSION/new-source/` 並保留相對路徑。不要把 venv、私鑰、known_hosts、
token、相機私有錄影或整個 `$HOME` 打包。若改了 estimator／binary，必須附新 build
來源與 hash，不能沿用這份參考包的 PASS。

打包並回傳（確認 RETURN.md 已寫好）：

```bash
test -s "$SESSION/RETURN.md"
cd "$TRANSFER_BASE/sessions"
tar -czf "$TRANSFER_BASE/$SESSION_ID-results.tar.gz" "$SESSION_ID"
cd "$TRANSFER_BASE"
sha256sum "$SESSION_ID-results.tar.gz" > "$SESSION_ID-results.tar.gz.sha256"
scp "$SESSION_ID-results.tar.gz" "$SESSION_ID-results.tar.gz.sha256" \
  "$BUILD_HOST:/home/shiuhou/Projects/rm27-m3c-schurvins-research/incoming/"
```

`incoming/` 已在開發主機準備好。無法 SSH 回主機就以 USB 搬運同兩檔，
不要為交接臨時開防火牆／公開服務。回到這個主機的對話提供：

> laptop 測試已回傳：Session `<實際值>`，archive `<主機 incoming 絕對路徑>`，
> SHA256 `<實際值>`。請依 M3C_LAPTOP_HANDOFF.md 核驗並分析，保留單目失敗證據，
> 修復後提供下一輪測試包；不修改 RK3576 工作樹。

## 10. 開發主機接手

先核對 archive checksum，再檢查 tar 成員無絕對路徑、`..`、symlink、device，
解壓到新的 session 目錄；不覆蓋原研究證據。
核對 RETURN、repo commit／diff、checked binary、輸入與實際板端身份。
以主機保留且 hash 相符的原始 EuRoC ZIP，對**完整** run 各自評分：

```bash
cd /home/shiuhou/Projects/rm27_vio-m3c
python3 -m tools.schurvins.headless_evaluate \
  --run /ABSOLUTE_NEW_SESSION/board-results/runs/mono-offline-full/native \
  --archive '/ABSOLUTE_VERIFIED_DATASET/V1_01_easy.zip' \
  --output /ABSOLUTE_NEW_SESSION/mono-offline-evaluation.json
```

上面 ABSOLUTE 路徑是接收後才能知道的值。ground truth 只在這一步讀取，不部署到板上。
smoke 有意選取部分幀，不得用它宣稱完整序列 PASS。已知 mono 評分會非零退出，
保留 JSON，不修改 `metric_initialization_verified`、不接雙目充當單目、不放寬 gate。
先區分 ABI／輸入／初始化／狀態生命週期／效能問題，再修復負責該行為的程式。
依新證據更新部署報告與唯一活動計畫，再交付下一個有新 identity 的測試版本。

設備已接到 laptop 並不等於真實 Camera+IMU VIO 可以驗收；這一輪的具體交付是
**可追溯的 M3C 板端單目公開資料執行／失敗證據與資源測量**。
