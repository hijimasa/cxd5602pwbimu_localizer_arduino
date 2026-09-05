# Spresense IMU 自己位置推定

[English](./README.md) | 日本語

このリポジトリは、Spresenseボードの内蔵IMU（慣性計測ユニット）を利用してセンサによる自己位置推定を行うArduino IDE向けプロジェクトを収録しています。Spresenseのマルチコアを「I/O専任のMainCore」と「処理専任のSubCore1」に分けて使い、**Fusionライブラリ**によるAHRS、ZUPT（ゼロ速度更新）、台形積分で姿勢・速度・位置を推定します。

アルゴリズム本体はArduino非依存のC++モジュール（`SubCore1/src/localizer/`）として実装されており、ホストPC上で単体テストと合成IMUデータによる回帰テストが実行できます。

## 概要

本システムは、1920HzでIMUの生データ（加速度、角速度、温度）を読み取り、2コアで処理します：

- **MainCore:** IMUデータ取得、共有リングバッファへの書き込み、シリアル出力
- **SubCore1:** 初期アライメント、ガウシアンフィルタ、地球自転補償、Fusion AHRS、ZUPT、バイアス学習、速度・位置積分

## アーキテクチャ

```
┌────────────────────┐   共有メモリ      ┌──────────────────────────────────┐
│      MainCore      │  リングバッファ   │             SubCore1             │
│  IMU読取 1920Hz    │ ───────────────▶ │ アライメント → ガウシアン →      │
│  シリアル出力 30Hz │ ◀─────────────── │ 地球自転補償 → Fusion AHRS →     │
│                    │  位置 (30Hz)      │ ZUPT → バイアス学習 → 積分       │
└────────────────────┘  統計 (0.2Hz)     └──────────────────────────────────┘
```

コア間のIMUデータは、MainCoreが所有する256サンプルのリングバッファ（約133ms分）を通して渡されます。SubCore1は自分の読み出し位置を持ち、通知メッセージが取りこぼされても未処理サンプルをすべて処理します。SubCore1が255サンプル以上遅れた場合のみデータが失われ、その数は「overrun」として統計に計上されます。

## 特徴

- **欠落しないデータ経路:** 単一バッファの上書きではなくリングバッファで受け渡し。欠落は検出・計数される
- **高速IMUデータ取得:** IMUの性能を最大限に活用した1920Hzサンプリング
- **ジャイロコンパス式の初期アライメント:** 静止時の加速度平均から上方向、角速度平均の水平成分から北方向を求め、Fusionの規約（NWU）に一致する初期クォータニオンを生成
- **ジャイロオフセットの初期推定と地球自転補償:** 較正中の角速度平均から地球自転分を差し引いた値を初期オフセットとし、運転中は姿勢に応じて地球自転を常時補償
- **Fusion AHRS統合:** 加速度計拒否機能と重力除去を備えたセンサフュージョン
- **ZUPT検出（分散＋平均）:** 信号の分散に加えて窓内の平均加速度・平均角速度も評価し、滑らかな加速中を静止と誤判定しない。推定速度が大きい間は確認期間を延ばし、等速区間での誤発火を防ぐ
- **AHRSゲインの切り替え:** 静止確定時は高ゲインで素早く収束、運動中は低ゲインで並進加速度による姿勢の引き込みを抑制
- **静止中の加速度バイアス学習**
- **台形積分とZUPT遡及補正:** ZUPT確定時に残っている速度は直前の移動区間で積み上がった誤差そのものなので、区間内で線形に増えたとみなして位置から差し引く
- **ホストで動く単体テスト:** 合成IMUデータによる回帰テストで機能劣化を防止

## ハードウェア要件

- **Sony Spresense メインボード**（CXD5602プロセッサ搭載）
- **Spresense IMUアドオンボード**（CXD5602PWBIMU）

## ソフトウェア要件

- **Arduino IDE**（1.8.x または 2.x）または **arduino-cli**
- **Spresense Arduino ボードパッケージ**（3.0.0以降）
- テスト実行には g++（C++17）と make

## はじめに

### インストール

1. **リポジトリのクローン:**

   ```bash
   git clone https://github.com/hijimasa/cxd5602pwbimu_localizer_arduino.git
   cd cxd5602pwbimu_localizer_arduino
   git submodule update --init
   ```

   Fusionライブラリはサブモジュールで取得し、`SubCore1/src/Fusion` からシンボリックリンクで参照しています。

2. **Spresense Arduino ボードパッケージのインストール:**

   - Arduino IDEを開く
   - **ファイル** > **基本設定** を選択
   - 「追加のボードマネージャのURL」に以下を追加：
     ```
     https://github.com/sonydevworld/spresense-arduino-compatible/releases/download/generic/package_spresense_index.json
     ```
   - **ツール** > **ボード** > **ボードマネージャ** を選択
   - 「Spresense」を検索してパッケージをインストール

### ビルドとアップロード（マルチコア）

SubCore1をMainCoreより**先に**アップロードしてください。

1. **SubCore1（全処理）:**
   - Arduino IDEで `SubCore1/SubCore1.ino` を開く
   - **ツール** > **Core** > **"SubCore 1"** を選択
   - **マイコンボードに書き込む**をクリック

2. **MainCore（IMU取得・出力）:**
   - Arduino IDEで `cxd5602pwbimu_localizer_arduino.ino` を開く
   - **ツール** > **Core** > **"MainCore"** を選択
   - **マイコンボードに書き込む**をクリック

arduino-cli を使う場合:

```bash
arduino-cli compile --fqbn SPRESENSE:spresense:spresense:Core=Sub1 SubCore1
arduino-cli compile --fqbn SPRESENSE:spresense:spresense:Core=Main .
```

### アプリケーションの実行

1. すべてのコアをアップロード後、シリアルモニタを開く
2. ボーレートを **115200** に設定
3. 起動後 **15秒間** はセンサを完全に静止させてください（初期姿勢・ジャイロオフセット・加速度バイアスの較正）
4. 較正後、30Hzで位置データが出力されます

### 出力フォーマット

1行に18個の値がカンマ区切りで出力されます。タイムスタンプ以外はfloatのIEEE754ビット列を16進8桁で表したものです：
```
timestamp, temp, gx, gy, gz, ax, ay, az, qw, qx, qy, qz, vx, vy, vz, px, py, pz
```

- `gx..gz`: オフセット補正済み角速度 [rad/s]（センサ座標系）
- `ax..az`: 重力除去・バイアス補正済み加速度 [m/s²]（地球座標系 NWU）
- `qw..qz`: 姿勢クォータニオン（センサ→地球、NWU）
- `vx..vz`, `px..pz`: 速度 [m/s]、位置 [m]（地球座標系）

`VERBOSE_OUTPUT` を定義してビルドすると、5秒ごとに処理レート、リングバッファのoverrun数、dt異常数、推定バイアスなどの統計が追加で出力されます。

## テスト

アルゴリズムはホストPC上でテストできます：

```bash
cd test
make            # ビルドして全テストを実行
make run ARGS=localizer   # 名前に localizer を含むテストだけ実行
```

テストの内容：

- クォータニオン変換がFusionライブラリの規約と一致すること
- 初期アライメントがランダムな姿勢を復元すること（傾けて起動した場合の回帰テストを含む）
- ガウシアンフィルタ、ZUPT検出器、積分器の単体テスト（旧実装を参照実装として同値性を確認）
- リングバッファの欠落なし・overrun検出・スレッド並行動作
- 合成IMUデータ（ノイズ・バイアス・地球自転を含む）による end-to-end 回帰テスト：静止、傾斜起動、回転、並進、タイムスタンプの桁あふれ、サンプル欠落時のdt

各テストは `metric` 行に現在の性能値（姿勢誤差、位置誤差など）を出力します。閾値は改善に合わせて厳しくし、緩めないでください。

## 実機データの記録と再生

MainCoreを `RAW_LOG_OUTPUT` 付きでビルドすると、全IMUサンプル（1920Hz）と実機の位置推定（30Hz）がバイナリフレームで 921600 baud のシリアルに出力されます。これをホストで記録し、同じアルゴリズムで再生できます。

```bash
# 記録用MainCoreをビルド・書き込み（SubCore1は通常のもの）
arduino-cli compile --upload -p /dev/ttyUSB0 \
  --build-property "build.extra_flags=-DRAW_LOG_OUTPUT" \
  --fqbn SPRESENSE:spresense:spresense:Core=Main .

# 60秒記録（ポートを開くとボードがリセットされ、15秒の較正から始まる）
python3 tools/record_raw.py /dev/ttyUSB0 60 logs/desk_1m.bin

# ホストで再生（--csv で30Hzの推定結果を書き出し、オプションで各機能をオフにして比較）
cd test && make replay
./out/replay ../logs/desk_1m.bin --csv desk_1m.csv
./out/replay ../logs/desk_1m.bin --no-retro
```

再生結果には実機の位置推定との最大差が表示されます。同じコードが動いているので、通常は位置差0、クォータニオン差1e-5以下です。

記録用ファームウェアは起動後2秒間はシリアル出力を止め、書き込みツールがブートROMと同期する時間を確保します。それでも921600 baudのストリームを別のボーレートで読むとホスト側のUSBシリアルブリッジ（CP210x）が固まり、以後何も受信できず書き込みも `flash_writer` のプロンプト待ちで止まることがあります。その場合は `flash_writer` を止め、`python3 tools/usb_reset_cp210x.py` でブリッジをUSBレベルでリセットしてから（ボードの抜き差し不要）書き込みを再試行してください。

## プロジェクト構成

```
cxd5602pwbimu_localizer_arduino/
├── cxd5602pwbimu_localizer_arduino.ino  # MainCore: IMU取得 & シリアル出力
├── shared_types.h                        # コア間通信の構造体とリングバッファ
├── SubCore1/
│   ├── SubCore1.ino                      # SubCore1: パイプライン実行と結果送信
│   ├── shared_types.h -> ../shared_types.h
│   └── src/
│       ├── Fusion -> ../../Fusion/Fusion # Fusionライブラリ（サブモジュール）
│       └── localizer/                    # Arduino非依存のアルゴリズム
│           ├── localizer_config.h        # 全パラメータ
│           ├── vec_math.h                # ベクトル・クォータニオン
│           ├── initial_alignment.*       # 初期姿勢・オフセット・バイアス
│           ├── gaussian_filter.*         # 因果型ガウシアンフィルタ
│           ├── zupt_detector.*           # 分散ベースZUPT
│           ├── velocity_integrator.*     # 台形積分
│           └── localizer.*               # パイプライン統合
├── test/                                 # ホスト用単体テスト（make）
│   ├── imu_simulator.h                   # 合成IMUデータ生成
│   ├── replay.cpp                        # 実機ログの再生（make replay）
│   └── test_*.cpp
├── tools/
│   ├── record_raw.py                     # 実機の生データ記録
│   └── usb_reset_cp210x.py               # 固まったUSBシリアルブリッジの復旧
└── Fusion/                               # Fusionライブラリ（サブモジュール）
```

## アルゴリズム詳細

### 初期アライメント（15秒、静止）

1. 加速度と角速度を平均する
2. 加速度平均の方向を上（Up）、角速度平均の水平成分を北（North）、West = Up × North としてNWU座標系を構成し、初期クォータニオンを求める
3. 緯度から求めた地球自転ベクトルをセンサ座標系に変換し、角速度平均から差し引いた残りをジャイロオフセットとする
4. 加速度平均から期待重力ベクトルを引いた値を加速度バイアスとする（重力に直交する成分は原理的に観測できない）

### データ処理パイプライン（1サンプルごと）

1. **dt計算:** IMUタイムスタンプ差から算出（32bit桁あふれ対応）。公称の1.5倍を超えると異常として計数、10倍で飽和
2. **ガウシアンフィルタ:** σ = 2サンプルの因果型フィルタ（現在サンプルに最大の重み）
3. **地球自転補償:** 現在の姿勢推定で地球自転ベクトルをセンサ座標系に回して角速度から減算
4. **ジャイロオフセット追跡:** FusionOffset（初期値はアライメント結果）
5. **AHRS更新:** Fusion（加速度拒否 10°、ゲインは7bで切り替え）
6. **重力除去とバイアス補正:** センサ座標系で補正した後、地球座標系へ回転
7. **ZUPT検出:** 16サンプル窓で「分散が閾値未満」かつ「平均加速度 < 0.1 m/s² かつ平均角速度 < 2°/s」の状態が96サンプル（50ms）続いたら静止と判定。推定速度が0.1 m/sを超えている間は1秒の連続静止を要求する（移動中の短い等速区間を停止と誤認しないため）。解除は即時
7b. **ゲイン切り替え:** 静止確定 → 0.5、運動中（高分散、または平均加速度・角速度が大きい） → 0.02、低分散かつ平均も小さいが未確定 → 0.1。「低分散だが平均加速度が大きい」状態が2秒以上続く場合は静止時の姿勢誤差とみなし0.1に戻す
8. **バイアス学習:** 静止中、残留加速度を時定数1秒で学習
9. **積分:** 速度・位置の台形積分。静止中は速度をゼロにリセット
10. **ZUPT遡及補正:** 移動区間（長さT）の終わりにZUPTが確定した時点の速度 v_err は誤差なので、位置から v_err × T / 2 を差し引く。出力の連続性を保つため、補正は段差ではなくZUPT確定後の1秒間に線形に配分する（未適用分が残る間に次のZUPTが来た場合は合算）。速度クランプが効いた区間や60秒を超える区間は補正しない

### 合成データでの現在の性能（`cd test && make` の metric 行）

| シナリオ | 位置誤差 |
|---|---|
| 0.3 m を 0.5 s で移動（急） | 0.6 mm |
| 1 m を 2 s で移動（滑らか） | 3.5 mm |
| 2 m を 3 s で移動 | 8.3 mm |
| 1 m を 4 s で移動（遅い、最大加速度0.36 m/s²） | 2.0 cm |
| 90° 旋回 | 1.5 cm、姿勢誤差 0.03° |

### 既知の制約（次の改善対象）

- **1秒を超える等速移動は静止と区別できません。** IMU単体の原理的な限界です。等速区間が1秒以内なら速度依存の確認期間で回避できます
- **ゆっくりした移動**（加速度が0.1 m/s²前後）は平均加速度の閾値付近になり、距離が短めに出ます
- **ジャイロコンパスの精度**はジャイロバイアスに依存します。バイアスが地球自転の水平成分（東京で約0.0034°/s）より大きい場合、初期の方位は任意になります（相対位置推定には影響しません。テストでは初期方位のずれを補正した座標系で位置誤差を評価しています）
- **実機の運動データでの閾値調整が未実施です。** 合成データはノイズ・バイアス・地球自転を含みますが、実際の振動特性は含みません。下記の生データ記録と再生で実機ログを回帰テストに取り込めます

## 設定

`SubCore1/src/localizer/localizer_config.h` の主要パラメータ：

| パラメータ | 値 | 説明 |
|-----------|------|------|
| `LOCALIZER_SAMPLE_RATE` | 1920 Hz | IMUサンプリングレート |
| `LOCALIZER_GRAVITY` | 9.7975 m/s² | 局所重力（東京） |
| `LOCALIZER_LATITUDE_DEG` | 35.68° | 緯度（地球自転補償、東京） |
| `LOCALIZER_CALIBRATION_SECONDS` | 15 s | 起動時の静止較正時間 |
| `FUSION_GAIN_STATIONARY` / `SETTLING` / `MOTION` | 0.5 / 0.1 / 0.02 | 状態別AHRSゲイン |
| `FUSION_ACCEL_REJECTION` | 10.0° | 加速度計拒否閾値 |
| `ZUPT_ACCEL_VARIANCE_THRESHOLD` | 0.004 (m/s²)² | ZUPT加速度分散閾値 |
| `ZUPT_GYRO_VARIANCE_THRESHOLD` | 0.00006 (rad/s)² | ZUPT角速度分散閾値 |
| `ZUPT_ACCEL_MEAN_THRESHOLD` | 0.1 m/s² | ZUPT平均加速度閾値 |
| `ZUPT_GYRO_MEAN_THRESHOLD` | 2°/s | ZUPT平均角速度閾値 |
| `ZUPT_SLOW_CONFIRM_SPEED` / `SAMPLES` | 0.1 m/s / 1 s | 推定速度が大きいときの確認期間 |
| `ZUPT_RETROACTIVE_CORRECTION` | 1 | ZUPT遡及補正の有効化 |
| `ZUPT_CORRECTION_BLEND_TIME` | 1.0 s | 遡及補正を配分する時間（0で即時） |
| `IMU_RING_SIZE`（shared_types.h） | 256 | コア間リングバッファ長 |
| `IMU_NOTIFY_INTERVAL`（shared_types.h） | 8 | SubCore1を起こす通知の間隔（サンプル数）。受信FIFOは8個しかないため頻度を抑える |

## 可視化

### ROS 2連携

リアルタイムの可視化とROSエコシステムとの統合には、ROS 2パッケージをご利用ください：

**[cxd5602pwbimu_localizer_node](https://github.com/hijimasa/cxd5602pwbimu_localizer_node)**

このROS 2ノードは、本Arduinoプロジェクトのシリアル出力を読み取り、標準ROSメッセージに変換します：
- `sensor_msgs/Imu` - IMUデータ（角速度、線形加速度、姿勢）
- `geometry_msgs/Pose` - 位置と姿勢
- RViz可視化用TF変換

#### クイックスタート

```bash
# ROS 2パッケージのインストール
cd ~/ros2_ws/src
git clone https://github.com/hijimasa/cxd5602pwbimu_localizer_node.git
cd ~/ros2_ws
colcon build --packages-select cxd5602pwbimu_localizer_node
source install/setup.bash

# ノードの実行
ros2 run cxd5602pwbimu_localizer_node localizer_node --ros-args -p serial_port:=/dev/ttyUSB0
```

RVizでTFとPoseディスプレイを追加することで、センサデータを可視化できます。

## トラブルシューティング

### よくある問題

1. **出力が始まらない:**
   - SubCore1がアップロードされていることを確認
   - 起動後15秒の較正が終わるまで出力はありません

2. **静止中の位置ドリフト:**
   - 初期較正中はセンサを完全に静止させてください
   - `VERBOSE_OUTPUT` で overrun と dt_anomalies が0であることを確認してください

3. **コンパイルエラー:**
   - `git submodule update --init` を実行済みか確認（`SubCore1/src/Fusion` がサブモジュールを指します）
   - Spresenseボードパッケージが正しくインストールされていることを確認
   - ツールメニューで正しいコアが選択されていることを確認

## Contributing

貢献は大歓迎です！リポジトリをフォークして、プルリクエストをお送りください。大きな変更を加える場合は、事前にIssueを立てて議論いただけると助かります。変更時は `cd test && make` でテストが通ることを確認してください。

## ライセンス

このプロジェクトはMITライセンスの下で配布されています。詳細は `LICENSE` ファイルをご確認ください。

---

ご質問や問題がございましたら、GitHub Issueを作成するか、リポジトリ管理者にお問い合わせください。
