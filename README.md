# shooter_calculator

ROS 2 で moving_bucket をターゲットとし、雑巾を射出するための軌道予測と可視化ノードです。

このパッケージは、PWM による理論計算ではなく、ベルト直動式射出機構の「リリース時終速度」を基準にした実測データベースを使う設計です。ROS からはベルト速度指令 [m/s] を送り、VESC 側で rpm 指令へ変換します。軌道計算では、雑巾がベルトから離れる瞬間の release velocity [m/s] を使います。

---

## 1. 起動方法

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select shooter_calculator
source install/setup.bash
ros2 launch shooter_calculator shooter_calculator.launch.py
```

ダミー環境の固定バケツを目標にする場合は、固定バケツの TF を指定します。

```bash
ros2 launch shooter_calculator shooter_calculator.launch.py \
	target_frame:=fixed_target_fixed_bucket_1
```

使用できる固定バケツの TF は次のとおりです。

- `fixed_target_fixed_bucket_1`
- `fixed_target_fixed_bucket_2`
- `fixed_target_fixed_bucket_3`

配信トピック:

- /shooter_calculator/trajectory_markers

RViz では Add -> By topic から MarkerArray を選択して表示します。

LiDAR のダミー環境と一緒に使う場合は、先に次のリポジトリを起動します。

```bash
ros2 launch lidar_perception_system test_dummy.launch.py
```

その後、このパッケージを起動します。固定バケツを使う場合は、目標 TF を指定します。

---

## 2. 必須 TF

起動前に次の TF が存在していることを確認してください。

- map -> base_link
- map -> moving_bucket または指定した固定バケツ TF

どちらかがない場合、lookupTransform に失敗して射撃計算ができません。

---

## 3. 計算モデル

このノードは次の順で計算を行います。

1. map 上の moving_bucket の位置を取得する
2. map 上の base_link の位置を取得する
3. ロボットの発射口位置と目標までの距離を算出する
4. 距離から必要な出力レベルを算出する
5. 出力レベルから release velocity を引く
6. その速度を使って軌道を可視化する

射出の前提は、ベルト直動が放出する瞬間の最高速度が雑巾の初速になるということです。ベルトへの速度指令値と雑巾の release velocity は同じ値とは限らないため、実験データで対応付けます。

---

## 4. 実測データベースの考え方

LUT は PWM ではなく、実験時のモーター出力と release velocity の対応表として扱います。実機制御時には、出力レベルをベルト速度指令または実測速度へ対応付けます。

- 出力レベル: 実験データ上のモーター出力
- release_velocity: ベルト直動から離れた瞬間の射出速度 [m/s]
- flight_distance: その速度で到達する推定飛距離 [m]

実機のデータを入れるときは、この形式に合わせて LUT を更新します。

現在コードに入っている実測値は、出力 1〜10、射出角度 50 deg の次の対応です。

| 出力 | release velocity [m/s] | flight distance [m] |
| ---: | ---: | ---: |
| 1 | 1.55 | 0.00 |
| 2 | 2.296666667 | 0.533333333 |
| 3 | 4.763333333 | 2.666666667 |
| 4 | 5.953333333 | 3.966666667 |
| 5 | 8.84 | 3.566666667 |
| 6 | 11.23333333 | 4.10 |
| 7 | 8.516666667 | 3.333333333 |
| 8 | 9.04 | 3.566666667 |
| 9 | 9.42 | 3.833333333 |
| 10 | 10.84666667 | 3.966666667 |

高さと滞空時間も元データには含まれますが、現在の選択処理では release velocity と軌道シミュレーションを使用します。実測値が増えた場合は、この表と `initializeLUT()` を更新してください。

---

## 5. 目標位置と着弾予測

ノードは目標位置を map 基準で見て、次のように距離を計算します。

- dx = target_x - (robot_x + 0.2)
- dy = target_y - robot_y
- horizontal_distance = sqrt(dx^2 + dy^2)

そして、目標の高さ差、射出角度、雑巾の質量、面積、抵抗係数を考慮して、出力 1〜10 の軌道をシミュレーションします。目標位置に最も近づく出力を 1 つ選び、その軌道だけを緑線で描画します。目標位置への強制補正は行いません。

---

## 6. RViz で何を描くか

- 緑の線: 選択された出力の予測軌道 1 本
- 白文字: 目標距離 / 推奨 belt_vel / モーター RPM / 予測 release velocity

通常起動ではモーターへ指令を送信しません。`enable_real_output:=true` の場合だけ、計算した `belt_vel` を `/belt/speed_ratio` へ送ります。表示される `Recommended belt_vel` とモーター RPM は計算上の推奨値であり、`Predicted release velocity` は実測 LUT に基づく予測値です。

## 7. 実機へ速度指令を送る場合

通常起動では実機へ何も送りません。`robot-port` を起動した状態で、次のように明示的に有効化した場合だけ `/belt/speed_ratio` へ速度指令を送ります。

```bash
ros2 launch shooter_calculator shooter_calculator.launch.py \
	enable_real_output:=true \
	auto_initialize_belt:=true
```

この機能が送るのはベルト速度指令だけです。`/belt/throw` は自動送信しないため、射出は別途確認してから実行します。

```bash
ros2 topic pub --once /belt/throw std_msgs/msg/Bool "{data: true}"
```

送信先は `robot-port` の次のトピックです。

- `/belt/speed_ratio`: ベルト速度指令 [m/s]
- `/belt/init`: ベルト初期化
- `/belt/throw`: 射出開始

現在の変換は、実測出力 1〜10 を `0.1〜1.0 m/s` に仮対応させています。実機で使う前に、`belt_vel` と実測 release velocity の対応を校正し、`belt_speed_command_max_mps` を確認してください。速度指令を有効にしても、物理的な安全確認なしに射出しないでください。

`enable_real_output:=true` にしても、雑巾を勝手に射出することはありません。計算ノードが送信するのは速度指令だけです。実際に射出するには、`robot-port` が起動している状態で、利用者が `/belt/throw` に `true` を明示的に送る必要があります。

予測だけを行う場合は、次の通常起動を使用してください。

```bash
ros2 launch shooter_calculator shooter_calculator.launch.py
```

---

## 8. パラメータ

launch ファイルで以下を調整します。

- launch_angle_deg: 発射角度 [deg]
- cloth_mass_kg: 雑巾の質量 [kg]
- cloth_area_m2: 受風面積 [m^2]
- drag_coefficient: 空気抵抗係数
- enable_real_output: 実機への速度指令を有効化。既定値は false
- auto_initialize_belt: `/belt/init` を自動送信。既定値は false
- belt_speed_command_max_mps: 出力 10 に対応させる最大速度 [m/s]

---

## 9. 運用時の確認ポイント

- map -> base_link が出ている
- map -> moving_bucket が出ている
- TF の座標系が map 基準である
- target_frame が指定した目標 TF に一致している
- ROS のベルト速度指令が [m/s] であることを確認している
- 実測したモーター出力 / ベルト速度 / release velocity を LUT に反映している

---

## 10. 重要な注意点

このノードは理論式だけで動いているわけではなく、実機計測ベースの LUT を使う設計です。実際の軌道の正確さは、次の更新で大きく変わります。

- 実際のベルト速度指令と VESC の rpm 変換値
- 実測の release velocity
- 実機での射出角度
- 雑巾の質量と面積

---

## 11. 現在のステータス

現時点の実装は、静止または動く TF ターゲットに対して、実測 LUT の出力 1〜10 を比較し、目標に最も近い 1 本の軌道を表示します。ターゲットの移動予測や実機へのモーター指令は、このパッケージには含まれていません。

---

## 12. 関連リポジトリ

今回のロボコンのベルト直動、エアシリンダー、ROS 2 通信、シミュレーション、設計資料は、次のリポジトリに分かれています。

- [gn10-mainboard](https://github.com/tmcit-ararobo-2026a/gn10-mainboard): メイン基板。ベルト速度指令、VESC 初期化、エアシリンダー用ソレノイドを制御
- [vesc-hub](https://github.com/tmcit-ararobo-2026a/vesc-hub): VESC とベルト直動の制御。速度比から rpm への変換、エンコーダーによる実速度計算
- [robot-port](https://github.com/tmcit-ararobo-2026a/robot-port): ROS 2 からメイン基板へ CAN 指令を送るノード。ベルト・エアシリンダーのトピックを提供
- [LiDAR-Perception-System](https://github.com/ta9ma12/LiDAR-Perception-System): LiDAR の実機・ダミー点群、固定バケツ・移動バケツの認識、`map` 基準の目標 TF、RViz2 設定を提供
- [system-design](https://github.com/tmcit-ararobo-2026a/system-design): VESC、BLDC-NEO、エア射出を含む配線・システム構成資料
- [robot-sim](https://github.com/tmcit-ararobo-2026a/robot-sim): Isaac Sim 2025 を使ったロボットシミュレーション

ベルト直動のデータフローは次のとおりです。

```text
/belt/speed_ratio [m/s]
	|
	v
robot-port
	|
	v
gn10-mainboard: belt_vel [m/s]
	|
	v
vesc-hub: target_rpm = belt_vel * -46000
	|
	v
ベルトの実速度 -> 雑巾の release velocity -> 軌道計算
```

LiDAR ダミー環境とのデータフローは次のとおりです。

```text
LiDAR-Perception-System/test_dummy.launch.py
	|
	+-> map -> base_link
	+-> map -> moving_bucket
	+-> map -> fixed_target_fixed_bucket_1/2/3
	|
	v
shooter_calculator
	|
	v
目標に最も近い出力の予測軌道を MarkerArray で表示
```

## STM32新仕様向け型生成（T01）

`-DBUILD_RUNTIME_NODES=OFF` で型生成を実行ノードから分離できる。
初速とベルト速度の変換・校正範囲・無効解処理はT13で実装し、校正未確定の間は実機受入を保留する。

## AimTarget・ShotSolution（T02b）

AimTargetは対象位置・速度・最終観測時刻・valid・session/世代を保持する。
ShotSolutionは射出初速とベルト速度、valid・校正範囲・校正IDを別々に保持する。
T02bは型定義まで。型付き入出力の接続・射出口姿勢・速度校正・invalid配信はT13の残作業。
旧0〜1比率を物理速度として新型へ流用しない。
