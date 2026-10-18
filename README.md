# shooter_calculator

目標位置から雑巾の射出初速を計算し、予測軌道を可視化するROS 2パッケージです。

ベルト直動式射出機構のリリース時終速度を軌道計算に使い、実測校正からベルト線速度へ変換します。射出初速とベルト速度は別の物理量として扱います。静止対象の計算に対応し、移動対象の発射時補償は未対応です。

---

## 1. 起動方法

ROS 2 Humbleを使用します。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/shooter_calculator --ignore-src -y --rosdistro humble
colcon build --symlink-install --packages-select shooter_calculator
source install/setup.bash
ros2 launch shooter_calculator shooter_calculator.launch.py calibration_file:=/absolute/path/calibration.json
```

既定の[config/calibration.json](config/calibration.json)は未承認のテンプレートです。承認済みの校正ファイルを指定するまで有効な射出解を出しません。bag再生時は`use_sim_time:=true`を追加します。

入力・配信トピック:

| 方向 | トピック | 型 | 内容 |
| :--- | :--- | :--- | :--- |
| 入力 | `/auto_mode/aim_target` | `shooter_calculator/msg/AimTarget` | 目標位置・観測時刻・session・世代・有効性 |
| 出力 | `/shooter/solution` | `shooter_calculator/msg/ShotSolution` | 射出初速・ベルト速度・校正ID・有効性・理由 |
| 出力 | `/shooter_calculator/trajectory_markers` | `visualization_msgs/msg/MarkerArray` | 予測軌道 |

型のみをビルドする場合は`--cmake-args -DBUILD_RUNTIME_NODES=OFF`を指定します。

---

## 2. 必須TFと入力条件

校正で指定する`map_frame → robot_frame`（通常は`map → base_link`）のTFが必要です。目標位置は`AimTarget`から受け取り、目標TFの代用は行いません。

現在の軌道モデルは水平なロボット姿勢を前提とします。入力の時刻・session・世代、受信からの単調期限、TFの進行と鮮度を検査し、古い入力やTFで有効な解を維持しません。移動対象や予測だけの目標は無効解を返します。

---

## 3. 計算モデル

1. `AimTarget`の目標位置と、TFからロボットの姿勢を取得します。
2. 校正済みの射出口オフセットをロボット姿勢でmapへ変換します。
3. 射出口から目標までの水平距離と高さ差を計算します。
4. 射出角度・雑巾の質量・面積・抵抗係数を使い、抗力付きモデルで必要初速を求めます。
5. 距離に対する到達高さを再評価し、許容誤差と校正範囲を検査します。
6. 実測の初速→ベルト速度ペアを補間し、射出解と予測軌道を配信します。

校正範囲外の値をクリップして有効解にはしません。未校正、入力断、不正値、TF断、到達不能、範囲外では`valid=false`と理由を公開します。

---

## 4. 実測データベースと校正

校正ファイルは次の情報を持ちます。

| 項目 | 内容 |
| :--- | :--- |
| `schema_version` / `approved` | `1` / `true` |
| `calibration_id` | 校正の識別子 |
| `map_frame` / `robot_frame` | 計算に使う座標系 |
| `shooter_offset_m` | ロボット座標系の射出口位置 `[x,y,z]` [m] |
| `launch_angle_deg` | 射出角度 [deg]。対応範囲は0度より大きく90度未満 |
| `cloth_mass_kg` / `cloth_area_m2` / `drag_coefficient` | 雑巾の質量・面積・抵抗係数 |
| `max_height_error_m` | 目標距離での高さ誤差の許容値 [m] |
| `calibration_pairs` | `[initial_speed_m_s, belt_speed_m_s]`の実測ペア |

質量は0.01kg以上、初速は1〜20m/sに対応します。校正ペアは2組以上を用意し、初速とベルト速度の絶対値を単調に増加させます。ベルト速度は同じ非ゼロ符号を持つ必要があります。測定した範囲内だけで補間します。

### 既存LUTの参考データ

コード内には、出力1〜10・射出角度50度の実験データも残っています。出力レベルはPWMや物理速度そのものではありません。このLUTはベルト線速度の校正には使用しません。

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

実際の計算には、校正ファイルの物理速度ペアとモデルを使います。実機の射出校正・精度確認は未完了です。

---

## 5. RVizでの表示

RVizのAdd → By topicから`/shooter_calculator/trajectory_markers`のMarkerArrayを追加します。有効解では予測軌道を線で表示し、無効解では軌道を消去します。`/shooter/solution`で初速・ベルト速度・有効性・理由を確認してください。

このノードは計算結果だけを配信し、アクチュエータ指令やベルト初期化は送信しません。無効解の速度0は、実際の射出禁止や物理停止を確認した値ではありません。

---

## 6. 検証

以下の試験にはローカル管理の`test/`ファイルが必要です。

```bash
colcon test --packages-select shooter_calculator
colcon test-result --verbose
```

合成した固定対象を使う試験はローカルのテストファイルがある場合だけ登録します。移動対象補償・実測校正・実射出の受入は別途必要です。
