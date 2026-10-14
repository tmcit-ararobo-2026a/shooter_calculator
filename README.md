# shooter_calculator

ROS 2 Humbleの型付き射出解・軌道可視化パッケージ。静止したAimTargetと承認済み校正を使い、
射出初速とベルト線速度を分離する。直接アクチュエータ指令やbelt初期化は配信しない。
移動対象の発射時補償は未承認で、現段階ではinvalidを返す。

## 起動

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select shooter_calculator
source install/setup.bash
ros2 launch shooter_calculator shooter_calculator.launch.py calibration_file:=/approved/calibration.json
```

既定テンプレートは未承認で、有効解を出さない。TF目標のfallbackとenable_real_outputは廃止。
入力は `/auto_mode/aim_target`、必要TFはmap→base_link。公開は `/shooter/solution` と
`/shooter_calculator/trajectory_markers`。invalidで軌道を消去する。
旧test_dummy.launch.py・旧target_frame指定は現行の起動手順として使わない。

## 計算

承認された射出口オフセットをbase_linkの姿勢でmapへ変換し、既存抗力モデルで初速を解く。
到達高さを検査し、実測の初速→ベルト速度ペアの範囲内だけで補間する。robot姿勢は現2Dモデルに一致する水平姿勢を必要とする。
未校正・入力断・不正値・TF断・到達不能・範囲外ではvalid=falseと理由を公開する。
solverの境界返値や比率を物理速度・有効解として扱わない。

## 旧LUT資料（保持・新ベルト校正には未採用）



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

高さと滞空時間も元データには含まれますが、新しい選択処理は承認済みモデルと速度校正を使い、このLUTを物理速度の校正へ流用しません。実測値が増えた場合は、この表と `initializeLUT()` を更新してください。

---

## STM32新仕様向け型生成（T01）

`-DBUILD_RUNTIME_NODES=OFF` で型生成を実行ノードから分離できる。
初速とベルト速度の変換・校正範囲・無効解処理をT13で実装。校正未確定の間は実機受入を保留する。

## AimTarget・ShotSolution（T02b）

AimTargetは対象位置・速度・最終観測時刻・valid・session/世代を保持する。
ShotSolutionは射出初速とベルト速度、valid・校正範囲・校正IDを別々に保持する。
T02bの型定義に加えT13で型付き入出力・射出口姿勢・校正枠組み・invalid配信を実装。実測校正と移動対象補償は未完了。
旧0〜1比率を物理速度として新型へ流用しない。

## 型付き射出解（T13）

入力は `/auto_mode/aim_target`、出力は `/shooter/solution`。旧TF目標・0〜1比率・直接belt/init配信は廃止。
承認されたshooter_offset_mをbase_link姿勢のQuaternionでmapへ変換し、既存の抗力付き計算を再利用する。
solverの返値だけで有効にせず、到達距離での高さを再評価し、到達不能と校正範囲外をinvalidにする。
射出初速とベルト線速度は承認済みの物理速度ペアで補間し、範囲外をクリップしない。
旧LUTは維持するが、belt速度の校正に流用しない。invalid速度0は射出禁止や物理停止の証拠ではない。

config/calibration.jsonは未承認テンプレート。承認ファイルはschema_version=1、approved=true、
calibration_id、map_frame、robot_frame、shooter_offset_m=[x,y,z]、launch_angle_deg、cloth_mass_kg、
cloth_area_m2、drag_coefficient、max_height_error_m、calibration_pairs=[[initial_speed_m_s,belt_speed_m_s],...]。
現モデルは角度(0,90)度・mass>=0.01kg・初速[1,20]m/sをサポートする。各値は実測契約が必要で既定値を置かない。
calibration_pairsは初速とbelt速度の絶対値が増加し、belt速度は同じ非ゼロ符号を持つこと。
移動対象の発射時補償は未承認・未実装でinvalidを返す（T11・T12との結合と校正後に対応）。

生成stamp・session・世代・受信からの単調期限、TFの進行と期限で古い解を維持しない。
露出する物理値・固定目標・実射出の校正受入は未完了。試験は合成固定対象のみ。
