# 型付き静止射出解の検証（T13）

実施日: 2026-10-14（Asia/Tokyo）。作業ディレクトリ: `/home/lambda/ros2_ws/src`。
対象: `feat/1-shot-solution-interfaces`、基準 `e7e6eaa2114cc6b14d97e44d5c958ba4c8665f12`からT01・T02b・T13の差分。

`source /opt/ros/humble/setup.bash` 後、colcon build --base-paths shooter_calculator --build-base /tmp/stm32-ros2/build --install-base /tmp/stm32-ros2/install --packages-select shooter_calculator。
ビルド: 成功／終了コード0。source /tmp/stm32-ros2/install/setup.bash 後、ctest --test-dir /tmp/stm32-ros2/build/shooter_calculator --output-on-failure。

試験: 成功／終了コード0。合成校正のみで無抗力の独立した閉形式との初速比較、初速→belt速度（比率1へのclipなし）、Yaw回転した射出口オフセット、到達不能・校正範囲外・未承認・入力断・重複・TF断・invalid入力・移動対象未対応・不正校正を検証。
ROS公式schema検査・launch Python構文: 成功。旧belt/initとspeed_ratioの生成元なしを実DDSで確認。
ログ: /tmp/stm32-ros2/shooter-t13-final-build.log、shooter-t13-final-tests.log。

localhost専用domain、合成校正とgeometryだけを使用。実射出・物理速度変換・移動対象補償・実機・HIL: 未実施。
実測profileがないため既定はapproved=false。到達不能・範囲外・未承認はvalid=false。速度0だけを射出禁止・物理停止と扱わない。
旧LUTは保持し、belt校正に流用しない。実機校正・モデル誤差・移動対象補償の受入は未完了。
