
# 過去の実験の記録

## SSH鍵生成からログインまでの手続き

```
# Client (tamago.arc.ics.keio.ac.jp)
$ ssh-keygen -t rsa
$ scp .ssh/my_id_rsa.pub user@server:~/.ssh/
$ chmod 600 ~/.ssh/my_id_rsa.pub

# Server (pynq-z2)
$ cat ~/.ssh/my_id_rsa.pub >> ~/.ssh/authorized_keys
$ chmod 755 ~
$ chmod 700 ~/.ssh
$ chmod 600 ~/.ssh/authorized_keys
$ Edit /etc/ssh/sshd_config and set AuthorizedKeysFile
$ sudo systemctl reload sshd.service

# Client
$ eval "$(ssh-agent)"
$ ssh-add ~/.ssh/my_id_rsa (Secret key)
$ ssh-agent -k
or
# Enable ssh-agent autostart by editing ~/.bashrc
# ~/.bashrc
# if [ -f ~/.ssh-agent ]; then
#     . ~/.ssh-agent
# fi
# if [ -z "$SSH_AGENT_PID" ] || ! kill -0 $SSH_AGENT_PID; then
#     ssh-agent > ~/.ssh-agent
#     . ~/.ssh-agent
# fi
# ssh-add -l >& /dev/null || ssh-add
# Enable autologin by editing ~/.ssh/config
# ~/.ssh/config
# Host pynq0
#     HostName 192.168.30.50
#     User xilinx
#     IdentityFile ~/.ssh/id_rsa_pynq
#     AddKeysToAgent yes
#     ServerAliveInterval 60
#     ServerAliveCountMax 10
#     ForwardX11 yes
#     Compression yes

# Password required for the first login (key is added to the agent)
$ ssh -X pynq0
# Password not required for the second login
$ ssh -X pynq0
```

## ROSによるデータ取得

```
# メガローバーがオドメトリ情報をPublish
$ rosrun megarover_samples pub_odom

# スキャンデータの取得を開始
$ rosrun urg_node urg_node _serial_port:=/dev/ttyACM0 \
    _frame_id:=laser _angle_min:=-1.57 _angle_max:=1.57

# ロボットの中心からレーザスキャナへの座標変換をPublish (不必要)
$ rosrun tf static_transform_publisher \
    0.1 0 0.08 3.14159 3.14159 0 base_link laser 100

# Bagファイルを保存
$ rosbag record -o matutani-lab /scan /odom /tf

# Carmen形式のログに変換
rosrun bag2carmen bag2carmen_node _carmen:=test.carmen _bag_file:=test.bag
```

## `top`コマンドの出力の保存

```
top -d 1 -b | grep --line-buffered hoge >> fuga.log
```

## 実験ログ

- 設定ファイル(settings-*.json)は各ディレクトリにそのまま残しておく
- `screen`、`screen -ls`、`screen -r`(アタッチ)、`Ctrl-a d`(デタッチ)

### pynq0 (192.168.30.50)
- lidar-graph-slam/results/hw-intel-0 [0-4]
- lidar-graph-slam/results/sw-intel-0 [0-4]
- lidar-graph-slam/results/hw-intel-1 (ループ検出のスコア閾値を0.5から0.45に) [0-9]
- lidar-graph-slam/results/sw-intel-1 (スキャンマッチングの範囲を0.2x0.2x0.5から0.25x0.25x0.5に) [0-4][5-9]
- my-gmapping/sw-fr079-0 [0-2]
- my-gmapping/hw-fr079-0 (sw-fr079-0と同じ設定) [0-4]
- my-gmapping/sw-fr079-1 [0-2]
- my-gmapping/hw-fr079-1 (sw-fr079-1と同じ設定) [0-4][5-9]
- my-gmapping/hw-mit-0 (pynq1/my-gmapping/sw-mit-3と同じ設定) [0-4][5-9][10-19][20-39]
- lidar-graph-slam/results/sw-sousoukan-1 (hw-sousoukan-10と同じ設定) [0-4]
- lidar-graph-slam/results/sw-mit-1-2 (hw-mit-7と同じ設定で、ソフトウェア上で実行) [0-4]
- 山登り法のスキャンマッチングを並列化したバージョン (バッチ処理)
- my-gmapping/sw-fr079-0-new [0-2]
- my-gmapping/hw-fr079-0-new [0-2]
- my-gmapping/sw-fr079-1-new [0-2]
- my-gmapping/hw-fr079-1-new [0-2]
- my-gmapping/hw-mit-0-new [0-2]

### pynq1 (192.168.30.51)
- lidar-graph-slam/results/hw-fr079-0 [0-4]
- lidar-graph-slam/results/sw-fr079-0 [0-4]
- lidar-graph-slam/results/hw-fr079-1 (ループ検出のスコア閾値を0.5から0.45に) [0-4]
- lidar-graph-slam/results/sw-fr079-1 (ループ検出のスコア閾値を0.55から0.5に) [0-4]
- lidar-graph-slam/results/hw-fr079-2 (ループ検出のスコア閾値を0.45から0.4に) [0-4]
- lidar-graph-slam/results/hw-fr079-3 (スキャンマッチングの範囲を0.25x0.25x0.5から0.2x0.2x0.25に) [0-4]
- lidar-graph-slam/results/hw-fr079-4 (ProbabilityHitを0.62から0.6に) [0-4]
- lidar-graph-slam/results/hw-fr079-5 (ループ検出の探索範囲を2.5x2.5x0.5から2.0x2.0x0.5に、スコア閾値を0.45に) [0-9]
- lidar-graph-slam/results/hw-fr079-6 (ループ検出の探索範囲を2.0x2.0x0.5から4.0x4.0x1.0に、スコア閾値を0.4に) [0-9]
- lidar-graph-slam/results/hw-fr079-7 (センサデータ取得の間隔を0.5radから0.25radに) [0-4]
- lidar-graph-slam/results/sw-fr079-2 (hw-fr079-7と同じ設定) [0-4]
- lidar-graph-slam/results/hw-fr079-8 (ループ検出の探索範囲を2.5x2.5x1.0に) [0-4][5-9]
- lidar-graph-slam/results/sw-fr079-3 (hw-fr079-8と同じ設定) [0-4]
- my-gmapping/sw-mit-0 [0-2]
- my-gmapping/sw-mit-1 [0-2] (センサデータ取得の間隔を0.25mから0.5mに、確率値を0.58から0.62に)
- my-gmapping/sw-mit-2 [0-2] (センサデータ取得の間隔を0.25radから0.5radに)
- my-gmapping-sw-mit-3 [0-2][3-10] (確率値を0.62に、尤度に対するスケーリングを0.75に)
- lidar-graph-slam/results/hw-sousoukan-7 [0-4] (スキャンマッチングの探索範囲を0.15x0.15x0.15に)
- lidar-graph-slam/results/hw-sousoukan-9 [0-4] (ループ検出の探索範囲を4.0x4.0x0.5に、スキャンマッチングの探索範囲を0.1x0.1x0.15に、確率値を0.62に、ループ検出の間隔を1mに)
- lidar-graph-slam/results/hw-sousoukan-10-2 [0-19] (lidar-graph-slam/hw-sousoukan-10と同じ)

### pynq2 (192.168.30.55)
- lidar-graph-slam/results/hw-mit-0 [0-4]
- lidar-graph-slam/results/sw-mit-0 [0-4]
- lidar-graph-slam/results/hw-mit-1 (ループ検出のスコア閾値を0.5から0.4に) [0-4]
- lidar-graph-slam/results/hw-mit-2 (スキャンマッチングの範囲を0.25x0.25x0.5から0.2x0.2x0.25に、ループ検出のスコア閾値を0.45に) [0-4]
- lidar-graph-slam/results/hw-mit-3 (スキャンマッチングの範囲を0.2x0.2x0.25から0.25x0.25x0.5に、ループ検出の探索範囲を2.5x2.5x0.5から4.0x4.0x1.0に) [0-4]
- lidar-graph-slam/results/hw-mit-4 (ループ検出の間隔を4mから6mに、確率値を0.63から0.62、0.46から0.45に、スキャンマッチングの範囲を0.2x0.2x0.25に、ループ検出の探索範囲を2.5x2.5x0.5に) [0-4]
- lidar-graph-slam/results/hw-mit-5 (確率値を0.62から0.63、0.45から0.46に戻す) [0-4]
- lidar-graph-slam/results/hw-mit-6 (ループ検出の間隔を6mから3mに) [0-4]
- lidar-graph-slam/results/hw-mit-7 [0-4][5-14][15-29]
- lidar-graph-slam/results/sw-sousoukan-0 [0-4]
- lidar-graph-slam/results/hw-sousoukan-0 [0-4] (sw-sousoukan-0と同じ設定)
- lidar-graph-slam/results/hw-sousoukan-1 (ループ検出の閾値を0.45から0.4に) [0-4]
- lidar-graph-slam/results/hw-sousoukan-2 (確率値を0.62に) [0-4]
- lidar-graph-slam/results/hw-sousoukan-3 (ループ検出の閾値を0.4から0.3に) [0-4]
- lidar-graph-slam/results/hw-sousoukan-4 (確率値を0.62に、ループ検出の閾値を0.4に、ループ検出の間隔を2mに、ループ検出の探索範囲を2.5x2.5x0.5から4.0x4.0x0.5に) [0-4]
- lidar-graph-slam/results/hw-sousoukan-5 (hw-sousoukan-4で用いた確率値0.62を、0.6に) [0-4]
- lidar-graph-slam/results/hw-sousoukan-6 (hw-sousoukan-0の設定から、スコアの閾値を0.35に)
- lidar-graph-slam/results/hw-sousoukan-11 (hw-sousoukan-6の設定から、スキャンマッチングの探索範囲を0.1x0.1x0.25に) [0-4]
- lidar-graph-slam/results/hw-sousoukan-12 (hw-sousoukan-10がようやくうまく行ったので、確率値を0.62から0.6へ変更) [0-4][5-19]
- lidar-graph-slam/results/sw-sousoukan-1 (hw-sousoukan-10と同じ設定で、ソフトウェア上で実行) [0-4]
- lidar-graph-slam/results/sw-mit-1 (hw-mit-7と同じ設定で、ソフトウェア上で実行) [0-4]
- lidar-graph-slam/results/hw-sousoukan-10-3 (hw-sousoukan-10と同じ設定) [0-49]
- 山登り法のスキャンマッチングを並列化したバージョン (バッチ処理)
- my-gmapping/sw-mit-0-new [0-2]
- my-gmapping/sw-mit-1-new [0-2]
- my-gmapping/sw-mit-2-new [0-2]
- my-gmapping/sw-mit-3-new [0-2]

### pynq3 (192.168.30.56)
- my-gmapping/sw-intel-0 [0-2] (大成功)
- my-gmapping/hw-intel-0 (スキャンマッチングの範囲を0.2x0.2x0.15) [0-4][5-9][0-19][20-39]
- my-gmapping/hw-intel-1 (尤度のスケールを0.5から0.75に) [0-9]
- my-gmapping/hw-intel-2 (尤度のスケールを0.5に、確率値を0.58に) [0-4]
- lidar-graph-slam/hw-sousoukan-8 [0-4] (hw-sousoukan-6の設定から、ループ検出の探索範囲を3.0x3.0x0.5に)
- lidar-graph-slam/hw-sousoukan-10 [0-4][5-19] (hw-sousoukan-9の設定から、スキャンマッチングの範囲を0.1x0.1x0.25に)
- lidar-graph-slam/hw-sousoukan-13 [0-4] (hw-sousoukan-10の設定から、0.62,0.45の確率値を0.61,0.45に)
- lidar-graph-slam/hw-sousoukan-14 [0-4] (hw-sousoukan-10の設定から、0.62,0.45の確率値を0.62,0.46に)
- lidar-graph-slam/hw-sousoukan-15 [0-4] (hw-sousoukan-10の設定から、0.62,0.45の確率値を0.615,0.45に)
- lidar-graph-slam/hw-sousoukan-16 [0-4] (hw-sousoukan-10の設定から、0.62,0.45の確率値を0.62,0.455に)
- lidar-graph-slam/hw-sousoukan-17 [0-4] (hw-sousoukan-10の設定から、0.62,0.45の確率値を0.625,0.45に)
- 山登り法のスキャンマッチング処理を並列化したバージョン (バッチ処理)
- my-gmapping/sw-intel-0-new [0-2]
- my-gmapping/hw-intel-0-new [0-2]
- my-gmapping/hw-intel-1-new [0-2]
- my-gmapping/hw-intel-2-new [0-2]

### 細かな話1

- 1つのIPコアにつき、coarseGridMapに24個, gridMapに21.5個のBRAMスライスを使用している
- CSM(Correlative Scan Matcher)コアが2つ搭載されているので、91個のBRAMスライスが地図のために使用される
- Pynq-Z2には140個のBRAMが搭載されているので、65%のBRAMリソース(91 / 140)が地図のために使用される
- CPUから受け取ったスキャンデータの距離値を格納するのに0.5個、角度を格納するのに0.5個
- 2Dの直交座標のスキャン点のX座標を格納するのに0.5個、Y座標を格納するのに0.5個
- 2つのコアが実装されているので、上記を2倍すれば、4個(2.86%)のBRAMスライス(4 / 140)がスキャンデータを格納するために使用されている
- 低解像度の地図を計算するための一時的なキャッシュとして、1.5個のBRAMスライスが使用される
- 2つのコアが実装されているので、上記を2倍すれば、3個(2.14%)のBRAMスライス(3 / 140)が一時的なキャッシュとして使用される

### 細かな話2

- Intelデータセット
  - 1404個のスキャンデータ(合計271,517個のスキャン点を含む)を処理している
  - 各スキャンデータは`Sensor::ScanData<double>`型のデータとして表現される
  - `Sensor::ScanData<double>`型は, オドメトリの姿勢(24バイト), 速度ベクトル(24バイト), 相対姿勢(24バイト), 最小距離(8バイト), 最大距離(8バイト), 最小角度(8バイト), 最大角度(8バイト), 各スキャン点の角度(8バイト * スキャン点の個数), 各スキャン点の距離(8バイト * スキャン点の個数)を保持している
  - 1つのスキャンデータにつき, 付加情報(姿勢や最小値, 最大値)が24 + 24 + 24 + 8 + 8 + 8 + 8 = 104バイトで表現されるほか, 1つのスキャン点につき, 距離と角度が8 + 8 = 16バイトで表現される
  - スキャンデータが消費するメモリ量は, 1404 * 104 + 271,517 * 16 = 4,490,288 B = 4.49 MBである
- FR079データセット
  - 889個のスキャンデータ(合計224,119個のスキャン点を含む)を処理している
  - スキャンデータが消費するメモリ量は, 889 * 104 + 224,119 * 16 = 3,678,360 B = 3.67 MBである
- MIT-CSAILデータセット [グラフベースとパーティクルベースで個数が違う]
  - 1,170個のスキャンデータ(合計459,969個のスキャン点を含む)を処理している
  - スキャンデータが消費するメモリ量は, 1,170 * 104 + 459,969 * 16 = 7,481,184 B = 7.48 MBである

### 細かな話3

- FR079データセット
  - グラフベースSLAMとパーティクルフィルタベースSLAMの場合の平均メモリ消費
  - lidar-graph-slam/hw-fr079-8: 66.55 MB - 67.51 MB
  - my-gmapping/hw-fr079-1: 51.50 MB - 52.89 MB
  - 差分: 67.03 - 52.195 = 14.835 MB (28.4%)
- Intelデータセット
  - lidar-graph-slam/hw-intel-0: 82.43 MB - 86.80 MB
  - my-gmapping/hw-intel-0: 67.70 MB - 69.25 MB
  - 差分: 84.615 - 68.475 = 16.14 MB (23.6%)
- MITデータセット
  - lidar-graph-slam/hw-mit-7: 49.36 MB - 52.89 MB
  - my-gmapping/hw-mit-0: 36.28 MB - 37.86 MB
  - 差分: 51.125 - 37.07 = 14.055 MB (37.9%)
- FR079データセット
  - lidar-graph-slam/hw-fr079-8: 66.55 MB - 67.51 MB
  - lidar-graph-slam/sw-fr079-3: 70.11 MB - 70.68 MB
  - 差分: 70.395 - 67.03 = 3.365 MB (5.02%)
- Intelデータセット
  - lidar-graph-slam/hw-intel-0: 82.43 MB - 86.80 MB
  - lidar-graph-slam/sw-intel-0: 92.54 MB - 94.38 MB
  - 差分: 93.46 - 84.615 = 8.845 MB (10.45%)
- MITデータセット
  - lidar-graph-slam/hw-mit-7: 49.36 MB - 52.89 MB
  - lidar-graph-slam/sw-mit-1-2: 51.70 MB - 53.18 MB
  - 差分: 52.44 - 51.125 = 1.315 MB (2.57%)

### 細かな話4

- 創想館データセット: 廊下の端から端まで22.56m / 11.195m
