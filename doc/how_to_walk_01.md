## 補助輪台車ありで歩く
### Mac側のデバッグ準備
1. Wired-LANでJetsonと接続する．

1. Macbook側でVSCodeからリポジトリを開く

1. 可視化専用のrviz等を起動（jetson側でjoint_state_publisherが起動しているため）
    ```bash
    # rviz2だけ
    pixi run ros2 launch aoba_description display_rviz_only.launch.py joint_states_topic:=/cmd/joint_states
    
    # rviz2 + PlotJuggler
    pixi run ros2 launch aoba_description display_rviz_only.launch.py joint_states_topic:=/cmd/joint_states plot:=true
    ```

1. terminalからJetsonにSSHする
1. tmuxを起動して，次節の操作を行う．

### Jetson側

1. terminalからCANを有効化
    ```bash
    sudo ip link set can1_rs up type can bitrate 1000000
    sudo ip link set can2_rs up type can bitrate 1000000
    ```
1. 以下のようにCAN1, CAN2がUPになっていればOK
    ```bash
    $ ip link show type can
    2: can0: <NOARP,ECHO> mtu 16 qdisc noop state DOWN mode DEFAULT group default qlen 10
    link/can 
    3: can1_rs: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
    link/can 
    4: can2_rs: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
    link/can 
    ```
    can0はJetson内蔵のmttcanなので使わない．


1. HWノードを起動
    ```bash
    cd ros2_ws
    pixi run ros2 launch aoba_hardware bringup_headless.launch.py joint_states_topic:=/hw/joint_states
    ```

1. 別ターミナルで歩容再生ノードを起動
    ```bash
    # Jetsonで実行するときは`rviz:=fals`とする（可視化はmacbookで机上検討をするとき用）
    cd ros2_ws
    pixi run ros2 launch biped_gait_control trajectory_replay.launch.py mode:=control rviz:=false
    ```