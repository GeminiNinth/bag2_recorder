# ROS2 Bag Recorder

ROS2向けのbagレコーダーツールである。

設定ファイル（`config/standard.config`）で記録対象のトピックを指定し、launchファイルのパラメータにより自動または手動で記録を制御することができる。

## 特記

ROS2 jazzyにおいて動作確認済みである。

このリポジトリは、もともとROS1用のC++で実装された[bag_recorder](https://github.com/joshs333/bag_recorder.git)リポジトリをフォークしたものである。

フォークにあたり、ROS2への移行および実装言語をC++からPythonへ変更した（C++が苦手なためである）。

# インストール方法と実行方法
ROS2ワークスペースのsrc/ディレクトリに本リポジトリをクローンする。その後、以下のコマンドを実行する。
```bash
colcon build
source install/setup.bash
```
ビルド完了後、ros2 launchコマンドでノードを起動できる。
```bash
ros2 launch bag2_recorder default.launch.py
```

# launchファイルの設定
本パッケージを使用するには、プロジェクトに応じてlaunchファイル（`launch/default.launch.py`）を設定する必要がある。設定可能なパラメータは以下の通りである：

- `data_directory`: バッグファイルの保存先ディレクトリ(デフォルト: ~/data/)
- `configuration_directory`: 設定ファイルの格納ディレクトリ(デフォルト: パッケージのconfigディレクトリ)
- `storage_format`: rosbagのストレージフォーマット(選択肢: ["mcap", "db3"]、デフォルト: "mcap")
- `seconds`: 記録時間（秒）(デフォルト: 10、0の場合は手動で停止するまで記録)
- `start_now`: 起動時に自動で記録を開始するかどうか(デフォルト: true)
- `config_files`: 使用する設定ファイルのリスト(デフォルト: ["standard"])
- `show_topics`: 記録対象のトピック一覧を表示するかどうか(デフォルト: false)

注意: data_directoryとconfiguration_directoryのいずれも未設定の場合、ノードは起動しない。

start_nowをtrueに設定すると、起動時に指定された設定ファイルを使用して"auto_record"という名前で自動的に記録を開始する。secondsが0より大きい場合、指定された時間が経過すると自動的に記録を停止する。ただし、自動停止が設定されていても、手動で停止することも可能である。

### 標準出力例と解説
以下に、様々なケースにおける標準出力例を示す。

#### 1. 正常な記録と自動停止の例
```
[INFO] [launch]: All log files can be found below /home/gemini-ninth/.ros/log/2025-02-13-21-35-07-228364-gemini-ninth-01-187395
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [bag2_recorder_py_node-1]: process started with pid [187399]
```
↑ 起動時の基本情報
- ログファイルの保存場所
- ログレベルの設定
- プロセスIDの情報

```
[bag2_recorder_py_node-1] [INFO] [1739450108.324657797] [rosbag2_recorder_node]: Found 1 topics in 'standard.config'
[bag2_recorder_py_node-1] [INFO] [1739450108.324973709] [rosbag2_recorder_node]: Found 3 topics in 'special.config'
[bag2_recorder_py_node-1] [INFO] [1739450108.325205883] [rosbag2_recorder_node]: 
[bag2_recorder_py_node-1] Combined Topic Statistics:
[bag2_recorder_py_node-1] Total topics specified across all files: 4
[bag2_recorder_py_node-1] Unique topics: 4
[bag2_recorder_py_node-1] Duplicate topics: 0
[bag2_recorder_py_node-1] Topics to be recorded: 4
```
↑ 設定ファイルの解析結果
- 各設定ファイルで指定されたトピック数
- トピックの重複チェック結果
- 実際に記録される総トピック数

```
[bag2_recorder_py_node-1] [INFO] [1739450109.375877486] [rosbag2_recorder_node]: Started recording with combined topics from configs: standard, special
[bag2_recorder_py_node-1] [INFO] [1739450109.376203673] [rosbag2_recorder_node]: Auto-stop timer set for 30 seconds for config 'standard+special'
```
↑ 記録開始の通知
- 使用される設定ファイルの組み合わせ
- 自動停止タイマーの設定

```
[bag2_recorder_py_node-1] [INFO] [1739450139.497939370] [rosbag2_recorder_node]: Stopped recording for config 'standard+special' due to timeout (Recording duration: 30.0 seconds)
[bag2_recorder_py_node-1] [INFO] [1739450139.498490484] [rosbag2_recorder_node]: Recording finished, requesting shutdown...
[INFO] [bag2_recorder_py_node-1]: process has finished cleanly [pid 187399]
```
↑ 記録終了の通知
- 停止理由（この場合はタイムアウト）
- 実際の記録時間
- プロセスの正常終了

#### 2. 手動停止の例
```
[bag2_recorder_py_node-1] [INFO] [1739450109.375877486] [rosbag2_recorder_node]: Started recording with combined topics from configs: standard, special
[bag2_recorder_py_node-1] [INFO] [1739450112.497939370] [rosbag2_recorder_node]: Stopped recording for config 'standard+special' due to stop topic received (Recording duration: 3.1 seconds)
[bag2_recorder_py_node-1] [INFO] [1739450112.498490484] [rosbag2_recorder_node]: Recording finished, requesting shutdown...
```
↑ 停止トピックを受信して記録を終了した場合の出力

#### 3. 設定ファイルエラーの例
```
[ERROR] [bag2_recorder_py_node-1]: Configuration file 'nonexistent.config' not found in directory '/home/gemini-ninth/ros2_ws/src/bag2_recorder/config'
[ERROR] [bag2_recorder_py_node-1]: Failed to load configuration files
[bag2_recorder_py_node-1]: process has finished with exit code 1
```
↑ 存在しない設定ファイルを指定した場合のエラー

```
[WARNING] [bag2_recorder_py_node-1]: Duplicate topic '/camera/image_raw' found in configs: standard.config, special.config
[WARNING] [bag2_recorder_py_node-1]: Topic will only be recorded once
```
↑ 複数の設定ファイルで同じトピックが指定された場合の警告

#### 4. その他のエラー例
```
[ERROR] [bag2_recorder_py_node-1]: Failed to create data directory: Permission denied
[ERROR] [bag2_recorder_py_node-1]: Unable to initialize recorder
[bag2_recorder_py_node-1]: process has finished with exit code 1
```
↑ データディレクトリの作成権限がない場合のエラー

```
[ERROR] [bag2_recorder_py_node-1]: Failed to open bag file: Disk space full
[ERROR] [bag2_recorder_py_node-1]: Recording failed
[bag2_recorder_py_node-1]: process has finished with exit code 1
```
↑ ディスク容量不足によるエラー

## ストレージフォーマット
ROS2 Jazzyでは、rosbagファイルに対して2種類のストレージフォーマットをサポートしている：

- `mcap`: デフォルトの推奨フォーマット。ロボティクスデータに最適化された新しいシリアライゼーションフォーマット
- `db3`: 旧バージョンのROS2で使用されていたSQLiteベースのレガシーフォーマット

launchファイルでフォーマットを指定する例：
```python
ros2 launch bag2_recorder default.launch.py storage_format:=mcap  # または storage_format:=db3
```

# 設定ファイルの使用方法
## 基本設定
設定ファイルは拡張子"`.config`"で識別される。例えば、"standard"という設定を作成する場合、設定ディレクトリに`standard.config`というファイルを作成し、その中にrosbag2_recorderが購読するトピックを列挙する。

`standard.config`の例：
```
/topic1
/topic2/subtopic1
/topic2/subtopic2
/topic3
/topic4
```

### 全トピックの購読
全トピックを購読する場合は、以下のいずれかの方法を使用することができる：

1. 設定ファイルに以下の1行を記述する：
    ```bash
    *
    ```

2. 設定ファイルを空にする（コメントのみの場合も空とみなされる）：
    ```bash
    # コメントのみなので、空とみなされる
    ```

3. 設定ファイルを作成しない
    
    注意: この場合、警告ログが出力される

### 複数の設定ファイルの使用
launchファイルの`config_files`パラメータで複数の設定ファイルを指定することができる：
```python
config_files:=['standard', 'special']
```

複数の設定ファイルを指定した場合：
1. 全ての指定されたファイルからトピックが結合される
2. 重複するトピックは自動的に除外される
3. 結合された設定は'+'区切りで名前が付けられる（例："standard+special"）
4. 合計トピック数、ユニークトピック数、重複トピック数などの統計情報が表示される
5. 記録停止時：
   - `"data: standard+special"`で全ての記録を停止
   - `"data: standard"`でstandard.configのトピックのみを停止
   - `"data: special"`でspecial.configのトピックのみを停止

設定ファイルの例：
```bash
# standard.config
/topica
/topicb
/topicc

# special.config
/extra/topic1
/extra/topic2
/extra/topic3
```

### コメントの記述
設定ファイルでは、行頭に'#'または空白文字を使用してコメントを記述できる。パーサーは空行や' '、'#'で始まる行を無視する。
```
# これはコメント
 これもコメントとして扱われる
/This/will/be/subscribed/to
```

# レコーダーの制御
## 記録開始
開始トピックはlaunchファイルで設定する：
```python
start_bag_topic:=/record/start
```
このトピックにrosbagメッセージを送信することで記録を開始できる。メッセージには以下の2つの値を指定する：
- config: 使用する設定ファイルの名前(拡張子なし)
- bag_name: 記録するバッグファイルのベース名(タイムスタンプは自動付与)

コマンド例：
```bash
ros2 topic pub /record/start bag2_recorder/msg/Rosbag "{config: 'standard', bag_name: 'test_bag'}" --times 5
```

`--once`オプションを使用するとトピックの送出が1回だけ行われるが、うまくいかない場合がある。`--times 5`などで複数回に分けて送信することをお勧めする。

完全なバッグファイル名を確認するには、以下のlaunchオプションを使用する：
```python
publish_name:=true
name_topic:=/record/bag_name
```
これにより、完全なバッグファイル名がname_topicに送信される。

確認方法：
```bash
ros2 topic echo /record/bag_name
```

### プログラムからの記録開始(C++)
本パッケージはROSトピックを介したプログラム制御が可能である。以下にC++での実装例を示す：

```cpp
#include <rclcpp/rclcpp.hpp>
#include "bag2_recorder/msg/rosbag.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_node");
    
    auto bag_pub = node->create_publisher<bag2_recorder::msg::Rosbag>("/record/start", 10);

    auto message = bag2_recorder::msg::Rosbag();
    message.config = "standard";
    message.bag_name = "test_bag";

    while(rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), "Publishing...");
        bag_pub->publish(message);
        rclpy::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
```

### プログラムからの記録開始(Python)
以下にPythonでの実装例を示す：

```python
import rclpy
from rclpy.node import Node
from bag2_recorder.msg import Rosbag

def main():
    rclpy.init()
    node = Node("test_node")
    
    bag_pub = node.create_publisher(Rosbag, "/record/start", 10)
    
    message = Rosbag()
    message.config = "standard"
    message.bag_name = "test_bag"
    
    try:
        while rclpy.ok():
            node.get_logger().info("Publishing...")
            bag_pub.publish(message)
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 記録停止
停止トピックもlaunchファイルで定義する：
```python
stop_bag_topic:=/record/stop
```
特定の設定による記録を停止するには、その設定名をstop_bag_topicにパブリッシュする。このトピックはstd_msgs::msg::String型を使用する。

複数の設定ファイルを使用している場合：
- `"data: standard+special"`で全ての記録を停止
- `"data: standard"`でstandard.configのトピックのみを停止
- `"data: special"`でspecial.configのトピックのみを停止

コマンド例：
```bash
ros2 topic pub /record/stop std_msgs/msg/String "data: standard" --times 5
```

`--once`オプションを使用するとトピックの送出が1回だけ行われるが、うまくいかない場合がある。`--times 5`などで複数回に分けて送信することをお勧めする。

### プログラムからの記録停止(C++)
以下にC++での停止処理の実装例を示す：

```cpp
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_node");
    
    auto bag_pub = node->create_publisher<std_msgs::msg::String>("/record/stop", 10);

    auto message = std_msgs::msg::String();
    message.data = "standard";

    while(rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), "Publishing...");
        bag_pub->publish(message);
        rclpy::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
```

### プログラムからの記録停止(Python)
以下にPythonでの停止処理の実装例を示す：

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main():
    rclpy.init()
    node = Node("test_node")
    
    bag_pub = node.create_publisher(String, "/record/stop", 10)
    
    message = String()
    message.data = "standard"
    
    try:
        while rclpy.ok():
            node.get_logger().info("Publishing...")
            bag_pub.publish(message)
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

# トラブルシューティング

## よくあるエラーと解決方法

### 1. 設定ファイル関連
- **エラー**: `Configuration file 'xxx.config' not found`
  - **原因**: 指定した設定ファイルが見つからない
  - **解決**: 設定ファイルの存在と場所を確認する。デフォルトではパッケージのconfigディレクトリに配置する

- **警告**: `Duplicate topic found in configs`
  - **説明**: 複数の設定ファイルで同じトピックが指定されている
  - **影響**: 警告のみ。トピックは1回だけ記録される

### 2. ディレクトリ関連
- **エラー**: `Failed to create data directory: Permission denied`
  - **原因**: データディレクトリの作成権限がない
  - **解決**: ディレクトリのパーミッションを確認し、必要に応じて権限を付与する

- **エラー**: `Failed to open bag file: Disk space full`
  - **原因**: ディスク容量が不足している
  - **解決**: 不要なファイルを削除するか、別のディスクを指定する

### 3. 記録制御関連
- **問題**: 記録が開始されない
  - **確認点**:
    1. トピックの型が正しいか（/record/start: bag2_recorder/msg/Rosbag）
    2. メッセージの内容が正しいか（config, bag_nameフィールド）
    3. `--times 5`オプションを使用しているか

- **問題**: 記録が停止されない
  - **確認点**:
    1. トピックの型が正しいか（/record/stop: std_msgs/msg/String）
    2. 設定名が正しいか（複数設定の場合は'+'で結合）
    3. `--times 5`オプションを使用しているか

# TODO
- [ ] テストの追加
- [ ] defalut.launch.pyの引数から設定ファイルを指定できるようにする（ただし`/record/start`トピックの`bag_name`との兼ね合いも考慮する必要あり）
- [ ] Ctrl + Cでノードを終了した際に、記録中のバッグファイルを自動で閉じるようにする
- [ ] `ros2 topic echo /record/bag_name`でバッグ名が表示されない問題を解決する
- [ ] 複数設定ファイル使用時の機能改善：
  - [ ] 結合記録使用時に個別の設定を停止できるようにする
  - [ ] "all"キーワードによる全記録停止機能の追加
  - [ ] 設定の組み合わせ処理の改善
