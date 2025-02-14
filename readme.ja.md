# ROS2 Bag Recorder

ROS2向けのbagレコーダーツールである。

設定ファイル（`config/standard.config`）で記録対象のトピックを指定し、launchファイルのパラメータにより自動または手動で記録を制御することができる。

## 特記

ROS2 jazzyにおいて動作確認済みである。

このリポジトリは、もともとROS1用のC++で実装された[bag_recorder](https://github.com/joshs333/bag_recorder.git)リポジトリをフォークしたものである。

フォークにあたり、ROS2への移行および実装言語をC++からPythonへ変更した。（C++が苦手なためである）

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

注意: data_directoryとconfiguration_directoryのいずれも未設定の場合、ノードは起動しない。

start_nowをtrueに設定すると、起動時に"standard"設定で"auto_record"という名前で自動的に記録を開始する。secondsが0より大きい場合、指定された時間が経過すると自動的に記録を停止する。ただし、自動停止が設定されていても、手動で停止することも可能である。

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

全トピックを購読する場合は、以下のいずれかの方法を使用することができる

1. 設定ファイルに以下の1行を記述する：
    ```
    *
    ```

2. 設定ファイルを空にする（コメントのみの場合も空とみなされる）

    ```bash
    # コメントのみなので、空とみなされる
    ```

3. 設定ファイルを作成しない

    この場合、警告ログが出力される

### 設定ファイルのリンク
他の設定ファイルの内容を取り込むことができる。例えば、以下のような`standard.config`があるとする：
```
# `standard.config`内の設定
/topica
/topicb
/topicc
```

新しい設定ファイル（例：`special.config`）で標準設定に加えて追加のトピックを記録したい場合、以下のように記述することができる：
```
# これは特別設定である
/extra/topic
# 標準設定を取り込む
$standard
```

これにより、`special.config`では`standard.config`のトピックに加えて、`/extra/topic`も記録対象となる。

なお、設定ファイルの循環参照（例：AがBを参照し、BがAを参照する）は自動的に検出され、警告ログが出力される。

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
ros2 topic pub /record/start bag2_recorder/msg/Rosbag "{config: 'standard', bag_name: 'test_bag'}"
```

`--once`オプションを使用するとトピックの送出が1回だけ行われるが、うまくいかない場合がある。`--times 5`などで複数回に分けて送信することをお勧めする。

```bash
ros2 topic pub /record/start bag2_recorder/msg/Rosbag "{config: 'standard', bag_name: 'test_bag'}" --times 5
```

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
        rclcpp::spin_some(node);
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

コマンド例：
```bash
ros2 topic pub /record/stop std_msgs/msg/String "data: standard"
```

`--once`オプションを使用するとトピックの送出が1回だけ行われるが、うまくいかない場合がある。`--times 5`などで複数回に分けて送信することをお勧めする。

```bash
ros2 topic pub /record/stop std_msgs/msg/String "data: standard" --times 5
```

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
        rclcpp::spin_some(node);
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

# TODO
- [ ] テストの追加
- [ ] defalut.launch.pyの引数から設定ファイルを指定できるようにする。（ただし`/record/start`トピックの`bag_name`との兼ね合いも考慮する必要あり）
- [ ] Ctrl + Cでノードを終了した際に、記録中のバッグファイルを自動で閉じるようにする
- [ ] `ros2 topic echo /record/bag_name`でバッグ名が表示されない問題を解決する