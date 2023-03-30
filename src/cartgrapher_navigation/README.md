# cartographer_navigation

# cartographer navigation
以下のlaunchファイルを実行することでcartographerで地図生成しながらnavigationを行える

```bash
roslaunch cartgrapher_navigation cartgrapher_navigation.launch
```

# オンラインでの地図生成
ロボットを走行させながら同時に地図生成を行うためには以下のlaunchファイルを実行

```bash
roslaunch cartgrapher_navigation cartographer.launch
```

# オフラインでの地図生成
rosbagファイルを再生しながら地図生成を行うためには以下のlaunchファイルを実行

```bash
roslaunch cartgrapher_navigation cartographer_rosbag.launch
```
