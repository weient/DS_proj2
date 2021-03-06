## 資料結構 project2
### 掃地機器人
概要：設計一個演算法能計算出掃地機器人的行走路徑。

首先輸入地面總面積m*n，和其最大電量（最多能走幾步）。接著輸入地面分佈圖，R代表掃地機器人的初始位置，0代表機器人可以走，1代表障礙物不能走。
```
7 16 30
1 1 1 1	1 1 1 1 1 1 1 1 1 1 1 1
1 0 0 0	1 0 0 1	0 0 0 1 0 0 0 1
1 0 0 0 1 0 0 1 1 0 0 0 0 0 0 1
1 0 0 0	1 0 0 0	0 0 1 0 1 0 0 1
1 0 1 1 1 1 1 1	0 0 1 0 0 0 0 1
1 0 0 0	0 0 R 0	0 0 1 0 0 0 0 1
1 1 1 1	1 1 1 1	1 1 1 1 1 1 1 1
```

程式會依據給予的data計算出走完所有可行走的地板（包括適時回去R充電）的步數及詳細路徑。
```
198
5 6
5 7
5 8
4 8
3 8
3 9
4 9
5 9
4 9
3 9
2 9
1 9
1 10
2 10
2 11
3 11
4 11
5 11
5 12
4 12
4 11
3 11
2 11
2 10
....
```
詳細sample output請參考附件final.path
