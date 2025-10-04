# ESP32 CANPacket header
## CANPacket class
コンストラクタ及びInitメソッド, パケットID, バッファを定義した基底クラス。多分直接は使わない。
## CANSendPacket class
セッターと送信メソッドを新しく定義したCANPacket classの継承クラス。
これを更に継承してRoboMasControlPacket classにする。(予定)
```cpp
CANSendPacket hoge = CANSendPacket(PacketID);
hoge.BufInit() // バッファのゼロクリア
hoge.SetByte(byteID, data) // バッファのn番目にデータをセット
hoge.SetBuf(data) // uint8_t配列をバッファにセット
hoge.Send() // 送信
```
## CANReceivePacket class
ゲッターと受信メソッドを新しく定義したCANPacket classの継承クラス。
これを更に継承してRoboMasFeedbackPacket classを作ってもらう。
```cpp
CANReceivePacket huga = CANReceivePacket(PacketID);
huga.BufInit()
huga.GetByte(byteID) // バッファのn番目のデータを返す
huga.Receive() // 受信
```