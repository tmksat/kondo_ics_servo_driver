#ifndef ICS_DRIVER_HPP_
#define ICS_DRIVER_HPP_

#include <cstdint>
#include <string>
#include <vector>

class IcsDriver
{
public:
    /**
     * @brief コンストラクタ．指定したシリアルポートを開く．
     * @param port シリアルポート（例: "/dev/ttyUSB0"）
     * @param baud 通信速度（例: 115200）
     */
    IcsDriver(const std::string &port, unsigned int baud);
    ~IcsDriver();

    bool isOpen() const;

    /**
     * @brief シリアル通信でtxデータを送信し，rxLengthバイト受信する．
     */
    bool synchronize(const std::vector<uint8_t> &tx, std::vector<uint8_t> &rx, size_t rxLength);

    /**
     * @brief サーボ角度指令．ICSプロトコルsetPosに準ずる．
     * @param id サーボID
     * @param pos サーボ位置（整数値：7500がニュートラル）
     * @return trueなら成功，falseなら失敗
     */
    bool setPositionCmd(uint8_t id, int pos);

    /**
     * @brief サーボ位置取得．ICSプロトコルgetPosに準ずる．
     * @param id サーボID
     * @return 取得したサーボ位置（整数値），失敗なら -1
     */
    int getPositionCmd(uint8_t id);

    /**
     * @brief サーボのID読み出し．ICSプロトコルgetIDに準ずる．
     * @return サーボID，失敗なら -1
     */
    int getIdCmd();

    /**
     * @brief サーボのID書き込み．ICSプロトコルsetIDに準ずる．
     * @param new_id 書き込みたいID
     * @return trueなら成功，falseなら失敗
     */
    bool setIdCmd(uint8_t new_id);

private:
    int fd_; // シリアルポートのファイルディスクリプタ
    bool openPort(const std::string &port, unsigned int baud);
    void closePort();
};

#endif // ICS_DRIVER_HPP_
