#include "kondo_ics_servo_driver/ics_driver.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <iostream>

// コンストラクタ：シリアルポートを指定してオープン
IcsDriver::IcsDriver(const std::string &port, unsigned int baud)
    : fd_(-1)
{
    openPort(port, baud);
}

// デストラクタ：ポートをクローズ
IcsDriver::~IcsDriver()
{
    closePort();
}

bool IcsDriver::isOpen() const
{
    return (fd_ >= 0);
}

bool IcsDriver::openPort(const std::string &port, unsigned int baud)
{
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0)
    {
        return false;
    }
    // 端末設定
    struct termios options;
    if (tcgetattr(fd_, &options) < 0)
    {
        return false;
    }

    speed_t speed;
    switch (baud)
    {
    case 115200:
        speed = B115200;
        break;
    // 必要に応じて他のbaud rateも追加
    default:
        speed = B115200;
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // 8bit, Even parity, 1 stop bit
    options.c_cflag = CS8 | CLOCAL | CREAD | PARENB;
    options.c_cflag &= ~PARODD;  // Even parity
    options.c_cflag &= ~CSTOPB;  // 1 stop bit
    options.c_cflag &= ~CRTSCTS; // No flow control

    // Input flags
    options.c_iflag = IGNPAR | INPCK;           // Enable parity check
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // No flow control

    // // Output flags
    options.c_oflag &= ~OPOST; // raw output

    // // Local flags
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw input

    // 読み出しタイムアウト設定（VTIME: 100ms単位, VMIN: 最低読み出しバイト数）
    // options.c_cc[VMIN] = 0; // non-blocking
    options.c_cc[VMIN] = 1;  // blocking
    options.c_cc[VTIME] = 1; // 100ms timeout

    // 送信バッファをフラッシュ
    tcflush(fd_, TCOFLUSH);
    // 受信バッファをフラッシュ
    tcflush(fd_, TCIFLUSH);

    if (tcsetattr(fd_, TCSANOW, &options) < 0)
    {
        return false;
    }
    return true;
}

void IcsDriver::closePort()
{
    if (fd_ >= 0)
    {
        close(fd_);
        fd_ = -1;
    }
}

bool IcsDriver::synchronize(const std::vector<uint8_t> &tx, std::vector<uint8_t> &rx, size_t rxLength)
{
    int flags;

    // 送信バッファをフラッシュ
    tcflush(fd_, TCOFLUSH);
    // 受信バッファをフラッシュ
    tcflush(fd_, TCIFLUSH);

    rx.resize(rxLength);
    size_t totalRead = 0;

    // 送信
    ssize_t written = write(fd_, tx.data(), tx.size());
    if (written != static_cast<ssize_t>(tx.size()))
    {
        return false;
    }
    tcdrain(fd_);

    // 指令コマンド分を受信バッファから読み出す（空読み出し）
    size_t total_read_dummy = 0;
    size_t tx_length = tx.size();
    std::vector<uint8_t> dummy;
    dummy.resize(tx.size());
    while (total_read_dummy < tx_length)
    {
        ssize_t n = read(fd_, dummy.data() + total_read_dummy, tx_length - total_read_dummy);
        if (n > 0)
        {
            total_read_dummy += n;
        }
        else if (n < 0)
        {
            // std::cout << "dummy error" << std::endl;
            // return false;
        }
    }
    // std::cout << "tx.size()=" << (int)tx.size() << std::endl;
    // for (int i = 0; i < tx.size(); i++)
    // {
    //     std::cout << "dummy[]=" << (int)dummy[i] << std::endl;
    // }

    // 受信バッファ読み出し
    while (totalRead < rxLength)
    {
        ssize_t n = read(fd_, rx.data() + totalRead, rxLength - totalRead);
        if (n > 0)
        {
            totalRead += n;
        }
        else if (n < 0)
        {
            // std::cout << "read from buffer error" << std::endl;
            return false;
        }
    }
    return (totalRead == rxLength);
}

bool IcsDriver::setPositionCmd(uint8_t id, int pos)
{
    std::vector<uint8_t> tx(3);
    tx[0] = 0x80 + id;
    tx[1] = (pos >> 7) & 0x7F;
    tx[2] = pos & 0x7F;

    std::vector<uint8_t> rx;
    bool res = synchronize(tx, rx, 3);
    if (!res || rx.size() != 3)
    {
        return false;
    }
    // 受信データからpos値を構築
    int rePos = ((rx[1] & 0x7F) << 7) | (rx[2] & 0x7F);
    return (std::abs(rePos - pos) < 50);
}

int IcsDriver::getPositionCmd(uint8_t id)
{
    std::vector<uint8_t> tx(2);
    tx[0] = 0xA0 + id;
    tx[1] = 0x05;
    std::vector<uint8_t> rx;
    bool res = synchronize(tx, rx, 4);
    if (!res || rx.size() != 4)
    {
        return -1;
    }
    int pos = ((rx[2] & 0x7F) << 7) | (rx[3] & 0x7F);
    return pos;
}

int IcsDriver::getIdCmd()
{
    std::vector<uint8_t> tx(4);
    tx[0] = 0xFF;
    tx[1] = 0x00;
    tx[2] = 0x00;
    tx[3] = 0x00;
    std::vector<uint8_t> rx;
    bool res = synchronize(tx, rx, 1);
    if (!res || rx.size() != 1)
    {
        return -1;
    }
    return rx[0] & 0x1F;
}

bool IcsDriver::setIdCmd(uint8_t new_id)
{
    std::vector<uint8_t> tx(4);
    tx[0] = 0xE0 + new_id;
    tx[1] = 0x01;
    tx[2] = 0x01;
    tx[3] = 0x01;
    std::vector<uint8_t> rx;
    bool res = synchronize(tx, rx, 1);
    if (!res || rx.size() != 1)
    {
        return false;
    }
    return ((rx[0] & 0x1F) == new_id);
}
