#pragma once
#include <map>
#include <string>
#include <random>
#include <stdexcept>

#include <utils/flog.h>
#include <utils/net.h>
// #include "http.h"


namespace net::websock {
    class WSClient {
    public:
        // PARTS were taken from https://github.com/katzarsky/WebSocket
        enum WebSocketFrameType {
            ERROR_FRAME = 0xFF00,
            INCOMPLETE_FRAME = 0xFE00,

            OPENING_FRAME = 0x3300,
            CLOSING_FRAME = 0x3400,

            INCOMPLETE_TEXT_FRAME = 0x01,
            INCOMPLETE_BINARY_FRAME = 0x02,

            TEXT_FRAME = 0x81,
            BINARY_FRAME = 0x82,

            PING_FRAME = 0x19,
            PONG_FRAME = 0x1A
        };


        WSClient() : socket(), rd(), e1(rd()), uniform_dist(0, 255) {
            flog::info("WSClient instance created");
        }

        void sendPong() {
            std::string str = " ";
            std::string buffer;
            buffer.resize(200, ' ');
            size_t len = createFrame(PONG_FRAME, (unsigned char*)str.c_str(), 0, (unsigned char*)buffer.data(), buffer.size());
            buffer.resize(len);
            socket->sendstr(buffer);
        }

        void sendString(const std::string& str) {
            std::string buffer;
            buffer.resize(str.length() + 200, ' ');
            size_t len = createFrame(TEXT_FRAME, (unsigned char*)str.c_str(), str.length(), (unsigned char*)buffer.data(), buffer.size());
            buffer.resize(len);
            socket->sendstr(buffer);
        }

        void sendBinary(const std::vector<uint8_t>& data) {
            std::string buffer;
            buffer.resize(data.size() + 200, ' ');
            size_t len = createFrame(BINARY_FRAME, (unsigned char*)data.data(), data.size(), (unsigned char*)buffer.data(), buffer.size());
            buffer.resize(len);
            socket->sendstr(buffer);
        }

        WebSocketFrameType getFrame(const std::vector<uint8_t>& in_buffer, unsigned char* out_buffer, int out_size, int* out_length, int* skipSize) {
            if (in_buffer.size() < 2) return INCOMPLETE_FRAME;

            unsigned char msg_opcode = in_buffer[0] & 0x0F;
            unsigned char msg_fin = (in_buffer[0] >> 7) & 0x01;
            unsigned char msg_masked = (in_buffer[1] >> 7) & 0x01;

            // *** message decoding
            uint64_t payload_length = 0;
            int pos = 2;
            uint8_t length_field = in_buffer[1] & (~0x80);

            if (length_field <= 125) {
                payload_length = (uint64_t)length_field;
            }
            else if (length_field == 126) { // msglen is 16bit!
                payload_length = (((uint64_t)in_buffer[2] << 8) |
                                  ((uint64_t)in_buffer[3]));
                pos += 2;
            }
            else if (length_field == 127) { // msglen is 64bit!
                payload_length = (((uint64_t)in_buffer[2] << 56) |
                                  ((uint64_t)in_buffer[3] << 48) |
                                  ((uint64_t)in_buffer[4] << 40) |
                                  ((uint64_t)in_buffer[5] << 32) |
                                  ((uint64_t)in_buffer[6] << 24) |
                                  ((uint64_t)in_buffer[7] << 16) |
                                  ((uint64_t)in_buffer[8] << 8) |
                                  ((uint64_t)in_buffer[9]));
                pos += 8;
            }

            if (in_buffer.size() < payload_length + pos) {
                return INCOMPLETE_FRAME;
            }

            if (payload_length > out_size) {
                flog::error("ERROR: output buffer is too small for the payload");
                return ERROR_FRAME;
            }

            if (msg_masked) {
                unsigned int mask = 0;
                mask = *((unsigned int*)(in_buffer.data() + pos));
                pos += 4;

                for (int i = 0 ; i < payload_length; i++) {
                    out_buffer[i] = in_buffer[pos + i] ^ ((uint8_t*)(&mask))[i % 4];
                }
            }
            else
            {
                memcpy((void*)out_buffer, (void*)(in_buffer.data() + pos), payload_length);
            }
            out_buffer[payload_length] = 0;


            if (out_length)
                *out_length = payload_length + 1;

            if (skipSize)
                *skipSize = payload_length + pos;

            switch (msg_opcode) {
            case 0x0: // continuation frame
            case 0x1:
                return (msg_fin) ? TEXT_FRAME : INCOMPLETE_TEXT_FRAME;
            case 0x2:
                return (msg_fin) ? BINARY_FRAME : INCOMPLETE_BINARY_FRAME;
            case 0x9:
                return PING_FRAME;
            case 0xA:
                return PONG_FRAME;
            default:
                return ERROR_FRAME;
            }
        }

        std::function<void(const std::string&)> onTextMessage = [](auto) {};
        std::function<void(const std::string&)> onBinaryMessage = [](auto) {};
        std::function<void()> onConnected = []() {};
        std::function<void()> onDisconnected = []() {};
        std::function<void()> onEveryReceive = []() {};

        void connectAndReceiveLoop(
            const std::string& host,
            int port,
            const std::string& path) {

                stopped = false;
                socket = net::connect(Address(host, port));

            if (stopped) {
                if (socket) {
                    socket->close();
                    socket.reset();
                    return;
                }
            }
            flog::info("WSClient socket connected");

            std::string initHeaders =
                "Accept-Encoding: gzip, deflate\r\n"
                "Accept-Language: en-US,en\r\n"
                "Cache-Control: no-cache\r\n"
                "Connection: Upgrade\r\n"
                "Cookie: ident=\r\n"
                "Pragma: no-cache\r\n"
                "Sec-GPC: 1\r\n"
                "Sec-WebSocket-Extensions: permessage-deflate; client_max_window_bits\r\n"
                "Sec-WebSocket-Key: tXZvmn8MbkVRhAoczhuyVQ==\r\n"
                "Sec-WebSocket-Version: 13\r\n"
                "Upgrade: websocket\r\n"
                "User-Agent: Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/108.0.0.0 Safari/537.36\r\n";
            initHeaders = "GET " + path + " HTTP/1.1\r\n" + initHeaders;
            initHeaders += "Host: " + host + ":" + std::to_string(port) + "\r\n";
            initHeaders += "Origin: http://" + host + ":" + std::to_string(port) + "\r\n";
            initHeaders += "\r\n";

            int len = socket->sendstr(initHeaders);
            int senderr = errno;
            flog::info("sent: {} of {}", len, (int64_t)initHeaders.size());

            uint8_t buf[64000];

            int recvd = socket->recv(buf, sizeof(buf), false, NO_TIMEOUT);
            if (recvd <= 0) {
                std::string msg = "websock: recv failed, errno=" + std::to_string(errno) + " (recvd=" + std::to_string(recvd) +
                                  " sent=" + std::to_string(len) + " senderr=" + std::to_string(senderr) + ")";
                socket->close();
                throw std::runtime_error(msg);
            }
            buf[recvd] = 0;

            flog::info("recvd: {}", recvd);
            std::vector<std::string> recvHeaders;
            std::string bufs = (char*)buf;
            auto pos = bufs.find("\r\n\r\n");
            if (pos == std::string::npos) {
                socket->close();
                throw std::runtime_error("websock: invalid response");
            }
            bufs.resize(pos + 2);
            {
                const std::string delim = "\r\n";
                size_t start = 0;
                while (start <= bufs.size()) {
                    size_t p = bufs.find(delim, start);
                    if (p == std::string::npos) {
                        std::string token = bufs.substr(start);
                        if (!token.empty()) recvHeaders.push_back(token);
                        break;
                    }
                    if (p > start) recvHeaders.push_back(bufs.substr(start, p - start));
                    start = p + delim.size();
                }
            }
            for (int i = 0; i < recvHeaders.size(); i++) {
                flog::debug("{}", recvHeaders[i]);
            }
            std::vector<uint8_t> data;
            for (int i = (int)pos + 4; i < recvd; i++) {
                data.push_back(buf[i]);
            }
            onConnected();
            while (true) {
                int len0 = tryDecode(data);
                if (len0 > 0) {
                    //                    printf("decoded/dropping bytes: %d\n", len0);
                    data.erase(data.begin(), data.begin() + len0);
                    continue;
                }
                recvd = socket->recv(buf, sizeof(buf), false, 100); // 100 msec
                if (recvd == 0) {
                    continue;
                }
                //                printf("recvd bytes in loop: %d\n", recvd);
                if (recvd <= 0) {
                    socket->close();
                    onDisconnected();
                    break;
                }
                onEveryReceive();
                for (int i = 0; i < recvd; i++) {
                    data.push_back(buf[i]);
                }
            }

            flog::info("WSClient connectAndReceiveLoop exiting");
        }
        void stopSocket() {
            if (socket) {
                socket->close();
            }
        }

    private:
        std::shared_ptr<::net::Socket> socket;
        std::string path;
        std::random_device rd;
        std::default_random_engine e1;
        std::uniform_int_distribution<int> uniform_dist;
        bool stopped = false;
        int count = 0;


        size_t createFrame(WebSocketFrameType frame_type, unsigned char* msg, size_t input_size, unsigned char* buffer, size_t buffer_size) {
            int pos = 0;
            buffer[pos++] = (unsigned char)frame_type; // fin included

            if (input_size <= 125) {
                buffer[pos++] = (uint8_t)input_size; // set mask bit
            }
            else if (input_size <= 65535) {
                buffer[pos++] = 126; // 16 bit length follows

                buffer[pos++] = (uint8_t)((input_size >> 8) & 0xFF); // leftmost first
                buffer[pos++] = (uint8_t)(input_size & 0xFF);
            }
            else {                   // >2^16-1 (65535)
                buffer[pos++] = 127; // 64 bit length follows

                // write 8 bytes length (significant first)

                // since msg_length is int it can be no longer than 4 bytes = 2^32-1
                // padd zeroes for the first 4 bytes
                for (int i = 3; i >= 0; i--) {
                    buffer[pos++] = 0;
                }
                // write the actual 32bit msg_length in the next 4 bytes
                for (int i = 3; i >= 0; i--) {
                    buffer[pos++] = ((input_size >> 8 * i) & 0xFF);
                }
            }
            if (frame_type != PONG_FRAME) {
                buffer[1] |= 0x80; // set mask bit
                auto maskIndex = pos;
                buffer[pos++] = uniform_dist(e1);
                buffer[pos++] = uniform_dist(e1);
                buffer[pos++] = uniform_dist(e1);
                buffer[pos++] = uniform_dist(e1);
                memcpy(buffer + pos, msg, input_size);
                for (int q = 0; q < input_size; q++) {
                    buffer[pos + q] ^= buffer[maskIndex + (q % 4)];
                }
            }
            return (input_size + pos);
        }

        int tryDecode(const std::vector<uint8_t>& data) {
            std::string buffer;
            if (data.empty()) {
                return 0;
            }
            buffer.resize(data.size() + 200, ' ');
            int outLen = 0;
            int skipSize = 0;
            int frameType = getFrame(data, (unsigned char*)buffer.data(), (int)buffer.length(), &outLen, &skipSize);
            switch (frameType) {
            case TEXT_FRAME:
                if (outLen > 3) {
                    buffer.resize(outLen - 1);
                    onTextMessage(buffer);
                }
                break;
            case BINARY_FRAME:
                if (outLen > 3) {
                    buffer.resize(outLen - 1);
                    onBinaryMessage(buffer);
                }
                break;
            case INCOMPLETE_FRAME:
                return 0;
            case ERROR_FRAME:
                flog::error("ERROR FRAME: {}", frameType);
                break;
            case PING_FRAME:
                sendPong();
                break;
            default:
                flog::error("Unknown frame type: {}", frameType);
                break;
            }
            count++;
            return skipSize;
        }
    };

}