#include <gpiod.hpp>

#include <array>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <unistd.h>
#include <vector>

#include <netinet/in.h>

#include <sys/select.h>
#include <sys/socket.h>
#include <termios.h>

#include "station_protocol.h"

// ===== GPIO / UART =====
static constexpr const char* UART_DEVICE = "/dev/serial0";
static constexpr unsigned int HC12_SET_GPIO = 21;   // BCM GPIO21
static constexpr int HC12_UART_BAUD = 9600;
static constexpr uint8_t HC12_CHANNEL = 5;
static constexpr uint16_t HTTP_SERVER_PORT = 8080;

// Если нужен внешний LED на GPIO, поставь номер пина вместо std::nullopt
static const std::optional<unsigned int> STATUS_LED_GPIO = std::nullopt;

// ===== Timings =====
static constexpr int AT_ENTER_DELAY_MS = 50;
static constexpr int UART_RETURN_DELAY_MS = 80;
static constexpr int AT_RESPONSE_TIMEOUT_MS = 300;
static constexpr int FIRST_RESPONSE_TIMEOUT_MS = 1000;
static constexpr int SECOND_RESPONSE_TIMEOUT_MS = 30000;
static constexpr int HTTP_CLIENT_READ_TIMEOUT_MS = 500;

// ===== Supported HC-12 AT baud rates =====
static constexpr std::array<int, 8> SUPPORTED_BAUDS = {
    9600, 19200, 4800, 2400, 1200, 38400, 57600, 115200
};

static constexpr std::array<std::string_view, 11> REPEATED_RESPONSE_BASE_IDS = {
    "6", "10", "11", "12", "16", "17", "20", "21", "22", "23", "24"
};

static uint8_t normalizeChannel(uint8_t channel) {
    if (channel < 1) return 1;
    if (channel > 127) return 127;
    return channel;
}

static void sleepMs(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

static const char* httpStatusText(int statusCode) {
    switch (statusCode) {
        case 200: return "OK";
        case 204: return "No Content";
        case 400: return "Bad Request";
        case 404: return "Not Found";
        case 409: return "Conflict";
        case 503: return "Service Unavailable";
        case 500: return "Internal Server Error";
        case 502: return "Bad Gateway";
        case 504: return "Gateway Timeout";
        default:  return "OK";
    }
}

static void writeAllToSocket(int fd, std::string_view data) {
    const char* cursor = data.data();
    size_t left = data.size();

    while (left > 0) {
        const ssize_t written = ::send(fd, cursor, left, MSG_NOSIGNAL);
        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            throw std::runtime_error("Socket write failed");
        }

        cursor += written;
        left -= static_cast<size_t>(written);
    }
}

static void sendHttpResponse(int clientFd,
                             int statusCode,
                             std::string_view contentType,
                             std::string_view body) {
    std::string headers;
    headers.reserve(256);
    headers += "HTTP/1.1 ";
    headers += std::to_string(statusCode);
    headers += " ";
    headers += httpStatusText(statusCode);
    headers += "\r\n";
    headers += "Content-Type: ";
    headers += contentType;
    headers += "\r\n";
    headers += "Access-Control-Allow-Origin: *\r\n";
    headers += "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n";
    headers += "Access-Control-Allow-Headers: Content-Type\r\n";
    headers += "Cache-Control: no-store\r\n";
    headers += "Connection: close\r\n";
    headers += "Content-Length: ";
    headers += std::to_string(body.size());
    headers += "\r\n\r\n";

    writeAllToSocket(clientFd, headers);
    if (!body.empty()) {
        writeAllToSocket(clientFd, body);
    }
}

static std::string escapeJsonString(std::string_view input) {
    std::string escaped;
    escaped.reserve(input.size() + 16);

    for (char ch : input) {
        switch (ch) {
            case '\\': escaped += "\\\\"; break;
            case '"':  escaped += "\\\""; break;
            case '\n': escaped += "\\n"; break;
            case '\r': escaped += "\\r"; break;
            case '\t': escaped += "\\t"; break;
            default:
                if (static_cast<unsigned char>(ch) < 0x20) {
                    char buffer[7];
                    std::snprintf(buffer,
                                  sizeof(buffer),
                                  "\\u%04x",
                                  static_cast<unsigned char>(ch));
                    escaped += buffer;
                } else {
                    escaped.push_back(ch);
                }
                break;
        }
    }

    return escaped;
}

static void sendJsonResult(int clientFd,
                           int statusCode,
                           bool ok,
                           std::string_view response,
                           std::string_view error,
                           const std::vector<std::string>* responses = nullptr,
                           const std::optional<bool>& awaitingSecondResponse = std::nullopt,
                           const control_station::master::SystemSummary* systemSummary = nullptr,
                           const std::vector<int>* boomSlots = nullptr) {
    std::string body;
    body.reserve(160 + response.size() + error.size());
    body += "{\"ok\":";
    body += ok ? "true" : "false";

    if (!response.empty()) {
        body += ",\"response\":\"";
        body += escapeJsonString(response);
        body += "\"";
    }

    if (responses != nullptr && !responses->empty()) {
        body += ",\"responses\":[";

        for (size_t i = 0; i < responses->size(); ++i) {
            if (i > 0) {
                body += ",";
            }

            body += "\"";
            body += escapeJsonString((*responses)[i]);
            body += "\"";
        }

        body += "]";
    }

    if (awaitingSecondResponse.has_value()) {
        body += ",\"awaitingSecondResponse\":";
        body += *awaitingSecondResponse ? "true" : "false";
    }

    if (systemSummary != nullptr) {
        body += ",\"systemSummary\":{";
        body += "\"robot\":";
        body += std::to_string(systemSummary->robot);
        body += ",\"pre\":";
        body += std::to_string(systemSummary->pre);
        body += ",\"crsf\":";
        body += std::to_string(systemSummary->crsf);
        body += ",\"batt\":";
        body += std::to_string(systemSummary->batt);
        body += ",\"voltage_cV\":";
        body += std::to_string(systemSummary->voltage_cV);
        body += "}";
    }

    if (boomSlots != nullptr && !boomSlots->empty()) {
        body += ",\"boomSlots\":[";
        for (size_t i = 0; i < boomSlots->size(); ++i) {
            if (i > 0) {
                body += ",";
            }
            body += std::to_string((*boomSlots)[i]);
        }
        body += "]";
    }

    if (!error.empty()) {
        body += ",\"error\":\"";
        body += escapeJsonString(error);
        body += "\"";
    }

    body += "}";
    sendHttpResponse(clientFd, statusCode, "application/json; charset=utf-8", body);
}

static int createHttpServerSocket(uint16_t port) {
    const int serverFd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (serverFd < 0) {
        throw std::runtime_error("socket() failed");
    }

    const int reuseAddress = 1;
    if (::setsockopt(serverFd,
                     SOL_SOCKET,
                     SO_REUSEADDR,
                     &reuseAddress,
                     sizeof(reuseAddress)) < 0) {
        ::close(serverFd);
        throw std::runtime_error("setsockopt(SO_REUSEADDR) failed");
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = htonl(INADDR_ANY);

    if (::bind(serverFd,
               reinterpret_cast<sockaddr*>(&address),
               sizeof(address)) < 0) {
        ::close(serverFd);
        throw std::runtime_error("bind() failed on HTTP server socket");
    }

    if (::listen(serverFd, 8) < 0) {
        ::close(serverFd);
        throw std::runtime_error("listen() failed on HTTP server socket");
    }

    return serverFd;
}

static bool waitForClientConnection(int serverFd, int timeoutMs, int& clientFdOut) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(serverFd, &rfds);

    timeval tv{};
    tv.tv_sec = timeoutMs / 1000;
    tv.tv_usec = (timeoutMs % 1000) * 1000;

    const int ret = select(serverFd + 1, &rfds, nullptr, nullptr, &tv);
    if (ret < 0) {
        throw std::runtime_error("select() failed on HTTP server socket");
    }
    if (ret == 0) {
        return false;
    }

    clientFdOut = ::accept(serverFd, nullptr, nullptr);
    if (clientFdOut < 0) {
        if (errno == EINTR) {
            return false;
        }
        throw std::runtime_error("accept() failed on HTTP server socket");
    }

    return true;
}

static bool readHttpRequest(int clientFd,
                            std::string& methodOut,
                            std::string& pathOut) {
    std::string request;
    auto startedAt = std::chrono::steady_clock::now();

    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - startedAt)
               .count() < HTTP_CLIENT_READ_TIMEOUT_MS) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(clientFd, &rfds);

        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 50 * 1000;

        const int ret = select(clientFd + 1, &rfds, nullptr, nullptr, &tv);
        if (ret < 0) {
            throw std::runtime_error("select() failed on HTTP client socket");
        }
        if (ret == 0) {
            continue;
        }

        char buffer[1024];
        const ssize_t bytesRead = ::recv(clientFd, buffer, sizeof(buffer), 0);
        if (bytesRead < 0) {
            if (errno == EINTR) {
                continue;
            }
            throw std::runtime_error("HTTP client read failed");
        }
        if (bytesRead == 0) {
            break;
        }

        request.append(buffer, static_cast<size_t>(bytesRead));

        if (request.find("\r\n\r\n") != std::string::npos ||
            request.find("\n\n") != std::string::npos ||
            request.size() >= 4096) {
            break;
        }
    }

    if (request.empty()) {
        return false;
    }

    size_t lineEnd = request.find("\r\n");
    if (lineEnd == std::string::npos) {
        lineEnd = request.find('\n');
    }
    if (lineEnd == std::string::npos) {
        return false;
    }

    const std::string requestLine = request.substr(0, lineEnd);
    const size_t firstSpace = requestLine.find(' ');
    const size_t secondSpace = requestLine.find(' ', firstSpace + 1);

    if (firstSpace == std::string::npos || secondSpace == std::string::npos) {
        return false;
    }

    methodOut = requestLine.substr(0, firstSpace);
    pathOut = requestLine.substr(firstSpace + 1, secondSpace - firstSpace - 1);

    const size_t queryPos = pathOut.find('?');
    if (queryPos != std::string::npos) {
        pathOut.resize(queryPos);
    }

    return true;
}

static speed_t baudToTermios(int baud) {
    switch (baud) {
        case 1200:   return B1200;
        case 2400:   return B2400;
        case 4800:   return B4800;
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        default:
            throw std::runtime_error("Unsupported baud rate");
    }
}

class SerialPort {
public:
    SerialPort(const std::string& device, int baud) : device_(device) {
        openPort(baud);
    }

    ~SerialPort() {
        closePort();
    }

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    void reopen(int baud) {
        closePort();
        openPort(baud);
    }

    void flushRx() {
        if (fd_ >= 0) {
            tcflush(fd_, TCIFLUSH);
        }
    }

    void writeString(std::string_view s) {
        const char* data = s.data();
        size_t left = s.size();

        while (left > 0) {
            const ssize_t n = ::write(fd_, data, left);
            if (n < 0) {
                throw std::runtime_error("UART write failed");
            }
            data += n;
            left -= static_cast<size_t>(n);
        }

        tcdrain(fd_);
    }

    std::string readQuietLine(int timeoutMs) {
        std::string out;
        auto start = std::chrono::steady_clock::now();
        auto lastByte = start;

        while (std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - start)
                   .count() < timeoutMs) {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(fd_, &rfds);

            timeval tv{};
            tv.tv_sec = 0;
            tv.tv_usec = 20 * 1000;

            const int ret = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
            if (ret < 0) {
                throw std::runtime_error("select() failed on UART");
            }

            if (ret > 0 && FD_ISSET(fd_, &rfds)) {
                char buf[64];
                const ssize_t n = ::read(fd_, buf, sizeof(buf));
                if (n < 0) {
                    throw std::runtime_error("UART read failed");
                }

                for (ssize_t i = 0; i < n; ++i) {
                    const char c = buf[i];
                    if (c == '\r' || c == '\n') {
                        continue;
                    }
                    out.push_back(c);
                }

                lastByte = std::chrono::steady_clock::now();
            }

            if (!out.empty()) {
                const auto quietMs =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - lastByte)
                        .count();

                if (quietMs > 40) {
                    break;
                }
            }
        }

        return out;
    }

    bool readLine(std::string& lineOut, int timeoutMs = 20) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_, &rfds);

        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = timeoutMs * 1000;

        const int ret = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
        if (ret < 0) {
            throw std::runtime_error("select() failed on UART");
        }
        if (ret == 0) {
            return false;
        }

        char buf[64];
        const ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n < 0) {
            throw std::runtime_error("UART read failed");
        }

        for (ssize_t i = 0; i < n; ++i) {
            const char c = buf[i];

            if (c == '\r') {
                continue;
            }

            if (c == '\n') {
                if (!rxBuffer_.empty()) {
                    lineOut = rxBuffer_;
                    rxBuffer_.clear();
                    return true;
                }
                continue;
            }

            if (rxBuffer_.size() < 47) {
                rxBuffer_.push_back(c);
            } else {
                rxBuffer_.clear();
            }
        }

        return false;
    }

private:
    void openPort(int baud) {
        fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ < 0) {
            throw std::runtime_error("Failed to open UART device: " + device_);
        }

        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            closePort();
            throw std::runtime_error("tcgetattr failed");
        }

        cfmakeraw(&tty);

        const speed_t speed = baudToTermios(baud);
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            closePort();
            throw std::runtime_error("tcsetattr failed");
        }

        tcflush(fd_, TCIOFLUSH);
    }

    void closePort() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    int fd_ = -1;
    std::string device_;
    std::string rxBuffer_;
};

class GpioOutputs {
public:
    GpioOutputs(unsigned int setGpio, std::optional<unsigned int> ledGpio)
        : chip_("/dev/gpiochip0"), setGpio_(setGpio), ledGpio_(ledGpio) {
        gpiod::line_settings settings;
        settings.set_direction(gpiod::line::direction::OUTPUT);
        settings.set_output_value(gpiod::line::value::INACTIVE);

        auto builder = chip_.prepare_request();
        builder.set_consumer("hc12-master-rpi");

        builder.add_line_settings(setGpio_, settings);

        if (ledGpio_) {
            builder.add_line_settings(*ledGpio_, settings);
        }

        request_.emplace(builder.do_request());

        setSetPin(true);
        if (ledGpio_) {
            setLed(false);
        }
    }

    void setSetPin(bool high) {
        request_->set_value(
            setGpio_,
            high ? gpiod::line::value::ACTIVE
                 : gpiod::line::value::INACTIVE
        );
    }

    void setLed(bool on) {
        if (!ledGpio_) {
            return;
        }

        request_->set_value(
            *ledGpio_,
            on ? gpiod::line::value::ACTIVE
               : gpiod::line::value::INACTIVE
        );
    }

    void toggleLed() {
        if (!ledGpio_) {
            return;
        }

        ledState_ = !ledState_;
        setLed(ledState_);
    }

private:
    gpiod::chip chip_;
    std::optional<gpiod::line_request> request_;
    unsigned int setGpio_;
    std::optional<unsigned int> ledGpio_;
    bool ledState_ = false;
};

static void enterATMode(GpioOutputs& gpio) {
    gpio.setSetPin(false);
    sleepMs(AT_ENTER_DELAY_MS);
}

static void exitATMode(GpioOutputs& gpio) {
    gpio.setSetPin(true);
    sleepMs(UART_RETURN_DELAY_MS);
}

static bool sendATCommand(SerialPort& serial,
                          std::string_view command,
                          std::string& response) {
    serial.flushRx();
    serial.writeString(command);
    response = serial.readQuietLine(AT_RESPONSE_TIMEOUT_MS);
    return !response.empty();
}

static int detectCurrentBaudInATMode(SerialPort& serial) {
    std::string response;

    for (int baud : SUPPORTED_BAUDS) {
        serial.reopen(baud);
        sleepMs(20);

        if (sendATCommand(serial, "AT", response) &&
            response.rfind("OK", 0) == 0) {
            return baud;
        }
    }

    return 0;
}

static bool applyRadioSettings(SerialPort& serial,
                               GpioOutputs& gpio,
                               uint8_t channel,
                               int uartBaud) {
    enterATMode(gpio);

    const int currentBaud = detectCurrentBaudInATMode(serial);
    if (currentBaud == 0) {
        exitATMode(gpio);
        return false;
    }

    serial.reopen(currentBaud);
    sleepMs(20);

    std::string response;
    char command[32];

    std::snprintf(command, sizeof(command), "AT+C%03u", normalizeChannel(channel));
    if (!sendATCommand(serial, command, response) ||
        response.rfind("OK", 0) != 0) {
        exitATMode(gpio);
        return false;
    }

    if (!sendATCommand(serial, "AT+FU3", response) ||
        response.rfind("OK", 0) != 0) {
        exitATMode(gpio);
        return false;
    }

    std::snprintf(command, sizeof(command), "AT+B%d", uartBaud);
    if (!sendATCommand(serial, command, response) ||
        response.rfind("OK", 0) != 0) {
        exitATMode(gpio);
        return false;
    }

    exitATMode(gpio);

    serial.reopen(uartBaud);
    sleepMs(20);
    serial.flushRx();

    return true;
}

static void blinkErrorPattern(GpioOutputs& gpio) {
    static auto lastToggle = std::chrono::steady_clock::now();
    static bool state = false;

    const auto now = std::chrono::steady_clock::now();
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - lastToggle).count();

    if (elapsed >= 200) {
        lastToggle = now;
        state = !state;
        gpio.setLed(state);
    }
}

static bool isValidRequestId(std::string_view requestId) {
    if (requestId == "-1") {
        return true;
    }

    if (requestId.empty() || requestId.front() == '_' || requestId.back() == '_') {
        return false;
    }

    bool previousWasUnderscore = false;

    for (const char ch : requestId) {
        const unsigned char byte = static_cast<unsigned char>(ch);

        if (std::isdigit(byte)) {
            previousWasUnderscore = false;
            continue;
        }

        if (ch == '_') {
            if (previousWasUnderscore) {
                return false;
            }

            previousWasUnderscore = true;
            continue;
        }

        return false;
    }

    return true;
}

static std::string_view extractBaseRequestId(std::string_view requestId) {
    const size_t separatorPos = requestId.find('_');
    if (separatorPos == std::string_view::npos) {
        return requestId;
    }

    return requestId.substr(0, separatorPos);
}

static bool requestIdNeedsRepeatedResponse(std::string_view requestId) {
    const std::string_view baseRequestId = extractBaseRequestId(requestId);

    for (const std::string_view repeatedBaseId : REPEATED_RESPONSE_BASE_IDS) {
        if (baseRequestId == repeatedBaseId) {
            return true;
        }
    }

    return false;
}

static std::optional<std::string> extractRequestIdFromPath(std::string_view path) {
    static constexpr std::string_view prefix = "/api/request/";

    if (path.size() < prefix.size() || path.substr(0, prefix.size()) != prefix) {
        return std::nullopt;
    }

    const std::string_view requestId = path.substr(prefix.size());
    if (!isValidRequestId(requestId)) {
        return std::nullopt;
    }

    return std::string(requestId);
}

static std::optional<std::string> extractRequestIdFromStatusPath(std::string_view path) {
    static constexpr std::string_view prefix = "/api/request-status/";

    if (path.size() < prefix.size() || path.substr(0, prefix.size()) != prefix) {
        return std::nullopt;
    }

    const std::string_view requestId = path.substr(prefix.size());
    if (!isValidRequestId(requestId)) {
        return std::nullopt;
    }

    return std::string(requestId);
}

static void stopSiteHostAndChromium() {
    std::thread([]() {
        sleepMs(200);

        const int hostExitCode =
            std::system("pkill -f \"python3 -m http.server 4173\"");
        std::cout << "Stop site host exit code: " << hostExitCode << std::endl;

        const int chromiumExitCode =
            std::system("pkill -f \"/usr/lib/chromium/chromium\"");
        std::cout << "Stop chromium exit code: " << chromiumExitCode << std::endl;
    }).detach();
}

struct ParsedSlaveResponse {
    std::string requestId;
    std::optional<int> state;
    std::string raw;
};

static std::optional<ParsedSlaveResponse> parseSlaveResponse(std::string_view line) {
    static constexpr std::string_view prefix = "RESP,";

    if (line.size() <= prefix.size() || line.substr(0, prefix.size()) != prefix) {
        return std::nullopt;
    }

    const std::string_view payload = line.substr(prefix.size());
    const size_t firstCommaPos = payload.find(',');

    if (firstCommaPos == std::string_view::npos) {
        if (!isValidRequestId(payload)) {
            return std::nullopt;
        }

        return ParsedSlaveResponse{std::string(payload), std::nullopt, std::string(line)};
    }

    const std::string_view requestId = payload.substr(0, firstCommaPos);
    const std::string_view statePart = payload.substr(firstCommaPos + 1);

    if (!isValidRequestId(requestId) || statePart.size() != 1 ||
        (statePart != "0" && statePart != "1")) {
        return std::nullopt;
    }

    return ParsedSlaveResponse{
        std::string(requestId),
        statePart == "0" ? 0 : 1,
        std::string(line)
    };
}

static std::string joinResponses(const std::vector<std::string>& responses) {
    if (responses.empty()) {
        return {};
    }

    std::string joined = responses.front();
    for (size_t i = 1; i < responses.size(); ++i) {
        joined += " | ";
        joined += responses[i];
    }

    return joined;
}

static int httpStatusForSlaveError(std::string_view errorCode) {
    if (errorCode == "BUSY") {
        return 409;
    }
    if (errorCode == "INVALID_SLOT" || errorCode == "BAD_REQUEST_ID" ||
        errorCode == "UNSUPPORTED" || errorCode == "INVALID_MM") {
        return 400;
    }
    if (errorCode == "NOT_READY") {
        return 503;
    }
    if (errorCode == "STEPPER_FAULT" || errorCode == "STEPPER_WRITE_FAIL" ||
        errorCode == "STEPPER_READ_FAIL" || errorCode == "EEPROM_SAVE_FAIL") {
        return 502;
    }
    return 502;
}

class MasterLogic {
public:
    struct RequestResult {
        bool ok = false;
        int httpStatus = 500;
        std::string response;
        std::vector<std::string> responses;
        std::string error;
        bool awaitingSecondResponse = false;
        std::optional<control_station::master::SystemSummary> systemSummary;
        std::optional<std::vector<int>> boomSlots;
    };

    void sendRequest(SerialPort& serial, const std::string& requestId) {
        serial.flushRx();
        serial.writeString("REQ," + requestId + "\r\n");
        waitingResponse_ = true;
        std::cout << "Sent: REQ," << requestId << std::endl;
    }

    RequestResult request(SerialPort& serial, GpioOutputs& gpio, const std::string& requestId) {
        if (waitingResponse_ || waitingSecondResponse_) {
            return {
                false,
                409,
                {},
                {},
                waitingSecondResponse_
                    ? "Master is still waiting for the second slave response"
                    : "Master is still waiting for the previous slave response",
                false,
                std::nullopt,
                std::nullopt
            };
        }

        sendRequest(serial, requestId);
        const bool needsRepeatedResponse = requestIdNeedsRepeatedResponse(requestId);
        std::vector<std::string> collectedResponses;
        int expectedState = needsRepeatedResponse ? 0 : -1;
        auto currentStageStartedAt = std::chrono::steady_clock::now();

        while (waitingResponse_) {
            std::string line;
            if (serial.readLine(line, 20)) {
                const auto parsedMessage =
                    control_station::master::parseSlaveMessage(line);
                if (!parsedMessage.has_value()) {
                    std::cout << "Ignoring malformed response: " << line << std::endl;
                    continue;
                }

                if (parsedMessage->request_id != requestId) {
                    std::cout << "Ignoring response for another request: " << line << std::endl;
                    continue;
                }

                if (parsedMessage->kind == control_station::master::ParsedSlaveMessageKind::Error) {
                    waitingResponse_ = false;
                    std::cout << "Received slave error: " << line << std::endl;
                    return {
                        false,
                        httpStatusForSlaveError(parsedMessage->error_code),
                        line,
                        {line},
                        parsedMessage->error_code,
                        false,
                        std::nullopt,
                        std::nullopt
                    };
                }

                if (parsedMessage->kind == control_station::master::ParsedSlaveMessageKind::SystemSummary) {
                    if (needsRepeatedResponse) {
                        waitingResponse_ = false;
                        std::cout << "Received STAT for repeated-response request: "
                                  << line << std::endl;
                        return {
                            false,
                            502,
                            line,
                            {line},
                            "Unexpected system summary for repeated-response request",
                            false,
                            std::nullopt,
                            std::nullopt
                        };
                    }

                    waitingResponse_ = false;
                    collectedResponses.push_back(parsedMessage->raw);
                    gpio.toggleLed();
                    std::cout << "Received system summary: " << line << std::endl;
                    return {
                        true,
                        200,
                        joinResponses(collectedResponses),
                        collectedResponses,
                        {},
                        false,
                        parsedMessage->summary,
                        std::nullopt
                    };
                }

                if (parsedMessage->kind == control_station::master::ParsedSlaveMessageKind::BoomSlots) {
                    if (needsRepeatedResponse) {
                        waitingResponse_ = false;
                        return {
                            false,
                            502,
                            line,
                            {line},
                            "Unexpected boom slots for repeated-response request",
                            false,
                            std::nullopt,
                            std::nullopt
                        };
                    }

                    waitingResponse_ = false;
                    collectedResponses.push_back(parsedMessage->raw);
                    gpio.toggleLed();
                    return {
                        true,
                        200,
                        joinResponses(collectedResponses),
                        collectedResponses,
                        {},
                        false,
                        std::nullopt,
                        parsedMessage->boom_slots
                    };
                }

                if (!needsRepeatedResponse) {
                    if (parsedMessage->state.has_value()) {
                        waitingResponse_ = false;
                        std::cout << "Received staged response for single-response request: "
                                  << line << std::endl;
                        return {
                            false,
                            502,
                            line,
                            {line},
                            "Unexpected staged response for single-response request",
                            false,
                            std::nullopt,
                            std::nullopt
                        };
                    }

                    waitingResponse_ = false;
                    collectedResponses.push_back(parsedMessage->raw);
                    gpio.toggleLed();
                    std::cout << "Received valid response: " << line << std::endl;
                    return {
                        true,
                        200,
                        joinResponses(collectedResponses),
                        collectedResponses,
                        {},
                        false,
                        std::nullopt,
                        std::nullopt
                    };
                }

                if (!parsedMessage->state.has_value() || *parsedMessage->state != expectedState) {
                    waitingResponse_ = false;
                    std::cout << "Received unexpected repeated response stage: " << line << std::endl;
                    return {
                        false,
                        502,
                        line,
                        {line},
                        expectedState == 0
                            ? "Expected first repeated response with state 0"
                            : "Expected second repeated response with state 1",
                        false,
                        std::nullopt,
                        std::nullopt
                    };
                }

                collectedResponses.push_back(parsedMessage->raw);

                if (expectedState == 0) {
                    waitingResponse_ = false;
                    waitingSecondResponse_ = true;
                    pendingSecondRequestId_ = requestId;
                    pendingSecondResponses_ = collectedResponses;
                    secondResponseStartedAt_ = std::chrono::steady_clock::now();
                    std::cout << "Received first repeated response: " << line << std::endl;
                    return {
                        true,
                        200,
                        joinResponses(collectedResponses),
                        collectedResponses,
                        {},
                        true,
                        std::nullopt,
                        std::nullopt
                    };
                }

                waitingResponse_ = false;
                gpio.toggleLed();
                std::cout << "Received second repeated response: " << line << std::endl;
                return {
                    true,
                    200,
                    joinResponses(collectedResponses),
                    collectedResponses,
                    {},
                    false,
                    std::nullopt,
                    std::nullopt
                };
            }

            const auto now = std::chrono::steady_clock::now();
            const auto waitMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    now - currentStageStartedAt)
                                    .count();
            const int timeoutMs =
                needsRepeatedResponse && expectedState == 1
                    ? SECOND_RESPONSE_TIMEOUT_MS
                    : FIRST_RESPONSE_TIMEOUT_MS;

            if (waitMs >= timeoutMs) {
                waitingResponse_ = false;

                if (needsRepeatedResponse && expectedState == 1) {
                    std::cout << "Second response timeout" << std::endl;
                    return {
                        false,
                        504,
                        joinResponses(collectedResponses),
                        collectedResponses,
                        "Second slave response timeout",
                        false,
                        std::nullopt,
                        std::nullopt
                    };
                }

                std::cout << "First response timeout" << std::endl;
                return {false, 504, {}, {}, "Slave response timeout", false, std::nullopt, std::nullopt};
            }
        }

        return {false, 500, {}, {}, "Request processing stopped unexpectedly", false, std::nullopt, std::nullopt};
    }

    RequestResult requestStatus(SerialPort& serial, GpioOutputs& gpio, const std::string& requestId) {
        if (!waitingSecondResponse_) {
            return {false, 404, {}, {}, "No pending second slave response", false, std::nullopt, std::nullopt};
        }

        if (pendingSecondRequestId_ != requestId) {
            return {
                false,
                409,
                {},
                pendingSecondResponses_,
                "Master is waiting for the second response of another request",
                true,
                std::nullopt,
                std::nullopt
            };
        }

        auto pollStartedAt = std::chrono::steady_clock::now();
        while (true) {
            std::string line;
            if (serial.readLine(line, 20)) {
                const auto parsedMessage =
                    control_station::master::parseSlaveMessage(line);
                if (!parsedMessage.has_value()) {
                    std::cout << "Ignoring malformed response while waiting second: " << line << std::endl;
                    continue;
                }

                if (parsedMessage->request_id != requestId) {
                    std::cout << "Ignoring response for another request while waiting second: "
                              << line << std::endl;
                    continue;
                }

                if (parsedMessage->kind == control_station::master::ParsedSlaveMessageKind::Error) {
                    waitingSecondResponse_ = false;
                    pendingSecondRequestId_.clear();
                    auto failedResponses = pendingSecondResponses_;
                    failedResponses.push_back(parsedMessage->raw);
                    pendingSecondResponses_.clear();
                    return {
                        false,
                        httpStatusForSlaveError(parsedMessage->error_code),
                        line,
                        failedResponses,
                        parsedMessage->error_code,
                        false,
                        std::nullopt,
                        std::nullopt
                    };
                }

                if (parsedMessage->kind != control_station::master::ParsedSlaveMessageKind::Response ||
                    !parsedMessage->state.has_value() || *parsedMessage->state != 1) {
                    waitingSecondResponse_ = false;
                    pendingSecondRequestId_.clear();
                    pendingSecondResponses_.clear();
                    return {
                        false,
                        502,
                        line,
                        {line},
                        "Expected second repeated response with state 1",
                        false,
                        std::nullopt,
                        std::nullopt
                    };
                }

                waitingSecondResponse_ = false;
                pendingSecondRequestId_.clear();
                pendingSecondResponses_.push_back(parsedMessage->raw);
                gpio.toggleLed();
                std::cout << "Received second repeated response: " << line << std::endl;

                auto completedResponses = pendingSecondResponses_;
                pendingSecondResponses_.clear();

                return {
                    true,
                    200,
                    joinResponses(completedResponses),
                    completedResponses,
                    {},
                    false,
                    std::nullopt,
                    std::nullopt
                };
            }

            const auto now = std::chrono::steady_clock::now();
            const auto secondWaitMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                          now - secondResponseStartedAt_)
                                          .count();
            if (secondWaitMs >= SECOND_RESPONSE_TIMEOUT_MS) {
                waitingSecondResponse_ = false;
                auto timedOutResponses = pendingSecondResponses_;
                pendingSecondRequestId_.clear();
                pendingSecondResponses_.clear();
                std::cout << "Second response timeout" << std::endl;
                return {
                    false,
                    504,
                    joinResponses(timedOutResponses),
                    timedOutResponses,
                    "Second slave response timeout",
                    false,
                    std::nullopt,
                    std::nullopt
                };
            }

            const auto pollWaitMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        now - pollStartedAt)
                                        .count();
            if (pollWaitMs >= 120) {
                return {
                    true,
                    200,
                    joinResponses(pendingSecondResponses_),
                    pendingSecondResponses_,
                    {},
                    true,
                    std::nullopt,
                    std::nullopt
                };
            }
        }
    }

private:
    bool waitingResponse_ = false;
    bool waitingSecondResponse_ = false;
    std::string pendingSecondRequestId_;
    std::vector<std::string> pendingSecondResponses_;
    std::chrono::steady_clock::time_point secondResponseStartedAt_{};
};

int main() {
    try {
        GpioOutputs gpio(HC12_SET_GPIO, STATUS_LED_GPIO);
        SerialPort serial(UART_DEVICE, 9600);

        const bool radioReady =
            applyRadioSettings(serial, gpio, HC12_CHANNEL, HC12_UART_BAUD);

        MasterLogic master;

        if (!radioReady) {
            std::cerr << "HC-12 configuration failed" << std::endl;
            while (true) {
                blinkErrorPattern(gpio);
                sleepMs(20);
            }
        }

        std::cout << "HC-12 master ready on " << UART_DEVICE
                  << ", channel=" << static_cast<int>(HC12_CHANNEL)
                  << ", baud=" << HC12_UART_BAUD << std::endl;

        const int httpServerFd = createHttpServerSocket(HTTP_SERVER_PORT);
        std::cout << "HTTP control API ready on port " << HTTP_SERVER_PORT << std::endl;

        while (true) {
            int clientFd = -1;

            if (!waitForClientConnection(httpServerFd, 200, clientFd)) {
                continue;
            }

            try {
                std::string method;
                std::string path;

                if (!readHttpRequest(clientFd, method, path)) {
                    sendJsonResult(clientFd, 400, false, {}, "Malformed HTTP request");
                    ::close(clientFd);
                    continue;
                }

                if (method == "OPTIONS") {
                    sendHttpResponse(clientFd, 204, "text/plain; charset=utf-8", "");
                    ::close(clientFd);
                    continue;
                }

                if (method == "GET" && path == "/api/health") {
                    sendJsonResult(clientFd, 200, true, "master-ready", {});
                    ::close(clientFd);
                    continue;
                }

                if ((method == "GET" || method == "POST") && path == "/api/close-site") {
                    stopSiteHostAndChromium();
                    sendJsonResult(clientFd, 200, true, "site-close-started", {});
                    ::close(clientFd);
                    continue;
                }

                if ((method == "GET" || method == "POST") && path == "/api/request42") {
                    const auto result = master.request(serial, gpio, "42");
                    sendJsonResult(clientFd,
                                   result.httpStatus,
                                   result.ok,
                                   result.response,
                                   result.error,
                                   &result.responses,
                                   result.awaitingSecondResponse,
                                   result.systemSummary ? &(*result.systemSummary) : nullptr,
                                   result.boomSlots ? &(*result.boomSlots) : nullptr);
                    ::close(clientFd);
                    continue;
                }

                if (method == "GET" || method == "POST") {
                    static constexpr std::string_view statusPrefix = "/api/request-status/";
                    static constexpr std::string_view requestPrefix = "/api/request/";
                    const std::string_view pathView(path);

                    if (pathView.size() >= statusPrefix.size() &&
                        pathView.substr(0, statusPrefix.size()) == statusPrefix) {
                        const auto requestId = extractRequestIdFromStatusPath(path);
                        if (!requestId.has_value()) {
                            sendJsonResult(clientFd, 400, false, {}, "Invalid request id");
                            ::close(clientFd);
                            continue;
                        }

                        const auto result = master.requestStatus(serial, gpio, *requestId);
                        sendJsonResult(clientFd,
                                       result.httpStatus,
                                       result.ok,
                                       result.response,
                                       result.error,
                                       &result.responses,
                                       result.awaitingSecondResponse,
                                       result.systemSummary ? &(*result.systemSummary) : nullptr,
                                       result.boomSlots ? &(*result.boomSlots) : nullptr);
                        ::close(clientFd);
                        continue;
                    }

                    if (pathView.size() >= requestPrefix.size() &&
                        pathView.substr(0, requestPrefix.size()) == requestPrefix) {
                        const auto requestId = extractRequestIdFromPath(path);
                        if (!requestId.has_value()) {
                            sendJsonResult(clientFd, 400, false, {}, "Invalid request id");
                            ::close(clientFd);
                            continue;
                        }

                        const auto result = master.request(serial, gpio, *requestId);
                        sendJsonResult(clientFd,
                                       result.httpStatus,
                                       result.ok,
                                       result.response,
                                       result.error,
                                       &result.responses,
                                       result.awaitingSecondResponse,
                                       result.systemSummary ? &(*result.systemSummary) : nullptr,
                                       result.boomSlots ? &(*result.boomSlots) : nullptr);
                        ::close(clientFd);
                        continue;
                    }
                }

                sendJsonResult(clientFd, 404, false, {}, "Route not found");
            } catch (const std::exception& e) {
                std::cerr << "HTTP request handling failed: " << e.what() << std::endl;
                if (clientFd >= 0) {
                    try {
                        sendJsonResult(clientFd, 500, false, {}, e.what());
                    } catch (...) {
                    }
                }
            }

            if (clientFd >= 0) {
                ::close(clientFd);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
}
