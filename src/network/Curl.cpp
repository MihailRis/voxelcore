#include "commons.hpp"

#include "debug/Logger.hpp"

#define NOMINMAX
#include <curl/curl.h>
#include <queue>

using namespace network;

static debug::Logger logger("curl");

inline constexpr int HTTP_OK = 200;
inline constexpr int HTTP_BAD_GATEWAY = 502;

static size_t write_callback(
    char* ptr, size_t size, size_t nmemb, void* userdata
) {
    auto& buffer = *reinterpret_cast<std::vector<char>*>(userdata);
    size_t psize = buffer.size();
    buffer.resize(psize + size * nmemb);
    std::memcpy(buffer.data() + psize, ptr, size * nmemb);
    return size * nmemb;
}

enum class RequestType {
    GET, POST
};

struct Request {
    RequestType type;
    std::string url;
    OnResponse onResponse;
    OnReject onReject;
    long maxSize;
    bool followLocation = false;
    std::string data;
    std::vector<std::string> headers;
};

class CurlRequests : public Requests {
    CURLM* multiHandle;
    CURL* curl;

    size_t totalUpload = 0;
    size_t totalDownload = 0;

    OnResponse onResponse;
    OnReject onReject;
    std::vector<char> buffer;
    std::string url;

    std::queue<Request> requests;
public:
    CurlRequests(CURLM* multiHandle, CURL* curl)
        : multiHandle(multiHandle), curl(curl) {
    }

    virtual ~CurlRequests() {
        curl_multi_remove_handle(multiHandle, curl);
        curl_easy_cleanup(curl);
        curl_multi_cleanup(multiHandle);
    }
    void get(
        const std::string& url,
        OnResponse onResponse,
        OnReject onReject,
        std::vector<std::string> headers,
        long maxSize
    ) override {
        Request request {
            RequestType::GET,
            url,
            onResponse,
            onReject,
            maxSize,
            true,
            "",
            std::move(headers)};
        processRequest(std::move(request));
    }

    void post(
        const std::string& url,
        const std::string& data,
        OnResponse onResponse,
        OnReject onReject=nullptr,
        std::vector<std::string> headers = {},
        long maxSize=0
    ) override {
        Request request {
            RequestType::POST,
            url,
            onResponse,
            onReject,
            maxSize,
            false,
            "",
            std::move(headers)};
        request.data = data;
        processRequest(std::move(request));
    }

    void processRequest(Request request) {
        if (!url.empty()) {
            requests.push(request);
            return;
        }
        onResponse = request.onResponse;
        onReject = request.onReject;
        url = request.url;

        buffer.clear();

        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_POST, request.type == RequestType::POST);
        
        curl_slist* hs = nullptr;
        
        for (const auto& header : request.headers) {
            hs = curl_slist_append(hs, header.c_str());
        }

        switch (request.type) {
            case RequestType::GET:
                break;
            case RequestType::POST: 
                hs = curl_slist_append(hs, "Content-Type: application/json");
                curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, request.data.length());
                curl_easy_setopt(curl, CURLOPT_COPYPOSTFIELDS, request.data.c_str());
                break;
            default:
                throw std::runtime_error("not implemented");
        }
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, hs);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, request.followLocation);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);
        curl_easy_setopt(curl, CURLOPT_USERAGENT, "curl/7.81.0");
        if (request.maxSize == 0) {
            curl_easy_setopt(
                curl, CURLOPT_MAXFILESIZE, std::numeric_limits<long>::max()
            );
        } else {
            curl_easy_setopt(curl, CURLOPT_MAXFILESIZE, request.maxSize);
        }
        curl_multi_add_handle(multiHandle, curl);
        int running;
        CURLMcode res = curl_multi_perform(multiHandle, &running);
        if (res != CURLM_OK) {
            auto message = curl_multi_strerror(res);
            logger.error() << message << " (" << url << ")";
            if (onReject) {
                onReject(HTTP_BAD_GATEWAY, {});
            }
            url = "";
        }
    }

    void update() override {
        int messagesLeft;
        int running;
        CURLMsg* msg;
        CURLMcode res = curl_multi_perform(multiHandle, &running);
        if (res != CURLM_OK) {
            auto message = curl_multi_strerror(res);
            logger.error() << message << " (" << url << ")";
            if (onReject) {
                onReject(HTTP_BAD_GATEWAY, {});
            }
            curl_multi_remove_handle(multiHandle, curl);
            url = "";
            return;
        }
        if ((msg = curl_multi_info_read(multiHandle, &messagesLeft)) != nullptr) {
            if(msg->msg == CURLMSG_DONE) {
                curl_multi_remove_handle(multiHandle, curl);
            }
            int response;
            curl_easy_getinfo(msg->easy_handle, CURLINFO_RESPONSE_CODE, &response);
            if (response == HTTP_OK) {
                long size;
                if (!curl_easy_getinfo(curl, CURLINFO_REQUEST_SIZE, &size)) {
                    totalUpload += size;
                }
                if (!curl_easy_getinfo(curl, CURLINFO_HEADER_SIZE, &size)) {
                    totalDownload += size;
                }
                totalDownload += buffer.size();
                if (onResponse) {
                    onResponse(std::move(buffer));
                }
            } else {
                logger.error()
                    << "response code " << response << " (" << url << ")"
                    << (buffer.empty()
                            ? ""
                            : std::to_string(buffer.size()) + " byte(s)");
                totalDownload += buffer.size();
                if (onReject) {
                    onReject(response, std::move(buffer));
                }
            }
            url = "";
        }
        if (url.empty() && !requests.empty()) {
            auto request = std::move(requests.front());
            requests.pop();
            processRequest(std::move(request));
        }
    }

    size_t getTotalUpload() const override {
        return totalUpload;
    }

    size_t getTotalDownload() const override {
        return totalDownload;
    }

    static std::unique_ptr<CurlRequests> create() {
        auto curl = curl_easy_init();
        if (curl == nullptr) {
            throw std::runtime_error("could not initialzie cURL");
        }
        auto multiHandle = curl_multi_init();
        if (multiHandle == nullptr) {
            curl_easy_cleanup(curl);
            throw std::runtime_error("could not initialzie cURL-multi");
        }
        return std::make_unique<CurlRequests>(multiHandle, curl);
    }
};

namespace network {
    std::unique_ptr<Requests> create_curl_requests() {
        return CurlRequests::create();
    }
}
