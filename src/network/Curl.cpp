#include "commons.hpp"

#include "debug/Logger.hpp"

#define NOMINMAX
#include <curl/curl.h>
#include <algorithm>

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

static size_t header_callback(
    char* buffer, size_t size, size_t nitems, void* userdata
) {
    auto* headers = static_cast<std::vector<std::string>*>(userdata);
    size_t len = size * nitems;
    std::string header(buffer, len);

    while (!header.empty() &&
           (header.back() == '\r' || header.back() == '\n')) {
        header.pop_back();
    }

    headers->push_back(std::move(header));
    return len;
}

struct ProcessingRequest {
    CURLM* multiHandle;
    CURL* curl;
    HttpRequest request;
    std::vector<char> buffer;
    std::vector<std::string> headers;

    ProcessingRequest(CURLM* multiHandle) : multiHandle(multiHandle) {
        curl = curl_easy_init();
    }

    ProcessingRequest(const ProcessingRequest&) = delete;
    ProcessingRequest& operator=(const ProcessingRequest&) = delete;

    ProcessingRequest(ProcessingRequest&& other) noexcept
        : multiHandle(other.multiHandle),
          curl(other.curl),
          request(std::move(other.request)),
          buffer(std::move(other.buffer)),
          headers(std::move(other.headers)) {
        other.curl = nullptr;
        other.multiHandle = nullptr;
    }

    ProcessingRequest& operator=(ProcessingRequest&& other) noexcept {
        if (this != &other) {
            if (curl) {
                curl_multi_remove_handle(multiHandle, curl);
                curl_easy_cleanup(curl);
            }
            multiHandle = other.multiHandle;
            curl = other.curl;
            request = std::move(other.request);
            buffer = std::move(other.buffer);
            headers = std::move(other.headers);
            other.curl = nullptr;
            other.multiHandle = nullptr;
        }
        return *this;
    }

    ~ProcessingRequest() {
        if (curl) {
            curl_multi_remove_handle(multiHandle, curl);
            curl_easy_cleanup(curl);
        }
    }
};

class CurlRequests : public Requests {
    CURLM* multiHandle;
    std::vector<std::unique_ptr<ProcessingRequest>> requests;

    size_t totalUpload = 0;
    size_t totalDownload = 0;
public:
    CurlRequests(CURLM* multiHandle) : multiHandle(multiHandle) {
    }

    virtual ~CurlRequests() {
        requests.clear();
        curl_multi_cleanup(multiHandle);
    }

    void request(HttpRequest request) override {
        processRequest(std::move(request));
    }

    void processRequest(HttpRequest request) {
        auto entry = std::make_unique<ProcessingRequest>(multiHandle);
        auto curl = entry->curl;

        curl_easy_setopt(curl, CURLOPT_URL, request.url.c_str());
        curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, request.method.c_str());
        
        curl_slist* hs = nullptr;
        
        for (const auto& header : request.headers) {
            hs = curl_slist_append(hs, header.c_str());
        }

        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(request.body.length()));
        if (!request.body.empty()) {
            curl_easy_setopt(curl, CURLOPT_COPYPOSTFIELDS, request.body.data());
        }
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, static_cast<long>(request.verifySSL));
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, static_cast<long>(request.verifySSL));
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, hs);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, static_cast<long>(request.followLocation));
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &entry->buffer);
        curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, header_callback);
        curl_easy_setopt(curl, CURLOPT_HEADERDATA, &entry->headers);
        curl_easy_setopt(curl, CURLOPT_USERAGENT, "curl/7.81.0");
        if (request.timeoutMs > 0) {
            curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, request.timeoutMs);
        }
#ifndef NDEBUG
        // curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
#endif
        if (request.maxSize == 0) {
            curl_easy_setopt(
                curl, CURLOPT_MAXFILESIZE, std::numeric_limits<long>::max()
            );
        } else {
            curl_easy_setopt(curl, CURLOPT_MAXFILESIZE, request.maxSize);
        }
        {
            CURLMcode res = curl_multi_add_handle(multiHandle, curl);
            if (res != CURLM_OK) {
                auto message = curl_multi_strerror(res);
                logger.error() << message << " (" << request.url << ")";
                if (request.onResponse) {
                    request.onResponse({0, {}, {}});
                }
                return;
            }
        }
        {
            int running;
            CURLMcode res = curl_multi_perform(multiHandle, &running);
            if (res != CURLM_OK) {
                auto message = curl_multi_strerror(res);
                logger.error() << message << " (" << request.url << ")";
                if (request.onResponse) {
                    request.onResponse({0, {}, {}});
                }
                return;
            }
        }
        entry->request = std::move(request);
        requests.push_back(std::move(entry));
    }

    void update() override {
        if (CURLMcode res = curl_multi_poll(
            multiHandle,
            nullptr,
            0,
            0,
            nullptr
        )) {
            auto message = curl_multi_strerror(res);
            logger.error() << message;
            return;
        }

        int messagesLeft;
        int running;
        if (CURLMcode res = curl_multi_perform(multiHandle, &running)) {
            auto message = curl_multi_strerror(res);
            logger.error() << message;
            return;
        }
        CURLMsg* msg = curl_multi_info_read(multiHandle, &messagesLeft);
        if (msg == nullptr || msg->msg != CURLMSG_DONE) {
            return;
        }
        auto curl = msg->easy_handle;
        auto found = std::find_if(
            requests.begin(),
            requests.end(),
            [curl](const std::unique_ptr<ProcessingRequest>& entry) {
                return entry && entry->curl == curl;
            }
        );
        if (requests.empty() || found == requests.end()) {
            logger.error() << "could not find request for cURL handle";
            return;
        }
        auto entry = std::move(*found);
        auto& req = entry->request;
        requests.erase(found);

        long response = -1;
        CURLcode result = msg->data.result;
        curl_easy_getinfo(msg->easy_handle, CURLINFO_RESPONSE_CODE, &response);
        auto headers = std::move(entry->headers);
        if (response == 0) {
            auto message = std::string(curl_easy_strerror(result));
            logger.error() << message << " (" << req.url << ")";
            if (req.onResponse) {
                req.onResponse(
                    {static_cast<int>(response),
                     std::move(headers),
                     std::vector<char>(
                         message.data(), message.data() + message.size()
                     )}
                );
            }
        } else {
            long size;
            if (!curl_easy_getinfo(curl, CURLINFO_REQUEST_SIZE, &size)) {
                totalUpload += size;
            }
            if (!curl_easy_getinfo(curl, CURLINFO_HEADER_SIZE, &size)) {
                totalDownload += size;
            }
            totalDownload += entry->buffer.size();
            if (req.onResponse) {
                req.onResponse({
                    static_cast<int>(response),
                    std::move(headers),
                    std::move(entry->buffer),
                });
            }
        }
    }

    size_t getTotalUpload() const override {
        return totalUpload;
    }

    size_t getTotalDownload() const override {
        return totalDownload;
    }

    static std::unique_ptr<CurlRequests> create() {
        auto multiHandle = curl_multi_init();
        if (multiHandle == nullptr) {
            throw std::runtime_error("could not initialize cURL-multi");
        }
        return std::make_unique<CurlRequests>(multiHandle);
    }
};

namespace network {
    std::unique_ptr<Requests> create_curl_requests() {
        return CurlRequests::create();
    }
}
