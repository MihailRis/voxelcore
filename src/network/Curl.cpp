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

struct ProcessingRequest {
    CURLM* multiHandle;
    CURL* curl;
    HttpRequest request;
    std::vector<char> buffer;

    ProcessingRequest(CURLM* multiHandle) : multiHandle(multiHandle) {
        curl = curl_easy_init();
    }

    ~ProcessingRequest() {
        curl_multi_remove_handle(multiHandle, curl);
        curl_easy_cleanup(curl);
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

        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, request.body.length());
        if (!request.body.empty()) {
            curl_easy_setopt(curl, CURLOPT_COPYPOSTFIELDS, request.body.data());
        }
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, request.verifySSL);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, request.verifySSL);
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, hs);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, request.followLocation);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &entry->buffer);
        curl_easy_setopt(curl, CURLOPT_USERAGENT, "curl/7.81.0");
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
        curl_multi_add_handle(multiHandle, curl);
        int running;
        CURLMcode res = curl_multi_perform(multiHandle, &running);
        if (res != CURLM_OK) {
            auto message = curl_multi_strerror(res);
            logger.error() << message << " (" << request.url << ")";
            if (request.onReject) {
                request.onReject(HTTP_BAD_GATEWAY, {});
            }
        }
        entry->request = std::move(request);
        requests.push_back(std::move(entry));
    }

    void update() override {
        int messagesLeft;
        int running;
        CURLMsg* msg;
        CURLMcode res = curl_multi_perform(multiHandle, &running);
        if (res != CURLM_OK) {
            auto message = curl_multi_strerror(res);
            logger.error() << message;
            return;
        }
        if ((msg = curl_multi_info_read(multiHandle, &messagesLeft)) != nullptr) {
            auto curl = msg->easy_handle;
            auto found = std::find_if(
                requests.begin(), requests.end(),
                [curl](const std::unique_ptr<ProcessingRequest>& entry) {
                    return entry->curl == curl;
                }
            );
            if (found == requests.end()) {
                logger.error() << "could not find request for cURL handle";
                return;
            }
            auto entry = std::move(*found);
            auto& req = entry->request;

            requests.erase(found);

            if(msg->msg == CURLMSG_DONE) {
                curl_multi_remove_handle(multiHandle, curl);
            }
            int response = -1;
            CURLcode result = msg->data.result;
            curl_easy_getinfo(msg->easy_handle, CURLINFO_RESPONSE_CODE, &response);
            if (response == HTTP_OK) {
                long size;
                if (!curl_easy_getinfo(curl, CURLINFO_REQUEST_SIZE, &size)) {
                    totalUpload += size;
                }
                if (!curl_easy_getinfo(curl, CURLINFO_HEADER_SIZE, &size)) {
                    totalDownload += size;
                }
                totalDownload += entry->buffer.size();
                if (req.onResponse) {
                    req.onResponse(std::move(entry->buffer));
                }
            } else if (response == 0) {
                auto message = std::string(curl_easy_strerror(result));
                logger.error() << message << " (" << req.url << ")";
                if (req.onReject) {
                    req.onReject(
                        response,
                        std::vector<char>(
                            message.data(), message.data() + message.size()
                        )
                    );
                }
            } else {
                logger.error()
                    << "response code " << response << " (" << req.url << ")"
                    << (entry->buffer.empty()
                            ? ""
                            : std::to_string(entry->buffer.size()) + " byte(s)");
                totalDownload += entry->buffer.size();
                if (req.onReject) {
                    req.onReject(response, std::move(entry->buffer));
                }
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
        auto curl = curl_easy_init();
        if (curl == nullptr) {
            throw std::runtime_error("could not initialize cURL");
        }
        auto multiHandle = curl_multi_init();
        if (multiHandle == nullptr) {
            curl_easy_cleanup(curl);
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
