#include "platform.hpp"

#include <time.h>

#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

#include "stringutil.hpp"
#include "typedefs.hpp"
#include "debug/Logger.hpp"
#include "frontend/locale.hpp"

#ifdef _WIN32
#include <Windows.h>
#pragma comment(lib, "winmm.lib")
#else
#include <unistd.h>
#endif

namespace platform::internal {
    std::filesystem::path get_executable_path();
}

static debug::Logger logger("platform");

#ifdef _WIN32
void platform::configure_encoding() {
    // set utf-8 encoding to console output
    SetConsoleOutputCP(CP_UTF8);
    setvbuf(stdout, nullptr, _IOFBF, 1000);
}

std::string platform::detect_locale() {
    LCID lcid = GetThreadLocale();
    wchar_t preferredLocaleName[LOCALE_NAME_MAX_LENGTH];  // locale name format:
                                                          // ll-CC
    if (LCIDToLocaleName(
            lcid, preferredLocaleName, LOCALE_NAME_MAX_LENGTH, 0
        ) == 0) {
        std::cerr
            << "error in platform::detect_locale! LCIDToLocaleName failed."
            << std::endl;
    }
    // ll_CC format
    return util::wstr2str_utf8(preferredLocaleName)
        .replace(2, 1, "_")
        .substr(0, 5);
}

void platform::sleep(size_t millis) {
    // Uses implementation from the SFML library
    // https://github.com/SFML/SFML/blob/master/src/SFML/System/Win32/SleepImpl.cpp

    // Get the minimum supported timer resolution on this system
    static const UINT periodMin = []{
        TIMECAPS tc;
        timeGetDevCaps(&tc, sizeof(TIMECAPS));
        return tc.wPeriodMin;
    }();

    // Set the timer resolution to the minimum for the Sleep call
    timeBeginPeriod(periodMin);

    // Wait...
    Sleep(static_cast<DWORD>(millis));

    // Reset the timer resolution back to the system default
    timeEndPeriod(periodMin);
}

int platform::get_process_id() {
    return GetCurrentProcessId(); 
}

bool platform::open_url(const std::string& url) {
    if (url.empty()) return false;
    // UTF-8 → UTF-16
    int wlen = MultiByteToWideChar(CP_UTF8, 0, url.c_str(), -1, nullptr, 0);
    if (wlen <= 0) return false;

    std::wstring wurl(wlen, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, url.c_str(), -1, &wurl[0], wlen);

    HINSTANCE result = ShellExecuteW(
        nullptr, L"open", wurl.c_str(), nullptr, nullptr, SW_SHOWNORMAL
    );

    return reinterpret_cast<intptr_t>(result) > 32;
}

#else // _WIN32

void platform::configure_encoding() {
}
 
std::string platform::detect_locale() {
    const char* const programLocaleName = setlocale(LC_ALL, nullptr);
    const char* const preferredLocaleName =
        setlocale(LC_ALL, "");  // locale name format: ll_CC.encoding
    if (programLocaleName && preferredLocaleName) {
        setlocale(LC_ALL, programLocaleName);

        return std::string(preferredLocaleName, 5);
    }
    return langs::FALLBACK_DEFAULT;
}

void platform::sleep(size_t millis) {
    std::this_thread::sleep_for(std::chrono::milliseconds(millis));
}

int platform::get_process_id() {
    return getpid();
}

bool platform::open_url(const std::string& url) {
    if (url.empty()) return false;

#ifdef __APPLE__
    auto cmd = "open " + util::quote(url);
    if (int res = system(cmd.c_str())) {
        logger.warning() << "'" << cmd << "' returned code " << res;
    } else {
        return false;
    }
#elif defined(_WIN32)
    auto res = ShellExecuteW(nullptr, L"open", util::quote(url).c_str(), nullptr, nullptr, SW_SHOWDEFAULT);
    if (res <= 32) {
        logger.warning() << "'open' returned code " << res;
    } else {
        return false;
    }
#else
    auto cmd = "xdg-open " + util::quote(url);
    if (int res = system(cmd.c_str())) {
        logger.warning() << "'" << cmd << "' returned code " << res;
    } else {
        return false;
    }
#endif
    return true;
}
#endif // _WIN32

void platform::open_folder(const std::filesystem::path& folder) {
    if (!std::filesystem::is_directory(folder)) {
        logger.warning() << folder << " is not a directory or does not exist";
        return;
    }
#ifdef __APPLE__
    auto cmd = "open " + util::quote(folder.u8string());
    if (int res = system(cmd.c_str())) {
        logger.warning() << "'" << cmd << "' returned code " << res;
    }
#elif defined(_WIN32)
    ShellExecuteW(nullptr, L"open", folder.wstring().c_str(), nullptr, nullptr, SW_SHOWDEFAULT);
#else
    auto cmd = "xdg-open " + util::quote(folder.u8string());
    if (int res = system(cmd.c_str())) {
        logger.warning() << "'" << cmd << "' returned code " << res;
    }

#endif
}

std::filesystem::path platform::get_executable_path() {
#ifdef _WIN32
    wchar_t buffer[MAX_PATH];
    DWORD result = GetModuleFileNameW(nullptr, buffer, MAX_PATH);
    if (result == 0) {
        DWORD error = GetLastError();
        throw std::runtime_error("GetModuleFileName failed with code: " + std::to_string(error));
    }

    int size = WideCharToMultiByte(
        CP_UTF8, 0, buffer, -1, nullptr, 0, nullptr, nullptr
    );
    if (size == 0) {
        throw std::runtime_error("could not get executable path");
    }
    std::string str(size, 0);
    size = WideCharToMultiByte(
        CP_UTF8, 0, buffer, -1, str.data(), size, nullptr, nullptr
    );
    if (size == 0) {
        DWORD error = GetLastError();
        throw std::runtime_error("WideCharToMultiByte failed with code: " + std::to_string(error));
    }
    str.resize(size - 1);
    return std::filesystem::path(str);

#elif defined(__APPLE__)
    auto path = platform::internal::get_executable_path();
    if (path.empty()) {
        throw std::runtime_error("could not get executable path");
    }
    return path;
#else
    char buffer[1024];
    ssize_t count = readlink("/proc/self/exe", buffer, sizeof(buffer));
    if (count != -1) {
        return std::filesystem::canonical(std::filesystem::path(
            std::string(buffer, static_cast<size_t>(count))
        ));
    }
    throw std::runtime_error("could not get executable path");
#endif
}

void platform::new_engine_instance(const std::vector<std::string>& args) {
    auto executable = get_executable_path();

#ifdef _WIN32
    std::stringstream ss;
    ss << util::quote(executable.u8string());
    for (int i = 0; i < args.size(); i++) {
        ss << " " << util::quote(args[i]);
    }

    auto toWString = [](const std::string& src) -> std::wstring {
        if (src.empty()) 
            return L"";
        int size = MultiByteToWideChar(CP_UTF8, 0, src.c_str(), -1, nullptr, 0);
        if (size == 0) {
            throw std::runtime_error(
                "MultiByteToWideChar failed with code: " +
                std::to_string(GetLastError())
            );
        }
        std::vector<wchar_t> buffer(size + 1);
        buffer[size] = 0;
        size = MultiByteToWideChar(CP_UTF8, 0, src.c_str(), -1, buffer.data(), size);
        if (size == 0) {
            throw std::runtime_error(
                "MultiByteToWideChar failed with code: " +
                std::to_string(GetLastError())
            );
        }
        return std::wstring(buffer.data(), size + 1);
    };

    auto commandString = toWString(ss.str());

    STARTUPINFOW si = { sizeof(si) };
    PROCESS_INFORMATION pi = { 0 };
    DWORD flags = CREATE_NEW_PROCESS_GROUP | DETACHED_PROCESS;
    // | CREATE_NO_WINDOW;
    BOOL success = CreateProcessW(
        nullptr,
        commandString.data(),
        nullptr,
        nullptr,
        FALSE,
        flags,
        nullptr,
        nullptr,
        &si,
        &pi
    );
    if (success) {
        CloseHandle(pi.hProcess);
        CloseHandle(pi.hThread);
    } else {
        throw std::runtime_error(
            "starting an engine instance failed with code: " +
            std::to_string(GetLastError())
        );
    }
#else
    std::stringstream ss;
    ss << executable;
    for (int i = 0; i < args.size(); i++) {
        ss << " " << util::quote(args[i]);
    }
    ss << " >/dev/null &";
    
    auto command = ss.str();
    if (int res = system(command.c_str())) {
        throw std::runtime_error(
            "starting an engine instance failed with code: " +
            std::to_string(res)
        );
    }
#endif
}
