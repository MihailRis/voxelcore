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
#include "interfaces/SystemProcess.hpp"
#include "logic/scripting/io_descriptors.hpp"
#include "io/fd_istream.hpp"
#include "io/fd_ostream.hpp"

#ifdef _WIN32
#include <Windows.h>
#include <conio.h>
#pragma comment(lib, "winmm.lib")
#else
#include <sys/poll.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#endif
#ifdef __linux__
    #include <sys/prctl.h>
    #include <sys/wait.h>
#endif

using namespace std::literals;

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

    timeBeginPeriod(periodMin);
    Sleep(static_cast<DWORD>(millis));
    timeEndPeriod(periodMin);
}

int platform::get_process_id() {
    return GetCurrentProcessId();
}

bool platform::open_url(const std::string& url) {
    if (url.empty()) return false;
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
    const char* const preferredLocaleName = setlocale(LC_ALL, "");
    if (programLocaleName && preferredLocaleName) {
        setlocale(LC_ALL, programLocaleName);

        if (std::strlen(preferredLocaleName) >= 5) {
            return std::string(preferredLocaleName, 5);
        } else {
            return std::string(preferredLocaleName);
        }
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

class SystemProcessImpl final : public SystemProcess {
public:
#ifdef __linux__
    SystemProcessImpl(
        pid_t pid, int stdinDescriptor = -1, int stdoutDescriptor = -1
    )
        : pid(pid),
          stdinDescriptor(stdinDescriptor),
          stdoutDescriptor(stdoutDescriptor) {
    }

    ~SystemProcessImpl() {
        terminate();
    }
#elif defined(_WIN32)
    SystemProcessImpl(
        const PROCESS_INFORMATION& pi,
        int stdinDescriptor = -1,
        int stdoutDescriptor = -1
    )
        : processHandle(pi.hProcess),
          pid(pi.dwProcessId),
          stdinDescriptor(stdinDescriptor),
          stdoutDescriptor(stdoutDescriptor) {
        if (pi.hThread != nullptr) {
            CloseHandle(pi.hThread);
        }
    }

    ~SystemProcessImpl() {
        terminate();
    }
#endif
    SystemProcessImpl(const SystemProcessImpl&) = delete;
    SystemProcessImpl(SystemProcessImpl&&) noexcept = default;
    SystemProcessImpl& operator=(const SystemProcessImpl&) = delete;

    SystemProcessImpl& operator=(SystemProcessImpl&& other) noexcept {
        if (this != &other) {
#ifdef __linux__
            pid = other.pid;
#elif defined(_WIN32)
            if (processHandle != nullptr) {
                CloseHandle(processHandle);
            }
            processHandle = other.processHandle;
            pid = other.pid;
            other.processHandle = nullptr;
            other.pid = 0;
#endif
            stdinDescriptor = other.stdinDescriptor;
            stdoutDescriptor = other.stdoutDescriptor;
            other.stdinDescriptor = -1;
            other.stdoutDescriptor = -1;
        }
        return *this;
    }

    bool isActive() const override {
#ifdef __linux__
        if (pid <= 0) {
            return false;
        }

        // fixme: ugly ugly ugly
        int status = 0;
        pid_t result = waitpid(pid, &status, WNOHANG);
        if (result == -1) {
            if (errno == ECHILD) {
                const_cast<SystemProcessImpl*>(this)->pid = 0;
                return false;
            }
            return false;
        }
        if (result == 0) {
            return true;
        }

        const_cast<SystemProcessImpl*>(this)->pid = 0;
        return false;
#elif defined(_WIN32)
        if (processHandle == nullptr) {
            return false;
        }
        DWORD exitCode;
        if (GetExitCodeProcess(processHandle, &exitCode)) {
            return exitCode == STILL_ACTIVE;
        }
        return false;
#elif defined(__APPLE__)
        // FIXME: completely implement SystemProcess for MacOS
#else
        return false;
#endif
    }

    void update() override {}

    void waitForEnd() override {
#ifdef __linux__
        if (pid <= 0) {
            return;
        }
        waitpid(pid, nullptr, 0);
        pid = 0;
#elif defined(_WIN32)
        if (processHandle != nullptr) {
            WaitForSingleObject(processHandle, INFINITE);
        }
#endif
    };

    void terminate() final override {
#ifdef __linux__
        if (pid <= 0) {
            return;
        }
        logger.info() << "terminating " << pid;
        kill(pid, SIGTERM);
#elif defined(_WIN32)
        if (processHandle != nullptr) {
            TerminateProcess(processHandle, 1);
            WaitForSingleObject(processHandle, INFINITE);
        }
#endif
    }

    int getStdinDescriptor() const override {
        return stdinDescriptor;
    }

    int getStdoutDescriptor() const override {
        return stdoutDescriptor;
    }

private:
#ifdef __linux__
    pid_t pid;
    int stdinDescriptor;
    int stdoutDescriptor;
#elif defined(_WIN32)
    HANDLE processHandle = nullptr;
    DWORD pid = 0;
    int stdinDescriptor;
    int stdoutDescriptor;
#endif
};

std::unique_ptr<SystemProcess> platform::new_engine_instance(
    const std::vector<std::string>& args,
    std::filesystem::path outputFile,
    bool subProcess
) {
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

    if (subProcess) {
        SECURITY_ATTRIBUTES pipeAttributes{};
        pipeAttributes.nLength = sizeof(pipeAttributes);
        pipeAttributes.bInheritHandle = TRUE;

        HANDLE stdinRead = nullptr;
        HANDLE stdinWrite = nullptr;
        HANDLE stdoutRead = nullptr;
        HANDLE stdoutWrite = nullptr;

        if (!CreatePipe(&stdinRead, &stdinWrite, &pipeAttributes, 0) ||
            !CreatePipe(&stdoutRead, &stdoutWrite, &pipeAttributes, 0)) {
            throw std::runtime_error(
                "CreatePipe failed with code: " + std::to_string(GetLastError())
            );
        }

        if (!SetHandleInformation(stdinWrite, HANDLE_FLAG_INHERIT, 0) ||
            !SetHandleInformation(stdoutRead, HANDLE_FLAG_INHERIT, 0)) {
            CloseHandle(stdinRead);
            CloseHandle(stdinWrite);
            CloseHandle(stdoutRead);
            CloseHandle(stdoutWrite);
            throw std::runtime_error(
                "SetHandleInformation failed with code: " + std::to_string(GetLastError())
            );
        }

        si.dwFlags = STARTF_USESTDHANDLES;
        si.hStdInput = stdinRead;
        si.hStdOutput = stdoutWrite;
        si.hStdError = stdoutWrite;

        const DWORD flags = CREATE_NEW_PROCESS_GROUP | DETACHED_PROCESS;
        const BOOL success = CreateProcessW(
            nullptr,
            commandString.data(),
            nullptr,
            nullptr,
            TRUE,
            flags,
            nullptr,
            nullptr,
            &si,
            &pi
        );

        CloseHandle(stdinRead);
        CloseHandle(stdoutWrite);

        if (!success) {
            CloseHandle(stdinWrite);
            CloseHandle(stdoutRead);
            throw std::runtime_error(
                "starting an engine instance failed with code: " +
                std::to_string(GetLastError())
            );
        }

        int stdinDescriptor = scripting::io_descriptors::add_descriptor(
            std::make_unique<fd_ostream>(stdinWrite)
        );
        int stdoutDescriptor = scripting::io_descriptors::add_descriptor(
            std::make_unique<fd_istream>(stdoutRead)
        );
        return std::make_unique<SystemProcessImpl>(pi, stdinDescriptor, stdoutDescriptor);
    }

    if (!outputFile.empty()) {
        si.dwFlags |= STARTF_USESTDHANDLES;
        const auto outputPath = toWString(outputFile.u8string());
        si.hStdOutput = CreateFileW(
            outputPath.c_str(),
            GENERIC_WRITE,
            FILE_SHARE_WRITE | FILE_SHARE_READ,
            nullptr,
            CREATE_ALWAYS,
            FILE_ATTRIBUTE_NORMAL,
            nullptr
        );
        if (si.hStdOutput == INVALID_HANDLE_VALUE) {
            throw std::runtime_error(
                "CreateFileW failed with code: " + std::to_string(GetLastError())
            );
        }
    }

    const DWORD flags = CREATE_NEW_PROCESS_GROUP | DETACHED_PROCESS;
    const BOOL success = CreateProcessW(
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
#ifndef __APPLE__
    if (subProcess) {
        int stdinPipe[2] = {-1, -1};
        int stdoutPipe[2] = {-1, -1};
        if (pipe(stdinPipe) != 0 || pipe(stdoutPipe) != 0) {
            if (stdinPipe[0] >= 0) {
                close(stdinPipe[0]);
            }
            if (stdinPipe[1] >= 0) {
                close(stdinPipe[1]);
            }
            if (stdoutPipe[0] >= 0) {
                close(stdoutPipe[0]);
            }
            if (stdoutPipe[1] >= 0) {
                close(stdoutPipe[1]);
            }
            throw std::runtime_error("failed to create subprocess pipes");
        }

        pid_t pid = fork();

        if (pid == 0) {
            prctl(PR_SET_PDEATHSIG, SIGTERM);
            if (getppid() == 1) {
                _exit(1);
            }

            close(stdinPipe[1]);
            close(stdoutPipe[0]);
            dup2(stdinPipe[0], STDIN_FILENO);
            dup2(stdoutPipe[1], STDOUT_FILENO);
            dup2(stdoutPipe[1], STDERR_FILENO);
            close(stdinPipe[0]);
            close(stdoutPipe[1]);

            std::vector<const char*> argv(args.size() + 2);
            argv[0] = const_cast<char*>(executable.c_str());
            for (int i = 0; i < args.size(); i++) {
                argv[i + 1] = args[i].c_str();
            }
            argv[args.size() + 1] = nullptr;

            prctl(PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0);
            execvp(executable.c_str(), const_cast<char**>(argv.data()));
            _exit(127);
        }

        close(stdinPipe[0]);
        close(stdoutPipe[1]);

        const int stdinDescriptorId = scripting::io_descriptors::add_descriptor(
            std::make_unique<fd_ostream>(stdinPipe[1])
        );
        const int stdoutDescriptorId = scripting::io_descriptors::add_descriptor(
            std::make_unique<fd_istream>(stdoutPipe[0])
        );

        usleep(1000);  // waiting for execvp call
        return std::make_unique<SystemProcessImpl>(pid, stdinDescriptorId, stdoutDescriptorId);
    }
#endif

    std::stringstream ss;
    ss << executable;
    for (int i = 0; i < args.size(); i++) {
        ss << " " << util::quote(args[i]);
    }
    if (outputFile.empty()) {
        ss << " >/dev/null &";
    } else {
        ss << " >" << util::quote(outputFile.u8string()) << " &";
    }
    
    auto command = ss.str();
    logger.info() << command;
    if (int res = system(command.c_str())) {
        throw std::runtime_error(
            "starting an engine instance failed with code: " +
            std::to_string(res)
        );
    }
#endif
    return nullptr;
}

bool platform::stdin_has_data() {
#ifdef _WIN32
    return _kbhit();
#else
    struct pollfd fds;
    fds.fd = STDIN_FILENO;
    fds.events = POLLIN;
    return poll(&fds, 1, 0) == 1;
#endif
}
