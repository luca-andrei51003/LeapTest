// Dear ImGui + DirectX11 dashboard for the Leap -> MAVLink drone controller.
// The control logic runs on a background thread (runControlLoop, in main.cpp)
// and publishes live values into a TelemetryState; this file just renders them.
//
// DX11/Win32 boilerplate adapted from
// third_party/imgui/examples/example_win32_directx11/main.cpp

#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx11.h"
#include <d3d11.h>
#include <tchar.h>

#include <atomic>
#include <thread>
#include <chrono>
#include <cstdio>
#include <cfloat>

#include "app.h"

// ---- Direct3D state ----
static ID3D11Device*            g_pd3dDevice = nullptr;
static ID3D11DeviceContext*     g_pd3dDeviceContext = nullptr;
static IDXGISwapChain*          g_pSwapChain = nullptr;
static bool                     g_SwapChainOccluded = false;
static UINT                     g_ResizeWidth = 0, g_ResizeHeight = 0;
static ID3D11RenderTargetView*  g_mainRenderTargetView = nullptr;

bool CreateDeviceD3D(HWND hWnd);
void CleanupDeviceD3D();
void CreateRenderTarget();
void CleanupRenderTarget();
LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

static long long nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// ---- Small UI helpers ----
static const ImVec4 kGreen(0.20f, 0.80f, 0.30f, 1.0f);
static const ImVec4 kRed  (0.90f, 0.25f, 0.25f, 1.0f);
static const ImVec4 kGray (0.55f, 0.55f, 0.55f, 1.0f);
static const ImVec4 kAmber(0.95f, 0.70f, 0.15f, 1.0f);

static void StatusDot(const char* label, bool ok, const char* onText = "ONLINE", const char* offText = "OFFLINE") {
    ImGui::TextColored(ok ? kGreen : kRed, ok ? "  *  " : "  -  ");
    ImGui::SameLine();
    ImGui::Text("%-22s", label);
    ImGui::SameLine();
    ImGui::TextColored(ok ? kGreen : kRed, "%s", ok ? onText : offText);
}

// Bidirectional bar centered at 0, value in [-maxAbs, +maxAbs].
static void SignedBar(const char* label, float value, float maxAbs, const char* fmt = "%6.1f") {
    float frac = value / maxAbs * 0.5f + 0.5f;
    frac = frac < 0.0f ? 0.0f : (frac > 1.0f ? 1.0f : frac);
    char overlay[48];
    snprintf(overlay, sizeof(overlay), fmt, value);
    ImGui::Text("%-8s", label);
    ImGui::SameLine();
    ImGui::ProgressBar(frac, ImVec2(-FLT_MIN, 0.0f), overlay);
}

int main(int, char**) {
    // ---- Start the control loop on its own thread ----
    TelemetryState state;
    std::atomic<bool> running{true};
    std::thread controlThread(runControlLoop, std::ref(state), std::ref(running), /*verbose=*/false);

    // ---- Window + Direct3D ----
    ImGui_ImplWin32_EnableDpiAwareness();
    float main_scale = ImGui_ImplWin32_GetDpiScaleForMonitor(::MonitorFromPoint(POINT{0, 0}, MONITOR_DEFAULTTOPRIMARY));

    WNDCLASSEXW wc = { sizeof(wc), CS_CLASSDC, WndProc, 0L, 0L, GetModuleHandle(nullptr), nullptr, nullptr, nullptr, nullptr, L"LeapDroneGui", nullptr };
    ::RegisterClassExW(&wc);
    HWND hwnd = ::CreateWindowW(wc.lpszClassName, L"Leap Drone Controller", WS_OVERLAPPEDWINDOW,
                                100, 100, (int)(620 * main_scale), (int)(720 * main_scale),
                                nullptr, nullptr, wc.hInstance, nullptr);

    if (!CreateDeviceD3D(hwnd)) {
        CleanupDeviceD3D();
        ::UnregisterClassW(wc.lpszClassName, wc.hInstance);
        running.store(false);
        if (controlThread.joinable()) controlThread.join();
        return 1;
    }

    ::ShowWindow(hwnd, SW_SHOWDEFAULT);
    ::UpdateWindow(hwnd);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.ScaleAllSizes(main_scale);
    style.FontScaleDpi = main_scale;

    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(g_pd3dDevice, g_pd3dDeviceContext);

    const ImVec4 clear_color = ImVec4(0.10f, 0.11f, 0.13f, 1.0f);

    bool done = false;
    while (!done) {
        MSG msg;
        while (::PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE)) {
            ::TranslateMessage(&msg);
            ::DispatchMessage(&msg);
            if (msg.message == WM_QUIT)
                done = true;
        }
        if (done)
            break;

        if (g_SwapChainOccluded && g_pSwapChain->Present(0, DXGI_PRESENT_TEST) == DXGI_STATUS_OCCLUDED) {
            ::Sleep(10);
            continue;
        }
        g_SwapChainOccluded = false;

        if (g_ResizeWidth != 0 && g_ResizeHeight != 0) {
            CleanupRenderTarget();
            g_pSwapChain->ResizeBuffers(0, g_ResizeWidth, g_ResizeHeight, DXGI_FORMAT_UNKNOWN, 0);
            g_ResizeWidth = g_ResizeHeight = 0;
            CreateRenderTarget();
        }

        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        // ---- Dashboard: one window filling the OS window ----
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(vp->WorkPos);
        ImGui::SetNextWindowSize(vp->WorkSize);
        ImGui::Begin("Leap Drone Controller", nullptr,
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);

        // Snapshot atomics once per frame.
        const bool  serialOpen   = state.serialOpen.load();
        const bool  leapConn     = state.leapConnected.load();
        const bool  vehHb        = state.vehicleHeartbeat.load();
        const bool  mainPresent  = state.mainHandPresent.load();
        const bool  altPresent   = state.altHandPresent.load();
        const bool  ctrlAlive    = running.load();
        const bool  tracking     = (nowMs() - state.lastFrameMs.load()) < 500;
        const float pitch        = state.pitch.load();
        const float roll         = state.roll.load();
        const float yaw          = state.yaw.load();
        const float thrustPct    = state.thrustPct.load();
        const float mainGrab     = state.mainGrab.load();
        const float altGrab      = state.altGrabAngle.load();
        const int   sx           = state.sentX.load();
        const int   sy           = state.sentY.load();
        const int   sz           = state.sentZ.load();
        const int   sr           = state.sentR.load();
        const bool  deadMan      = state.deadMan.load();

        ImGui::SeparatorText("Connection");
        StatusDot("Control loop", ctrlAlive, "RUNNING", "STOPPED");
        StatusDot("SiK radio (serial)", serialOpen);
        StatusDot("Leap connection", leapConn);
        StatusDot("Leap tracking", tracking, "TRACKING", "NO FRAMES");
        StatusDot("Vehicle heartbeat", vehHb, "LINKED", "NO LINK");

        ImGui::SeparatorText("Hands");
        StatusDot("Right hand (attitude)", mainPresent, "DETECTED", "MISSING");
        StatusDot("Left hand (thrust)", altPresent, "DETECTED", "MISSING");

        ImGui::SeparatorText("Mode");
        if (deadMan)
            ImGui::TextColored(kAmber, "DEAD-MAN ENGAGED  (right fist -> hover, sticks neutral)");
        else if (mainPresent)
            ImGui::TextColored(kGreen, "ACTIVE  (right hand controlling attitude)");
        else
            ImGui::TextColored(kGray, "IDLE  (no right hand)");

        ImGui::SeparatorText("Attitude (degrees)");
        SignedBar("Pitch", pitch, 90.0f);
        SignedBar("Roll",  roll,  90.0f);
        SignedBar("Yaw",   yaw,   180.0f);

        ImGui::SeparatorText("Thrust");
        {
            char ov[32];
            snprintf(ov, sizeof(ov), "%.1f %%", thrustPct);
            ImGui::Text("%-8s", "Thrust");
            ImGui::SameLine();
            ImGui::ProgressBar(thrustPct / 100.0f, ImVec2(-FLT_MIN, 0.0f), ov);
        }

        ImGui::SeparatorText("MANUAL_CONTROL output (sent to vehicle)");
        SignedBar("X pitch", (float)sx, 1000.0f, "%5.0f");
        SignedBar("Y roll",  (float)sy, 1000.0f, "%5.0f");
        SignedBar("R yaw",   (float)sr, 1000.0f, "%5.0f");
        {
            char ov[32];
            snprintf(ov, sizeof(ov), "%d", sz);
            ImGui::Text("%-8s", "Z thrust");
            ImGui::SameLine();
            ImGui::ProgressBar((float)sz / 1000.0f, ImVec2(-FLT_MIN, 0.0f), ov);
        }
        // Derived direction hint
        const char* dirX = sx < 0 ? "FORWARD" : (sx > 0 ? "BACKWARD" : "-");
        const char* dirY = sy < 0 ? "LEFT"    : (sy > 0 ? "RIGHT"    : "-");
        ImGui::Text("Direction:  %s   %s", dirX, dirY);

        ImGui::SeparatorText("Raw gesture inputs");
        ImGui::Text("Right grab strength : %.2f", mainGrab);
        ImGui::Text("Left  grab angle    : %.2f rad", altGrab);

        ImGui::SeparatorText("Health");
        ImGui::Text("Frames processed : %llu", (unsigned long long)state.framesProcessed.load());
        ImGui::Text("Packets sent     : %llu", (unsigned long long)state.packetsSent.load());
        ImGui::Text("GUI              : %.1f FPS (%.2f ms)", io.Framerate, 1000.0f / io.Framerate);

        if (!ctrlAlive) {
            ImGui::Spacing();
            ImGui::TextColored(kRed, "Control loop stopped. Check the console for the error");
            ImGui::TextColored(kRed, "(serial port busy/wrong COM, or Leap service not running).");
        }

        ImGui::End();

        ImGui::Render();
        const float cc[4] = { clear_color.x, clear_color.y, clear_color.z, clear_color.w };
        g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView, nullptr);
        g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView, cc);
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

        HRESULT hr = g_pSwapChain->Present(1, 0); // vsync
        g_SwapChainOccluded = (hr == DXGI_STATUS_OCCLUDED);
    }

    // ---- Stop the control loop and wait for it ----
    running.store(false);
    if (controlThread.joinable()) controlThread.join();

    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    CleanupDeviceD3D();
    ::DestroyWindow(hwnd);
    ::UnregisterClassW(wc.lpszClassName, wc.hInstance);
    return 0;
}

// ---- D3D helpers ----
bool CreateDeviceD3D(HWND hWnd) {
    DXGI_SWAP_CHAIN_DESC sd;
    ZeroMemory(&sd, sizeof(sd));
    sd.BufferCount = 2;
    sd.BufferDesc.Width = 0;
    sd.BufferDesc.Height = 0;
    sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferDesc.RefreshRate.Numerator = 60;
    sd.BufferDesc.RefreshRate.Denominator = 1;
    sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow = hWnd;
    sd.SampleDesc.Count = 1;
    sd.SampleDesc.Quality = 0;
    sd.Windowed = TRUE;
    sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

    UINT createDeviceFlags = 0;
    D3D_FEATURE_LEVEL featureLevel;
    const D3D_FEATURE_LEVEL featureLevelArray[2] = { D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_0 };
    HRESULT res = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, createDeviceFlags, featureLevelArray, 2, D3D11_SDK_VERSION, &sd, &g_pSwapChain, &g_pd3dDevice, &featureLevel, &g_pd3dDeviceContext);
    if (res == DXGI_ERROR_UNSUPPORTED)
        res = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_WARP, nullptr, createDeviceFlags, featureLevelArray, 2, D3D11_SDK_VERSION, &sd, &g_pSwapChain, &g_pd3dDevice, &featureLevel, &g_pd3dDeviceContext);
    if (res != S_OK)
        return false;

    CreateRenderTarget();
    return true;
}

void CleanupDeviceD3D() {
    CleanupRenderTarget();
    if (g_pSwapChain) { g_pSwapChain->Release(); g_pSwapChain = nullptr; }
    if (g_pd3dDeviceContext) { g_pd3dDeviceContext->Release(); g_pd3dDeviceContext = nullptr; }
    if (g_pd3dDevice) { g_pd3dDevice->Release(); g_pd3dDevice = nullptr; }
}

void CreateRenderTarget() {
    ID3D11Texture2D* pBackBuffer;
    g_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&pBackBuffer));
    g_pd3dDevice->CreateRenderTargetView(pBackBuffer, nullptr, &g_mainRenderTargetView);
    pBackBuffer->Release();
}

void CleanupRenderTarget() {
    if (g_mainRenderTargetView) { g_mainRenderTargetView->Release(); g_mainRenderTargetView = nullptr; }
}

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam))
        return true;

    switch (msg) {
    case WM_SIZE:
        if (wParam == SIZE_MINIMIZED)
            return 0;
        g_ResizeWidth = (UINT)LOWORD(lParam);
        g_ResizeHeight = (UINT)HIWORD(lParam);
        return 0;
    case WM_SYSCOMMAND:
        if ((wParam & 0xfff0) == SC_KEYMENU)
            return 0;
        break;
    case WM_DESTROY:
        ::PostQuitMessage(0);
        return 0;
    }
    return ::DefWindowProcW(hWnd, msg, wParam, lParam);
}
