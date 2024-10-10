#pragma once

#include <map>
#include <vector>
#include <openxr/openxr.h>
#include "xr_dependencies.h"
#include <openxr/openxr_platform.h>

class GLInterceptor {
private:
	std::map<XrSwapchain, std::vector<XrSwapchainImageOpenGLKHR>> m_swapchainImages;
	std::map<XrSwapchain, int> m_currentSwapChainImage;
public:
	GLInterceptor() {}
	XRAPI_ATTR XrResult XRAPI_CALL xrCreateSwapchain(XrSession session, const XrSwapchainCreateInfo* createInfo, XrSwapchain* swapchain);
	XRAPI_ATTR XrResult XRAPI_CALL xrAcquireSwapchainImage(XrSwapchain swapchain, const XrSwapchainImageAcquireInfo* acquireInfo, uint32_t* index);
	XRAPI_ATTR XrResult XRAPI_CALL xrReleaseSwapchainImage(XrSwapchain swapchain, const XrSwapchainImageReleaseInfo* releaseInfo);
};
