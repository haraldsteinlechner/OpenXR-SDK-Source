#include "simulatedLayer.h"
#include <openxr/openxr_platform.h>
#include <tests/hello_xr/check.h>
#include <GL/GL.h>


#include <common/gfxwrapper_opengl.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

static bool useInterception = true;



XRAPI_ATTR XrResult XRAPI_CALL GLInterceptor::xrCreateSwapchain(XrSession session, const XrSwapchainCreateInfo* createInfo, XrSwapchain* swapchain) {
	auto result = ::xrCreateSwapchain(session, createInfo, swapchain);
	if (useInterception)
	{
		uint32_t imageCount;
		CHECK_XRCMD(xrEnumerateSwapchainImages(*swapchain, 0, &imageCount, nullptr));
		std::vector<XrSwapchainImageOpenGLKHR> swapchainImageBuffer(imageCount, { XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR });
		std::vector<XrSwapchainImageBaseHeader*> swapchainImageBase;
		for (XrSwapchainImageOpenGLKHR& image : swapchainImageBuffer) {
			swapchainImageBase.push_back(reinterpret_cast<XrSwapchainImageBaseHeader*>(&image));
		}
		CHECK_XRCMD(xrEnumerateSwapchainImages(*swapchain, imageCount, &imageCount, swapchainImageBase[0]));

		m_swapchainImages.insert(std::make_pair(*swapchain, std::move(swapchainImageBuffer)));
	}
	return result;
}


XRAPI_ATTR XrResult XRAPI_CALL GLInterceptor::xrAcquireSwapchainImage(XrSwapchain swapchain, const XrSwapchainImageAcquireInfo* acquireInfo, uint32_t* index) {
	XrResult res = ::xrAcquireSwapchainImage(swapchain, acquireInfo, index);
	m_currentSwapChainImage.insert_or_assign(swapchain, *index);
	return res;
}
XRAPI_ATTR XrResult XRAPI_CALL GLInterceptor::xrReleaseSwapchainImage(XrSwapchain swapchain, const XrSwapchainImageReleaseInfo* releaseInfo) {
	XrResult res = ::xrReleaseSwapchainImage(swapchain, releaseInfo);
	auto images = m_swapchainImages[swapchain];
	auto currentImage = m_currentSwapChainImage[swapchain];
	XrSwapchainImageOpenGLKHR& image = images[m_currentSwapChainImage[swapchain]];
	//glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, image.image, 0);
	glBindTexture(GL_TEXTURE_2D, image.image);
	int width; 
	int height;
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width);
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height);
	std::vector<unsigned char> frameData(width * height * 4);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, &frameData[0]);

    char buff[256]; // testing only
	sprintf(buff, "C:\\temp\\img_%d_%d.jpg", (int)swapchain, currentImage);
	stbi_write_jpg((const char*)buff, width, height, 4, &frameData[0], 100);
	return res;
}
