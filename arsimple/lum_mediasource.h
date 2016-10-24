#ifndef LUM_MEDIASOURCE_H
#define LUM_MEDIASOURCE_H
#include <vector>
#include <cstdint>

#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <atlbase.h>

namespace lum {
	// wraps Windows MediaFoundation
	class mediasource {
	public:
		mediasource();
		~mediasource();
		bool configure(int w, int h, const wchar_t* symname, int native_stream = -1);
		bool grab();
		// drop alpha
		void copyrgb(uint8_t* buff, int sz);
		void copy(uint8_t* buff, int sz);
		void* getPtr(int* len);
		void releasePtr();

		bool m_connected;
	private:
		int m_w;
		int m_h;
		CComPtr<IMFSourceReader> m_pReader;
		CComPtr<IMFMediaBuffer> m_pBuffer;
		std::vector<uint8_t> m_buffer;
	};

	HRESULT MediaSourceFromFriendly(const wchar_t* symbolicname, IMFMediaSource** ppSource);
	HRESULT MediaSourceFromSymbolic(const wchar_t* friendlyname, IMFMediaSource** ppSource);
}

#endif