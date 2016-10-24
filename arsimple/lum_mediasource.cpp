#include "lum_mediasource.h"
//#include "lum_imgproc.h"

#include <Mfapi.h>
#include <Mfidl.h>
#include <Mferror.h>
#include <Mfreadwrite.h>
#include <CommCtrl.h>

#include <cassert>

//#include "lum_win32util.h"

namespace lum {
	mediasource::mediasource() : m_w(0), m_h(0) {
	}
	mediasource::~mediasource() {
	}
	bool mediasource::configure(int w, int h, const wchar_t* symname, int native_stream) {
		m_w = 0;
		m_h = 0;
		CComPtr<IMFMediaSource> pInogeni = NULL;
		//HRESULT hr = MediaSourceFromFriendly(L"1838-INOGENI 4K2USB3", &pInogeni);
		HRESULT hr = MediaSourceFromSymbolic(symname, &pInogeni);
		if (FAILED(hr)) {
		    m_connected = false;
			return false;
		} else {
			m_connected = true;
		}

		if (native_stream >= 0) {
			/* Create SourceReader from Media Source */
			CComPtr<IMFAttributes> pAttributes = NULL;
			MFCreateAttributes(&pAttributes, 0);
			pAttributes->SetUINT32(MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING, 1);
			//pAttributes->SetUINT32(MF_READWRITE_ENABLE_HARDWARE_TRANSFORMS, 1);
			//pAttributes->SetUINT32(MF_LOW_LATENCY, 1);

			hr = MFCreateSourceReaderFromMediaSource(pInogeni, pAttributes, &m_pReader);
			assert(SUCCEEDED(hr));

			IMFMediaType* pType = NULL;
			hr = m_pReader->GetNativeMediaType(0, native_stream, &pType);
			assert(SUCCEEDED(hr));

			/* Get Current Media Type */
			hr = m_pReader->SetCurrentMediaType(0, NULL, pType);
			assert(SUCCEEDED(hr));
			if (SUCCEEDED(hr)) {
				m_w = w;
				m_h = h;
				return true;
			} else {
				return false;
			}
		} else {
			/* Create SourceReader from Media Source */
			CComPtr<IMFAttributes> pAttributes = NULL;
			MFCreateAttributes(&pAttributes, 3);
			pAttributes->SetUINT32(MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING, 1);
			pAttributes->SetUINT32(MF_READWRITE_ENABLE_HARDWARE_TRANSFORMS, 1);
			pAttributes->SetUINT32(MF_LOW_LATENCY, 1);

			hr = MFCreateSourceReaderFromMediaSource(pInogeni, pAttributes, &m_pReader);
			assert(SUCCEEDED(hr));

			/* Get Current Media Type */
			CComPtr<IMFMediaType> pType = NULL;
			hr = m_pReader->GetCurrentMediaType(0, &pType);
			assert(SUCCEEDED(hr));

			/* Set Desired Media Type
					TODO: Figure out why no native full hd resolution doesn't work. */
			UINT64 framesz = ((UINT64)w << 32) | h;
			hr = pType->SetUINT64(MF_MT_FRAME_SIZE, framesz);
			assert(SUCCEEDED(hr));
			// RGB24 returns weird 4:2:2 encoding.
			// RGB32 returns zero alpha channel...
			hr = pType->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_RGB32);
			assert(SUCCEEDED(hr));
			hr = m_pReader->SetCurrentMediaType(0, NULL, pType);

			if (SUCCEEDED(hr)) {
				m_w = w;
				m_h = h;
				return true;

			} else {
				return false;

			}
		}
	}
	bool mediasource::grab() {
		CComPtr<IMFSample> pSample = NULL;
		LONGLONG timestamp = 0;
		DWORD streamIndex = 0;
		DWORD streamFlags = 0;
		HRESULT hrStatus = 0;
		if (!m_pReader) {
			return false;
		}
		IMFMediaType* pType;
		m_pReader->GetCurrentMediaType(0, &pType);
		hrStatus = m_pReader->ReadSample(0, 0, &streamIndex, &streamFlags, &timestamp, &pSample);
		if (SUCCEEDED(hrStatus) && pSample) {
			m_pBuffer.Release();
			pSample->ConvertToContiguousBuffer(&m_pBuffer);
			return true;
		} else {
			return false;
		}
	}

	void mediasource::copyrgb(uint8_t* buff, int sz) {
		assert(sz == m_w * m_h * 3);

		DWORD curlen = 0;
		BYTE* pFrom = nullptr;
		DWORD len = 0;
		HRESULT hrStatus = m_pBuffer->Lock(&pFrom, &len, &curlen);
		assert(len == m_w * m_h * 4);
		// lum::convert_rgba_to_rgb(pFrom, buff, len);
		// only need single-channel depth.
		assert(false);
		m_pBuffer->Unlock();
	}
	void mediasource::copy(uint8_t* buff, int sz) {
		DWORD curlen = 0;
		BYTE* pFrom = nullptr;
		DWORD len = 0;
		HRESULT hrStatus = m_pBuffer->Lock(&pFrom, &len, &curlen);
		assert(len == m_w * m_h * 2); // this assumes 16-bit data - hackish
		assert(len == sz);
		std::copy(pFrom, pFrom + len, buff);
		// only need single-channel depth.
		m_pBuffer->Unlock();
	}
	void* mediasource::getPtr(int* plen) {
		DWORD curlen = 0;
		BYTE* pFrom = nullptr;
		DWORD len = 0;
		HRESULT hrStatus = m_pBuffer->Lock(&pFrom, &len, &curlen);
		assert(SUCCEEDED(hrStatus));
		for (uint32_t i = 0; i < len; ++i) {
			if (pFrom[i]) {
				//printf("Got depth data %d\n", pFrom[i]);
			}
		}
		if (plen) {
			*plen = len;
		}
		return pFrom;
	}
	void mediasource::releasePtr() {
		m_pBuffer->Unlock();
	}

	HRESULT MediaSourceFromSymbolic(const wchar_t* symbolicname, IMFMediaSource** ppSource) {
		CComPtr<IMFAttributes> pAttributes = NULL;
		HRESULT hr = MFCreateAttributes(&pAttributes, 2);
		if (FAILED(hr)) {
			return hr;
		}

		hr = pAttributes->SetString(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, symbolicname);
		if (FAILED(hr)) {
			return hr;
		}

		hr = pAttributes->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
		if (FAILED(hr)) {
			return hr;
		}

		hr = MFCreateDeviceSource(pAttributes, ppSource);
		return hr;
	}

	HRESULT MediaSourceFromFriendly(const wchar_t* friendlyname, IMFMediaSource** ppSource) {
		CComPtr<IMFAttributes> pConfig = NULL;
		IMFActivate** ppDevices = NULL;
		HRESULT hr = MFCreateAttributes(&pConfig, 1);
		UINT32 count = 0;

		// Request video capture devices.
		if (FAILED(hr)) {
			return hr;
		}
		hr = pConfig->SetGUID(
			MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
			MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID
			);
		if (FAILED(hr)) {
			return hr;
		}

		hr = MFEnumDeviceSources(pConfig, &ppDevices, &count);
		if (FAILED(hr)) {
			return hr;
		}

		// If enumeration succeeds, we must sure to deallocate
		// the received pointer. Hence the goto done calles.
		static const size_t c_buffsz = 160;
		wchar_t buff[c_buffsz];
		size_t correct_index = count;
		for (size_t i = 0; i < count; ++i) {
			IMFActivate* pa = ppDevices[i];
			UINT32 cchName;
#pragma warning(suppress: 6001)
			hr = pa->GetString(MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME, buff, c_buffsz, &cchName);
			if (FAILED(hr)) {
				goto done;
			}
			if (wcscmp(buff, friendlyname) == 0) {
				correct_index = i;
			}
		}

		if (correct_index != count) {
			hr = ppDevices[correct_index]->ActivateObject(IID_PPV_ARGS(ppSource));
			if (FAILED(hr)) {
				goto done;
			}
		} else {
			hr = E_FAIL;
		}

	done:
		for (DWORD i = 0; i < count; i++) {
			ppDevices[i]->Release();
		}
		CoTaskMemFree(ppDevices);
		return hr;
	}
}

