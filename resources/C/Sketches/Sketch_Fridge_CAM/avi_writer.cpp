/**********************************************************************
  AVI MJPEG Writer for ESP32
  Writes a minimal AVI (RIFF) container with MJPEG video stream to SD.
  
  AVI structure:
    RIFF .... 'AVI '
      LIST .... 'hdrl'
        'avih' (main AVI header)
        LIST .... 'strl'
          'strh' (stream header — vids/MJPG)
          'strf' (stream format — BITMAPINFOHEADER)
      LIST .... 'movi'
        '00dc' frame0
        '00dc' frame1
        ...
      'idx1' (index)
**********************************************************************/
#include "avi_writer.h"
#include <Arduino.h>

// ---- Internal state ----
static File      _file;
static bool      _open = false;
static uint16_t  _width;
static uint16_t  _height;
static uint8_t   _fps;
static uint32_t  _frameCount;
static uint32_t  _totalFrameSize;  // sum of all JPEG payload sizes (without chunk headers)

// Positions we need to patch later
static uint32_t  _riffSizePos;     // offset to RIFF size field
static uint32_t  _avihFramePos;    // offset to dwTotalFrames in avih
static uint32_t  _strhFramePos;    // offset to dwLength in strh
static uint32_t  _moviSizePos;     // offset to movi LIST size field
static uint32_t  _moviStartPos;    // offset right after movi LIST size (start of movi data)

// In-memory index: offset from movi start, size of each frame chunk
struct FrameIdx {
  uint32_t offset;  // offset of '00dc' FourCC relative to movi start
  uint32_t size;    // JPEG payload size (without 8-byte chunk header)
};
static FrameIdx *_idx = NULL;

// ---- Helpers ----
static void writeFourCC(const char *cc) {
  _file.write((const uint8_t *)cc, 4);
}

static void writeU16(uint16_t v) {
  _file.write((const uint8_t *)&v, 2);
}

static void writeU32(uint32_t v) {
  _file.write((const uint8_t *)&v, 4);
}

static void writeZeros(size_t n) {
  uint8_t z = 0;
  for (size_t i = 0; i < n; i++) {
    _file.write(&z, 1);
  }
}

// ---- Public API ----

bool avi_start(fs::FS &fs, const char *path, uint16_t width, uint16_t height, uint8_t fps) {
  if (_open) return false;

  _file = fs.open(path, FILE_WRITE);
  if (!_file) {
    Serial.printf("[AVI] Failed to open %s\n", path);
    return false;
  }

  _width = width;
  _height = height;
  _fps = fps > 0 ? fps : 6;
  _frameCount = 0;
  _totalFrameSize = 0;

  // Allocate index
  _idx = (FrameIdx *)malloc(AVI_MAX_FRAMES * sizeof(FrameIdx));
  if (!_idx) {
    Serial.println("[AVI] Failed to allocate index");
    _file.close();
    return false;
  }

  uint32_t usPerFrame = 1000000 / _fps;

  // ==== RIFF header ====
  writeFourCC("RIFF");
  _riffSizePos = _file.position();
  writeU32(0);            // placeholder — patched in avi_end()
  writeFourCC("AVI ");

  // ==== LIST hdrl ====
  writeFourCC("LIST");
  uint32_t hdrlSizePos = _file.position();
  writeU32(0);            // placeholder
  writeFourCC("hdrl");
  uint32_t hdrlStart = _file.position();

  // ---- avih (main AVI header, 56 bytes) ----
  writeFourCC("avih");
  writeU32(56);                   // chunk size
  writeU32(usPerFrame);           // dwMicroSecPerFrame
  writeU32(0);                    // dwMaxBytesPerSec (0 = not specified)
  writeU32(0);                    // dwPaddingGranularity
  writeU32(0x10);                 // dwFlags: AVIF_HASINDEX
  _avihFramePos = _file.position();
  writeU32(0);                    // dwTotalFrames — patched later
  writeU32(0);                    // dwInitialFrames
  writeU32(1);                    // dwStreams
  writeU32(0);                    // dwSuggestedBufferSize
  writeU32(_width);               // dwWidth
  writeU32(_height);              // dwHeight
  writeZeros(16);                 // dwReserved[4]

  // ---- LIST strl ----
  writeFourCC("LIST");
  uint32_t strlSizePos = _file.position();
  writeU32(0);                    // placeholder
  writeFourCC("strl");
  uint32_t strlStart = _file.position();

  // ---- strh (stream header, 56 bytes) ----
  writeFourCC("strh");
  writeU32(56);                   // chunk size
  writeFourCC("vids");            // fccType
  writeFourCC("MJPG");            // fccHandler
  writeU32(0);                    // dwFlags
  writeU16(0);                    // wPriority
  writeU16(0);                    // wLanguage
  writeU32(0);                    // dwInitialFrames
  writeU32(1);                    // dwScale
  writeU32(_fps);                 // dwRate
  writeU32(0);                    // dwStart
  _strhFramePos = _file.position();
  writeU32(0);                    // dwLength — patched later
  writeU32(0);                    // dwSuggestedBufferSize
  writeU32((uint32_t)-1);         // dwQuality
  writeU32(0);                    // dwSampleSize
  writeU16(0);                    // rcFrame left
  writeU16(0);                    // rcFrame top
  writeU16(_width);               // rcFrame right
  writeU16(_height);              // rcFrame bottom

  // ---- strf (stream format — BITMAPINFOHEADER, 40 bytes) ----
  writeFourCC("strf");
  writeU32(40);                   // chunk size
  writeU32(40);                   // biSize
  writeU32(_width);               // biWidth
  writeU32(_height);              // biHeight
  writeU16(1);                    // biPlanes
  writeU16(24);                   // biBitCount
  writeFourCC("MJPG");            // biCompression
  writeU32(_width * _height * 3); // biSizeImage
  writeU32(0);                    // biXPelsPerMeter
  writeU32(0);                    // biYPelsPerMeter
  writeU32(0);                    // biClrUsed
  writeU32(0);                    // biClrImportant

  // Patch strl LIST size
  uint32_t strlEnd = _file.position();
  _file.seek(strlSizePos);
  writeU32(strlEnd - strlStart - 4 + 4); // size = content + 'strl' fourcc
  _file.seek(strlEnd);

  // Patch hdrl LIST size
  uint32_t hdrlEnd = _file.position();
  _file.seek(hdrlSizePos);
  writeU32(hdrlEnd - hdrlStart - 4 + 4); // size = content + 'hdrl' fourcc
  _file.seek(hdrlEnd);

  // ==== LIST movi ====
  writeFourCC("LIST");
  _moviSizePos = _file.position();
  writeU32(0);            // placeholder — patched in avi_end()
  writeFourCC("movi");
  _moviStartPos = _file.position();

  _open = true;
  Serial.printf("[AVI] Started: %s (%ux%u @ %u fps)\n", path, _width, _height, _fps);
  return true;
}

bool avi_add_frame(const uint8_t *jpg_buf, size_t jpg_len) {
  if (!_open || !_file) return false;
  if (_frameCount >= AVI_MAX_FRAMES) return false;

  uint32_t chunkOffset = _file.position() - _moviStartPos;

  // Write '00dc' chunk header
  writeFourCC("00dc");
  uint32_t paddedLen = (jpg_len + 1) & ~1; // AVI chunks must be 16-bit aligned
  writeU32(jpg_len);
  _file.write(jpg_buf, jpg_len);

  // Pad to even boundary
  if (jpg_len & 1) {
    uint8_t z = 0;
    _file.write(&z, 1);
  }

  // Record index entry
  _idx[_frameCount].offset = chunkOffset;
  _idx[_frameCount].size = jpg_len;
  _frameCount++;
  _totalFrameSize += jpg_len;

  return true;
}

bool avi_end() {
  if (!_open || !_file) return false;

  uint32_t moviEndPos = _file.position();

  // ==== Write idx1 ====
  writeFourCC("idx1");
  writeU32(_frameCount * 16);  // index chunk size

  for (uint32_t i = 0; i < _frameCount; i++) {
    writeFourCC("00dc");               // ckid
    writeU32(0x10);                    // dwFlags: AVIIF_KEYFRAME
    writeU32(_idx[i].offset);          // dwOffset (relative to movi start)
    writeU32(_idx[i].size);            // dwSize
  }

  uint32_t fileEnd = _file.position();

  // ==== Patch sizes ====
  // RIFF size = fileEnd - 8
  _file.seek(_riffSizePos);
  writeU32(fileEnd - 8);

  // avih dwTotalFrames
  _file.seek(_avihFramePos);
  writeU32(_frameCount);

  // strh dwLength
  _file.seek(_strhFramePos);
  writeU32(_frameCount);

  // movi LIST size
  _file.seek(_moviSizePos);
  writeU32(moviEndPos - _moviStartPos + 4); // +4 for 'movi' fourcc

  _file.close();
  _open = false;

  if (_idx) {
    free(_idx);
    _idx = NULL;
  }

  Serial.printf("[AVI] Finished: %u frames, %u bytes\n", _frameCount, fileEnd);
  return true;
}

bool avi_is_open() {
  return _open;
}

uint32_t avi_frame_count() {
  return _frameCount;
}
