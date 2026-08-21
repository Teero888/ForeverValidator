#include "format/pack/installed/plug_file_pack.h"
#include <climits>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <openssl/crypto.h>
#include <openssl/evp.h>
#include <zlib.h>

#include "format/archive/classic_buffer_crypted.h"
#include "format/archive/archive_binary.h"
#include "format/archive/archive_class_ids.h"
#include "format/pack/installed/installed_pack_key_catalog.h"
#include "format/pack/installed/scene_descriptor_folder_paths.h"
#include "format/pack/installed/plug_file_pack_crypto_constants.h"
#include "format/static_solid/static_scene_archive_loader.h"
#include "format/static_solid/static_solid_archive_payload_decoder.h"
#include <new>

static size_t pack_cstring_length(const char *text) {
    size_t length = 0u;
    while (text[length] != 0) {
        length++;
    }
    return length;
}

int CommitArchiveFeedbackExtraction(
        ByteBuffer &&decoded,
        int readerError,
        ByteBuffer *out) {
    if (out == nullptr) {
        return 0;
    }
    out->Clear();
    if (readerError != 0) {
        return 0;
    }
    *out = std::move(decoded);
    return 1;
}

static int pack_strings_equal(const char *lhs, const char *rhs) {
    if (lhs == nullptr || rhs == nullptr) {
        return lhs == rhs;
    }
    size_t index = 0u;
    for (;;) {
        if (lhs[index] != rhs[index]) {
            return 0;
        }
        if (lhs[index] == 0) {
            return 1;
        }
        index++;
    }
}

static int pack_file_path_matches_reverse(
        const CPlugFilePack &pack,
        const CPlugFileFidContainer_SFileDesc &file,
        const char *selectedPath,
        size_t selectedPathLength) {
    size_t remaining = selectedPathLength;
    const char *fileName = file.name.c_str();
    const size_t fileNameLength = pack_cstring_length(fileName);
    if (fileNameLength > remaining) {
        return 0;
    }
    remaining -= fileNameLength;
    if (std::memcmp(selectedPath + remaining,
                    fileName,
                    fileNameLength) != 0) {
        return 0;
    }

    u32 folderIndex = file.folderIndex;
    size_t visitedFolderCount = 0u;
    while (folderIndex != 0xffffffffu) {
        if (static_cast<size_t>(folderIndex) >= pack.folders.size() ||
            visitedFolderCount++ >= pack.folders.size()) {
            return 0;
        }
        const CPlugFilePackFolderDesc &folder = pack.folders[folderIndex];
        const char *folderName = folder.name.c_str();
        const size_t folderNameLength = pack_cstring_length(folderName);
        if (folderNameLength > remaining) {
            return 0;
        }
        remaining -= folderNameLength;
        if (std::memcmp(selectedPath + remaining,
                        folderName,
                        folderNameLength) != 0) {
            return 0;
        }
        folderIndex = folder.parent;
    }
    return remaining == 0u;
}

static void copy_pack_bytes(unsigned char *dst,
                            const unsigned char *src,
                            size_t count) {
    for (size_t i = 0u; i < count; i++) {
        dst[i] = src[i];
    }
}

static void copy_pack_chars(char *dst, const char *src, size_t count) {
    for (size_t i = 0u; i < count; i++) {
        dst[i] = src[i];
    }
}

static int copy_cstr_to_fixed(char *dst, size_t dstSize, const char *src) {
    if (dst == nullptr || dstSize == 0u || src == nullptr) {
        return 0;
    }
    const size_t len = pack_cstring_length(src);
    if (len >= dstSize) {
        return 0;
    }
    copy_pack_chars(dst, src, len + 1u);
    return 1;
}

int CPlugFileFidContainer_SFileDesc::IsCompressed() const {
    return (flags & CompressionFlagsMask) != 0u;
}

int CPlugFileFidContainer_SFileDesc::IsPublicFile() const {
    return (flags & PublicFileFlag) != 0u;
}

int CPlugFileFidContainer_SFileDesc::ForceNoCrypt() const {
    return (flags & ForceNoCryptFlag) != 0u;
}

int CPlugFileFidContainer_SFileDesc::IsEncryptedPayload() const {
    return !(IsPublicFile() || ForceNoCrypt());
}

u32 CPlugFileFidContainer_SFileDesc::PackedPayloadByteCount() const {
    return IsCompressed() ? compressedSize : uncompressedSize;
}

int CPlugFileFidContainer_SFileDesc::IsDefaultVehicleTuningsPath(
        const char *selectedPath) const {
    return selectedPath != nullptr &&
           pack_strings_equal(selectedPath, TmnfDefaultVehicleTuningsPackPath);
}

int CPlugFileFidContainer_SFileDesc::ShouldTryStreamFeedbackExtraction(
        const char *selectedPath) const {
    const int isDefaultVehicleTunings =
            IsDefaultVehicleTuningsPath(selectedPath);
    const int isVehicleTunings =
            classId == TMNF_CLASS_CSceneVehicleTunings;
    const int isVehicleStruct =
            classId == TMNF_CLASS_CSceneVehicleStruct;
    const int isMaterial =
            classId == TMNF_CLASS_CPlugMaterial;
    const int isShaderApply =
            classId == TMNF_CLASS_CPlugShaderApply;
    const int isBitmap = classId == TMNF_CLASS_CPlugBitmap;
    const int isZoneDescriptor =
            classId == TMNF_CLASS_CGameCtnZoneFlat ||
            classId == TMNF_CLASS_CGameCtnZoneFrontier;
    const int streamFeedbackDescriptor =
            selectedPath != nullptr &&
            (SceneDescriptorFolderPaths::IsBlockInfoDescriptorPath(
                     selectedPath) ||
             SceneDescriptorFolderPaths::IsZoneDescriptorPath(selectedPath) ||
             SceneDescriptorFolderPaths::IsMobilDescriptorPath(selectedPath) ||
             SceneDescriptorFolderPaths::IsMediaSolidPath(
                     selectedPath) ||
             isVehicleTunings || isVehicleStruct || isZoneDescriptor ||
             isMaterial || isShaderApply || isBitmap ||
             isDefaultVehicleTunings);
    return (IsEncryptedPayload() || isVehicleTunings ||
            isDefaultVehicleTunings) &&
           streamFeedbackDescriptor;
}

struct CPlugFilePackLoadHeadersReader {
    std::unique_ptr<CClassicBufferCrypted> crypted;
    ByteBuffer plainHeader;

    int Read(void *out, size_t byteCount);
    int ReadU32(u32 *out);
    int ReadU64(uint64_t *out);
    int ReadString(std::string *out);
};

static int Md5Bytes(const unsigned char *data,
                           size_t byteCount,
                           unsigned char digest[16]) {
    EVP_MD_CTX *ctx = EVP_MD_CTX_new();
    unsigned int digestSize = 0u;
    if (ctx == nullptr) {
        return 0;
    }
    const int ok =
            EVP_DigestInit_ex(ctx, EVP_md5(), nullptr) == 1 &&
            EVP_DigestUpdate(ctx, data, byteCount) == 1 &&
            EVP_DigestFinal_ex(ctx, digest, &digestSize) == 1 &&
            digestSize == 16u;
    EVP_MD_CTX_free(ctx);
    return ok;
}

static int Md5CString(const char *text, unsigned char digest[16]) {
    return text != nullptr &&
           Md5Bytes((const unsigned char *)text, pack_cstring_length(text), digest);
}

int CPlugFilePack::ComputeKey(const char *keyString, SNat128 &keyOut) {
    char seed[128];
    unsigned char digest[16];
    if (keyString == nullptr) {
        return 0;
    }
    const int seedSize =
            snprintf(seed, sizeof(seed), "%sNadeoPak", keyString);
    if (seedSize < 0 || seedSize >= (int)sizeof(seed) ||
        !Md5CString(seed, digest)) {
        return 0;
    }
    for (u32 wordIndex = 0u; wordIndex < keyOut.words.size(); ++wordIndex) {
        const u32 offset = wordIndex * 4u;
        keyOut.words[wordIndex] =
                (static_cast<u32>(digest[offset]) << 24u) |
                (static_cast<u32>(digest[offset + 1u]) << 16u) |
                (static_cast<u32>(digest[offset + 2u]) << 8u) |
                static_cast<u32>(digest[offset + 3u]);
    }
    return 1;
}

static int inflate_static_solid_part(const unsigned char *input,
                                     u32 inputByteCount,
                                     unsigned char *output,
                                     u32 outputByteCount) {
    if (input == nullptr || (output == nullptr && outputByteCount != 0u)) {
        return 0;
    }
    unsigned char emptyOutput = 0u;
    uLongf actualOutputByteCount = outputByteCount;
    uLong actualInputByteCount = inputByteCount;
    return uncompress2(
                   outputByteCount != 0u ? output : &emptyOutput,
                   &actualOutputByteCount,
                   input,
                   &actualInputByteCount) == Z_OK &&
           actualOutputByteCount == static_cast<uLongf>(outputByteCount) &&
           actualInputByteCount == static_cast<uLong>(inputByteCount);
}

CPlugFilePack::CPlugFilePack() {
    key = {};
    dataStart = 0u;
}

CPlugFilePack::~CPlugFilePack() {
    FreeLoadedPack();
}

int CPlugFilePack::IsCrypted(void) const {
    return !key.IsNull();
}

const std::string &CPlugFilePack::PackName(void) const {
    return packName_;
}

int CPlugFilePackLoadHeadersReader::Read(void *out, size_t requestedByteCount) {
    return crypted != nullptr && out != nullptr &&
           requestedByteCount <= UINT32_MAX &&
           crypted->Read(out,
                         static_cast<unsigned long>(requestedByteCount)) ==
                   requestedByteCount &&
           plainHeader.Append(out, requestedByteCount);
}

int CPlugFilePackLoadHeadersReader::ReadU32(u32 *out) {
    unsigned char wordBytes[4];
    if (out == nullptr || !Read(wordBytes, sizeof(wordBytes))) {
        return 0;
    }
    *out = TmnfFormat::ArchiveBinary::ReadU32LE(wordBytes);
    return 1;
}

int CPlugFilePackLoadHeadersReader::ReadU64(uint64_t *out) {
    unsigned char qwordBytes[8];
    if (out == nullptr || !Read(qwordBytes, sizeof(qwordBytes))) {
        return 0;
    }
    *out = TmnfFormat::ArchiveBinary::ReadU64LE(qwordBytes);
    return 1;
}

int CPlugFilePackLoadHeadersReader::ReadString(std::string *out) {
    u32 byteCount = 0u;
    if (out == nullptr ||
        !ReadU32(&byteCount) ||
        byteCount > 0x0000ffffu) {
        return 0;
    }
    try {
        out->assign(byteCount, '\0');
    } catch (const std::bad_alloc &) {
        return 0;
    }
    if (byteCount != 0u &&
        !Read(&(*out)[0], byteCount)) {
        return 0;
    }
    return 1;
}

void CPlugFilePack::FreeLoadedPack() {
    folders.clear();
    files.clear();
    bytes.Clear();
    key = {};
    dataStart = 0u;
    packName_.clear();
}

int CPlugFilePack::LoadHeadersFromMemory() {
    if (bytes.Size() < 20u || bytes.Size() > UINT32_MAX) {
        return 0;
    }
    CPlugFilePackLoadHeadersReader reader;
    reader.crypted = CClassicBufferCrypted::CreateBlowfishReaderForMemory(
            bytes.Data(),
            static_cast<unsigned long>(bytes.Size()),
            20ul,
            key);
    if (reader.crypted == nullptr) {
        return 0;
    }
    unsigned char discard16[16];
    u32 discard = 0u;
    u32 loadedFolderCount = 0u;
    if (!reader.Read(discard16, sizeof(discard16)) ||
        !reader.ReadU32(&discard) ||
        !reader.ReadU32(&dataStart) ||
        !reader.ReadU32(&discard) ||
        !reader.ReadU32(&discard) ||
        !reader.Read(discard16, sizeof(discard16)) ||
        !reader.ReadU32(&discard) ||
        !reader.ReadU32(&loadedFolderCount) ||
        loadedFolderCount > 4096u) {
        return 0;
    }
    try {
        folders.clear();
        folders.resize(loadedFolderCount);
    } catch (const std::bad_alloc &) {
        return 0;
    }
    for (u32 i = 0u; i < loadedFolderCount; i++) {
        if (!reader.ReadU32(&folders[i].parent) ||
            !reader.ReadString(&folders[i].name) ||
            (folders[i].parent != 0xffffffffu &&
             (folders[i].parent >= loadedFolderCount ||
              folders[i].parent == i))) {
            return 0;
        }
    }
    if (loadedFolderCount > 2u && folders[2].name.size() > 4u) {
        unsigned char utf16[8] = {0};
        for (u32 i = 0u; i < 4u; i++) {
            utf16[i * 2u] = (unsigned char)folders[2].name[i + 2u];
        }
        reader.crypted->MixPageFeedback(utf16, 4ul);
    }
    u32 loadedFileCount = 0u;
    if (!reader.ReadU32(&loadedFileCount) ||
        loadedFileCount > 200000u) {
        return 0;
    }
    try {
        files.clear();
        files.resize(loadedFileCount);
    } catch (const std::bad_alloc &) {
        return 0;
    }
    for (u32 i = 0u; i < loadedFileCount; i++) {
        if (!reader.ReadU32(&files[i].folderIndex) ||
            !reader.ReadString(&files[i].name) ||
            !reader.ReadU32(&discard) ||
            !reader.ReadU32(&files[i].uncompressedSize) ||
            !reader.ReadU32(&files[i].compressedSize) ||
            !reader.ReadU32(&files[i].offsetRel) ||
            !reader.ReadU32(&files[i].classId) ||
            !reader.ReadU64(&files[i].flags) ||
            (files[i].folderIndex != 0xffffffffu &&
             files[i].folderIndex >= loadedFolderCount)) {
            return 0;
        }
    }
    if (reader.crypted->Error() || dataStart > bytes.Size() ||
        dataStart < 20u ||
        reader.plainHeader.Size() > dataStart - 20u ||
        reader.plainHeader.Size() < 16u) {
        return 0;
    }
    unsigned char expectedDigest[16];
    copy_pack_bytes(
            expectedDigest, reader.plainHeader.Data(), sizeof(expectedDigest));
    for (std::size_t index = 0u; index < sizeof(expectedDigest); ++index) {
        reader.plainHeader.Data()[index] = 0u;
    }
    unsigned char computedDigest[16];
    return Md5Bytes(
                   reader.plainHeader.Data(),
                   reader.plainHeader.Size(),
                   computedDigest) &&
           CRYPTO_memcmp(
                   expectedDigest, computedDigest, sizeof(expectedDigest)) == 0;
}

int CPlugFilePack::OpenFromMemory(
        const void *pakBytes,
        size_t pakByteCount,
        const void *packlistBytes,
        size_t packlistByteCount) {
    return OpenFromMemory(
            pakBytes,
            pakByteCount,
            packlistBytes,
            packlistByteCount,
            "Stadium");
}

int CPlugFilePack::OpenFromMemory(
        const void *pakBytes,
        size_t pakByteCount,
        const void *packlistBytes,
        size_t packlistByteCount,
        const char *packName) {
    if ((pakBytes == nullptr && pakByteCount != 0u) ||
        (packlistBytes == nullptr && packlistByteCount != 0u)) {
        return 0;
    }
    InstalledPackKeyCatalog keyCatalog;
    if (!keyCatalog.LoadFromMemory(packlistBytes, packlistByteCount)) {
        FreeLoadedPack();
        return 0;
    }
    return OpenFromMemory(pakBytes, pakByteCount, keyCatalog, packName);
}

int CPlugFilePack::OpenFromMemory(
        const void *pakBytes,
        size_t pakByteCount,
        const InstalledPackKeyCatalog &keyCatalog,
        const char *packName) {
    FreeLoadedPack();
    if (pakBytes == nullptr || pakByteCount < 20u ||
        std::memcmp(pakBytes, "NadeoPak", 8u) != 0 ||
        TmnfFormat::ArchiveBinary::ReadU32LE(
                static_cast<const unsigned char *>(pakBytes) + 8u) != 3u) {
        return 0;
    }
    const InstalledPackKeyEntry *entry = keyCatalog.Find(packName);
    if (entry == nullptr ||
        !ComputeKey(entry->keyString.c_str(), key) ||
        !bytes.Append(pakBytes, pakByteCount) ||
        !LoadHeadersFromMemory()) {
        FreeLoadedPack();
        return 0;
    }
    try {
        packName_ = entry->name;
        if (!packName_.empty() && packName_[0] >= 'a' &&
            packName_[0] <= 'z') {
            packName_[0] = static_cast<char>(
                    packName_[0] - 'a' + 'A');
        }
    } catch (const std::bad_alloc &) {
        FreeLoadedPack();
        return 0;
    }
    return 1;
}

int CPlugFilePack::FolderPathRecursive(
        u32 folderIndex,
        char *out,
        size_t outSize) const {
    if (out == nullptr || outSize == 0u ||
        (size_t)folderIndex >= folders.size()) {
        return 0;
    }
    const CPlugFilePackFolderDesc *folder = &folders[folderIndex];
    if (folder->parent != 0xffffffffu &&
        !FolderPathRecursive(folder->parent, out, outSize)) {
        return 0;
    }
    return SceneDescriptorFolderPaths::AppendCString(
            out, outSize, folder->name.c_str());
}

int CPlugFilePack::FileDescPlainPath(
        const CPlugFileFidContainer_SFileDesc *file,
        char *out,
        size_t outSize) const {
    if (file == nullptr || out == nullptr || outSize == 0u) {
        return 0;
    }
    out[0] = '\0';
    return (file->folderIndex == 0xffffffffu ||
            FolderPathRecursive(file->folderIndex, out, outSize)) &&
           SceneDescriptorFolderPaths::AppendCString(
                   out, outSize, file->name.c_str());
}

int CPlugFilePack::FileDescPathMatches(
        const CPlugFileFidContainer_SFileDesc *file,
        const char *selectedPath) const {
    char path[512];
    if (file == nullptr || selectedPath == nullptr) {
        return 0;
    }
    return FileDescPlainPath(file, path, sizeof(path)) &&
           pack_strings_equal(path, selectedPath);
}

const CPlugFileFidContainer_SFileDesc *CPlugFilePack::FindFileDescByPath(
        const char *selectedPath) const {
    if (selectedPath == nullptr || selectedPath[0] == '\0') {
        return nullptr;
    }
    const size_t selectedPathLength = pack_cstring_length(selectedPath);
    // FileDescPathMatches builds into a 512-byte local buffer, so a path that
    // does not fit can never match there either.
    if (selectedPathLength >= 512u) {
        return nullptr;
    }
    for (const CPlugFileFidContainer_SFileDesc &file : files) {
        if (pack_file_path_matches_reverse(
                    *this, file, selectedPath, selectedPathLength)) {
            return &file;
        }
    }
    return nullptr;
}

int CPlugFilePack::SelectedPathForPlainRef(
        const char *plainPath,
        char *out,
        size_t outSize) const {
    if (plainPath == nullptr || plainPath[0] == '\0' ||
        out == nullptr || outSize == 0u) {
        return 0;
    }
    out[0] = '\0';
    if (FindFileDescByPath(plainPath) != nullptr) {
        return copy_cstr_to_fixed(out, outSize, plainPath);
    }
    char hashedPath[512];
    hashedPath[0] = '\0';
    if (!SceneDescriptorFolderPaths::HashPackSelectedPath(
                plainPath,
                hashedPath,
                sizeof(hashedPath)) ||
        FindFileDescByPath(hashedPath) == nullptr) {
        return 0;
    }
    return copy_cstr_to_fixed(out, outSize, hashedPath);
}

int CPlugFilePack::PayloadOffset(
        const CPlugFileFidContainer_SFileDesc &file,
        size_t *payloadOffsetOut) const {
    if (payloadOffsetOut == nullptr ||
        dataStart > UINT32_MAX - file.offsetRel) {
        return 0;
    }
    const size_t payloadOffset = (size_t)dataStart + file.offsetRel;
    if (payloadOffset > bytes.Size()) {
        return 0;
    }
    *payloadOffsetOut = payloadOffset;
    return 1;
}

int CPlugFilePack::ReadPackedPayload(
        const CPlugFileFidContainer_SFileDesc &file,
        size_t payloadOffset,
        u32 payloadByteCount,
        unsigned char *payloadOut) const {
    if ((payloadOut == nullptr && payloadByteCount != 0u) ||
        payloadOffset > bytes.Size() ||
        payloadByteCount > bytes.Size() - payloadOffset) {
        return 0;
    }
    if (file.IsEncryptedPayload()) {
        if (payloadByteCount > UINT32_MAX - 7u) {
            return 0;
        }
        const size_t encryptedByteCount =
                (static_cast<size_t>(payloadByteCount) + 7u) & ~size_t{7u};
        if (encryptedByteCount > UINT32_MAX - 8u ||
            8u + encryptedByteCount > bytes.Size() - payloadOffset) {
            return 0;
        }
        auto reader = CClassicBufferCrypted::CreateBlowfishReaderForMemory(
                bytes.Data() + payloadOffset,
                static_cast<unsigned long>(8u + encryptedByteCount),
                8ul,
                key);
        return reader != nullptr &&
               reader->Read(payloadOut, payloadByteCount) ==
                       payloadByteCount;
    }
    if (payloadByteCount != 0u) {
        copy_pack_bytes(payloadOut, bytes.Data() + payloadOffset, payloadByteCount);
    }
    return 1;
}

int CPlugFilePack::ExtractPathWithPackedPayload(
        const CPlugFileFidContainer_SFileDesc &file,
        ByteBuffer *out) const {
    if (out == nullptr) {
        return 0;
    }
    const u32 payloadByteCount = file.PackedPayloadByteCount();
    size_t payloadOffset = 0u;
    if (!PayloadOffset(file, &payloadOffset) ||
        payloadByteCount > bytes.Size() - payloadOffset) {
        return 0;
    }
    std::vector<unsigned char> payload(payloadByteCount);
    if (!ReadPackedPayload(file,
                           payloadOffset,
                           payloadByteCount,
                           payloadByteCount != 0u ? payload.data() : nullptr)) {
        return 0;
    }
    out->Clear();
    if (file.IsCompressed()) {
        if (!out->Resize(file.uncompressedSize)) {
            return 0;
        }
        if (!inflate_static_solid_part(payloadByteCount != 0u
                                               ? payload.data()
                                               : nullptr,
                                       payloadByteCount,
                                       out->Data(),
                                       file.uncompressedSize)) {
            out->Clear();
            return 0;
        }
    } else {
        if (!out->Resize(payloadByteCount)) {
            return 0;
        }
        if (payloadByteCount != 0u) {
            copy_pack_bytes(out->Data(), payload.data(), payloadByteCount);
        }
    }
    return 1;
}

int CPlugFilePack::ExtractPathWithStreamFeedback(
        const CPlugFileFidContainer_SFileDesc *file,
        const char *selectedPath,
        ByteBuffer *out) const {
    if (out != nullptr) {
        out->Clear();
    }
    if (file == nullptr || out == nullptr) {
        return 0;
    }
    StaticSolidArchivePayload payload;
    if (!payload.BuildEncryptedPackFilePayload({
                *file,
                dataStart,
                0u,
                0u,
                "",
                selectedPath != nullptr ? selectedPath : ""}) ||
        static_cast<size_t>(payload.PackPayloadOffset()) > bytes.Size() ||
        payload.RawByteCount() >
                bytes.Size() -
                        static_cast<size_t>(payload.PackPayloadOffset())) {
        return 0;
    }

    CGameCtnReplayStaticSolidDecodedPayload decoded;
    CGameCtnReplayStaticSolidArchiveDecodeStats stats;
    if (!CGameCtnReplayStaticSolidArchivePayloadReader::DecodeWithStreamFeedback(
                key,
                payload,
                StaticSolidArchiveRawBytes::FromBytes(
                        bytes.Data() + payload.PackPayloadOffset(),
                        payload.RawByteCount()),
                nullptr,
                nullptr,
                StaticSolidArchiveId::Invalid(),
                &decoded,
                &stats) ||
        !decoded.IsReady() || decoded.ByteCount() != file->uncompressedSize) {
        return 0;
    }
    return decoded.CopyToByteBuffer(out);
}

int CPlugFilePack::ExtractPath(
        const char *selectedPath,
        ByteBuffer *out) const {
    if (selectedPath == nullptr || out == nullptr) {
        return 0;
    }
    out->Clear();
    const CPlugFileFidContainer_SFileDesc *file =
            FindFileDescByPath(selectedPath);
    if (file == nullptr) {
        return 0;
    }
    if (file->ShouldTryStreamFeedbackExtraction(selectedPath)) {
        return ExtractPathWithStreamFeedback(file, selectedPath, out);
    }
    return ExtractPathWithPackedPayload(*file, out);
}

int CPlugFilePack::ExtractPathWithStreamFeedbackStrict(
        const char *selectedPath,
        ByteBuffer *out) const {
    if (out == nullptr) {
        return 0;
    }
    out->Clear();
    if (selectedPath == nullptr || selectedPath[0] == '\0') {
        return 0;
    }
    const CPlugFileFidContainer_SFileDesc *file =
            FindFileDescByPath(selectedPath);
    if (file == nullptr) {
        return 0;
    }
    if (!file->IsEncryptedPayload()) {
        ByteBuffer decoded;
        return ExtractPathWithPackedPayload(*file, &decoded) &&
               CommitArchiveFeedbackExtraction(
                       std::move(decoded), 0, out);
    }
    return ExtractPathWithStreamFeedback(file, selectedPath, out);
}

int CPlugFilePack::ExtractReferenceTablePrefix(
        const char *selectedPath,
        ByteBuffer *out) const {
    if (selectedPath == nullptr || out == nullptr) {
        return 0;
    }
    out->Clear();
    const CPlugFileFidContainer_SFileDesc *file =
            FindFileDescByPath(selectedPath);
    if (file == nullptr) {
        return 0;
    }
    if (!file->IsEncryptedPayload()) {
        return ExtractPathWithPackedPayload(*file, out);
    }

    StaticSolidArchivePayload payload;
    if (!payload.BuildEncryptedPackFilePayload({
                *file,
                dataStart,
                0u,
                0u,
                "",
                selectedPath}) ||
        static_cast<size_t>(payload.PackPayloadOffset()) > bytes.Size() ||
        payload.RawByteCount() >
                bytes.Size() -
                        static_cast<size_t>(payload.PackPayloadOffset())) {
        return 0;
    }
    CGameCtnReplayStaticSolidDecodedPayload decoded;
    return CGameCtnReplayStaticSolidArchivePayloadReader::
                   DecodeReferenceTablePrefixWithStreamFeedback(
                           key,
                           payload,
                           StaticSolidArchiveRawBytes::FromBytes(
                                   bytes.Data() + payload.PackPayloadOffset(),
                                   payload.RawByteCount()),
                           &decoded) &&
           decoded.IsReady() && decoded.CopyToByteBuffer(out);
}
