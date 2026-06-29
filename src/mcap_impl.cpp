// Single translation unit that compiles the header-only MCAP 2.1.1
// implementation (vendored under third_party/, since the system package ships
// headers without the .inl files).
//
// libdata_logger.a (pulled in transitively by message_manager) contains a
// PARTIAL copy of the same mcap symbols — enough for its writer-only use, but
// missing reader decompression classes (LZ4Reader / ZStdReader) that the
// player needs. We therefore compile the complete implementation here and link
// with --allow-multiple-definition so this complete copy satisfies everything
// and the partial data_logger duplicates (identical 2.1.1 source) are ignored.
#define MCAP_IMPLEMENTATION
#include <mcap/reader.hpp>
#include <mcap/writer.hpp>
