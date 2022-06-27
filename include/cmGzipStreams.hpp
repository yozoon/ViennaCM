#ifndef CM_GZIP_STREAMS_HPP
#define CM_GZIP_STREAMS_HPP

#include <cassert> // assert
#include <fstream> // file writer
#include <ios>
#include <istream>
#include <ostream>
#include <streambuf>
#include <zlib.h> // compression

class gzip_streambuf : public std::streambuf {
  static const unsigned chunksize = 16384;

  std::streambuf *m_buf;
  char m_buffer[chunksize];
  z_stream m_strm;

  void do_output() {
    int write_size = pptr() - pbase();
    if (write_size > 0) {
      int ret;

      unsigned have;
      unsigned char out[chunksize];

      m_strm.avail_in = write_size;

      m_strm.next_in = (unsigned char *)pbase();

      do {
        m_strm.avail_out = chunksize;
        m_strm.next_out = out;
        // Do the compression
        ret = deflate(&m_strm, Z_NO_FLUSH);

        assert(ret != Z_STREAM_ERROR);

        have = chunksize - m_strm.avail_out;
        // Write the compressed data to the output stream buffer
        m_buf->sputn((char *)out, have);
      } while (m_strm.avail_out == 0);

      assert(m_strm.avail_in == 0);

      // Reset buffer to empty position
      setp(m_buffer, m_buffer + sizeof(m_buffer) - 1);
    }
  }

  void do_flush() {
    int ret;
    unsigned have;
    unsigned char out[chunksize];
    m_strm.avail_in = 0;
    m_strm.next_in = (unsigned char *)pbase();
    do {
      m_strm.avail_out = chunksize;
      m_strm.next_out = out;
      // Do the compression
      ret = deflate(&m_strm, Z_FINISH);

      assert(ret != Z_STREAM_ERROR);
      have = chunksize - m_strm.avail_out;
      // Write the compressed data to the output stream buffer
      m_buf->sputn((char *)out, have);
    } while (m_strm.avail_out == 0);

    assert(m_strm.avail_in == 0);
  }

public:
  gzip_streambuf(std::streambuf *buf) : m_buf(buf) {
    int ret;

    // allocate deflate state
    m_strm.zalloc = Z_NULL;
    m_strm.zfree = Z_NULL;
    m_strm.opaque = Z_NULL;

    // Initialize deflation for writing with gzip headers
    ret = deflateInit2(&m_strm, Z_BEST_COMPRESSION, Z_DEFLATED, 31, 8,
                       Z_DEFAULT_STRATEGY);

    if (ret != Z_OK)
      throw std::ios::failure("Opening gzip stream failed.");

    setp(m_buffer, m_buffer + sizeof(m_buffer) - 1);
  }

  int_type overflow(int_type value) override {
    if (value != traits_type::eof()) {
      *pptr() = value;
      pbump(1);
      do_output();
    }
    return value;
  };

  int_type sync() override {
    do_output();
    return 0;
  }

  ~gzip_streambuf() {
    sync();
    do_flush();
    (void)deflateEnd(&m_strm);
  }
};

// Wrapper class for writing data into a gzip compressed ouput stream
class gzip_ostream : public std::ostream {
public:
  gzip_ostream(std::ostream &stream)
      : std::ostream(new gzip_streambuf(stream.rdbuf())) {}

  ~gzip_ostream() { delete rdbuf(); }
};

// Wrapper class that allows for direct output to compressed file
template <typename STR> class gzip_ofstream : public std::ostream {
  std::ofstream f;

public:
  gzip_ofstream(STR s)
      : f(s, std::ios::binary), std::ostream(new gzip_streambuf(f.rdbuf())) {}

  ~gzip_ofstream() { delete rdbuf(); }
};

// // Wrapper class for reading data from a gzip compressed input stream
// TODO: implement this!
// class gzip_istream : public std::istream {
// public:
//   gzip_istream(std::istream &stream)
//       : std::istream(new gzip_streambuf(stream.rdbuf())) {}
//
//   ~gzip_istream() { delete rdbuf(); }
// };

#endif