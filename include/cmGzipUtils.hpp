#ifndef CM_GZIP_UTILS_HPP
#define CM_GZIP_UTILS_HPP

#include <iostream>

#include <algorithm>
#include <cassert>
#include <istream>
#include <ostream>

#include <zlib.h> // compression

inline const unsigned CHUNKSIZE = 16384;

struct CharStreamWrapper : std::streambuf {
  CharStreamWrapper(char *begin, char *end) { setg(begin, begin, end); }
};

template <typename Iterator>
int writeCompressed(std::istream &istr, Iterator output_iterator) {
  int ret, flush;
  unsigned have;
  z_stream strm;
  char in[CHUNKSIZE];
  char out[CHUNKSIZE];

  /* allocate deflate state */
  strm.zalloc = Z_NULL;
  strm.zfree = Z_NULL;
  strm.opaque = Z_NULL;

  // Initialize deflation for writing with gzip headers
  ret = deflateInit2(&strm, Z_BEST_COMPRESSION, Z_DEFLATED, 31, 8,
                     Z_DEFAULT_STRATEGY);

  if (ret != Z_OK)
    return ret;

  do {
    istr.read(in, CHUNKSIZE);
    strm.avail_in = istr.gcount();
    if (istr.bad()) {
      std::cout << "Error while reading from input stream.\n";
      deflateEnd(&strm);
      return Z_ERRNO;
    }
    flush = istr.eof() ? Z_FINISH : Z_NO_FLUSH;
    strm.next_in = (unsigned char *)in;

    /* run deflate() on input until output buffer not full, finish
       compression if all of source has been read in */
    do {
      strm.avail_out = CHUNKSIZE;
      strm.next_out = (unsigned char *)out;
      ret = deflate(&strm, flush);   /* no bad return value */
      assert(ret != Z_STREAM_ERROR); /* state not clobbered */
      have = CHUNKSIZE - strm.avail_out;

      std::copy(std::begin(out), std::begin(out) + have, output_iterator);
    } while (strm.avail_out == 0);
    assert(strm.avail_in == 0); /* all input will be used */

    /* done when last data in file processed */
  } while (flush != Z_FINISH);
  assert(ret == Z_STREAM_END); /* stream will be complete */

  /* clean up and return */
  deflateEnd(&strm);
  return Z_OK;
}

template <typename Iterator>
int readCompressed(std::istream &istr, Iterator output_iterator) {
  int ret;
  unsigned have;
  z_stream strm;
  char in[CHUNKSIZE];
  char out[CHUNKSIZE];

  /* allocate inflate state */
  strm.zalloc = Z_NULL;
  strm.zfree = Z_NULL;
  strm.opaque = Z_NULL;
  strm.avail_in = 0;
  strm.next_in = Z_NULL;
  ret = inflateInit2(&strm, 31);
  if (ret != Z_OK)
    return ret;

  /* decompress until deflate stream ends or end of file */
  do {
    istr.read(in, CHUNKSIZE);
    strm.avail_in = istr.gcount();
    if (istr.bad()) {
      inflateEnd(&strm);
      return Z_ERRNO;
    }
    if (strm.avail_in == 0)
      break;
    strm.next_in = (unsigned char *)in;

    /* run inflate() on input until output buffer not full */
    do {
      strm.avail_out = CHUNKSIZE;
      strm.next_out = (unsigned char *)out;
      ret = inflate(&strm, Z_NO_FLUSH);
      assert(ret != Z_STREAM_ERROR); /* state not clobbered */
      switch (ret) {
      case Z_NEED_DICT:
        ret = Z_DATA_ERROR; /* and fall through */
      case Z_DATA_ERROR:
      case Z_MEM_ERROR:
        inflateEnd(&strm);
        return ret;
      }
      have = CHUNKSIZE - strm.avail_out;
      std::copy(std::begin(out), std::begin(out) + have, output_iterator);
    } while (strm.avail_out == 0);

    /* done when inflate() says it's done */
  } while (ret != Z_STREAM_END);

  /* clean up and return */
  inflateEnd(&strm);
  return ret == Z_STREAM_END ? Z_OK : Z_DATA_ERROR;
}

#endif