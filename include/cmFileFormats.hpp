#ifndef CM_FILE_FORMATS_HPP
#define CM_FILE_FORMATS_HPP

enum cmFileFormatEnum : unsigned {
  VTK_LEGACY = 0,
  VTP = 1,
  MULTI = 2,
  MSGPACK = 3,
  MSGPACK_GZIP = 4,
  AUTO = 5,
};

#endif