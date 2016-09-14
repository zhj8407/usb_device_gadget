#ifndef FILE_UTILS_H_INCLUDED
#define FILE_UTILS_H_INCLUDED

int read_value_from_file(const char* filename, const char* format, ...);

int write_value_to_file(const char* filename, const char* format, ...);

#endif // FILE_UTILS_H_INCLUDED