#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "file_open.h"

uint8_t* load_binary_file(const char* file_path, unsigned int *loaded) {
    FILE *fp = fopen(file_path, "rb");

    if (fp == NULL) {
        perror("fopen()");
        return NULL;
    }

    if (fseek(fp, 0, SEEK_END) != 0) {
        perror("fseek()");
        return NULL;
    }

    size_t file_size = ftell(fp);
    if (file_size < 1) { perror("File is empty!");
        return NULL;
    }
    rewind(fp);

    uint8_t* data = (uint8_t*)malloc(sizeof(uint8_t)*file_size);

    if (data == NULL) {
        perror("calloc()");
        return NULL;
    }

    size_t n_reads = fread(data, sizeof(*data), file_size, fp);
    if (n_reads != file_size){
        perror("fread()");
        free(data);
        return NULL;
    }

    *loaded = file_size;
    fclose(fp);
    return data;
}
