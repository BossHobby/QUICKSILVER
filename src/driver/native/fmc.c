#include "driver/fmc.h"

#include <stdio.h>
#include <string.h>

#include "core/project.h"

#define FILENAME "flash.bin"

void fmc_lock() {
}

void fmc_unlock() {
}

void fmc_erase() {
  remove(FILENAME);
}

static FILE *open_file() {
  FILE *file = fopen(FILENAME, "rb+");
  if (file == NULL) {
    file = fopen(FILENAME, "wb+");
  }
  return file;
}

flash_word_t fmc_read(uint32_t addr) {
  FILE *file = open_file();
  fseek(file, addr, SEEK_SET);
  flash_word_t value;
  fread(&value, sizeof(flash_word_t), 1, file);
  fclose(file);
  return value;
}

void fmc_read_buf(uint32_t addr, uint8_t *data, uint32_t size) {
  FILE *file = open_file();
  fseek(file, addr, SEEK_SET);
  fread(data, size, 1, file);
  fclose(file);
}

void fmc_write(uint32_t addr, flash_word_t value) {
  FILE *file = open_file();
  fseek(file, addr, SEEK_SET);
  fwrite(&value, sizeof(flash_word_t), 1, file);
  fclose(file);
}

void fmc_write_buf(uint32_t addr, uint8_t *data, uint32_t size) {
  FILE *file = open_file();
  fseek(file, addr, SEEK_SET);
  fwrite(data, size, 1, file);
  fclose(file);
}