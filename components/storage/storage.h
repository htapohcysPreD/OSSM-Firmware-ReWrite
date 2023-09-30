/**
 ******************************************************************************
 *  file           : storage.h
 *  brief          : Non-Volatile Storage
 ******************************************************************************
 */

#ifndef COMPONENTS_STORAGE_STORAGE_H_
#define COMPONENTS_STORAGE_STORAGE_H_

#include "esp_err.h" // NOLINT

#ifdef __cplusplus
extern "C"
{
#endif

    esp_err_t STOR_Init(void);
    esp_err_t STOR_Write_u32(const char* key, const uint32_t value);
    esp_err_t STOR_Read_u32(const char* key, uint32_t* value);
    esp_err_t STOR_Read_str(const char* key, char* value, size_t* size);

#ifdef __cplusplus
}
#endif

#endif // COMPONENTS_STORAGE_STORAGE_H_
