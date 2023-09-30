/**
 ******************************************************************************
 *  file           : storage.c
 *  brief          : Non-Volatile Storage
 ******************************************************************************
 */

/****************************** Includes  */
#include "storage.h"

#include "../../main/ossm_config.h"
#include "esp_err.h"    // NOLINT
#include "esp_log.h"    // NOLINT
#include "esp_system.h" // NOLINT
#include "nvs_flash.h"  // NOLINT

/****************************** Configuration */

/****************************** Statics */
static const char* TAG = "STOR";
static bool nvsIsReady = false;

/****************************** Functions */

/**
 * @brief Init the Non-Volatile Storage
 */
esp_err_t STOR_Init(void)
{
    nvsIsReady = false;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // On Error: Erase and retry init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "Open NVS");
    nvs_handle_t nvsHandle;
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvsHandle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
#if 0
        // Debug-Code to set NVS Keys
        nvs_set_str(nvsHandle, "WIFI_SSID", "TheName");
        nvs_set_str(nvsHandle, "WIFI_PASS", "Secret");
        nvs_commit(nvsHandle);
#endif
        nvs_close(nvsHandle);
    }
#if 1
    // Dev Info: NVS Stats
    nvs_stats_t nvs_stats;
    nvs_get_stats("nvs", &nvs_stats);
    ESP_LOGI(TAG, "------------------------------------------");
    ESP_LOGI(TAG, "Used NVS entries = %d", nvs_stats.used_entries);
    ESP_LOGI(TAG, "Free NVS entries = %d", nvs_stats.free_entries);
    ESP_LOGI(TAG, " All NVS entries = %d", nvs_stats.total_entries);
    ESP_LOGI(TAG, "------------------------------------------");

    nvs_iterator_t iter = NULL;
    esp_err_t res = nvs_entry_find("nvs", NULL, NVS_TYPE_ANY, &iter);
    while (res == ESP_OK)
    {
        nvs_entry_info_t info;
        nvs_entry_info(iter, &info);
        ESP_LOGI(TAG, "Key '%s', Type '%d'", info.key, info.type);
        res = nvs_entry_next(&iter);
    }
    nvs_release_iterator(iter);
    ESP_LOGI(TAG, "------------------------------------------");
#endif

    nvsIsReady = true;
    return ESP_OK;
}

/**
 * @brief Write a U32 to NVS
 */
esp_err_t STOR_Write_u32(const char* key, const uint32_t value)
{
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    err |= nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    err |= nvs_set_u32(handle, key, value);
    err |= nvs_commit(handle);
    nvs_close(handle);
    return err;
}

/**
 * @brief Read a U32 from NVS
 */
esp_err_t STOR_Read_u32(const char* key, uint32_t* value)
{
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    err |= nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    err |= nvs_get_u32(handle, key, value);
    nvs_close(handle);
    return err;
}

/**
 * @brief Read a string from NVS
 */
esp_err_t STOR_Read_str(const char* key, char* value, size_t* size)
{
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    err |= nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    err |= nvs_get_str(handle, key, value, size);
    nvs_close(handle);
    return err;
}
