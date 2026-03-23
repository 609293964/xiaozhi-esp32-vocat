#include "wifi_board.h"
#include "codecs/box_audio_codec.h"
#include "display/lcd_display.h"
#include "display/emote_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "backlight.h"
#include "esp_video.h"
#include "settings.h"

#include <esp_log.h>

#include <driver/i2c_master.h>
#include <driver/i2c.h>
#include "i2c_device.h"
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_st77916.h>
#include "esp_lcd_touch_cst816s.h"
#include "touch.h"

#include "driver/temperature_sensor.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_sleep.h>
#include <esp_lvgl_port.h>
#include <lvgl.h>
#include <string>
#include <ctime>
#include <cstdlib>
#include <atomic>
#if CONFIG_USE_EMOTE_MESSAGE_STYLE
#include "gfx.h"
#endif

#define TAG "ESP-VoCat"
LV_FONT_DECLARE(BUILTIN_TEXT_FONT);
LV_FONT_DECLARE(font_puhui_30_4);


temperature_sensor_handle_t temp_sensor = NULL;
static const st77916_lcd_init_cmd_t vendor_specific_init_yysj[] = {
    {0xF0, (uint8_t []){0x28}, 1, 0},
    {0xF2, (uint8_t []){0x28}, 1, 0},
    {0x73, (uint8_t []){0xF0}, 1, 0},
    {0x7C, (uint8_t []){0xD1}, 1, 0},
    {0x83, (uint8_t []){0xE0}, 1, 0},
    {0x84, (uint8_t []){0x61}, 1, 0},
    {0xF2, (uint8_t []){0x82}, 1, 0},
    {0xF0, (uint8_t []){0x00}, 1, 0},
    {0xF0, (uint8_t []){0x01}, 1, 0},
    {0xF1, (uint8_t []){0x01}, 1, 0},
    {0xB0, (uint8_t []){0x56}, 1, 0},
    {0xB1, (uint8_t []){0x4D}, 1, 0},
    {0xB2, (uint8_t []){0x24}, 1, 0},
    {0xB4, (uint8_t []){0x87}, 1, 0},
    {0xB5, (uint8_t []){0x44}, 1, 0},
    {0xB6, (uint8_t []){0x8B}, 1, 0},
    {0xB7, (uint8_t []){0x40}, 1, 0},
    {0xB8, (uint8_t []){0x86}, 1, 0},
    {0xBA, (uint8_t []){0x00}, 1, 0},
    {0xBB, (uint8_t []){0x08}, 1, 0},
    {0xBC, (uint8_t []){0x08}, 1, 0},
    {0xBD, (uint8_t []){0x00}, 1, 0},
    {0xC0, (uint8_t []){0x80}, 1, 0},
    {0xC1, (uint8_t []){0x10}, 1, 0},
    {0xC2, (uint8_t []){0x37}, 1, 0},
    {0xC3, (uint8_t []){0x80}, 1, 0},
    {0xC4, (uint8_t []){0x10}, 1, 0},
    {0xC5, (uint8_t []){0x37}, 1, 0},
    {0xC6, (uint8_t []){0xA9}, 1, 0},
    {0xC7, (uint8_t []){0x41}, 1, 0},
    {0xC8, (uint8_t []){0x01}, 1, 0},
    {0xC9, (uint8_t []){0xA9}, 1, 0},
    {0xCA, (uint8_t []){0x41}, 1, 0},
    {0xCB, (uint8_t []){0x01}, 1, 0},
    {0xD0, (uint8_t []){0x91}, 1, 0},
    {0xD1, (uint8_t []){0x68}, 1, 0},
    {0xD2, (uint8_t []){0x68}, 1, 0},
    {0xF5, (uint8_t []){0x00, 0xA5}, 2, 0},
    {0xDD, (uint8_t []){0x4F}, 1, 0},
    {0xDE, (uint8_t []){0x4F}, 1, 0},
    {0xF1, (uint8_t []){0x10}, 1, 0},
    {0xF0, (uint8_t []){0x00}, 1, 0},
    {0xF0, (uint8_t []){0x02}, 1, 0},
    {0xE0, (uint8_t []){0xF0, 0x0A, 0x10, 0x09, 0x09, 0x36, 0x35, 0x33, 0x4A, 0x29, 0x15, 0x15, 0x2E, 0x34}, 14, 0},
    {0xE1, (uint8_t []){0xF0, 0x0A, 0x0F, 0x08, 0x08, 0x05, 0x34, 0x33, 0x4A, 0x39, 0x15, 0x15, 0x2D, 0x33}, 14, 0},
    {0xF0, (uint8_t []){0x10}, 1, 0},
    {0xF3, (uint8_t []){0x10}, 1, 0},
    {0xE0, (uint8_t []){0x07}, 1, 0},
    {0xE1, (uint8_t []){0x00}, 1, 0},
    {0xE2, (uint8_t []){0x00}, 1, 0},
    {0xE3, (uint8_t []){0x00}, 1, 0},
    {0xE4, (uint8_t []){0xE0}, 1, 0},
    {0xE5, (uint8_t []){0x06}, 1, 0},
    {0xE6, (uint8_t []){0x21}, 1, 0},
    {0xE7, (uint8_t []){0x01}, 1, 0},
    {0xE8, (uint8_t []){0x05}, 1, 0},
    {0xE9, (uint8_t []){0x02}, 1, 0},
    {0xEA, (uint8_t []){0xDA}, 1, 0},
    {0xEB, (uint8_t []){0x00}, 1, 0},
    {0xEC, (uint8_t []){0x00}, 1, 0},
    {0xED, (uint8_t []){0x0F}, 1, 0},
    {0xEE, (uint8_t []){0x00}, 1, 0},
    {0xEF, (uint8_t []){0x00}, 1, 0},
    {0xF8, (uint8_t []){0x00}, 1, 0},
    {0xF9, (uint8_t []){0x00}, 1, 0},
    {0xFA, (uint8_t []){0x00}, 1, 0},
    {0xFB, (uint8_t []){0x00}, 1, 0},
    {0xFC, (uint8_t []){0x00}, 1, 0},
    {0xFD, (uint8_t []){0x00}, 1, 0},
    {0xFE, (uint8_t []){0x00}, 1, 0},
    {0xFF, (uint8_t []){0x00}, 1, 0},
    {0x60, (uint8_t []){0x40}, 1, 0},
    {0x61, (uint8_t []){0x04}, 1, 0},
    {0x62, (uint8_t []){0x00}, 1, 0},
    {0x63, (uint8_t []){0x42}, 1, 0},
    {0x64, (uint8_t []){0xD9}, 1, 0},
    {0x65, (uint8_t []){0x00}, 1, 0},
    {0x66, (uint8_t []){0x00}, 1, 0},
    {0x67, (uint8_t []){0x00}, 1, 0},
    {0x68, (uint8_t []){0x00}, 1, 0},
    {0x69, (uint8_t []){0x00}, 1, 0},
    {0x6A, (uint8_t []){0x00}, 1, 0},
    {0x6B, (uint8_t []){0x00}, 1, 0},
    {0x70, (uint8_t []){0x40}, 1, 0},
    {0x71, (uint8_t []){0x03}, 1, 0},
    {0x72, (uint8_t []){0x00}, 1, 0},
    {0x73, (uint8_t []){0x42}, 1, 0},
    {0x74, (uint8_t []){0xD8}, 1, 0},
    {0x75, (uint8_t []){0x00}, 1, 0},
    {0x76, (uint8_t []){0x00}, 1, 0},
    {0x77, (uint8_t []){0x00}, 1, 0},
    {0x78, (uint8_t []){0x00}, 1, 0},
    {0x79, (uint8_t []){0x00}, 1, 0},
    {0x7A, (uint8_t []){0x00}, 1, 0},
    {0x7B, (uint8_t []){0x00}, 1, 0},
    {0x80, (uint8_t []){0x48}, 1, 0},
    {0x81, (uint8_t []){0x00}, 1, 0},
    {0x82, (uint8_t []){0x06}, 1, 0},
    {0x83, (uint8_t []){0x02}, 1, 0},
    {0x84, (uint8_t []){0xD6}, 1, 0},
    {0x85, (uint8_t []){0x04}, 1, 0},
    {0x86, (uint8_t []){0x00}, 1, 0},
    {0x87, (uint8_t []){0x00}, 1, 0},
    {0x88, (uint8_t []){0x48}, 1, 0},
    {0x89, (uint8_t []){0x00}, 1, 0},
    {0x8A, (uint8_t []){0x08}, 1, 0},
    {0x8B, (uint8_t []){0x02}, 1, 0},
    {0x8C, (uint8_t []){0xD8}, 1, 0},
    {0x8D, (uint8_t []){0x04}, 1, 0},
    {0x8E, (uint8_t []){0x00}, 1, 0},
    {0x8F, (uint8_t []){0x00}, 1, 0},
    {0x90, (uint8_t []){0x48}, 1, 0},
    {0x91, (uint8_t []){0x00}, 1, 0},
    {0x92, (uint8_t []){0x0A}, 1, 0},
    {0x93, (uint8_t []){0x02}, 1, 0},
    {0x94, (uint8_t []){0xDA}, 1, 0},
    {0x95, (uint8_t []){0x04}, 1, 0},
    {0x96, (uint8_t []){0x00}, 1, 0},
    {0x97, (uint8_t []){0x00}, 1, 0},
    {0x98, (uint8_t []){0x48}, 1, 0},
    {0x99, (uint8_t []){0x00}, 1, 0},
    {0x9A, (uint8_t []){0x0C}, 1, 0},
    {0x9B, (uint8_t []){0x02}, 1, 0},
    {0x9C, (uint8_t []){0xDC}, 1, 0},
    {0x9D, (uint8_t []){0x04}, 1, 0},
    {0x9E, (uint8_t []){0x00}, 1, 0},
    {0x9F, (uint8_t []){0x00}, 1, 0},
    {0xA0, (uint8_t []){0x48}, 1, 0},
    {0xA1, (uint8_t []){0x00}, 1, 0},
    {0xA2, (uint8_t []){0x05}, 1, 0},
    {0xA3, (uint8_t []){0x02}, 1, 0},
    {0xA4, (uint8_t []){0xD5}, 1, 0},
    {0xA5, (uint8_t []){0x04}, 1, 0},
    {0xA6, (uint8_t []){0x00}, 1, 0},
    {0xA7, (uint8_t []){0x00}, 1, 0},
    {0xA8, (uint8_t []){0x48}, 1, 0},
    {0xA9, (uint8_t []){0x00}, 1, 0},
    {0xAA, (uint8_t []){0x07}, 1, 0},
    {0xAB, (uint8_t []){0x02}, 1, 0},
    {0xAC, (uint8_t []){0xD7}, 1, 0},
    {0xAD, (uint8_t []){0x04}, 1, 0},
    {0xAE, (uint8_t []){0x00}, 1, 0},
    {0xAF, (uint8_t []){0x00}, 1, 0},
    {0xB0, (uint8_t []){0x48}, 1, 0},
    {0xB1, (uint8_t []){0x00}, 1, 0},
    {0xB2, (uint8_t []){0x09}, 1, 0},
    {0xB3, (uint8_t []){0x02}, 1, 0},
    {0xB4, (uint8_t []){0xD9}, 1, 0},
    {0xB5, (uint8_t []){0x04}, 1, 0},
    {0xB6, (uint8_t []){0x00}, 1, 0},
    {0xB7, (uint8_t []){0x00}, 1, 0},
    {0xB8, (uint8_t []){0x48}, 1, 0},
    {0xB9, (uint8_t []){0x00}, 1, 0},
    {0xBA, (uint8_t []){0x0B}, 1, 0},
    {0xBB, (uint8_t []){0x02}, 1, 0},
    {0xBC, (uint8_t []){0xDB}, 1, 0},
    {0xBD, (uint8_t []){0x04}, 1, 0},
    {0xBE, (uint8_t []){0x00}, 1, 0},
    {0xBF, (uint8_t []){0x00}, 1, 0},
    {0xC0, (uint8_t []){0x10}, 1, 0},
    {0xC1, (uint8_t []){0x47}, 1, 0},
    {0xC2, (uint8_t []){0x56}, 1, 0},
    {0xC3, (uint8_t []){0x65}, 1, 0},
    {0xC4, (uint8_t []){0x74}, 1, 0},
    {0xC5, (uint8_t []){0x88}, 1, 0},
    {0xC6, (uint8_t []){0x99}, 1, 0},
    {0xC7, (uint8_t []){0x01}, 1, 0},
    {0xC8, (uint8_t []){0xBB}, 1, 0},
    {0xC9, (uint8_t []){0xAA}, 1, 0},
    {0xD0, (uint8_t []){0x10}, 1, 0},
    {0xD1, (uint8_t []){0x47}, 1, 0},
    {0xD2, (uint8_t []){0x56}, 1, 0},
    {0xD3, (uint8_t []){0x65}, 1, 0},
    {0xD4, (uint8_t []){0x74}, 1, 0},
    {0xD5, (uint8_t []){0x88}, 1, 0},
    {0xD6, (uint8_t []){0x99}, 1, 0},
    {0xD7, (uint8_t []){0x01}, 1, 0},
    {0xD8, (uint8_t []){0xBB}, 1, 0},
    {0xD9, (uint8_t []){0xAA}, 1, 0},
    {0xF3, (uint8_t []){0x01}, 1, 0},
    {0xF0, (uint8_t []){0x00}, 1, 0},
    {0x21, (uint8_t []){}, 0, 0},
    {0x11, (uint8_t []){}, 0, 0},
    {0x00, (uint8_t []){}, 0, 120},
};
float tsens_value;
gpio_num_t AUDIO_I2S_GPIO_DIN = AUDIO_I2S_GPIO_DIN_1;
gpio_num_t AUDIO_CODEC_PA_PIN = AUDIO_CODEC_PA_PIN_1;
gpio_num_t QSPI_PIN_NUM_LCD_RST = QSPI_PIN_NUM_LCD_RST_1;
gpio_num_t TOUCH_PAD2 = TOUCH_PAD2_1;
gpio_num_t UART1_TX = UART1_TX_1;
gpio_num_t UART1_RX = UART1_RX_1;

class Charge : public I2cDevice {
public:
    Charge(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr)
    {
        read_buffer_ = new uint8_t[8];
    }
    ~Charge()
    {
        delete[] read_buffer_;
    }
    void Printcharge()
    {
        ReadRegs(0x08, read_buffer_, 2);
        ReadRegs(0x0c, read_buffer_ + 2, 2);
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &tsens_value));

        int16_t voltage = static_cast<uint16_t>(read_buffer_[1] << 8 | read_buffer_[0]);
        int16_t current = static_cast<int16_t>(read_buffer_[3] << 8 | read_buffer_[2]);
        latest_voltage_ = voltage;
        latest_current_ = current;
        if (current > 30) {
            external_power_connected_ = true;
        } else if (current < -30) {
            external_power_connected_ = false;
        }
    }
    static void TaskFunction(void *pvParameters)
    {
        Charge* charge = static_cast<Charge*>(pvParameters);
        while (true) {
            charge->Printcharge();
            vTaskDelay(pdMS_TO_TICKS(300));
        }
    }
    bool IsExternalPowerConnected() const
    {
        return external_power_connected_.load();
    }

    int16_t GetCurrent() const
    {
        return latest_current_.load();
    }

private:
    uint8_t* read_buffer_ = nullptr;
    std::atomic<int16_t> latest_voltage_{0};
    std::atomic<int16_t> latest_current_{0};
    std::atomic<bool> external_power_connected_{true};
};

class Cst816s : public I2cDevice {
public:
    struct TouchPoint_t {
        int num = 0;
        int x = -1;
        int y = -1;
    };

    enum TouchEvent {
        TOUCH_NONE,
        TOUCH_PRESS,
        TOUCH_RELEASE,
        TOUCH_LONG_PRESS,
        TOUCH_SWIPE_LEFT,
        TOUCH_SWIPE_RIGHT,
        TOUCH_HOLD
    };

    Cst816s(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr)
    {
        read_buffer_ = new uint8_t[6];
        was_touched_ = false;
        press_count_ = 0;
        touch_start_time_us_ = 0;
        long_press_triggered_ = false;
        touch_start_x_ = -1;
        touch_start_y_ = -1;
        touch_last_x_ = -1;
        touch_last_y_ = -1;

        // Create touch interrupt semaphore
        touch_isr_mux_ = xSemaphoreCreateBinary();
        if (touch_isr_mux_ == NULL) {
            ESP_LOGE(TAG, "Failed to create touch semaphore");
        }
    }

    ~Cst816s()
    {
        delete[] read_buffer_;

        // Delete semaphore if it exists
        if (touch_isr_mux_ != NULL) {
            vSemaphoreDelete(touch_isr_mux_);
            touch_isr_mux_ = NULL;
        }
    }

    void UpdateTouchPoint()
    {
        ReadRegs(0x02, read_buffer_, 6);
        tp_.num = read_buffer_[0] & 0x0F;
        tp_.x = ((read_buffer_[1] & 0x0F) << 8) | read_buffer_[2];
        tp_.y = ((read_buffer_[3] & 0x0F) << 8) | read_buffer_[4];
    }

    const TouchPoint_t &GetTouchPoint()
    {
        return tp_;
    }

    TouchEvent CheckTouchEvent()
    {
        bool is_touched = (tp_.num > 0);
        TouchEvent event = TOUCH_NONE;
        int64_t now = esp_timer_get_time();

        if (is_touched && !was_touched_) {
            press_count_++;
            touch_start_time_us_ = now;
            long_press_triggered_ = false;
            touch_start_x_ = tp_.x;
            touch_start_y_ = tp_.y;
            touch_last_x_ = tp_.x;
            touch_last_y_ = tp_.y;
            event = TOUCH_PRESS;
            ESP_LOGI(TAG, "TOUCH PRESS - count: %d, x: %d, y: %d", press_count_, tp_.x, tp_.y);
        } else if (!is_touched && was_touched_) {
            if (long_press_triggered_) {
                event = TOUCH_NONE;
            } else {
                int dx = touch_last_x_ - touch_start_x_;
                int dy = touch_last_y_ - touch_start_y_;
                if (dx <= -60 && std::abs(dx) > std::abs(dy)) {
                    event = TOUCH_SWIPE_LEFT;
                } else if (dx >= 60 && std::abs(dx) > std::abs(dy)) {
                    event = TOUCH_SWIPE_RIGHT;
                } else {
                    event = TOUCH_RELEASE;
                }
            }
            ESP_LOGI(TAG, "TOUCH RELEASE - total presses: %d", press_count_);
        } else if (is_touched && was_touched_) {
            touch_last_x_ = tp_.x;
            touch_last_y_ = tp_.y;
            if (!long_press_triggered_ && (now - touch_start_time_us_) >= 1500000) {
                long_press_triggered_ = true;
                event = TOUCH_LONG_PRESS;
                ESP_LOGI(TAG, "TOUCH LONG PRESS - x: %d, y: %d", tp_.x, tp_.y);
            } else {
                event = TOUCH_HOLD;
                ESP_LOGD(TAG, "TOUCH HOLD - x: %d, y: %d", tp_.x, tp_.y);
            }
        }

        was_touched_ = is_touched;
        return event;
    }

    int GetPressCount() const
    {
        return press_count_;
    }

    void ResetPressCount()
    {
        press_count_ = 0;
    }

    // Semaphore management methods
    SemaphoreHandle_t GetTouchSemaphore()
    {
        return touch_isr_mux_;
    }

    bool WaitForTouchEvent(TickType_t timeout = portMAX_DELAY)
    {
        if (touch_isr_mux_ != NULL) {
            return xSemaphoreTake(touch_isr_mux_, timeout) == pdTRUE;
        }
        return false;
    }

    void NotifyTouchEvent()
    {
        if (touch_isr_mux_ != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(touch_isr_mux_, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

private:
    uint8_t* read_buffer_ = nullptr;
    TouchPoint_t tp_;

    // Touch state tracking
    bool was_touched_;
    int press_count_;
    int64_t touch_start_time_us_;
    bool long_press_triggered_;
    int touch_start_x_;
    int touch_start_y_;
    int touch_last_x_;
    int touch_last_y_;

    // Touch interrupt semaphore
    SemaphoreHandle_t touch_isr_mux_;
};

class EspVocat : public WifiBoard {
private:
    struct ClockDigit {
        lv_obj_t* seg[7] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    };

    i2c_master_bus_handle_t i2c_bus_;
    Cst816s* cst816s_;
    Charge* charge_;
    Button boot_button_;
    Display* display_ = nullptr;
    PwmBacklight* backlight_ = nullptr;
    EspVideo* camera_ = nullptr;
    TaskHandle_t charge_task_handle_ = nullptr;
    TaskHandle_t touch_task_handle_ = nullptr;
    TaskHandle_t power_policy_task_handle_ = nullptr;
    int touch_brightness_step_ = 10;
    lv_obj_t* clock_screen_ = nullptr;
    ClockDigit clock_digits_[4];
    lv_obj_t* clock_colon_top_ = nullptr;
    lv_obj_t* clock_colon_bottom_ = nullptr;
    esp_timer_handle_t clock_timer_ = nullptr;
    bool clock_wallpaper_mode_ = false;
    // When voice conversation interrupts the full-screen clock overlay,
    // restore it automatically after the conversation ends.
    esp_timer_handle_t clock_restore_timer_ = nullptr;
    bool clock_restore_pending_ = false;
    // When hiding clock due to voice state changes, don't force emote idle state.
    bool suppress_emote_idle_on_hide_ = false;
#if CONFIG_USE_EMOTE_MESSAGE_STYLE
    bool emote_clock_mode_ = false;
    struct EmoteClockDigit {
        gfx_obj_t* seg[7] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    };
    EmoteClockDigit emote_clock_digits_[4];
    gfx_obj_t* emote_clock_colon_top_ = nullptr;
    gfx_obj_t* emote_clock_colon_bottom_ = nullptr;
#endif
    bool external_power_connected_ = true;
    bool power_loss_countdown_active_ = false;
    int64_t power_loss_deadline_us_ = 0;
    bool screen_off_by_schedule_ = false;
    int32_t shutdown_delay_min_ = 10;
    int32_t screen_off_start_hour_ = 1;
    int32_t screen_off_end_hour_ = 8;
    int64_t next_settings_reload_us_ = 0;

    void SetSegmentOn(lv_obj_t* segment, bool on)
    {
        if (segment == nullptr) {
            return;
        }
        if (on) {
            lv_obj_clear_flag(segment, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(segment, LV_OBJ_FLAG_HIDDEN);
        }
    }

    void SetDigitValue(ClockDigit& digit, int value)
    {
        static const uint8_t map[10] = {
            0x3F, 0x06, 0x5B, 0x4F, 0x66,
            0x6D, 0x7D, 0x07, 0x7F, 0x6F
        };
        uint8_t mask = 0;
        if (value >= 0 && value <= 9) {
            mask = map[value];
        }
        for (int i = 0; i < 7; ++i) {
            SetSegmentOn(digit.seg[i], ((mask >> i) & 0x01) != 0);
        }
    }

    lv_obj_t* CreateClockSegment(lv_obj_t* parent, int x, int y, int w, int h)
    {
        lv_obj_t* seg = lv_obj_create(parent);
        lv_obj_remove_style_all(seg);
        lv_obj_set_pos(seg, x, y);
        lv_obj_set_size(seg, w, h);
        lv_obj_set_style_bg_opa(seg, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(seg, lv_color_white(), 0);
        lv_obj_set_style_radius(seg, h / 2, 0);
        return seg;
    }

    void CreateClockDigit(ClockDigit& digit, lv_obj_t* parent, int x, int y, int w, int h, int t)
    {
        const int middle_y = (h - t) / 2;
        const int right_x = w - t;
        const int bottom_y = h - t;
        const int half_h = h / 2;

        digit.seg[0] = CreateClockSegment(parent, x + t, y, w - 2 * t, t);
        digit.seg[1] = CreateClockSegment(parent, x + right_x, y + t, t, half_h - t);
        digit.seg[2] = CreateClockSegment(parent, x + right_x, y + half_h, t, half_h - t);
        digit.seg[3] = CreateClockSegment(parent, x + t, y + bottom_y, w - 2 * t, t);
        digit.seg[4] = CreateClockSegment(parent, x, y + half_h, t, half_h - t);
        digit.seg[5] = CreateClockSegment(parent, x, y + t, t, half_h - t);
        digit.seg[6] = CreateClockSegment(parent, x + t, y + middle_y, w - 2 * t, t);
    }

    #if CONFIG_USE_EMOTE_MESSAGE_STYLE
    void SetEmoteSegmentOn(gfx_obj_t* segment, bool on)
    {
        if (segment == nullptr) {
            return;
        }
        gfx_obj_set_visible(segment, on);
    }

    void SetEmoteDigitValue(EmoteClockDigit& digit, int value)
    {
        static const uint8_t map[10] = {
            0x3F, 0x06, 0x5B, 0x4F, 0x66,
            0x6D, 0x7D, 0x07, 0x7F, 0x6F
        };
        uint8_t mask = 0;
        if (value >= 0 && value <= 9) {
            mask = map[value];
        }
        for (int i = 0; i < 7; ++i) {
            SetEmoteSegmentOn(digit.seg[i], ((mask >> i) & 0x01) != 0);
        }
    }

    gfx_obj_t* CreateEmoteClockSegment(emote_handle_t handle, const std::string& name, int x, int y, int w, int h)
    {
        auto* seg = emote_create_obj_by_type(handle, EMOTE_OBJ_TYPE_LABEL, name.c_str());
        if (seg == nullptr) {
            return nullptr;
        }
        gfx_label_set_text(seg, "");
        gfx_obj_set_pos(seg, x, y);
        gfx_obj_set_size(seg, w, h);
        gfx_label_set_bg_enable(seg, true);
        gfx_label_set_bg_color(seg, GFX_COLOR_HEX(0xFFFFFF));
        gfx_label_set_color(seg, GFX_COLOR_HEX(0xFFFFFF));
        gfx_obj_set_visible(seg, false);
        return seg;
    }

    void CreateEmoteClockDigit(emote_handle_t handle, EmoteClockDigit& digit, int index, int x, int y, int w, int h, int t)
    {
        const int middle_y = (h - t) / 2;
        const int right_x = w - t;
        const int bottom_y = h - t;
        const int half_h = h / 2;

        digit.seg[0] = CreateEmoteClockSegment(handle, "vocat_clk_" + std::to_string(index) + "_0", x + t, y, w - 2 * t, t);
        digit.seg[1] = CreateEmoteClockSegment(handle, "vocat_clk_" + std::to_string(index) + "_1", x + right_x, y + t, t, half_h - t);
        digit.seg[2] = CreateEmoteClockSegment(handle, "vocat_clk_" + std::to_string(index) + "_2", x + right_x, y + half_h, t, half_h - t);
        digit.seg[3] = CreateEmoteClockSegment(handle, "vocat_clk_" + std::to_string(index) + "_3", x + t, y + bottom_y, w - 2 * t, t);
        digit.seg[4] = CreateEmoteClockSegment(handle, "vocat_clk_" + std::to_string(index) + "_4", x, y + half_h, t, half_h - t);
        digit.seg[5] = CreateEmoteClockSegment(handle, "vocat_clk_" + std::to_string(index) + "_5", x, y + t, t, half_h - t);
        digit.seg[6] = CreateEmoteClockSegment(handle, "vocat_clk_" + std::to_string(index) + "_6", x + t, y + middle_y, w - 2 * t, t);
    }
    #endif

    static void clock_timer_callback(void* arg)
    {
        auto* board = static_cast<EspVocat*>(arg);
        if (board != nullptr) {
            board->UpdateClockText();
        }
    }

    void UpdateClockText()
    {
    #if CONFIG_USE_EMOTE_MESSAGE_STYLE
        if (emote_clock_mode_) {
            auto* emote_display = dynamic_cast<emote::EmoteDisplay*>(display_);
            if (emote_display == nullptr || emote_display->GetEmoteHandle() == nullptr || emote_clock_digits_[0].seg[0] == nullptr) {
                return;
            }
            std::time_t now = std::time(nullptr);
            std::tm local_tm = {};
            localtime_r(&now, &local_tm);
            emote_lock(emote_display->GetEmoteHandle());
            SetEmoteDigitValue(emote_clock_digits_[0], local_tm.tm_hour / 10);
            SetEmoteDigitValue(emote_clock_digits_[1], local_tm.tm_hour % 10);
            SetEmoteDigitValue(emote_clock_digits_[2], local_tm.tm_min / 10);
            SetEmoteDigitValue(emote_clock_digits_[3], local_tm.tm_min % 10);
            emote_unlock(emote_display->GetEmoteHandle());
            emote_notify_all_refresh(emote_display->GetEmoteHandle());
            return;
        }
    #endif
        if (clock_digits_[0].seg[0] == nullptr) {
            return;
        }
        std::time_t now = std::time(nullptr);
        std::tm local_tm = {};
        localtime_r(&now, &local_tm);

        if (!lvgl_port_lock(30000)) {
            return;
        }
        SetDigitValue(clock_digits_[0], local_tm.tm_hour / 10);
        SetDigitValue(clock_digits_[1], local_tm.tm_hour % 10);
        SetDigitValue(clock_digits_[2], local_tm.tm_min / 10);
        SetDigitValue(clock_digits_[3], local_tm.tm_min % 10);
        lvgl_port_unlock();
    }

    void ShowClockScreen()
    {
#if CONFIG_USE_EMOTE_MESSAGE_STYLE
        auto* emote_display = dynamic_cast<emote::EmoteDisplay*>(display_);
        if (emote_display == nullptr || emote_display->GetEmoteHandle() == nullptr) {
            return;
        }
        // Enter full-screen clock page: prevent Application state updates from re-showing small clock_label.
        emote_display->SetClockPageMode(true);
        auto handle = emote_display->GetEmoteHandle();
        emote_set_event_msg(handle, EMOTE_MGR_EVT_IDLE, NULL);
        emote_lock(handle);
        if (emote_clock_digits_[0].seg[0] == nullptr) {
            const int digit_w = (DISPLAY_WIDTH >= 320) ? 64 : 48;
            const int digit_h = (DISPLAY_HEIGHT >= 320) ? 140 : 100;
            const int thickness = (DISPLAY_WIDTH >= 320) ? 12 : 9;
            const int gap = (DISPLAY_WIDTH >= 320) ? 10 : 8;
            const int colon_w = thickness;
            const int total_w = digit_w * 4 + gap * 4 + colon_w;
            const int start_x = (DISPLAY_WIDTH - total_w) / 2;
            const int start_y = (DISPLAY_HEIGHT - digit_h) / 2;

            CreateEmoteClockDigit(handle, emote_clock_digits_[0], 0, start_x, start_y, digit_w, digit_h, thickness);
            CreateEmoteClockDigit(handle, emote_clock_digits_[1], 1, start_x + digit_w + gap, start_y, digit_w, digit_h, thickness);
            CreateEmoteClockDigit(handle, emote_clock_digits_[2], 2, start_x + digit_w * 2 + gap * 3 + colon_w, start_y, digit_w, digit_h, thickness);
            CreateEmoteClockDigit(handle, emote_clock_digits_[3], 3, start_x + digit_w * 3 + gap * 4 + colon_w, start_y, digit_w, digit_h, thickness);

            const int dot = thickness;
            const int colon_x = start_x + digit_w * 2 + gap * 2;
            const int top_dot_y = start_y + digit_h / 2 - dot - 8;
            const int bottom_dot_y = start_y + digit_h / 2 + 8;
            emote_clock_colon_top_ = CreateEmoteClockSegment(handle, "vocat_clk_colon_top", colon_x, top_dot_y, dot, dot);
            emote_clock_colon_bottom_ = CreateEmoteClockSegment(handle, "vocat_clk_colon_bottom", colon_x, bottom_dot_y, dot, dot);
        }
        gfx_obj_t* default_clock = emote_get_obj_by_name(handle, EMT_DEF_ELEM_CLOCK_LABEL);
        gfx_obj_t* eye_anim = emote_get_obj_by_name(handle, EMT_DEF_ELEM_EYE_ANIM);
        gfx_obj_t* status_icon = emote_get_obj_by_name(handle, EMT_DEF_ELEM_STATUS_ICON);
        gfx_obj_t* charge_icon = emote_get_obj_by_name(handle, EMT_DEF_ELEM_CHARGE_ICON);
        gfx_obj_t* bat_label = emote_get_obj_by_name(handle, EMT_DEF_ELEM_BAT_LEFT_LABEL);
        gfx_obj_t* clock_timer = emote_get_obj_by_name(handle, EMT_DEF_ELEM_TIMER_STATUS);
        if (default_clock) gfx_obj_set_visible(default_clock, false);
        // Pause the emote internal timer that keeps updating the small clock label.
        // Otherwise even if we hide it, it may reappear on next status refresh.
        if (clock_timer) {
            gfx_timer_pause((gfx_timer_handle_t)clock_timer);
            gfx_obj_set_visible(clock_timer, false);
        }
        if (eye_anim) gfx_obj_set_visible(eye_anim, false);
        if (status_icon) gfx_obj_set_visible(status_icon, false);
        if (charge_icon) gfx_obj_set_visible(charge_icon, false);
        if (bat_label) gfx_obj_set_visible(bat_label, false);
        for (int d = 0; d < 4; ++d) {
            for (int s = 0; s < 7; ++s) {
                if (emote_clock_digits_[d].seg[s]) {
                    gfx_obj_set_visible(emote_clock_digits_[d].seg[s], true);
                }
            }
        }
        if (emote_clock_colon_top_) gfx_obj_set_visible(emote_clock_colon_top_, true);
        if (emote_clock_colon_bottom_) gfx_obj_set_visible(emote_clock_colon_bottom_, true);
        emote_unlock(handle);
        emote_notify_all_refresh(handle);
        emote_clock_mode_ = true;
        UpdateClockText();
        return;
    #else
        if (!lvgl_port_lock(30000)) {
            return;
        }

        if (clock_screen_ == nullptr) {
            clock_screen_ = lv_obj_create(lv_screen_active());
            lv_obj_remove_style_all(clock_screen_);
            lv_obj_set_size(clock_screen_, DISPLAY_WIDTH, DISPLAY_HEIGHT);
            lv_obj_set_style_bg_opa(clock_screen_, LV_OPA_COVER, 0);
            lv_obj_set_style_bg_color(clock_screen_, lv_color_black(), 0);
            const int digit_w = (DISPLAY_WIDTH >= 320) ? 64 : 48;
            const int digit_h = (DISPLAY_HEIGHT >= 320) ? 140 : 100;
            const int thickness = (DISPLAY_WIDTH >= 320) ? 12 : 9;
            const int gap = (DISPLAY_WIDTH >= 320) ? 10 : 8;
            const int colon_w = thickness;
            const int total_w = digit_w * 4 + gap * 4 + colon_w;
            const int start_x = (DISPLAY_WIDTH - total_w) / 2;
            const int start_y = (DISPLAY_HEIGHT - digit_h) / 2;

            CreateClockDigit(clock_digits_[0], clock_screen_, start_x, start_y, digit_w, digit_h, thickness);
            CreateClockDigit(clock_digits_[1], clock_screen_, start_x + digit_w + gap, start_y, digit_w, digit_h, thickness);
            CreateClockDigit(clock_digits_[2], clock_screen_, start_x + digit_w * 2 + gap * 3 + colon_w, start_y, digit_w, digit_h, thickness);
            CreateClockDigit(clock_digits_[3], clock_screen_, start_x + digit_w * 3 + gap * 4 + colon_w, start_y, digit_w, digit_h, thickness);

            const int dot = thickness;
            const int colon_x = start_x + digit_w * 2 + gap * 2;
            const int top_dot_y = start_y + digit_h / 2 - dot - 8;
            const int bottom_dot_y = start_y + digit_h / 2 + 8;

            clock_colon_top_ = lv_obj_create(clock_screen_);
            lv_obj_remove_style_all(clock_colon_top_);
            lv_obj_set_pos(clock_colon_top_, colon_x, top_dot_y);
            lv_obj_set_size(clock_colon_top_, dot, dot);
            lv_obj_set_style_bg_opa(clock_colon_top_, LV_OPA_COVER, 0);
            lv_obj_set_style_bg_color(clock_colon_top_, lv_color_white(), 0);
            lv_obj_set_style_radius(clock_colon_top_, LV_RADIUS_CIRCLE, 0);

            clock_colon_bottom_ = lv_obj_create(clock_screen_);
            lv_obj_remove_style_all(clock_colon_bottom_);
            lv_obj_set_pos(clock_colon_bottom_, colon_x, bottom_dot_y);
            lv_obj_set_size(clock_colon_bottom_, dot, dot);
            lv_obj_set_style_bg_opa(clock_colon_bottom_, LV_OPA_COVER, 0);
            lv_obj_set_style_bg_color(clock_colon_bottom_, lv_color_white(), 0);
            lv_obj_set_style_radius(clock_colon_bottom_, LV_RADIUS_CIRCLE, 0);
        }

        lv_obj_clear_flag(clock_screen_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(clock_screen_);
        clock_wallpaper_mode_ = true;
        lvgl_port_unlock();

        UpdateClockText();

        if (clock_timer_ == nullptr) {
            const esp_timer_create_args_t timer_args = {
                .callback = clock_timer_callback,
                .arg = this,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "vocat_clock",
                .skip_unhandled_events = true,
            };
            ESP_ERROR_CHECK(esp_timer_create(&timer_args, &clock_timer_));
            ESP_ERROR_CHECK(esp_timer_start_periodic(clock_timer_, 1000000));
        }
#endif
    }

    void HideClockScreen()
    {
#if CONFIG_USE_EMOTE_MESSAGE_STYLE
        auto* emote_display = dynamic_cast<emote::EmoteDisplay*>(display_);
        if (emote_display == nullptr || emote_display->GetEmoteHandle() == nullptr) {
            return;
        }
        // Exit full-screen clock page mode.
        emote_display->SetClockPageMode(false);
        auto handle = emote_display->GetEmoteHandle();
        /* If this hide is triggered by voice conversation, don't force emote into IDLE.
         * Otherwise it may temporarily override listen/speak status updates and cause overlap. */
        if (!suppress_emote_idle_on_hide_) {
            emote_set_event_msg(handle, EMOTE_MGR_EVT_IDLE, NULL);
        }
        emote_lock(handle);
        for (int d = 0; d < 4; ++d) {
            for (int s = 0; s < 7; ++s) {
                if (emote_clock_digits_[d].seg[s]) {
                    gfx_obj_set_visible(emote_clock_digits_[d].seg[s], false);
                }
            }
        }
        if (emote_clock_colon_top_) gfx_obj_set_visible(emote_clock_colon_top_, false);
        if (emote_clock_colon_bottom_) gfx_obj_set_visible(emote_clock_colon_bottom_, false);
        gfx_obj_t* default_clock = emote_get_obj_by_name(handle, EMT_DEF_ELEM_CLOCK_LABEL);
        gfx_obj_t* eye_anim = emote_get_obj_by_name(handle, EMT_DEF_ELEM_EYE_ANIM);
        gfx_obj_t* status_icon = emote_get_obj_by_name(handle, EMT_DEF_ELEM_STATUS_ICON);
        gfx_obj_t* charge_icon = emote_get_obj_by_name(handle, EMT_DEF_ELEM_CHARGE_ICON);
        gfx_obj_t* bat_label = emote_get_obj_by_name(handle, EMT_DEF_ELEM_BAT_LEFT_LABEL);
        gfx_obj_t* clock_timer = emote_get_obj_by_name(handle, EMT_DEF_ELEM_TIMER_STATUS);
        if (default_clock) gfx_obj_set_visible(default_clock, true);
        // Resume emote internal timer for the small clock label.
        if (clock_timer) {
            gfx_timer_resume((gfx_timer_handle_t)clock_timer);
            gfx_obj_set_visible(clock_timer, true);
        }
        if (eye_anim) gfx_obj_set_visible(eye_anim, true);
        if (status_icon) gfx_obj_set_visible(status_icon, true);
        if (charge_icon) gfx_obj_set_visible(charge_icon, true);
        if (bat_label) gfx_obj_set_visible(bat_label, true);
        emote_unlock(handle);
        // During voice state transitions, EmoteDisplay will refresh on LISTEN/SPEAK anyway.
        // Skipping notify here avoids heavy redraw blocking audio-related state handling.
        if (!suppress_emote_idle_on_hide_) {
            emote_notify_all_refresh(handle);
        }
        emote_clock_mode_ = false;
        return;
#endif

        if (!clock_wallpaper_mode_) {
            return;
        }
        if (!lvgl_port_lock(30000)) {
            return;
        }
        if (clock_screen_ != nullptr) {
            lv_obj_add_flag(clock_screen_, LV_OBJ_FLAG_HIDDEN);
        }
        clock_wallpaper_mode_ = false;
        /* Ensure the uncovered UI is redrawn immediately.
         * Without this, LVGL may keep stale pixels until a later full/UI event (e.g. voice wake). */
        lv_refr_now(lv_display_get_default());
        lvgl_port_unlock();
    }

    static void clock_restore_timer_callback(void* arg) {
        auto* board = static_cast<EspVocat*>(arg);
        if (board != nullptr) {
            board->OnClockRestoreTimeout();
        }
    }

    void StopClockRestoreTimer() {
        if (clock_restore_timer_ != nullptr) {
            esp_timer_stop(clock_restore_timer_);
        }
    }

    void CancelClockRestore() {
        clock_restore_pending_ = false;
        StopClockRestoreTimer();
    }

    void StartClockRestoreTimer() {
        clock_restore_pending_ = true;
        StopClockRestoreTimer();

        if (clock_restore_timer_ == nullptr) {
            const esp_timer_create_args_t timer_args = {
                .callback = clock_restore_timer_callback,
                .arg = this,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "vocat_clock_restore",
                .skip_unhandled_events = true,
            };
            ESP_ERROR_CHECK(esp_timer_create(&timer_args, &clock_restore_timer_));
        }

        // Restore clock 1 minute after conversation ends.
        ESP_LOGI(TAG, "Schedule clock restore in 60s");
        ESP_ERROR_CHECK(esp_timer_start_once(clock_restore_timer_, 60LL * 1000000LL));
    }

    void OnClockRestoreTimeout() {
        if (!clock_restore_pending_) {
            return;
        }
        auto& app = Application::GetInstance();
        if (app.GetDeviceState() != kDeviceStateIdle) {
            return;
        }
        clock_restore_pending_ = false;
        ESP_LOGI(TAG, "Clock restore timeout, show clock page");
        ShowClockScreen();
    }

    void ChangeBacklightBrightness(int delta)
    {
        if (backlight_ == nullptr) {
            return;
        }
        int value = static_cast<int>(backlight_->brightness()) + delta;
        if (value < 1) {
            value = 1;
        }
        if (value > 100) {
            value = 100;
        }
        backlight_->SetBrightness(static_cast<uint8_t>(value), true);
        if (display_ != nullptr) {
            display_->ShowNotification(std::string("Brightness: ") + std::to_string(value), 1000);
        }
    }

    void HandleTouchRelease()
    {
        auto &app = Application::GetInstance();
        if (app.GetDeviceState() == kDeviceStateStarting) {
            EnterWifiConfigMode();
            return;
        }

        auto point = cst816s_->GetTouchPoint();
        if (point.y >= 0 && point.y < (DISPLAY_HEIGHT / 3)) {
            ChangeBacklightBrightness(touch_brightness_step_);
            return;
        }
        if (point.y >= ((DISPLAY_HEIGHT * 2) / 3) && point.y < DISPLAY_HEIGHT) {
            ChangeBacklightBrightness(-touch_brightness_step_);
            return;
        }
        app.ToggleChatState();
    }

    bool IsNightScreenOffHour(int hour) const
    {
        if (screen_off_start_hour_ == screen_off_end_hour_) {
            return false;
        }
        if (screen_off_start_hour_ < screen_off_end_hour_) {
            return hour >= screen_off_start_hour_ && hour < screen_off_end_hour_;
        }
        return hour >= screen_off_start_hour_ || hour < screen_off_end_hour_;
    }

    void ReloadPowerPolicySettings(bool force = false)
    {
        int64_t now_us = esp_timer_get_time();
        if (!force && now_us < next_settings_reload_us_) {
            return;
        }

        Settings settings("wifi", false);
        int32_t shutdown_delay = settings.GetInt("power_off_delay_min", 10);
        if (shutdown_delay < 1) {
            shutdown_delay = 1;
        }
        if (shutdown_delay > 720) {
            shutdown_delay = 720;
        }
        shutdown_delay_min_ = shutdown_delay;

        int32_t start_hour = settings.GetInt("screen_off_start_hour", 1);
        if (start_hour < 0) {
            start_hour = 0;
        }
        if (start_hour > 23) {
            start_hour = 23;
        }
        screen_off_start_hour_ = start_hour;

        int32_t end_hour = settings.GetInt("screen_off_end_hour", 8);
        if (end_hour < 0) {
            end_hour = 0;
        }
        if (end_hour > 23) {
            end_hour = 23;
        }
        screen_off_end_hour_ = end_hour;
        next_settings_reload_us_ = now_us + 60LL * 1000000LL;
    }

    void UpdateNightDisplayPolicy()
    {
        if (backlight_ == nullptr) {
            return;
        }
        std::time_t now = std::time(nullptr);
        std::tm local_tm = {};
        localtime_r(&now, &local_tm);
        int year = local_tm.tm_year + 1900;
        if (year < 2024) {
            return;
        }

        bool should_off = IsNightScreenOffHour(local_tm.tm_hour);
        if (should_off && !screen_off_by_schedule_) {
            backlight_->SetBrightness(0);
            screen_off_by_schedule_ = true;
        } else if (!should_off && screen_off_by_schedule_) {
            backlight_->RestoreBrightness();
            screen_off_by_schedule_ = false;
        }
    }

    void ShutdownDueToPowerLoss()
    {
        ESP_LOGW(TAG, "Power disconnected timeout reached, shutting down");
        gpio_set_level(POWER_CTRL, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_deep_sleep_start();
    }

    void UpdatePowerConnectionPolicy()
    {
        if (charge_ == nullptr) {
            return;
        }
        bool connected = charge_->IsExternalPowerConnected();
        int64_t now_us = esp_timer_get_time();
        if (connected != external_power_connected_) {
            external_power_connected_ = connected;
            if (connected) {
                gpio_set_level(POWER_CTRL, 0);
                power_loss_countdown_active_ = false;
                if (!screen_off_by_schedule_ && backlight_ != nullptr) {
                    backlight_->RestoreBrightness();
                }
                ESP_LOGI(TAG, "External power connected, auto power-on policy applied");
            } else {
                power_loss_countdown_active_ = true;
                power_loss_deadline_us_ = now_us + static_cast<int64_t>(shutdown_delay_min_) * 60LL * 1000000LL;
                ESP_LOGI(TAG, "External power disconnected, shutdown in %ld minutes", static_cast<long>(shutdown_delay_min_));
            }
        }

        if (!external_power_connected_ && power_loss_countdown_active_ && now_us >= power_loss_deadline_us_) {
            ShutdownDueToPowerLoss();
        }
    }

    static void power_policy_task(void* arg)
    {
        auto* board = static_cast<EspVocat*>(arg);
        if (board == nullptr) {
            vTaskDelete(NULL);
            return;
        }
        board->ReloadPowerPolicySettings(true);
        while (true) {
            board->ReloadPowerPolicySettings();
            board->UpdatePowerConnectionPolicy();
            board->UpdateNightDisplayPolicy();
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }

    void InitializeI2c()
    {
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));

        temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
        ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));
        ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));

    }
    uint8_t DetectPcbVersion()
    {
        esp_err_t ret = i2c_master_probe(i2c_bus_, 0x18, 100);
        uint8_t pcb_version = 0;
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "PCB version V1.0");
            pcb_version = 0;
        } else {
            gpio_config_t gpio_conf = {
                .pin_bit_mask = (1ULL << GPIO_NUM_48),
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE
            };
            ESP_ERROR_CHECK(gpio_config(&gpio_conf));
            ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_48, 1));
            vTaskDelay(pdMS_TO_TICKS(100));
            ret = i2c_master_probe(i2c_bus_, 0x18, 100);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "PCB version V1.2");
                pcb_version = 1;
                AUDIO_I2S_GPIO_DIN = AUDIO_I2S_GPIO_DIN_2;
                AUDIO_CODEC_PA_PIN = AUDIO_CODEC_PA_PIN_2;
                QSPI_PIN_NUM_LCD_RST = QSPI_PIN_NUM_LCD_RST_2;
                TOUCH_PAD2 = TOUCH_PAD2_2;
                UART1_TX = UART1_TX_2;
                UART1_RX = UART1_RX_2;
            } else {
                ESP_LOGE(TAG, "PCB version detection error");

            }
        }
        return pcb_version;
    }

    static void touch_isr_callback(void* arg)
    {
        Cst816s* touchpad = static_cast<Cst816s*>(arg);
        if (touchpad != nullptr) {
            touchpad->NotifyTouchEvent();
        }
    }

    static void touch_event_task(void* arg)
    {
        Cst816s* touchpad = static_cast<Cst816s*>(arg);
        if (touchpad == nullptr) {
            ESP_LOGE(TAG, "Invalid touchpad pointer in touch_event_task");
            vTaskDelete(NULL);
            return;
        }

        while (true) {
            if (touchpad->WaitForTouchEvent()) {
                auto &board = (EspVocat &)Board::GetInstance();

                ESP_LOGD(TAG, "Touch event, TP_PIN_NUM_INT: %d", gpio_get_level(TP_PIN_NUM_INT));
                touchpad->UpdateTouchPoint();
                auto touch_event = touchpad->CheckTouchEvent();

                if (touch_event == Cst816s::TOUCH_RELEASE) {
                    if (board.clock_wallpaper_mode_
#if CONFIG_USE_EMOTE_MESSAGE_STYLE
                        || board.emote_clock_mode_
#endif
                    ) {
                        continue;
                    }
                    board.HandleTouchRelease();
                } else if (touch_event == Cst816s::TOUCH_LONG_PRESS) {
                    board.EnterWifiConfigMode();
                } else if (touch_event == Cst816s::TOUCH_SWIPE_LEFT) {
                    board.CancelClockRestore();
                    board.ShowClockScreen();
                } else if (touch_event == Cst816s::TOUCH_SWIPE_RIGHT) {
                    board.CancelClockRestore();
                    board.HideClockScreen();
                }
            }
        }
    }

    void InitializeCharge()
    {
        charge_ = new Charge(i2c_bus_, 0x55);
        xTaskCreatePinnedToCore(Charge::TaskFunction, "batterydecTask", 3 * 1024, charge_, 6, &charge_task_handle_, 0);
    }

    void InitializePowerPolicy()
    {
        external_power_connected_ = true;
        power_loss_countdown_active_ = false;
        xTaskCreatePinnedToCore(power_policy_task, "power_policy_task", 4 * 1024, this, 4, &power_policy_task_handle_, 1);
    }

    void InitializeCst816sTouchPad()
    {
        cst816s_ = new Cst816s(i2c_bus_, 0x15);

        xTaskCreatePinnedToCore(touch_event_task, "touch_task", 4 * 1024, cst816s_, 5, &touch_task_handle_, 1);

        const gpio_config_t int_gpio_config = {
            .pin_bit_mask = (1ULL << TP_PIN_NUM_INT),
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_ANYEDGE
        };
        ESP_ERROR_CHECK(gpio_config(&int_gpio_config));
        esp_err_t isr_ret = gpio_install_isr_service(0);
        if (isr_ret != ESP_OK && isr_ret != ESP_ERR_INVALID_STATE) {
            ESP_ERROR_CHECK(isr_ret);
        }
        ESP_ERROR_CHECK(gpio_intr_enable(TP_PIN_NUM_INT));
        ESP_ERROR_CHECK(gpio_isr_handler_add(TP_PIN_NUM_INT, EspVocat::touch_isr_callback, cst816s_));
    }

    void InitializeSpi()
    {
        const spi_bus_config_t bus_config = TAIJIPI_ST77916_PANEL_BUS_QSPI_CONFIG(QSPI_PIN_NUM_LCD_PCLK,
                                                                                  QSPI_PIN_NUM_LCD_DATA0,
                                                                                  QSPI_PIN_NUM_LCD_DATA1,
                                                                                  QSPI_PIN_NUM_LCD_DATA2,
                                                                                  QSPI_PIN_NUM_LCD_DATA3,
                                                                                  QSPI_LCD_H_RES * 80 * sizeof(uint16_t));
        ESP_ERROR_CHECK(spi_bus_initialize(QSPI_LCD_HOST, &bus_config, SPI_DMA_CH_AUTO));
    }

    void InitializeSt77916Display(uint8_t pcb_version)
    {

        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        const esp_lcd_panel_io_spi_config_t io_config = ST77916_PANEL_IO_QSPI_CONFIG(QSPI_PIN_NUM_LCD_CS, NULL, NULL);
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)QSPI_LCD_HOST, &io_config, &panel_io));
        st77916_vendor_config_t vendor_config = {
            .init_cmds = vendor_specific_init_yysj,
            .init_cmds_size = sizeof(vendor_specific_init_yysj) / sizeof(st77916_lcd_init_cmd_t),
            .flags = {
                .use_qspi_interface = 1,
            },
        };
        const esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = QSPI_PIN_NUM_LCD_RST,
            .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
            .bits_per_pixel = QSPI_LCD_BIT_PER_PIXEL,
            .flags = {
                .reset_active_high = pcb_version,
            },
            .vendor_config = &vendor_config,
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_st77916(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_disp_on_off(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

#if CONFIG_USE_EMOTE_MESSAGE_STYLE
        display_ = new emote::EmoteDisplay(panel, panel_io, DISPLAY_WIDTH, DISPLAY_HEIGHT);
#else
        display_ = new SpiLcdDisplay(panel_io, panel,
            DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
#endif
        backlight_ = new PwmBacklight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        backlight_->RestoreBrightness();
    }

    void InitializeButtons()
    {
        boot_button_.OnClick([this]() {
            auto &app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                ESP_LOGI(TAG, "Boot button pressed, enter WiFi configuration mode");
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
        gpio_config_t power_gpio_config = {
            .pin_bit_mask = (BIT64(POWER_CTRL)),
            .mode = GPIO_MODE_OUTPUT,

        };
        ESP_ERROR_CHECK(gpio_config(&power_gpio_config));

        gpio_set_level(POWER_CTRL, 0);
    }

#ifdef CONFIG_ESP_VIDEO_ENABLE_USB_UVC_VIDEO_DEVICE
    void InitializeCamera() {
        esp_video_init_usb_uvc_config_t usb_uvc_config = {
            .uvc = {
                .uvc_dev_num = 1,
                .task_stack = 4096,
                .task_priority = 5,
                .task_affinity = -1,
            },
            .usb = {
                .init_usb_host_lib = true,
                .task_stack = 4096,
                .task_priority = 5,
                .task_affinity = -1,
            },
        };

        esp_video_init_config_t video_config = {
            .usb_uvc = &usb_uvc_config,
        };

        camera_ = new EspVideo(video_config);
    }
#endif // CONFIG_ESP_VIDEO_ENABLE_USB_UVC_VIDEO_DEVICE

public:
    void OnDeviceStateChanged(DeviceState /*old_state*/, DeviceState new_state) override
    {
        bool clock_active = false;
#if CONFIG_USE_EMOTE_MESSAGE_STYLE
        clock_active = emote_clock_mode_;
#else
        clock_active = clock_wallpaper_mode_;
#endif

        const bool entering_conversation =
            (new_state == kDeviceStateConnecting ||
             new_state == kDeviceStateListening ||
             new_state == kDeviceStateSpeaking ||
             new_state == kDeviceStateActivating);

        if (entering_conversation) {
            // Conversation starts -> stop any scheduled restore.
            StopClockRestoreTimer();

            if (clock_active) {
                // Remember we need to restore clock when conversation ends.
                clock_restore_pending_ = true;

                // Hide clock overlay immediately.
                ESP_LOGI(TAG, "Voice state -> hide clock overlay (restore after dialog)");
                suppress_emote_idle_on_hide_ = true;
                HideClockScreen();
                suppress_emote_idle_on_hide_ = false;
            }
        } else if (new_state == kDeviceStateIdle) {
            if (clock_restore_pending_) {
                StartClockRestoreTimer();
            }
        }
    }

    ~EspVocat() {
        // Stop tasks
        if (charge_task_handle_ != nullptr) {
            vTaskDelete(charge_task_handle_);
        }
        if (touch_task_handle_ != nullptr) {
            vTaskDelete(touch_task_handle_);
        }
        if (power_policy_task_handle_ != nullptr) {
            vTaskDelete(power_policy_task_handle_);
        }
        if (clock_timer_ != nullptr) {
            esp_timer_stop(clock_timer_);
            esp_timer_delete(clock_timer_);
            clock_timer_ = nullptr;
        }
        if (clock_restore_timer_ != nullptr) {
            esp_timer_stop(clock_restore_timer_);
            esp_timer_delete(clock_restore_timer_);
            clock_restore_timer_ = nullptr;
        }

        // Delete objects
        delete charge_;
        delete cst816s_;
        delete display_;
        // Note: backlight_ (PwmBacklight) and camera_ (EspVideo) are not deleted here
        // because their base classes (Backlight, Camera) don't have virtual destructors.
        // Since EspVocat is a singleton that lives for the device lifetime, this is acceptable.

        // Remove GPIO ISR handler
        gpio_isr_handler_remove(TP_PIN_NUM_INT);

        // Disable temperature sensor
        if (temp_sensor != NULL) {
            temperature_sensor_disable(temp_sensor);
            temperature_sensor_uninstall(temp_sensor);
            temp_sensor = NULL;
        }
    }

    EspVocat() : boot_button_(BOOT_BUTTON_GPIO)
    {
        InitializeI2c();
        uint8_t pcb_version = DetectPcbVersion();
        InitializeCharge();
        InitializePowerPolicy();
        InitializeCst816sTouchPad();

        InitializeSpi();
        InitializeSt77916Display(pcb_version);
        InitializeButtons();
#ifdef CONFIG_ESP_VIDEO_ENABLE_USB_UVC_VIDEO_DEVICE
        InitializeCamera();
#endif // CONFIG_ESP_VIDEO_ENABLE_USB_UVC_VIDEO_DEVICE
    }

    virtual AudioCodec* GetAudioCodec() override
    {
        static BoxAudioCodec audio_codec(
            i2c_bus_,
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK,
            AUDIO_I2S_GPIO_BCLK,
            AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN,
            AUDIO_CODEC_ES8311_ADDR,
            AUDIO_CODEC_ES7210_ADDR,
            AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override
    {
        return display_;
    }

    Cst816s* GetTouchpad()
    {
        return cst816s_;
    }

    virtual Backlight* GetBacklight() override
    {
        return backlight_;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }
};

DECLARE_BOARD(EspVocat);
