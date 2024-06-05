#pragma once

#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <atomic>
#include <chrono>
#include <future>
#include <list>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

class Ultrasonic {
private:
    enum class State {
        NotInitialized,
        Idle,
        Measuring,
        Finished,
    };

    struct SensorData {
        const gpio_num_t trigPin;
        const gpio_num_t echoPin;

        std::atomic<State> state = State::NotInitialized;

        std::promise<double> promise;

        std::int64_t start = 0;
        std::int64_t end = 0;

        SensorData(gpio_num_t trigPin, gpio_num_t echoPin)
            : trigPin(trigPin)
            , echoPin(echoPin)
        {
        }

        SensorData(const SensorData&) = delete;
        SensorData(SensorData&&) = delete;

        SensorData& operator=(const SensorData&) = delete;
        SensorData& operator=(SensorData&&) = delete;

        ~SensorData() = default;
    };

    static std::list<SensorData>& getSensors() {
        static std::list<SensorData> sensors;
        return sensors;
    }

    std::list<SensorData>& s_sensors;
    std::list<SensorData>::iterator m_handle;

    static void staticInit() {
        static std::once_flag flag;
        std::call_once(flag,
            []() { ESP_ERROR_CHECK(gpio_install_isr_service(0)); });
    }

    static void threadFunction() {
        using namespace std::chrono_literals;
        try {
            while (true) {
                for (auto& sensor : getSensors()) {
                    if (sensor.state == State::Finished) {
                        sensor.promise.set_value((sensor.end - sensor.start) / 2.0 / 2.91);
                        sensor.state = State::Idle;
                    }
                }
                std::this_thread::sleep_for(10ms); // default=10ms
            }
        } catch (const std::exception& e) {
            ESP_LOGE("Ultrasonic", "Exception occured: %s", e.what());
            while (true) {
                std::this_thread::sleep_for(1000ms);
            }
        }
    }

    static void echoTrampoline(void* arg) {
        static_cast<Ultrasonic*>(arg)->echoHandler();
    }

    void echoHandler() {
        if (gpio_get_level(m_handle->echoPin) == 1) {
            m_handle->start = esp_timer_get_time();
        } else {
            if (m_handle->state == State::Measuring) {
                m_handle->end = esp_timer_get_time();
                m_handle->state = State::Finished;
            }
        }
    }

    void trigger() {
        using namespace std::chrono_literals;

        if (m_handle->state != State::Idle) {
            m_handle->promise.set_exception(
                std::make_exception_ptr(std::runtime_error("Already measuring")));
            return;
        }

        gpio_set_level(m_handle->trigPin, 1);
        std::this_thread::sleep_for(10us);
        gpio_set_level(m_handle->trigPin, 0);

        m_handle->state = State::Measuring;
    }

public:
    Ultrasonic(gpio_num_t trigPin, gpio_num_t echoPin)
        : s_sensors(getSensors())
        , m_handle(s_sensors.emplace(s_sensors.end(), trigPin, echoPin)) {
        // m_handle = s_sensors.emplace(s_sensors.end(), trigPin, echoPin);
        static std::thread thread(threadFunction);
    }

    ~Ultrasonic() {
        if (m_handle->state != State::NotInitialized)
            deinit();
        s_sensors.erase(m_handle);
    }

    Ultrasonic(const Ultrasonic&) = delete;
    Ultrasonic(Ultrasonic&& other) = default;

    Ultrasonic& operator=(const Ultrasonic&) = delete;
    Ultrasonic& operator=(Ultrasonic&&) = default;

    void init() {
        if (m_handle->state != State::NotInitialized)
            return;

        staticInit();

        gpio_config_t o_conf = {
            .pin_bit_mask = (1ULL << m_handle->trigPin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&o_conf));

        gpio_config_t i_conf = {
            .pin_bit_mask = (1ULL << m_handle->echoPin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE,
        };
        ESP_ERROR_CHECK(gpio_config(&i_conf));

        ESP_ERROR_CHECK(
            gpio_isr_handler_add(m_handle->echoPin, echoTrampoline, this));
        m_handle->state = State::Idle;
    }

    void deinit() {
        ESP_ERROR_CHECK(gpio_isr_handler_remove(m_handle->echoPin));
        m_handle->state = State::NotInitialized;
    }

    /**
     * @brief Measure the distance to the object in front of the sensor.
     *
     * @return std::future<double> A future that will be set when the measurement
     * is finished, containing the distance in mm.
     */
    std::future<double> measure() {
        m_handle->promise = std::promise<double>();
        trigger();
        return m_handle->promise.get_future();
    }

    /**
     * @brief Measure the distance to the object in front of the sensor.
     *
     * @param timeout The maximum time to wait for the measurement to finish.
     * @return std::optional<double> The distance in mm, or std::nullopt if the
     * measurement timed out.
     */
    std::optional<double>
    measureSync(std::chrono::milliseconds timeout = std::chrono::milliseconds(1000)) {
        auto result = measure();
        auto status = result.wait_for(timeout);
        if (status == std::future_status::timeout)
            return std::nullopt;
        try {
            return result.get();
        } catch (const std::exception& e) {
            ESP_LOGE("Ultrasonic", "Exception occured: %s", e.what());
            return std::nullopt;
        }
    }
};

class UltrasonicGroup {
private:
    std::vector<Ultrasonic> m_sensors;

public:
    UltrasonicGroup(
        std::initializer_list<std::tuple<gpio_num_t, gpio_num_t>> sensors) {
        m_sensors.reserve(sensors.size());
        for (auto& [trigPin, echoPin] : sensors) {
            m_sensors.emplace_back(trigPin, echoPin);
        }
    }

    ~UltrasonicGroup() = default;

    UltrasonicGroup(const UltrasonicGroup&) = delete;
    UltrasonicGroup(UltrasonicGroup&&) = default;

    UltrasonicGroup& operator=(const UltrasonicGroup&) = delete;
    UltrasonicGroup& operator=(UltrasonicGroup&&) = default;

    void addSensor(gpio_num_t trigPin, gpio_num_t echoPin) {
        m_sensors.emplace_back(trigPin, echoPin);
    }

    void init() {
        for (auto& sensor : m_sensors) {
            sensor.init();
        }
    }

    void deinit() {
        for (auto& sensor : m_sensors) {
            sensor.deinit();
        }
    }

    /**
     * @brief Measure the distance to the objects in front of the sensors.
     *
     * @param timeout The maximum time to wait for the measurements to finish.
     * @return std::vector<std::future<double>> A vector containing futures that
     * will be set when the measurements are finished, containing the distances in
     * mm.
     */

    std::future<std::vector<double>> measure() {
        std::vector<std::future<double>> results;
        results.reserve(m_sensors.size());
        for (auto& sensor : m_sensors) {
            results.push_back(sensor.measure());
        }

        return std::async(std::launch::async,
            [results = std::move(results)]() mutable {
                std::vector<double> distances;
                distances.reserve(results.size());
                for (auto& result : results) {
                    distances.push_back(result.get());
                }
                return distances;
            });
    }

    /**
     * @brief Measure the distance to the objects in front of the sensors.
     *
     * @param timeout The maximum time to wait for the measurements to finish.
     * @return std::vector<std::optional<double>> A vector containing the
     * distances in mm, or std::nullopt if the measurement timed out.
     */
    std::vector<std::optional<double>>
    measureSync(std::chrono::milliseconds timeout) {
        std::vector<std::future<double>> results;
        results.reserve(m_sensors.size());
        for (auto& sensor : m_sensors) {
            results.push_back(sensor.measure());
        }

        std::vector<std::optional<double>> distances;
        distances.reserve(m_sensors.size());

        auto now = std::chrono::steady_clock::now();
        auto end = now + timeout;
        for (auto& result : results) {
            auto status = result.wait_until(end);
            if (status == std::future_status::timeout) {
                distances.push_back(std::nullopt);
            } else {
                distances.push_back(result.get());
            }
        }

        return distances;
    }
};