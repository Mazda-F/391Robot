#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

void Kprint(float Kp, float Ki, float Kd);

class TimerDecorator {
private:
    const char* function_id;

public:
    TimerDecorator(const char* id);
    
    template <typename Func, typename... Args>
    auto operator()(Func func, Args&&... args) {
    unsigned long start_time = micros();
    
    // if constexpr (std::is_void_v<std::invoke_result_t<Func, Args...>>) {
    func(std::forward<Args>(args)...); 
    unsigned long end_time = micros();
    Serial.print("Function ID: ");
    Serial.print(function_id);
    Serial.print(" | Execution Time: ");
    Serial.print(end_time - start_time);
    Serial.println(" us");
    // } else {
    //     auto result = func(std::forward<Args>(args)...); 
    //     unsigned long end_time = micros();
    //     Serial.print("Function ID: ");
    //     Serial.print(function_id);
    //     Serial.print(" | Execution Time: ");
    //     Serial.print(end_time - start_time);
    //     Serial.println(" us");
    //     return result;
    // }
}
};

#endif
