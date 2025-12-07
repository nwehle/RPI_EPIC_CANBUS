/*
 * GPIO Interface for Raspberry Pi using libgpiod
 * Handles digital inputs, outputs, VSS, and PWM
 * Supports dynamic pin configuration
 */

#ifndef GPIO_INTERFACE_H
#define GPIO_INTERFACE_H

#include <cstdint>
#include <string>
#include <vector>
#include <gpiod.h>

class GPIOInterface {
public:
    // Maximum supported pins per category
    static constexpr int MAX_DIGITAL_INPUTS = 16;
    static constexpr int MAX_DIGITAL_OUTPUTS = 16;
    static constexpr int MAX_VSS_INPUTS = 4;
    static constexpr int MAX_PWM_OUTPUTS = 10;

    GPIOInterface();
    ~GPIOInterface();

    // Initialize GPIO interface
    // chipPath: e.g., "/dev/gpiochip0" or "/dev/gpiochip4" (Pi 5)
    bool init(const std::string& chipPath = "/dev/gpiochip4");

    // Configure pins dynamically
    bool configureDigitalInputs(const std::vector<int>& pins);
    bool configureDigitalOutputs(const std::vector<int>& pins);
    bool configureVssInputs(const std::vector<int>& pins);
    bool configurePwmOutputs(const std::vector<int>& pins);

    // Close GPIO interface
    void close();

    // Check if interface is open
    bool isOpen() const { return chip_ != nullptr; }

    // Get configured pin counts
    int getNumDigitalInputs() const { return numDigitalInputs_; }
    int getNumDigitalOutputs() const { return numDigitalOutputs_; }
    int getNumVssInputs() const { return numVssInputs_; }
    int getNumPwmOutputs() const { return numPwmOutputs_; }

    // Read all digital inputs as bitfield (up to 16 bits)
    // Returns inverted values (LOW = 1, HIGH = 0) to match Arduino behavior
    uint16_t readDigitalInputs();

    // Set digital outputs from bitfield
    void setDigitalOutputs(uint16_t bits);

    // Set PWM outputs from bitfield (on/off only for now)
    void setPwmOutputs(uint16_t bits);

    // Get current VSS edge counts (for rate calculation)
    void getVSSCounts(uint32_t* counts);

    // Update VSS counts by polling (call frequently)
    void pollVSS();

private:
    struct gpiod_chip* chip_;

    // Line handles (dynamically sized)
    std::vector<struct gpiod_line*> digitalInputLines_;
    std::vector<struct gpiod_line*> digitalOutputLines_;
    std::vector<struct gpiod_line*> vssInputLines_;
    std::vector<struct gpiod_line*> pwmOutputLines_;

    // Pin counts
    int numDigitalInputs_;
    int numDigitalOutputs_;
    int numVssInputs_;
    int numPwmOutputs_;

    // VSS state tracking
    std::vector<uint32_t> vssEdgeCounts_;
    std::vector<int> vssLastState_;

    bool configured_;
};

#endif // GPIO_INTERFACE_H
