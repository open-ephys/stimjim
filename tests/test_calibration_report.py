#!/usr/bin/env python3
"""Test the sketch's D-command formatter without hardware or Arduino libraries.

Run: python3 tests/test_calibration_report.py
Requires a C++ compiler with AddressSanitizer and UndefinedBehaviorSanitizer.
Set CXX to select a compiler (defaults to c++).
"""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile


HARNESS = r'''
#include <cassert>
#include <climits>
#include <cstdio>
#include <limits>
#include <string>

struct Calibration {
    // Example calibration values from the repository README.
    float adcOffset25[2] = {-11.03f, -11.15f};
    float adcOffset10[2] = {-10.22f, -10.34f};
    int currentOffsets[2] = {11, 8};
    int voltageOffsets[2] = {0, 0};
} Stimjim;

struct SerialStub {
    std::string output;
    void println(const char* text) { output += text; output += "\r\n"; }
} Serial;

void report() {
__D_COMMAND_BODY__
}

int main() {
    const std::string expected =
        "ADC offsets (+-2.5V): -11.030000, -11.150000\r\n"
        "ADC offsets (+-10V): -10.220000, -10.340000\r\n"
        "current offsets: 11, 8\r\nvoltage offsets: 0, 0\r\n\r\n";
    for (int i = 0; i < 1000; ++i) {
        Serial.output.clear();
        report();
        assert(Serial.output == expected);
    }

    // Force a report too large for the buffer: reject rather than print a
    // plausible but incomplete calibration report or write past the buffer.
    Stimjim.adcOffset25[0] = Stimjim.adcOffset25[1] = std::numeric_limits<float>::max();
    Stimjim.adcOffset10[0] = Stimjim.adcOffset10[1] = -std::numeric_limits<float>::max();
    Stimjim.currentOffsets[0] = INT_MIN;
    Stimjim.currentOffsets[1] = INT_MAX;
    Serial.output.clear();
    report();
    assert(Serial.output == "ERROR: calibration report formatting failed or truncated\r\n");

    // Non-finite calibration values must also remain memory-safe. Their exact
    // spelling is libc-dependent, so do not impose a particular representation.
    Stimjim.adcOffset25[0] = std::numeric_limits<float>::infinity();
    Stimjim.adcOffset25[1] = -std::numeric_limits<float>::infinity();
    Stimjim.adcOffset10[0] = Stimjim.adcOffset10[1] = std::numeric_limits<float>::quiet_NaN();
    Serial.output.clear();
    report();
    assert(!Serial.output.empty());
}
'''


def main():
    sketch = Path(__file__).resolve().parents[1] / "stimjimPulser/stimjimPulser.ino"
    source = sketch.read_text()
    # Compile the actual command body, so reverting to sprintf reproduces the
    # overflow under ASan; a copied formatter could silently diverge from it.
    start = "} else if (comBuf[0] == 'D') {"
    end = "} else if (comBuf[0] == 'R') {"
    if source.count(start) != 1 or source.count(end) != 1:
        raise RuntimeError("Command layout changed; update the D-command extraction")
    body = source.split(start, 1)[1].split(end, 1)[0]

    with tempfile.TemporaryDirectory(prefix="stimjim-calibration-test-") as directory:
        cpp = Path(directory) / "calibration_report.cpp"
        executable = Path(directory) / "calibration_report"
        cpp.write_text(HARNESS.replace("__D_COMMAND_BODY__", body))
        compiler = shlex.split(os.environ.get("CXX", "c++"))
        subprocess.run(compiler + [
            "-std=c++11", "-g", "-O1", "-fsanitize=address,undefined",
            "-fno-sanitize-recover=all", "-fno-omit-frame-pointer",
            str(cpp), "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True)
    print("PASS: repeated normal reports, truncation rejection, and non-finite values")


if __name__ == "__main__":
    main()
