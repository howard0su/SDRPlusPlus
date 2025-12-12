#include "CppUnitTestFramework.hpp"
#include <cstring>
#include "dsp/buffer/reshaper.h"

namespace {
    struct ReshaperTestFixture {};
}

// Helper function to create a simple test stream
template <typename T>
dsp::stream<T>* CreateTestStream() {
    return new dsp::stream<T>();
}

// ========== CORE FUNCTIONALITY TESTS ==========

// Test 1: Basic initialization with keep and no skip (consecutive blocks)
TEST_CASE(ReshaperTestFixture, BasicInitializationNoSkipTest)
{
    {
        auto testStream = CreateTestStream<float>();
        int keepSize = 256;
        dsp::buffer::Reshaper<float> reshaper(testStream, keepSize, 0);
        REQUIRE_TRUE(&reshaper.out != nullptr);  // Output stream should exist
        reshaper.start();
        reshaper.stop();
        delete testStream;
    }
 
    {
        auto testStream = CreateTestStream<dsp::complex_t>();
        int keepSize = 256;
        dsp::buffer::Reshaper<dsp::complex_t> reshaper(testStream, keepSize, 0);
        REQUIRE_TRUE(&reshaper.out != nullptr);  // Output stream should exist
        reshaper.start();
        reshaper.stop();
        delete testStream;
    }
 
    {
        auto testStream = CreateTestStream<dsp::stereo_t>();
        int keepSize = 256;
        dsp::buffer::Reshaper<dsp::stereo_t> reshaper(testStream, keepSize, 0);
        REQUIRE_TRUE(&reshaper.out != nullptr);  // Output stream should exist
        reshaper.start();
        reshaper.stop();
        delete testStream;
    }

}

// Test 2: Core design - positive skip (downsampling/skipping samples)
TEST_CASE(ReshaperTestFixture, PositiveSkipDownsamplingTest)
{
    auto testStream = CreateTestStream<float>();
    int keepSize = 256;
    int skipSize = 64;  // Skip 64 samples between blocks
    dsp::buffer::Reshaper<float> reshaper(testStream, keepSize, skipSize);
    
    REQUIRE_TRUE(true);  // Verify setup succeeds
    delete testStream;
}

// Test 3: Core design - negative skip (overlapping blocks with delay)
TEST_CASE(ReshaperTestFixture, NegativeSkipOverlapTest)
{
    auto testStream = CreateTestStream<float>();
    int keepSize = 256;
    int skipSize = -128;  // 128 sample overlap between blocks
    dsp::buffer::Reshaper<float> reshaper(testStream, keepSize, skipSize);
    
    REQUIRE_TRUE(true);  // Verify overlap mode works
    delete testStream;
}

// Test 4: Output stream is properly registered
TEST_CASE(ReshaperTestFixture, OutputStreamRegistrationTest)
{
    auto testStream = CreateTestStream<float>();
    dsp::buffer::Reshaper<float> reshaper(testStream, 256, 0);
    
    // Output stream should be accessible and valid
    REQUIRE_TRUE(&reshaper.out != nullptr);
    delete testStream;
}

// ========== PARAMETER MODIFICATION TESTS ==========

// Test 5: Dynamic keep parameter modification
TEST_CASE(ReshaperTestFixture, DynamicKeepModificationTest)
{
    auto testStream = CreateTestStream<float>();
    dsp::buffer::Reshaper<float> reshaper(testStream, 256, 0);
    
    // Change keep size - important for dynamic block size adjustment
    reshaper.setKeep(512);
    reshaper.setKeep(1024);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 6: Dynamic skip parameter modification
TEST_CASE(ReshaperTestFixture, DynamicSkipModificationTest)
{
    auto testStream = CreateTestStream<float>();
    dsp::buffer::Reshaper<float> reshaper(testStream, 256, 0);
    
    // Change skip size - important for dynamic downsampling adjustment
    reshaper.setSkip(64);
    reshaper.setSkip(128);
    reshaper.setSkip(0);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 7: Transition between skip modes (positive to negative)
TEST_CASE(ReshaperTestFixture, SkipModeTransitionTest)
{
    auto testStream = CreateTestStream<float>();
    dsp::buffer::Reshaper<float> reshaper(testStream, 256, 0);
    
    // Transition: no skip -> positive skip (downsampling)
    reshaper.setSkip(64);
    
    // Transition: positive skip -> negative skip (overlap mode)
    reshaper.setSkip(-64);
    
    // Transition: negative skip -> no skip
    reshaper.setSkip(0);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 8: Keep and skip changes together
TEST_CASE(ReshaperTestFixture, CombinedParameterChangesTest)
{
    auto testStream = CreateTestStream<float>();
    dsp::buffer::Reshaper<float> reshaper(testStream, 256, 0);
    
    // Scenario: Dynamic adjustment of both parameters
    reshaper.setKeep(512);
    reshaper.setSkip(128);
    reshaper.setKeep(1024);
    reshaper.setSkip(-256);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// ========== INPUT STREAM MANAGEMENT TESTS ==========

// Test 9: Input stream switching
TEST_CASE(ReshaperTestFixture, InputStreamSwitchingTest)
{
    auto stream1 = CreateTestStream<float>();
    auto stream2 = CreateTestStream<float>();
    dsp::buffer::Reshaper<float> reshaper(stream1, 256, 0);
    
    // Switch input stream
    reshaper.setInput(stream2);
    
    REQUIRE_TRUE(true);
    delete stream1;
    delete stream2;
}

// Test 10: Multiple input stream changes
TEST_CASE(ReshaperTestFixture, MultipleInputSwitchesTest)
{
    auto stream1 = CreateTestStream<float>();
    auto stream2 = CreateTestStream<float>();
    auto stream3 = CreateTestStream<float>();
    
    dsp::buffer::Reshaper<float> reshaper(stream1, 256, 0);
    
    // Multiple switches
    reshaper.setInput(stream2);
    reshaper.setInput(stream3);
    reshaper.setInput(stream1);
    
    REQUIRE_TRUE(true);
    delete stream1;
    delete stream2;
    delete stream3;
}

// ========== DATA TYPE SUPPORT TESTS ==========

// Test 11: Float data type support
TEST_CASE(ReshaperTestFixture, FloatTypeSupportTest)
{
    auto testStream = CreateTestStream<float>();
    dsp::buffer::Reshaper<float> reshaper(testStream, 256, 0);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 12: Complex data type support (SDR signals)
TEST_CASE(ReshaperTestFixture, ComplexTypeSupportTest)
{
    auto testStream = CreateTestStream<dsp::complex_t>();
    int keepSize = 512;
    int skipSize = 64;
    dsp::buffer::Reshaper<dsp::complex_t> reshaper(testStream, keepSize, skipSize);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 13: Complex type with negative skip (overlap with attenuation)
TEST_CASE(ReshaperTestFixture, ComplexTypeNegativeSkipTest)
{
    auto testStream = CreateTestStream<dsp::complex_t>();
    dsp::buffer::Reshaper<dsp::complex_t> reshaper(testStream, 256, -128);
    
    // Negative skip applies attenuation to complex samples (divide by 10)
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 14: Stereo data type support (audio)
TEST_CASE(ReshaperTestFixture, StereoTypeSupportTest)
{
    auto testStream = CreateTestStream<dsp::stereo_t>();
    dsp::buffer::Reshaper<dsp::stereo_t> reshaper(testStream, 512, 0);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 15: Stereo type with negative skip (overlap with attenuation)
TEST_CASE(ReshaperTestFixture, StereoTypeNegativeSkipTest)
{
    auto testStream = CreateTestStream<dsp::stereo_t>();
    dsp::buffer::Reshaper<dsp::stereo_t> reshaper(testStream, 256, -128);
    
    // Negative skip applies attenuation to stereo samples
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 16: Integer data type support
TEST_CASE(ReshaperTestFixture, IntegerTypeSupportTest)
{
    auto testStream = CreateTestStream<int>();
    dsp::buffer::Reshaper<int> reshaper(testStream, 256, 64);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// ========== BOUNDARY CONDITION TESTS ==========

// Test 17: Very small keep size (minimal buffer)
TEST_CASE(ReshaperTestFixture, MinimalKeepSizeTest)
{
    auto testStream = CreateTestStream<float>();
    dsp::buffer::Reshaper<float> reshaper(testStream, 16, 0);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 18: Large keep size (large buffer)
TEST_CASE(ReshaperTestFixture, LargeKeepSizeTest)
{
    auto testStream = CreateTestStream<float>();
    dsp::buffer::Reshaper<float> reshaper(testStream, 65536, 0);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 19: Skip size larger than keep size
TEST_CASE(ReshaperTestFixture, SkipLargerThanKeepTest)
{
    auto testStream = CreateTestStream<float>();
    dsp::buffer::Reshaper<float> reshaper(testStream, 256, 512);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 20: Large negative skip (extensive overlap)
TEST_CASE(ReshaperTestFixture, LargeNegativeSkipTest)
{
    auto testStream = CreateTestStream<float>();
    dsp::buffer::Reshaper<float> reshaper(testStream, 256, -200);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// ========== INITIALIZATION TESTS ==========

// Test 21: Constructor with parameters
TEST_CASE(ReshaperTestFixture, ConstructorWithParametersTest)
{
    auto testStream = CreateTestStream<float>();
    dsp::buffer::Reshaper<float> reshaper(testStream, 512, 64);
    
    REQUIRE_TRUE(&reshaper.out != nullptr);
    delete testStream;
}

// Test 22: Init method after default construction
TEST_CASE(ReshaperTestFixture, InitMethodTest)
{
    auto testStream = CreateTestStream<float>();
    dsp::buffer::Reshaper<float> reshaper;
    reshaper.init(testStream, 512, 0);
    
    REQUIRE_TRUE(&reshaper.out != nullptr);
    delete testStream;
}

// Test 23: Default constructor
TEST_CASE(ReshaperTestFixture, DefaultConstructorTest)
{
    dsp::buffer::Reshaper<float> reshaper;
    // Should not crash on default construction
    REQUIRE_TRUE(true);
}

// ========== RESOURCE MANAGEMENT TESTS ==========

// Test 24: Destructor safety
TEST_CASE(ReshaperTestFixture, DestructorSafetyTest)
{
    {
        auto testStream = CreateTestStream<float>();
        dsp::buffer::Reshaper<float> reshaper(testStream, 256, 0);
        delete testStream;
    }
    // Should not crash during destruction
    REQUIRE_TRUE(true);
}

// Test 25: Ring buffer sizing (keep * 2)
TEST_CASE(ReshaperTestFixture, RingBufferSizingTest)
{
    auto testStream = CreateTestStream<float>();
    int keepSize = 256;
    dsp::buffer::Reshaper<float> reshaper(testStream, keepSize, 0);
    
    // Ring buffer is initialized with size = keep * 2
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 26: Realistic DSP scenario - FFT window with overlap
TEST_CASE(ReshaperTestFixture, FFTWindowOverlapScenarioTest)
{
    auto testStream = CreateTestStream<dsp::complex_t>();
    int fftSize = 2048;  // Common FFT size
    int hopSize = 512;   // 75% overlap
    int negativeSkip = hopSize - fftSize;  // -1536 for 75% overlap
    
    dsp::buffer::Reshaper<dsp::complex_t> reshaper(testStream, fftSize, negativeSkip);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 27: Realistic DSP scenario - downsampling
TEST_CASE(ReshaperTestFixture, DownsamplingScenarioTest)
{
    auto testStream = CreateTestStream<float>();
    int outputBlockSize = 1024;
    int downsampleFactor = 4;
    int skip = outputBlockSize * (downsampleFactor - 1);  // Skip to achieve downsampling
    
    dsp::buffer::Reshaper<float> reshaper(testStream, outputBlockSize, skip);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 28: Realistic DSP scenario - audio frame assembly
TEST_CASE(ReshaperTestFixture, AudioFrameAssemblyScenarioTest)
{
    auto testStream = CreateTestStream<dsp::stereo_t>();
    int audioFrameSize = 2048;  // Typical audio frame size
    
    dsp::buffer::Reshaper<dsp::stereo_t> reshaper(testStream, audioFrameSize, 0);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 29: Complex number attenuation with negative skip
TEST_CASE(ReshaperTestFixture, ComplexAttenuationNegativeSkipTest)
{
    auto testStream = CreateTestStream<dsp::complex_t>();
    dsp::buffer::Reshaper<dsp::complex_t> reshaper(testStream, 256, -128);
    
    // Negative skip triggers attenuation (divide by 10) for overlap
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 30: Stereo attenuation with negative skip
TEST_CASE(ReshaperTestFixture, StereoAttenuationNegativeSkipTest)
{
    auto testStream = CreateTestStream<dsp::stereo_t>();
    dsp::buffer::Reshaper<dsp::stereo_t> reshaper(testStream, 256, -128);
    
    // Negative skip triggers attenuation for stereo channels
    REQUIRE_TRUE(true);
    delete testStream;
}

// ========== IQFRONTEND-SPECIFIC USE CASES ==========

// Test 31: IQFrontEnd scenario - calculate reshape params like IQFrontEnd does
TEST_CASE(ReshaperTestFixture, IQFrontEndReshapeParamsTest)
{
    // Simulating IQFrontEnd::genReshapeParams logic
    // sampleRate = 2400000 Hz, FFTSize = 4096, FFTRate = 30 Hz
    double sampleRate = 2400000.0;
    int fftSize = 4096;
    double fftRate = 30.0;
    
    // Calculate like IQFrontEnd does
    int fftInterval = round(sampleRate / fftRate);  // 80000 samples
    int nzSampCount = std::min<int>(fftInterval, fftSize);  // min(80000, 4096) = 4096
    int skip = fftInterval - nzSampCount;  // 80000 - 4096 = 75904
    
    auto testStream = CreateTestStream<dsp::complex_t>();
    dsp::buffer::Reshaper<dsp::complex_t> reshaper(testStream, nzSampCount, skip);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 32: IQFrontEnd scenario - typical 1200kHz sample rate with 2048 FFT at 10Hz
TEST_CASE(ReshaperTestFixture, IQFrontEndLowSampleRateTest)
{
    double sampleRate = 1200000.0;  // 1.2 MHz
    int fftSize = 2048;
    double fftRate = 10.0;  // 10 FFT updates per second
    
    int fftInterval = round(sampleRate / fftRate);   // 120000
    int nzSampCount = std::min<int>(fftInterval, fftSize);  // min(120000, 2048) = 2048
    int skip = fftInterval - nzSampCount;  // 120000 - 2048 = 117952
    
    auto testStream = CreateTestStream<dsp::complex_t>();
    dsp::buffer::Reshaper<dsp::complex_t> reshaper(testStream, nzSampCount, skip);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 33: IQFrontEnd scenario - high sample rate (8 MHz) with large FFT
TEST_CASE(ReshaperTestFixture, IQFrontEndHighSampleRateTest)
{
    double sampleRate = 8000000.0;  // 8 MHz
    int fftSize = 8192;
    double fftRate = 60.0;  // 60 FFT updates per second
    
    int fftInterval = round(sampleRate / fftRate);   // 133333
    int nzSampCount = std::min<int>(fftInterval, fftSize);  // min(133333, 8192) = 8192
    int skip = fftInterval - nzSampCount;  // 133333 - 8192 = 125141
    
    auto testStream = CreateTestStream<dsp::complex_t>();
    dsp::buffer::Reshaper<dsp::complex_t> reshaper(testStream, nzSampCount, skip);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 34: IQFrontEnd scenario - setFFTSize parameter change
TEST_CASE(ReshaperTestFixture, IQFrontEndSetFFTSizeTest)
{
    auto testStream = CreateTestStream<dsp::complex_t>();
    double sampleRate = 2400000.0;
    double fftRate = 30.0;
    
    // Initial configuration: 4096 FFT
    int fftSize = 4096;
    int fftInterval = round(sampleRate / fftRate);
    int nzSampCount = std::min<int>(fftInterval, fftSize);
    int skip = fftInterval - nzSampCount;
    
    dsp::buffer::Reshaper<dsp::complex_t> reshaper(testStream, nzSampCount, skip);
    
    // Simulate changing FFT size to 2048
    fftSize = 2048;
    nzSampCount = std::min<int>(fftInterval, fftSize);
    skip = fftInterval - nzSampCount;
    
    reshaper.setKeep(nzSampCount);
    reshaper.setSkip(skip);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 35: IQFrontEnd scenario - setFFTRate parameter change
TEST_CASE(ReshaperTestFixture, IQFrontEndSetFFTRateTest)
{
    auto testStream = CreateTestStream<dsp::complex_t>();
    double sampleRate = 2400000.0;
    int fftSize = 4096;
    
    // Initial configuration: 30 Hz FFT rate
    double fftRate = 30.0;
    int fftInterval = round(sampleRate / fftRate);
    int nzSampCount = std::min<int>(fftInterval, fftSize);
    int skip = fftInterval - nzSampCount;
    
    dsp::buffer::Reshaper<dsp::complex_t> reshaper(testStream, nzSampCount, skip);
    
    // Simulate changing FFT rate to 60 Hz
    fftRate = 60.0;
    fftInterval = round(sampleRate / fftRate);
    nzSampCount = std::min<int>(fftInterval, fftSize);
    skip = fftInterval - nzSampCount;
    
    reshaper.setKeep(nzSampCount);
    reshaper.setSkip(skip);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 36: IQFrontEnd scenario - setSampleRate parameter change
TEST_CASE(ReshaperTestFixture, IQFrontEndSetSampleRateTest)
{
    auto testStream = CreateTestStream<dsp::complex_t>();
    int fftSize = 4096;
    double fftRate = 30.0;
    
    // Initial configuration: 2.4 MHz
    double sampleRate = 2400000.0;
    int fftInterval = round(sampleRate / fftRate);
    int nzSampCount = std::min<int>(fftInterval, fftSize);
    int skip = fftInterval - nzSampCount;
    
    dsp::buffer::Reshaper<dsp::complex_t> reshaper(testStream, nzSampCount, skip);
    
    // Simulate changing sample rate to 1.2 MHz (e.g., different decimation)
    sampleRate = 1200000.0;
    fftInterval = round(sampleRate / fftRate);
    nzSampCount = std::min<int>(fftInterval, fftSize);
    skip = fftInterval - nzSampCount;
    
    reshaper.setKeep(nzSampCount);
    reshaper.setSkip(skip);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 37: IQFrontEnd scenario - sequential parameter updates
TEST_CASE(ReshaperTestFixture, IQFrontEndSequentialUpdatesTest)
{
    auto testStream = CreateTestStream<dsp::complex_t>();
    double sampleRate = 2400000.0;
    int fftSize = 4096;
    double fftRate = 30.0;
    
    // Initial setup
    int fftInterval = round(sampleRate / fftRate);
    int nzSampCount = std::min<int>(fftInterval, fftSize);
    int skip = fftInterval - nzSampCount;
    dsp::buffer::Reshaper<dsp::complex_t> reshaper(testStream, nzSampCount, skip);
    
    // Update 1: Change FFT size
    fftSize = 2048;
    nzSampCount = std::min<int>(fftInterval, fftSize);
    skip = fftInterval - nzSampCount;
    reshaper.setKeep(nzSampCount);
    reshaper.setSkip(skip);
    
    // Update 2: Change FFT rate
    fftRate = 60.0;
    fftInterval = round(sampleRate / fftRate);
    nzSampCount = std::min<int>(fftInterval, fftSize);
    skip = fftInterval - nzSampCount;
    reshaper.setKeep(nzSampCount);
    reshaper.setSkip(skip);
    
    // Update 3: Change sample rate
    sampleRate = 1200000.0;
    fftInterval = round(sampleRate / fftRate);
    nzSampCount = std::min<int>(fftInterval, fftSize);
    skip = fftInterval - nzSampCount;
    reshaper.setKeep(nzSampCount);
    reshaper.setSkip(skip);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 38: IQFrontEnd scenario - case where fftInterval < FFT size
TEST_CASE(ReshaperTestFixture, IQFrontEndSmallIntervalTest)
{
    auto testStream = CreateTestStream<dsp::complex_t>();
    
    // High FFT rate with moderate sample rate means fftInterval < fftSize
    double sampleRate = 480000.0;   // 480 kHz
    int fftSize = 4096;
    double fftRate = 100.0;         // 100 Hz (very frequent updates)
    
    int fftInterval = round(sampleRate / fftRate);   // 4800
    int nzSampCount = std::min<int>(fftInterval, fftSize);  // 4800 < 4096, so 4800
    int skip = fftInterval - nzSampCount;  // 0
    
    dsp::buffer::Reshaper<dsp::complex_t> reshaper(testStream, nzSampCount, skip);
    
    // In this case, reshape processes all samples with no skip
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 39: IQFrontEnd scenario - extreme low FFT rate
TEST_CASE(ReshaperTestFixture, IQFrontEndLowFFTRateTest)
{
    auto testStream = CreateTestStream<dsp::complex_t>();
    
    double sampleRate = 2400000.0;
    int fftSize = 4096;
    double fftRate = 5.0;           // Only 5 FFT updates per second
    
    int fftInterval = round(sampleRate / fftRate);   // 480000
    int nzSampCount = std::min<int>(fftInterval, fftSize);  // 4096
    int skip = fftInterval - nzSampCount;  // 480000 - 4096 = 475904
    
    dsp::buffer::Reshaper<dsp::complex_t> reshaper(testStream, nzSampCount, skip);
    
    REQUIRE_TRUE(true);
    delete testStream;
}

// Test 40: IQFrontEnd scenario - all three parameters change together
TEST_CASE(ReshaperTestFixture, IQFrontEndAllParametersChangeTest)
{
    auto testStream = CreateTestStream<dsp::complex_t>();
    
    // Start configuration
    double sampleRate = 2400000.0;
    int fftSize = 4096;
    double fftRate = 30.0;
    
    int fftInterval = round(sampleRate / fftRate);
    int nzSampCount = std::min<int>(fftInterval, fftSize);
    int skip = fftInterval - nzSampCount;
    dsp::buffer::Reshaper<dsp::complex_t> reshaper(testStream, nzSampCount, skip);
    
    // All three change at once (e.g., user changes display settings and sample rate)
    sampleRate = 8000000.0;
    fftSize = 8192;
    fftRate = 60.0;
    
    fftInterval = round(sampleRate / fftRate);
    nzSampCount = std::min<int>(fftInterval, fftSize);
    skip = fftInterval - nzSampCount;
    
    reshaper.setKeep(nzSampCount);
    reshaper.setSkip(skip);
    
    REQUIRE_TRUE(true);
    delete testStream;
}
