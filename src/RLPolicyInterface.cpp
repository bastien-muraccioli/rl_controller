#include "RLPolicyInterface.h"
#include <mc_rtc/logging.h>
#include <cmath>
#include <random>
#include <filesystem>

RLPolicyInterface::RLPolicyInterface(const std::string & policyPath)
: isLoaded_(false), policyPath_(policyPath)
{
  loadPolicy(policyPath);
}

RLPolicyInterface::RLPolicyInterface()
: isLoaded_(true), policyPath_("")
{
  mc_rtc::log::info("RLPolicyInterface: Using dummy policy for testing");
}

RLPolicyInterface::~RLPolicyInterface()
{
  //TODO : cleanup any loaded models here
}

void RLPolicyInterface::loadPolicy(const std::string & path)
{
  try
  {
    mc_rtc::log::info("Loading RL policy from: {}", path);
    
#ifdef USE_ONNX
    if(path.size() >= 5 && path.substr(path.size() - 5) == ".onnx")
    {
      if(!std::filesystem::exists(path))
      {
        mc_rtc::log::error("Policy file does not exist: {}", path);
        isLoaded_ = false;
        return;
      }
      
      mc_rtc::log::info("Loading ONNX model...");
      
      onnxEnv_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "RLPolicy");
      
      Ort::SessionOptions sessionOptions;
      sessionOptions.SetIntraOpNumThreads(1);
      sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
      
      // try CUDA if available
      try 
      {
        OrtCUDAProviderOptions cuda_options{};
        sessionOptions.AppendExecutionProvider_CUDA(cuda_options);
        mc_rtc::log::info("CUDA provider added for ONNX inference");
      }
      catch(const std::exception&)
      {
        mc_rtc::log::info("CUDA not available, using CPU for ONNX inference");
      }
      
      onnxSession_ = std::make_unique<Ort::Session>(*onnxEnv_, path.c_str(), sessionOptions);
      
      Ort::AllocatorWithDefaultOptions allocator;
      
      size_t numInputNodes = onnxSession_->GetInputCount();
      if(numInputNodes != 1)
      {
        mc_rtc::log::error("Expected 1 input, got {}", numInputNodes);
        isLoaded_ = false;
        return;
      }
      
      inputNames_.clear();
      inputNamePtrs_.clear();
      std::string inputName = onnxSession_->GetInputNameAllocated(0, allocator).get();
      inputNames_.push_back(inputName);
      inputNamePtrs_.push_back(inputNames_[0].c_str());
      
      auto inputTypeInfo = onnxSession_->GetInputTypeInfo(0);
      auto inputTensorInfo = inputTypeInfo.GetTensorTypeAndShapeInfo();
      inputShape_ = inputTensorInfo.GetShape();
      
      size_t numOutputNodes = onnxSession_->GetOutputCount();
      if(numOutputNodes != 1)
      {
        mc_rtc::log::error("Expected 1 output, got {}", numOutputNodes);
        isLoaded_ = false;
        return;
      }
      
      outputNames_.clear();
      outputNamePtrs_.clear();
      std::string outputName = onnxSession_->GetOutputNameAllocated(0, allocator).get();
      outputNames_.push_back(outputName);
      outputNamePtrs_.push_back(outputNames_[0].c_str());
      
      auto outputTypeInfo = onnxSession_->GetOutputTypeInfo(0);
      auto outputTensorInfo = outputTypeInfo.GetTensorTypeAndShapeInfo();
      outputShape_ = outputTensorInfo.GetShape();
      
      if(inputShape_.size() != 2 || inputShape_[1] != getObservationSize())
      {
        mc_rtc::log::error("Input shape mismatch: expected [batch, {}], got [{}, {}]", 
                          getObservationSize(), inputShape_[0], inputShape_[1]);
        isLoaded_ = false;
        return;
      }
      
      if(outputShape_.size() != 2 || outputShape_[1] != getActionSize())
      {
        mc_rtc::log::error("Output shape mismatch: expected [batch, {}], got [{}, {}]", 
                          getActionSize(), outputShape_[0], outputShape_[1]);
        isLoaded_ = false;
        return;
      }
      
      memoryInfo_ = std::make_unique<Ort::MemoryInfo>(
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
      
      mc_rtc::log::info("Model metadata:");
      mc_rtc::log::info("  Input name: {}", inputNames_[0]);
      mc_rtc::log::info("  Input shape: [{}, {}]", inputShape_[0], inputShape_[1]);
      mc_rtc::log::info("  Output name: {}", outputNames_[0]);
      mc_rtc::log::info("  Output shape: [{}, {}]", outputShape_[0], outputShape_[1]);
      
      // test
      std::vector<float> testInput(getObservationSize(), 0.0f);
      std::vector<int64_t> testInputShape = {1, getObservationSize()};
      
      Ort::Value testInputTensor = Ort::Value::CreateTensor<float>(
        *memoryInfo_, testInput.data(), testInput.size(), 
        testInputShape.data(), testInputShape.size());
      
      auto testOutputTensors = onnxSession_->Run(
        Ort::RunOptions{nullptr}, inputNamePtrs_.data(), &testInputTensor, 1, 
        outputNamePtrs_.data(), outputNamePtrs_.size());
      
      if(testOutputTensors.size() != 1)
      {
        mc_rtc::log::error("Expected 1 output tensor, got {}", testOutputTensors.size());
        isLoaded_ = false;
        return;
      }
      
      isLoaded_ = true;
      mc_rtc::log::success("ONNX policy loaded successfully (input: {}, output: {})", 
                          getObservationSize(), getActionSize());
      return;
    }
#endif
    isLoaded_ = true;
  }
  catch(const std::exception & e)
  {
    mc_rtc::log::error_and_throw("Failed to load policy: {}", e.what());
  }
}

Eigen::VectorXd RLPolicyInterface::predict(const Eigen::VectorXd & observation)
{
  if(!isLoaded_)
  {
    mc_rtc::log::error("Policy not loaded, returning zero action");
    return Eigen::VectorXd::Zero(19);
  }
  
  if(observation.size() != getObservationSize())
  {
    mc_rtc::log::error("Observation size mismatch: expected {}, got {}", 
                       getObservationSize(), observation.size());
    return Eigen::VectorXd::Zero(19);
  }
  
#ifdef USE_ONNX
  if(!policyPath_.empty() && policyPath_.size() >= 5)
  {
    std::string ext = policyPath_.substr(policyPath_.size() - 5);
    if(ext == ".onnx")
    {
      try
      {
        return runOnnxInference(observation);
      }
      catch(const std::exception & e)
      {
        mc_rtc::log::error_and_throw("ONNX inference failed: {}", e.what());
      }
    }
  }
#endif
  return Eigen::VectorXd::Zero(19);
}

#ifdef USE_ONNX
Eigen::VectorXd RLPolicyInterface::runOnnxInference(const Eigen::VectorXd & observation)
{
  std::vector<float> inputData(observation.size());
  for(int i = 0; i < observation.size(); ++i)
  {
    inputData[i] = static_cast<float>(observation(i));
  }
  
  std::vector<int64_t> inputShape = {1, static_cast<int64_t>(observation.size())};
  Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
    *memoryInfo_, inputData.data(), inputData.size(),
    inputShape.data(), inputShape.size());
    
  auto outputTensors = onnxSession_->Run(
    Ort::RunOptions{nullptr}, 
    inputNamePtrs_.data(), &inputTensor, 1,
    outputNamePtrs_.data(), outputNamePtrs_.size());
  
  if(outputTensors.size() != 1)
  {
    throw std::runtime_error("Expected 1 output tensor, got " + std::to_string(outputTensors.size()));
  }
  
  float* outputData = outputTensors[0].GetTensorMutableData<float>();
  auto outputShape = outputTensors[0].GetTensorTypeAndShapeInfo().GetShape();
  
  if(outputShape.size() != 2 || outputShape[1] != getActionSize())
  {
    throw std::runtime_error("Output shape mismatch");
  }
  
  Eigen::VectorXd action(getActionSize());
  for(int i = 0; i < getActionSize(); ++i)
  {
    action(i) = static_cast<double>(outputData[i]);
  }
  
  return action;
}
#endif 