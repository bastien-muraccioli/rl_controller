#include "RLPolicyInterface.h"
#include <mc_rtc/logging.h>
#include <cmath>
#include <random>
#include <filesystem>

RLPolicyInterface::RLPolicyInterface(const std::string & policyPath)
: isLoaded_(false), policyPath_(policyPath), inputSize_(0), outputSize_(0)
{
  loadPolicy(policyPath);
}

RLPolicyInterface::~RLPolicyInterface()
{
}

void RLPolicyInterface::loadPolicy(const std::string & path)
{
  try
  {
    mc_rtc::log::info("Loading RL policy from: {}", path);
    
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
      
      // Extract input size from shape (handle both 1D and 2D cases)
      if(inputShape_.size() == 1)
      {
        // 1D case: [obs_size] - implicit batch dimension
        inputSize_ = static_cast<int>(inputShape_[0]);
        mc_rtc::log::info("Raw input shape from model: [{}] (1D - implicit batch size)", inputShape_[0]);
      }
      else if(inputShape_.size() == 2)
      {
        // 2D case: [batch_size, obs_size] or [obs_size, batch_size]
        mc_rtc::log::info("Raw input shape from model: [{}, {}] (2D - explicit batch size)", inputShape_[0], inputShape_[1]);
        
        // Determine which dimension is the observation size
        if(inputShape_[0] == 1 || inputShape_[0] == -1)
        {
          // Standard format: [batch_size, obs_size]
          inputSize_ = static_cast<int>(inputShape_[1]);
          mc_rtc::log::info("Model uses standard format: [batch_size, obs_size]");
        }
        else if(inputShape_[1] == 1 || inputShape_[1] == -1)
        {
          // Transposed format: [obs_size, batch_size]
          inputSize_ = static_cast<int>(inputShape_[0]);
          mc_rtc::log::info("Model uses transposed format: [obs_size, batch_size]");
        }
        else
        {
          // Neither dimension is 1 or -1, assume standard format
          inputSize_ = static_cast<int>(inputShape_[1]);
          mc_rtc::log::warning("Ambiguous input shape [{}, {}], assuming standard format [batch_size, obs_size]", 
                              inputShape_[0], inputShape_[1]);
        }
      }
      else
      {
        mc_rtc::log::error("Input shape should be 1D [obs_size] or 2D [batch_size, obs_size], got {}D", 
                          inputShape_.size());
        isLoaded_ = false;
        return;
      }
      
      // Extract output size from shape (handle both 1D and 2D cases)
      if(outputShape_.size() == 1)
      {
        // 1D case: [action_size] - implicit batch dimension
        outputSize_ = static_cast<int>(outputShape_[0]);
        mc_rtc::log::info("Raw output shape from model: [{}] (1D - implicit batch size)", outputShape_[0]);
      }
      else if(outputShape_.size() == 2)
      {
        // 2D case: [batch_size, action_size] or [action_size, batch_size]
        mc_rtc::log::info("Raw output shape from model: [{}, {}] (2D - explicit batch size)", outputShape_[0], outputShape_[1]);
        
        // Determine which dimension is the action size
        if(outputShape_[0] == 1 || outputShape_[0] == -1)
        {
          // Standard format: [batch_size, action_size]
          outputSize_ = static_cast<int>(outputShape_[1]);
          mc_rtc::log::info("Model uses standard output format: [batch_size, action_size]");
        }
        else if(outputShape_[1] == 1 || outputShape_[1] == -1)
        {
          // Transposed format: [action_size, batch_size]
          outputSize_ = static_cast<int>(outputShape_[0]);
          mc_rtc::log::info("Model uses transposed output format: [action_size, batch_size]");
        }
        else
        {
          // Neither dimension is 1 or -1, assume standard format
          outputSize_ = static_cast<int>(outputShape_[1]);
          mc_rtc::log::warning("Ambiguous output shape [{}, {}], assuming standard format [batch_size, action_size]", 
                              outputShape_[0], outputShape_[1]);
        }
      }
      else
      {
        mc_rtc::log::error("Output shape should be 1D [action_size] or 2D [batch_size, action_size], got {}D", 
                          outputShape_.size());
        isLoaded_ = false;
        return;
      }
      
      memoryInfo_ = std::make_unique<Ort::MemoryInfo>(
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
      
      mc_rtc::log::info("Model metadata:");
      mc_rtc::log::info("  Input name: {}", inputNames_[0]);
      if(inputShape_.size() == 1)
      {
        mc_rtc::log::info("  Input shape: [{}] (1D)", inputShape_[0]);
      }
      else
      {
        mc_rtc::log::info("  Input shape: [{}, {}] (2D)", inputShape_[0], inputShape_[1]);
      }
      mc_rtc::log::info("  Output name: {}", outputNames_[0]);
      if(outputShape_.size() == 1)
      {
        mc_rtc::log::info("  Output shape: [{}] (1D)", outputShape_[0]);
      }
      else
      {
        mc_rtc::log::info("  Output shape: [{}, {}] (2D)", outputShape_[0], outputShape_[1]);
      }
      
      // Test inference with appropriate tensor shape
      std::vector<float> testInput(getObservationSize(), 0.0f);
      std::vector<int64_t> testInputShape;
      if(inputShape_.size() == 1)
      {
        testInputShape = {getObservationSize()};
      }
      else
      {
        testInputShape = {1, getObservationSize()};
      }
      
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
    return Eigen::VectorXd::Zero(outputSize_);
  }
  
  if(observation.size() != getObservationSize())
  {
    mc_rtc::log::error("Observation size mismatch: expected {}, got {}", 
                       getObservationSize(), observation.size());
    return Eigen::VectorXd::Zero(outputSize_);
  }
  
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
        mc_rtc::log::error("ONNX inference failed: {}", e.what());
        mc_rtc::log::error("Returning zero action as fallback");
        return Eigen::VectorXd::Zero(outputSize_);
      }
    }
  }
  
  // Fallback: return zero action
  mc_rtc::log::warning("Using fallback zero action (no valid policy loaded)");
  return Eigen::VectorXd::Zero(outputSize_);
}

Eigen::VectorXd RLPolicyInterface::runOnnxInference(const Eigen::VectorXd & observation)
{
  // Convert Eigen vector to float vector
  std::vector<float> inputData(observation.size());
  for(int i = 0; i < observation.size(); ++i)
  {
    inputData[i] = static_cast<float>(observation(i));
  }
  
  // Create input tensor with shape matching the model's expected input
  std::vector<int64_t> inputShape;
  if(inputShape_.size() == 1)
  {
    // 1D model: [obs_size]
    inputShape = {static_cast<int64_t>(observation.size())};
  }
  else
  {
    // 2D model: [batch_size, obs_size] 
    inputShape = {1, static_cast<int64_t>(observation.size())};
  }
  
  Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
    *memoryInfo_, inputData.data(), inputData.size(),
    inputShape.data(), inputShape.size());
    
  // Verify tensor was created correctly
  auto createdShape = inputTensor.GetTensorTypeAndShapeInfo().GetShape();
  
  // Run inference
  auto outputTensors = onnxSession_->Run(
    Ort::RunOptions{nullptr}, 
    inputNamePtrs_.data(), &inputTensor, 1,
    outputNamePtrs_.data(), outputNamePtrs_.size());
  
  if(outputTensors.size() != 1)
  {
    throw std::runtime_error("Expected 1 output tensor, got " + std::to_string(outputTensors.size()));
  }
  
  // Get output data and shape
  float* outputData = outputTensors[0].GetTensorMutableData<float>();
  auto outputShape = outputTensors[0].GetTensorTypeAndShapeInfo().GetShape();
  
  if(outputShape.size() == 1 and outputShape[0] != getActionSize())
  {
    throw std::runtime_error("Output action size mismatch: expected " + 
                            std::to_string(getActionSize()) + 
                            ", got " + std::to_string(outputShape[0]));
  }
  else if(outputShape.size() == 2)
  {    
    // For standard format [batch, action], action size is in second dimension
    // For transposed format [action, batch], action size is in first dimension
    int64_t outputActionSize = outputShape[1];
    if(outputShape[0] == getActionSize() && outputShape[1] == 1)
    {
      // Handle transposed output case
      outputActionSize = outputShape[0];
      mc_rtc::log::warning("Detected transposed output format, adjusting...");
    }
    
    if(outputActionSize != getActionSize())
    {
      throw std::runtime_error("Output action size mismatch: expected " + 
                              std::to_string(getActionSize()) + 
                              ", got " + std::to_string(outputActionSize));
    }
  }
  else
  {
    throw std::runtime_error("Expected 1D or 2D output tensor, got " + std::to_string(outputShape.size()) + "D");
  }
  
  // Convert output to Eigen vector
  Eigen::VectorXd action(getActionSize());
  for(int i = 0; i < getActionSize(); ++i)
  {
    action(i) = static_cast<double>(outputData[i]);
  }
  
  return action;
}